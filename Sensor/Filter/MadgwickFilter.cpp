#include "MadgwickFilter.h"

/**
 * Fast inverse square root function.
 * Uses some magic numbers to achieve better performance
 * for 1/sqrt(x). Precise to 5 digits and almost 3 times
 * faster.
 *
 * Credit goes to Jan Kadlec, <rrrola@gmail.com>
 * See: http://rrrola.wz.cz/inv_sqrt.html
 *
 * WARNING: This can only be used as long as a float is 32 bits long.
 * Not to be used on 64bit systems. Can be changed with
 * by using the define SYSTEM_64BIT
 */
#ifndef SYSTEM_64BIT
float invSqrt(float x) {
	union { float f; uint32_t u; } y = {x};
	y.u = 0x5F1FFFF9ul - (y.u >> 1);
	return 0.703952253f * y.f * (2.38924456f - x * y.f * y.f);
}
#else
float invSqrt(float x) {
	return 1.0f/sqrt(x);
}
#endif

/**
 * The constructor for a new filter
 *
 * @param float filter_interval:	The update interval in seconds
 * @param float filter_gain:		The gain of the filter
 */
MadgwickFilter::MadgwickFilter(float filter_interval, float filter_gain){
	this->beta = filter_gain;
	this->interval = filter_interval;
}


/*
 * This method uses data from gyroscope, accelerometer and magnetometer to estimate
 * the position of a spacecraft. Uses a variation of Sebastian Madgwicks filter,
 * See:
 * "An efficient orientation filter for inertial and inertial/magnetic sensor arrays",
 * Sebastian Madgwicks, April 2010.
 *
 * The original code comes from:
 * http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * Includes an updated version of the gradient descend algorithm.
 * See:
 * http://diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root?id=705844%3ATopic%3A1018435&page=4#comments
 * Jeroen van de Mortel, August 2014
 *
 * Changes were made by me to avoid repeated calculations and speed up the filter
 *
 * Uses a fast inverse square root implementation.
 *
 * The gyroscope measurements need to be scaled to rad/s, the accelerometer and magnetometer data
 * does not need to be properly scaled but should to be calibrated. This holds especially for the
 * magnetometer values, a simple offset/scale model might not be sufficient
 *
 * Make sure that the coordinate systems of the individual sensors are correctly aligned.
 *
 * For the magnetometer calibration an ellipsoid fit algorithm might be useful.
 * See: http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
 * Yury Petrov, September 2015
 *
 * @params: 	float 	gx, gy, gz:		The gyroscope measurements in rad/s
 * 				float	ax, ay, az:		The accelerometer values
 * 				float	mx, my, mz:		The magnetometer values
 */
void MadgwickFilter::filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float scale; // Used to scale xyz values to length 1

	/* Rate of change of quaternion from gyroscope */
	float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	/* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		/* Normalise accelerometer measurement */
		scale = invSqrt(ax * ax + ay * ay + az * az);
		ax *= scale;
		ay *= scale;
		az *= scale;

		/* Normalise magnetometer measurement */
		scale = invSqrt(mx * mx + my * my + mz * mz);
		mx *= scale;
		my *= scale;
		mz *= scale;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q0mx = 2.0f * q0 * mx;
		float _2q0my = 2.0f * q0 * my;
		float _2q0mz = 2.0f * q0 * mz;
		float _2q1mx = 2.0f * q1 * mx;
		float _2q0 = 2.0f * q0;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q0q2 = 2.0f * q0 * q2;
		float _2q2q3 = 2.0f * q2 * q3;
		float q0q0 = q0 * q0;
		float q0q1 = q0 * q1;
		float q0q2 = q0 * q2;
		float q0q3 = q0 * q3;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		float _2bx = sqrt(hx * hx + hy * hy);
		float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		float _4bx = 2.0f * _2bx;
		float _4bz = 2.0f * _2bz;
		float _8bx = 2.0f * _4bx;
		float _8bz = 2.0f * _4bz;

		// Gradient decent algorithm corrective step
		float t0 = 0.5 - q1q1 - q2q2;
		float t1 = (2.0f*(q1q3 - q0q2) - ax);
		float t2 = (2.0f*(q0q1 + q2q3) - ay);
		float t3 = 4.0f * (2.0f*t0 - az);
		float t4 = (_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx);
		float t5 = _4bx*(q0q2 + q1q3) + _4bz*t0 - mz;
		float t6 = _4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my;

		float s0 = -_2q2 * t1 + _2q1 * t2                          - _4bz * q2  * t4 + ( -_4bx*q3 + _4bz*q1) * t6 +  _4bx * q2              * t5;
		float s1 =  _2q3 * t1 + _2q0 * t2 - q1 * t3                + _4bz * q3  * t4 + (  _4bx*q2 + _4bz*q0) * t6 + (_4bx * q3 - _8bz * q1) * t5;
		float s2 = -_2q0 * t1 + _2q3 * t2 - q2 * t3  + (-_8bx * q2 - _4bz * q0) * t4 + (  _4bx*q1 + _4bz*q3) * t6 + (_4bx * q0 - _8bz * q2) * t5;
		float s3 =  _2q1 * t1 + _2q2 * t2            + (-_8bx * q3 + _4bz * q1) * t4 + ( -_4bx*q0 + _4bz*q2) * t6 +  _4bx * q1              * t5;

		// Normalize step magnitude
		scale = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= scale;
		s1 *= scale;
		s2 *= scale;
		s3 *= scale;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * interval;
	q1 += qDot2 * interval;
	q2 += qDot3 * interval;
	q3 += qDot4 * interval;

	// Normalize quaternion
	scale = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= scale;
	q1 *= scale;
	q2 *= scale;
	q3 *= scale;
}

/**
 * Calculates Euler angles (roll,pitch,yaw) from the quaternion q.
 * The resulting angles range from -180 to 180 degrees.
 *
 * The pitch angle is positive if the nose of the spacecraft is pointing up
 *	TODO: Direction of rotations
 *
 * @param float* data: A pointer to a float[3] array. The values are
 * 						stored as:	data[0] = roll
 * 									data[1] = pitch
 * 									data[2] = yaw
 */
void MadgwickFilter::getEulerAnglesDeg(float* data) {
	float temp = 2*q0*q0 - 1;	// Avoid recalculating this
	data[0] = 57.295779513 * atan2f(2.0f * (q0*q1 + q2*q3), temp + 2*q3*q3);	// roll
	data[1] = 57.295779513 * asinf( 2.0f * (q0*q2 - q1*q3));					// pitch
	data[2] = 57.295779513 * atan2f(2.0f * (q1*q2 + q0*q3), temp + 2*q1*q1);	// yaw
}

/**
 * Calculates Euler angles (roll,pitch,yaw) from the quaternion q.
 * The resulting angles range from -pi to pi degrees.
 *
 * The pitch angle is positive if the nose of the spacecraft is pointing up
 *	TODO: Direction of rotations
 *
 * @param float* data: A pointer to a float[3] array. The values are
 * 						stored as:	data[0] = roll
 * 									data[1] = pitch
 * 									data[2] = yaw
 */
void MadgwickFilter::getEulerAnglesRad(float* data) {
	float temp = 2*q0*q0 - 1;	// Avoid recalculating this
	data[0] = atan2f(2.0f * (q0*q1 + q2*q3), temp + 2*q3*q3);	// roll
	data[1] = asinf( 2.0f * (q0*q2 - q1*q3));					// pitch
	data[2] = atan2f(2.0f * (q1*q2 + q0*q3), temp + 2*q1*q1);	// yaw
}

/**
 * Resets the quaternion to (1,0,0,0)
 */
void MadgwickFilter::resetFilter(){
	q0 = 1.0f;
	q1 = q2 = q3 = 0.0f;
}

/**
 * Sets the filter gain beta
 */
void MadgwickFilter::setGain(float gain) {
	beta = gain;
}
