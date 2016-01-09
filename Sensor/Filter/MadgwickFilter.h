/*
 * AUTHOR:		Christoph Hagen
 * EMAIL:		christoph.hagen@me.com
 * DATE:		04.12.2015
 *
 * This file includes an implementation of a madgwick filter
 * in order to combine measurements of a gyroscope, accelerometer
 * and magnetometer.
 *
 * It includes a lot of comments to make the usage easier.
 *
 * If you find this implementation useful, you can buy me a beer.
 *
 * This implementation is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This class is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * For more information see <http://www.gnu.org/licenses/>.
 */

#ifndef Madgwick_Filter_h
#define Madgwick_Filter_h

/**
 * stdint.h is used for standard data types
 */
#include "stdint.h"

/*
 * Math.h is needed for the trigonometric functions
 * sqrt(),atan2f() and asinf()
 */
#include "math.h"

/**
 * Enable this macro on 64bit systems, because the
 * fast inverse square root function is not going to work
 */
//#define SYSTEM_64BIT

/*
 * This class uses data from gyroscope, accelerometer and magnetometer to estimate
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
 * magnetometer values, a simple offset/scale model might not be sufficient.
 *
 * Make sure that the coordinate systems of the individual sensors are correctly aligned.
 *
 * For the magnetometer calibration an ellipsoid fit algorithm might be useful.
 * See: http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
 * Yury Petrov, September 2015
 */
class MadgwickFilter {

private:
	/**
	 * The quaternion q represents the estimated attitude of the spacecraft.
	 * It describes the position of the sensor frame relative to the auxiliary frame
	 *
	 * The initial orientation is pointing along the x axis
	 *
	 * Sometimes q1, q2, q3 are referred to as q_x, q_y, q_z.
	 */
	volatile float q0, q1, q2, q3;

	/**
	 * This is the gain of the filter. Beta is defined as 2 * proportional gain (Kp)
	 * of the controller that merges gyro quaternion with acc/mag quaternion
	 * In the beginning it should be high (e.g. 10) for calibration,
	 * after that it should be set low (e.g. 0.1).
	 * The value mainly depends on the drift of the gyroscope.
	 * Higher drift -> higher gain
	 */
	volatile float beta;

	/**
	 * The update interval of the filter. For sample frequency
	 * f this is T = 1/f. The Interval is given in seconds, since the
	 * gyro measurements are given in rad/s
	 * This should be set to the timing that corresponds to the call of
	 * the filterUpdate() function.
	 */
	volatile float interval;

public:

	/**
	 * The constructor for a new filter
	 *
	 * @param float filter_interval:	The update interval in seconds
	 * @param float filter_gain:		The gain of the filter
	 */
	MadgwickFilter(float filter_interval = 0.01f, float filter_gain = 0.1f);

	/**
	 * Call this function with the new sensor values to perform a filter update.
	 *
	 * @params: 	float 	gx, gy, gz:		The gyroscope measurements in rad/s
	 * 				float	ax, ay, az:		The accelerometer values
	 * 				float	mx, my, mz:		The (scaled) magnetometer values
	 */
	void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

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
	void getEulerAnglesDeg(float* data);
    
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
    void getEulerAnglesRad(float* data);

	/**
	 * Resets the quaternion to (1,0,0,0)
	 */
	void resetFilter();

	/**
	 * Sets the filter gain beta
	 */
	void setGain(float gain);

};

#endif
