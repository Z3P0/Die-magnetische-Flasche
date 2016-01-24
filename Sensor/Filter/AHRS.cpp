/*
 * AHRS.cpp
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#include "AHRS.h"
#include "math.h"
#include "../../Define/Define.h"

AHRS::AHRS(float sampleRate) {
	this->sampleRate = sampleRate;

	setAlpha(ALPHA);
	setYawEqual = false;

	setAllValuesToZero();

	gx = 0;
	gy = 0;
	gz = 0;

	rFus = 0;
	pFus = 0;
	yFus = 0;
}

AHRS::~AHRS() {
}

void AHRS::setAllValuesToZero() {
	fP = fR = fY = aR = aP = gR = gP = gY = mY = 0;
	setYawEqual = true;
}

void AHRS::setGyrYawToMagYaw() {
	gY = mY;
}

void AHRS::filterUpdate(float dGx, float dGy, float dGz, float ax, float ay, float az, float mx, float my, float mz) {

	// Accelerometer: Roll, pitch angle
	aR = atan2((double) ax, sqrt((double) (ay * ay + az * az)));
	aP = atan2((double) ay, sqrt((double) (ax * ax + az * az)));

	// Gyroscope: Roll, pitch, yaw angle
	gR -= (dGy * sampleRate);
	gP += (dGx * sampleRate);
	gY -= (dGz * sampleRate);

	// Tilt compensation of the Magnetometer with filtered values
	double mxH = mx * cos(aP) + mz * sin(aP);
	double myH = mx * sin(aR) * sin(aP) + my * cos(aR) - mz * cos(aR) * cos(aP);

	mY = atan2(myH, mxH);

	// Fusion of gyroscope and Accelerometer data -> filtered: roll pitch
	fR = (alpha * gR * DEG_TO_RAD + oneSubAlpha * aR);
	fP = (alpha * gP * DEG_TO_RAD + oneSubAlpha * aP);
	fY = (alpha * gY * DEG_TO_RAD + oneSubAlpha * mY);

	// TODO scale values
}

void AHRS::filterUpdate2(IMU_Acc* acc, IMU_Gyro* gyr, IMU_Mag* mag) {

	// Tilt compensation of the magnetometer with filtered values
	double mxH = mag->x * cos(acc->p) + mag->z * sin(acc->p);
	double myH = mag->x * sin(acc->r) * sin(acc->p) + mag->y * cos(acc->r) - mag->z * sin(acc->r) * cos(acc->p);

	mY = atan2(myH, mxH) * RAD_TO_DEG;

	// Fusion for roll and pitch are degree values
	rFus = (0.6 * acc->r + 0.4 * gyr->r);
	pFus = (0.6 * acc->p + 0.4 * gyr->p);
	yFus = (0.7 * gyr->y + 0.3 * mY);
}

void AHRS::setAlpha(float alpha) {

	this->alpha = alpha;

	oneSubAlpha = 1 - this->alpha;
}
