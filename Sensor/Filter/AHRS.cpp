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

	alpha = 0;
	pre_yFus = 0;

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

void AHRS::filterUpdate2(IMU_Acc* acc, IMU_Gyro* gyr, IMU_Mag* mag) {

	// Tilt compensation of the magnetometer with filtered values
	double mxH = mag->x * cos(acc->p) + mag->z * sin(acc->p);
	double myH = mag->x * sin(acc->r) * sin(acc->p) + mag->y * cos(acc->r) - mag->z * sin(acc->r) * cos(acc->p);

	mY = atan2(myH, mxH) * RAD_TO_DEG;

	//This is for a smooth 360 to 0 and vice versa transition
	if ((gyr->y - mY) > 90)
		mY += 180;
	else if ((mY - gyr->y) > 90)
		mY -= 180;

	// Fusion for roll and pitch are degree values
	rFus = (0.3 * acc->r + 0.7 * gyr->r);
	pFus = (0.3 * acc->p + 0.7 * gyr->p);
	yFus = (0.85 * gyr->y + 0.15 * mY);

	int n = (int) (ABS(yFus) / 360);

	if (yFus > 360)
		yFus -= n * 360;
	else if (yFus < 0)
		yFus += (n+1) * 360;

}

void AHRS::simplePredict() {
//	float di	ff = yFus - pre_yFus;
//
//	float k = ABS(diff / yFus);
//
//	yFusPre = pre_yFus + yFus;
//
//	// Prediction of the new value
//	pre_yFus += (yFus * sampleRate);
}

void AHRS::setAlpha(float alpha) {

	this->alpha = alpha;

	oneSubAlpha = 1 - this->alpha;
}
