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

void AHRS::filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

	//Accelerometer: Roll, pitch angle
	aR = atan2((double) ax, sqrt((double) (ay * ay + az * az)));
	aP = atan2((double) ay, sqrt((double) (ax * ax + az * az)));

	//Gyroscope: Roll, pitch, yaw angle
	gR -= (gy * sampleRate);
	gP += (gx * sampleRate);
	gY -= (gz * sampleRate);

	//Fusion of gyroscope and accelerometer data -> filtered: roll pitch
	fR = (alpha * gR * DEG_TO_RAD + oneSubAlpha * aR);
	fP = (alpha * gP * DEG_TO_RAD + oneSubAlpha * aP);

	//Tilt compensation of the magnetometer with filtered values
	double mxH = mx * cos(fP) + mz * sin(fP);
	double myH = mx * sin(fR) * sin(fP) + my * cos(fR) - mz * cos(fR) * cos(fP);

	mY = atan2(myH, mxH);

	fY = (alpha * gY * DEG_TO_RAD + oneSubAlpha * mY);

	//TODO scale values 
}

void AHRS::setValToGrad() {

	aR *= RAD_TO_DEG;
	aP *= RAD_TO_DEG;
	mY *= RAD_TO_DEG;

	fR *= RAD_TO_DEG;
	fP *= RAD_TO_DEG;
	fY *= RAD_TO_DEG;

}

void AHRS::setAlpha(float alpha) {

	this->alpha = alpha;

	oneSubAlpha = 1 - this->alpha;
}
