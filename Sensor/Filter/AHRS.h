/*
 * AHRS.h
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_FILTER_AHRS_H_
#define SENSOR_FILTER_AHRS_H_
#include "../I2C/IMU/IMU.h"

#define ALPHA  	0.80

class AHRS {
public:
	float fR, fP, fY;
	float aR, aP;
	float gR, gP, gY;

	AHRS(float SAMPLE_RATE);
	virtual ~AHRS();
	void filterUpdate2(IMU_Acc* acc, IMU_Gyro* gyr, IMU_Mag* mag);
	void setAlpha(float alpha);
	void setAllValuesToZero();
	void setGyrYawToMagYaw();
	void simplePredict();

	float rFus;
	float pFus;
	float yFus;
	float yFusPre;

	// simple predict variables
	float pre_yFus;		// predicted value

	float gx;
	float gy;
	float gz;

	float mY;

private:
	float sampleRate;
	float alpha;
	float oneSubAlpha;
	bool setYawEqual;
};

#endif /* SENSOR_FILTER_AHRS_H_ */
