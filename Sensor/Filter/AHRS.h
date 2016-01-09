/*
 * AHRS.h
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_FILTER_AHRS_H_
#define SENSOR_FILTER_AHRS_H_

#define ALPHA  	65

class AHRS {
public:
	float fR, fP, fY;
	float aR, aP;
	float gR, gP, gY;
	float mY;

	AHRS(float SAMPLE_RATE);
	virtual ~AHRS();
	void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void setAlpha(float alpha);
	void setAllValuesToZero();
	void setGyrYawToMagYaw();
	void setValToGrad();

private:
	float sampleRate;
	float alpha;
	float oneSubAlpha;
	bool setYawEqual;

};

#endif /* SENSOR_FILTER_AHRS_H_ */
