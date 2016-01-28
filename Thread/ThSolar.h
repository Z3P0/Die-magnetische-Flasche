/*
 * ThSolar.h
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#ifndef THREAD_THSOLAR_H_
#define THREAD_THSOLAR_H_
#include "../Extern/Include.h"
#include "../Extern/Extern.h"
#include "ThImuRead.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Sensor/I2C/Lighsensor/Light.h"


#define DEPLOY_TIME 		18	 //Seconds

class ThSolar: public Thread {
public:
	ThSolar(const char* name, Hbridge *bridge, ThImuRead* controlLoop, AHRS* ahrs, Light* lightSensor);
	virtual ~ThSolar();
	void init();
	void run();
	void start();
private:
	Hbridge *bridge;
	Hbridge *flyWheel;
	AHRS* ahrs;
	Light* lightSensor;
	ThImuRead* controlLoop;

};

#endif /* THREAD_THSOLAR_H_ */
