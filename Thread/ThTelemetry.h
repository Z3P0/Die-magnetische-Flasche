/*
 * ThTelemetry.h
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#ifndef THREAD_THTELEMETRY_H_
#define THREAD_THTELEMETRY_H_
#include "../Extern/Include.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Sensor/ADC/SolarPannel.h"
//I2C Sensors
#include "../Sensor/I2C/IMU/IMU.h"
#include "../Sensor/I2C/Lighsensor/Light.h"
#include "../TCTM/SerializationUtil.h"


class ThTelemetry: public Thread{
public:
	ThTelemetry(const char* name, AHRS *ahrs, SolarPannel *solPan, Light *lightSensor);
	virtual ~ThTelemetry();
	void init();
	void run();

private:
	AHRS *ahrs;
	SolarPannel *solPan;
	Light *lightSensor;

};

#endif /* THREAD_THTELEMETRY_H_ */
