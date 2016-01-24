/*
 * ThImuRead.cpp
 *
 *  Created on: 24.12.2015
 *      Author: pinker
 */

#include "ThImuRead.h"
#include "../Define/Define.h"
#include "../Sensor/I2C/IMU/IMU.h"
#include "../Sensor/I2C/Lighsensor/Light.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Extern/Extern.h"
#include "../Actuators/Hbridge.h"
#include "../Sensor/I2C/Current/Current.h"
#include <stdio.h>

ThImuRead::ThImuRead(const char* name, Hbridge *flWheel) {
	// Read the battery values
	// Current batteries(ADDR_BATT);


	//Light ls;
	float batVal[2];

	// Reference to the H-Bridge
	this->flWheel = flWheel;


	flag = false;
	changeAlphaFlag = false;
	epsilonFlag = false;
	magCalFlag = false;
	accCalFlag = false;
	kiFlag = false;
	kpFlag = false;
	kdFlag = false;

	value = 0;				// Default value used for the TC

	send = true;     		// Sending continuously values to the GS
	gyrCalFlag = false;     	// Gyroscope calibration by default enabled

	motorCtrl = false;
	setPointFlag = false;
	setPoint = 0;    	 	// Setpoint for the controller
}

ThImuRead::~ThImuRead() {
}

void ThImuRead::init() {
}

void ThImuRead::run() {
	// Sample rate = 0.015 s
	IMU imu(this, 0.015);
	imu.accSetDefaultValues();

	int cnt = 0;
	int cnt2 = 0;
	float gain = 10;

	float eulAng[3];

	AHRS ahrs(0.01);
	char sprintOut[100];

	//sample time set to 10 Milliseconds
	PiController controller;
	unsigned int duty = 0;

	flWheel->init();
	setPoint = 0;     //setpoint for the controller

	//endless loop with calibration selection
	while (1) {
		PRINTF("Now im in the imu read thead");
		//Gyro calibration with TC. Sets all AHRS values to zero.
		if (gyrCalFlag) {
			imu.gyrCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Magnetometer calibration with TC. Sets all AHRS values to zero.
		if (magCalFlag) {
			imu.magCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Gyro calibration with TC. Sets all AHRS values to zero.
		if (accCalFlag) {
			imu.accCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Change the alpha factor of the AHRS
		if (changeAlphaFlag) {
			PRINTF("AHRS: New alpha %f \r\n", (value / 100.00));
			ahrs.setAlpha((value / 100.00));
		}

		//Controller setpoint change
		if (epsilonFlag) {
			controller.setEpsion((value / 1000.00));
			PRINTF("Controller new epsilon value %f \r\n", controller.epsilon);
		}

		if (kdFlag) {
			controller.setDerivative((value / 1000.00));
			PRINTF("Controller new kd value %f \r\n", controller.ki);
		}

		if (kiFlag) {
			controller.setIntegral((value / 1000.00));
			PRINTF("Controller new ki value %f \r\n", controller.ki);
		}

		if (kpFlag) {
			controller.setProportional((value / 1000.00));
			PRINTF("Controller new kp value %f \r\n", controller.kp);
		}

		if (setPointFlag) {
			this->setPoint = value;
			PRINTF("New setpoint %f \r\n", this->setPoint);
		}

		flag = magCalFlag = gyrCalFlag = accCalFlag = changeAlphaFlag = setPointFlag = kpFlag = kdFlag = kiFlag = epsilonFlag = false;

		// Inner read loop
		while (1) {
			imu.accRead();
			imu.gyrRead();
			imu.magReadLSM303DLH();

			ahrs.filterUpdate2(&imu.acc, &imu.gyr, &imu.mag);

			// input value is just the gyro dz value!
			//duty = controller.pi(setPoint, imu.gyr.dz);

			// input value is just the gyro dz value!
			duty = controller.pid(setPoint, ahrs.gY);


			if ((cnt++ > 22) && (send)) {
				cnt = 0;

				char out[80];

				sprintf(out, "MAG raw %d %d %.d\r\n", (int)imu.mag.x,(int)imu.mag.y,(int)imu.mag.z);
				PRINTF(out);


				//sprintf(out, "xl %d xh %d  yl %d yh %d zl %d xh %d \r\n", imu.xlow, imu.xhigh, imu.ylow, imu.yhigh, imu.zlow, imu.zhigh);
				//PRINTF(out);

				sprintf(out, "AM r:%.1f p:%.1f y:%.1f\r\n", imu.acc.r, imu.acc.p, ahrs.mY);
				PRINTF(out);

				sprintf(out, "G  r:%.1f p:%.1f y:%.1f\r\n", imu.gyr.r, imu.gyr.r, imu.gyr.r);
				PRINTF(out);
				PRINTF("----------------------------\r\n");

				sprintf(out, "F  r:%.1f p:%.1f y:%.1f\r\n\n\n", ahrs.rFus, ahrs.pFus, ahrs.yFus);
				PRINTF(out);
//
//				//enables the motor to controll the S/C
//				if (motorCtrl) {
//					PRINTF("Motorcontroller ON!\r\n");
//				} else {
//					PRINTF("Motorcontroller OFF!\r\n");
//				}
			}

			//enables/disables motor controller
			if (motorCtrl) {
				flWheel->setDuty(duty);
			} else {
				flWheel->setDuty(0);
			}

			if (flag)
				break;
			suspendCallerUntil(NOW()+15*MILLISECONDS);
		}
	}
}

