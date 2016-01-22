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
	//Read the battery values
	//Current batteries(ADDR_BATT);
	float batval[2];

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
	gyrCalFlag = true;     	// Gyroscope calibration by default enabled

	motorCtrl = false;
	setPointFlag = false;
	setPoint = 0;    	 	// Setpoint for the controller
}

ThImuRead::~ThImuRead() {
}

void ThImuRead::init() {
}

void ThImuRead::run() {
	// Sample rate = 0.01 s
	IMU imu(this, 0.01);
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

		//inner read loop
		while (1) {
			imu.accRead();
			imu.gyrRead();
			//imu.magRead();
			imu.magReadLSM303DLH();

			//ahrs.filterUpdate2(&imu.acc, &imu.gyr, &imu.mag);

			// input value is just the gyro dz value!
			//duty = controller.pi(setPoint, imu.gyr.dz);

			// input value is just the gyro dz value!
			duty = controller.pid(setPoint, ahrs.gY);

			if ((cnt++ > 22) && (send)) {
				cnt = 0;

				char out[80];

				sprintf(out, "x %f y %f z %f\r\n", imu.mag.x,  imu.mag.y, imu.mag.z);
				PRINTF(out);
//
//				sprintf(out, "tilt   My %d\r\n", (int)ahrs.mY);
//				PRINTF(out);
//
//				sprintf(out, "simple My %d\r\n", (int)(atan2(imu.mag.y, imu.mag. x))*RAD_TO_DEG);
//				PRINTF(out);
//
//
//				sprintf(out, "Gy %+.2f\r\n", imu.gyr.y);
//				PRINTF(out);

//				sprintf(out, "G y%+d\r\n", (int) imu.gyr.y);
//				PRINTF(out);
//				sprintf(out, "M y%+d\r\n", (int) ahrs.mY);
//				PRINTF(out);
//				sprintf(out, "F y%+d\r\n", (int) ahrs.yFus);
//				PRINTF(out);
				//sprintf(out, "A R%+d  P%+d\r\n", (int) (imu.acc.r), (int) (imu.acc.p));
				//PRINTF(out);
				//sprintf(out, "F x%d  y%d   z%d\r\n", (int) (ahrs.yFusion), (int) (ahrs.xFusion), (int) (ahrs.zFusion));

				//PRINTF("F x%d  y%d   z%d\r\n", (int) (ahrs.yFusion), (int) (ahrs.xFusion), (int) (ahrs.zFusion));

//				char out[80];
//				//sprintf(out, "kp: %f ki %f eps: %f\r\n" controller.kp, controller.ki, controller.epsilon);
//
//
//				sprintf(out, "kp %.3f ki %.3f kd %.3f eps%.3f\r\n", controller.kp, controller.ki, controller.kd, controller.epsilon );
//				PRINTF(out);
//				sprintf(out, "G dz %.3f setpoint %f\r\n", ahrs.gY, setPoint);
//				PRINTF(out);
//				sprintf(out, "ERR %+.3f INT %+.3f OUTP %+.3f\r\n", controller.error, controller.integral, controller.output);
//				PRINTF(out);
//				PRINTF("Motor duty: %d\r\n", duty);
//
//				//PRINTF("kp: %f ki %f eps: %f\r\n", controller.kp, controller.ki, controller.epsilon);
//				//PRINTF("G dz %f setpoint %f\r\n", imu.gyr.dz, setPoint);
//				//PRINTF("ERR %f INT %f OUTP %f\r\n", controller.error, controller.integral, controller.output);
//				//PRINTF("Motor duty: %d\r\n", duty);
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

