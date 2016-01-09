/*
 * ThImuRead.cpp
 *
 *  Created on: 24.12.2015
 *      Author: pinker
 */

#include "ThImuRead.h"
#include "../Define/Define.h"
#include "../Sensor/I2C/IMU/IMU.h"
#include "../Sensor/I2C/Light.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Sensor/Filter/MadgwickFilter.h"
#include "../Extern/Extern.h"
#include "../Actuators/Hbridge.h"


ThImuRead::ThImuRead(const char* name, Hbridge *flWheel) {
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
	IMU imu(this);
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

	//endless loop with calibartion
	while (1) {
		//Gyro calibration with TC. Sets all ahrs values to zero.
		if (gyrCalFlag) {
			imu.gyrCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Magnetometer calibration with TC. Sets all ahrs values to zero.
		else if (magCalFlag) {
			imu.magCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Gyro calibration with TC. Sets all ahrs values to zero.
		else if (accCalFlag) {
			imu.accCalibrate();
			ahrs.setAllValuesToZero();
		}

		//Change the alpha factor of the ahrs
		else if (changeAlphaFlag) {
			PRINTF("AHRS: New alpha %f \r\n", (value / 100.00));
			ahrs.setAlpha((value / 100.00));
		}

		//Controller setpoint change
		else if (epsilonFlag) {
			controller.setEpsion((value / 1000.00));
			PRINTF("Controller new epsilon value %f \r\n", controller.epsilon);
		}

		else if (kdFlag) {
			controller.setDerivative((value / 1000.00));
			PRINTF("Controller new kd value %f \r\n", controller.ki);
		}

		else if (kiFlag) {
			controller.setIntegral((value / 1000.00));
			PRINTF("Controller new ki value %f \r\n", controller.ki);
		}

		else if (kpFlag) {
			controller.setProportional((value / 1000.00));
			PRINTF("Controller new kp value %f \r\n", controller.kp);
		}

		else if (setPointFlag) {
			this->setPoint = value;
			PRINTF("New setpoint %f \r\n", this->setPoint);
		}

		flag = magCalFlag = gyrCalFlag = accCalFlag = changeAlphaFlag = setPointFlag = kpFlag = kdFlag = kiFlag = epsilonFlag = false;

		//inner read loop
		while (1) {
			imu.accRead();
			imu.gyrRead();
			//imu.magRead();

			//ahrs.filteUpdate(imu.gyr.dx, imu.gyr.dy, imu.gyr.dz, imu.acc.x, imu.acc.y, imu.acc.z, imu.mag.x, imu.mag.y, imu.mag.z);

			// input value is just the gyro dz value!
			duty = controller.velocityPI(setPoint, imu.gyr.dz);

			if ((cnt++ > 10) && (send)) {
				cnt = 0;
				//ahrs.setValToGrad();

				//PRINTF("A R%d   P%d\r\n", (int) ahrs.aR, (int) ahrs.aP);
				//PRINTF("G R%d   P%d   Y%d\r\n", (int) ahrs.gR, (int) ahrs.gP, (int) ahrs.gY);
				//PRINTF("M           Y%d\r\n", (int) ahrs.mY);
				//PRINTF("F R%d   P%d   Y%d\r\n", (int) ahrs.fR, (int) ahrs.fP, (int) ((int) ahrs.fY % 360));
				PRINTF("kp: %f ki %f eps: %f\r\n", controller.kp, controller.ki, controller.epsilon);
				PRINTF("G dz %f setpoint %f\r\n", imu.gyr.dz, setPoint);
				PRINTF("Error %f integral %f\r\n", controller.error, controller.integral);
				PRINTF("Motor duty: %d\r\n", duty);;
				if(motorCtrl){
					PRINTF("Laura the shit is ON!\r\n");
				}
				else{
					PRINTF("NOW its OFF!\r\n");
				}
				PRINTF("-------------------\r\n");
			}

			//enables/disables motor controller
				if (motorCtrl) {
					flWheel->setDuty(duty);
				} else {
					flWheel->setDuty(0);
				}


			if (flag)
				break;
			suspendCallerUntil(NOW()+10*MILLISECONDS);
		}
	}
}



