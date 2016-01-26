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
#include "../Sensor/ADC/IR.h"
#include "../Sensor/ADC/SolarPannel.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Extern/Extern.h"
#include "../Actuators/Hbridge.h"
#include "../Sensor/I2C/Current/Current.h"
#include <stdio.h>

ThImuRead::ThImuRead(const char* name, Hbridge *flWheel) {
	// Read the battery values
	// Current batteries(ADDR_BATT);
	float batVal[2];

	// Reference to the H-Bridge
	this->flWheel = flWheel;

	// Flags for reading/ printing
	accPrint = false;
	gyrPrint = false;
	// This one is for calibration
	magPrint = false;
	// This is the normal stuff
	filPrint = true;
	lightPrint = false;
	solarPrint = false;
	irPrint = false;

	flag = false;
	changeAlphaFlag = false;
	epsilonFlag = false;

	kiFlag = false;
	kpFlag = false;
	kdFlag = false;

	// Flags for calibration
	accCalFlag = false;
	gyrCalFlag = false;
	magCalFlag = false;

	value = 0;				// Default value used for the TC

	send = true;     		// Sending continuously values to the GS
	gyrCalFlag = false;     // Gyroscope calibration by default enabled

	motorCtrl = false;
	setPointFlag = false;
	setPoint = 0;    	 	// Setpoint for the controller
}

ThImuRead::~ThImuRead() {
}

void ThImuRead::init() {
	// Initialize of the I2C one speed just once here!
	I2C_1.init(400000);
	// Configure ADCs just here
	ADC_1.config(ADC_PARAMETER_RESOLUTION, 12);
	ADC_2.config(ADC_PARAMETER_RESOLUTION, 12);
}

void ThImuRead::run() {
	// Sample rate = 0.015 s
	IMU imu(this, 0.015);
	imu.accSetDefaultValues();

	// Sensors
	Light ls;
	float lightVal;

	// ADC
	IR ir1(&ADC_1, ADC_CHANNEL_IR1);
	ir1.init();

	IR ir2(&ADC_1, ADC_CHANNEL_IR2);
	ir2.init();

	// Solar panel object for voltage and current
	SolarPannel solPan(&ADC_1, ADC_CHANNEL_SOL_A, ADC_CHANNEL_SOL_V);
	solPan.init();

	char printOutput[80];

	int cnt = 0;
	int cnt2 = 0;
	float gain = 10;

	float eulAng[3];

	AHRS ahrs(0.01);
	char sprintOut[100];

	// Sample time set to 15 Milliseconds
	PiController controller;
	unsigned int duty = 0;

	flWheel->init();
	setPoint = 0;     // Setpoint for the controller

	// Endless loop with calibration selection
	while (1) {

		// Accelerometer calibration with TC. Sets all AHRS values to zero.
		if (accCalFlag) {
			imu.accCalibrate();
			ahrs.setAllValuesToZero();
		}

		// Magnetometer calibration with TC. Sets all AHRS values to zero.
		if (magCalFlag) {
			imu.magCalibrate();
			ahrs.setAllValuesToZero();
		}

		// Gyroscope calibration with TC. Sets all AHRS values to zero.
		if (gyrCalFlag) {
			imu.gyrCalibrate();
			ahrs.setAllValuesToZero();
		}

		// Change the alpha factor of the AHRS
		if (changeAlphaFlag) {
			PRINTF("AHRS: New alpha %f \r\n", (value / 100.00));
			ahrs.setAlpha((value / 100.00));
		}

		// Controller setpoint change
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
			//duty = controller.pid(setPoint, ahrs.gY);


			if (magPrint) {
				sprintf(printOutput, "%d,%d,%d\r", imu.mag.xRAW, imu.mag.yRAW, imu.mag.zRAW);
				PRINTF(printOutput);
			}

			// Print
			if ((cnt++ > 30) && (send)) {
				cnt = 0;

				// Read Light sensor
				if (lightPrint) {
					ls.read();
					sprintf(printOutput, "light ch1%d ch1%d\r\n", ls.ch0, ls.ch1);
					PRINTF(printOutput);
				}

				// Print Magnetometer data
				if (accPrint) {
					sprintf(printOutput, "A x%.1f y%.1f z%.1f\r\n", imu.acc.x, imu.acc.y, imu.acc.z);
					PRINTF(printOutput);
				}

				if (gyrPrint) {
					sprintf(printOutput, "G dx%.1f dy%.1f dz%.1f\r\n", imu.gyr.dx, imu.gyr.dy, imu.gyr.dz);
					PRINTF(printOutput);
				}

//				if (magPrint) {
//					sprintf(printOutput, "%d %d %d \r\n", imu.mag.xRAW, imu.mag.yRAW, imu.mag.zRAW);
//					PRINTF(printOutput);
//				}

				if (filPrint) {
					sprintf(printOutput, "G r%.1f p %.1f y%.1f\r\n", imu.gyr.r, imu.gyr.p, imu.gyr.y);
					PRINTF(printOutput);

					PRINTF("----------------------------\r\n");

					sprintf(printOutput, "F r%.1f p %.1f y%.1f\r\n", ahrs.fR, ahrs.fP, ahrs.fY);
					PRINTF(printOutput);
				}

				if (solarPrint) {
					sprintf(printOutput, "Solar vol: %.2f\r\n", solPan.getVoltage());
					PRINTF(printOutput);

//					sprintf(printOutput, "Solar cur: %.1f\r\n", solPan.getCurrent());
//					PRINTF(printOutput);
				}

				if (irPrint) {
					sprintf(printOutput, "IR1 %f\r\n", ir1.read());
					PRINTF(printOutput);

					sprintf(printOutput, "IR2 %f\r\n", ir2.read());
					PRINTF(printOutput);

				}

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
