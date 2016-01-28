/*
 * ThImuRead.cpp
 *
 *  Created on: 24.12.2015
 *      Author: pinker
 */

#include "ThImuRead.h"
#include "../Define/Define.h"
#include "../Sensor/I2C/Lighsensor/Light.h"
#include "../Sensor/ADC/IR.h"
#include "../Sensor/ADC/SolarPannel.h"
#include "../Sensor/Filter/AHRS.h"
#include "../Extern/Extern.h"
#include "../Actuators/Hbridge.h"
#include "../Sensor/I2C/Current/Current.h"
#include <stdio.h>

ThImuRead::ThImuRead(const char* name, Hbridge *flWheel, AHRS *ahrs) {

	this->ahrs = ahrs;
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
	currePrint = false;
	solarPrint = false;
	irPrint = false;
	motorPrintPos = false;
	motorPrintVel = false,

	flag = false;
	changeAlphaFlag = false;
	epsilonFlag = false;

	kiFlag = false;
	kpFlag = false;
	kdFlag = false;

	// Flags for calibration
	accCalFlag = false;
	gyrCalFlag = true;
	magCalFlag = false;
	softIrFlag = false;

	value = 0;				// Default value used for the TC

	send = true;     		// Sending continuously values to the GS

	motorCtrl = true;
	setPointFlag = false;
	setPoint = 0;    	 	// Setpoint for the controller
	controlType = 0;		//1-velocity 2- position
}

ThImuRead::~ThImuRead() {
}

void ThImuRead::init() {
	// Initialize of the I2C one speed just once here!
	//I2C_1.init(100000);
	// Configure ADCs just here
	ADC_1.config(ADC_PARAMETER_RESOLUTION, 12);
	ADC_2.config(ADC_PARAMETER_RESOLUTION, 12);
}

void ThImuRead::run() {
	IMU imu(0.015, this);
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

	// Current
	Current bat(ADDR_BATT);

	char printOutput[80];

	int cnt = 0;
	int cnt2 = 0;
	float gain = 10;

	float eulAng[3];

	char sprintOut[100];

	// Sample time set to 15 Milliseconds
	PiController controller;
	unsigned int duty = 0;

	flWheel->init();

	// Endless loop with calibration selection
	while (1) {

		// Accelerometer calibration with TC. Sets all AHRS values to zero.
		if (accCalFlag) {
			imu.accCalibrate();
			ahrs->setAllValuesToZero();
		}

		// Magnetometer soft iron calibration with TC. Sets all AHRS values to zero.
		if (magCalFlag) {

			// Read and print out values for the soft iron calibration
			int cnt = 0;
			PRINTF("Magnetometer calibration starts in 5 seconds.....reset the commuication\r\n");
			suspendCallerUntil(NOW()+7*SECONDS);

			while (cnt < SAMPLES_SOFTCAL) {
				cnt++;
				imu.magReadLSM303DLH();

				sprintf(printOutput, "%d,%d,%d\r", imu.mag.xRAW, imu.mag.yRAW, imu.mag.zRAW);
				PRINTF(printOutput);
				suspendCallerUntil(NOW()+25*MILLISECONDS);
			}

			// Suspend the print so that the user can store the values;
			suspendCallerUntil(NOW()+15*SECONDS);
			ahrs->setAllValuesToZero();
		}

		// Gyroscope calibration with TC. Sets all AHRS values to zero.
		if (gyrCalFlag) {
			imu.gyrCalibrate();
			ahrs->setAllValuesToZero();
		}

		// Change the alpha factor of the AHRS
		if (changeAlphaFlag) {
			PRINTF("AHRS: New alpha %f \r\n", (value / 100.00));
			ahrs->setAlpha((value / 100.00));
		}

		// Controller setpoint change
		if (epsilonFlag) {
			controller.setEpsion((value / 1000.00));
			PRINTF("Controller new epsilon value %f \r\n", controller.epsilonI);
		}

		if (kdFlag) {
			controller.setDerivative((value / 1000.00));
			PRINTF("Controller new kd value %f \r\n", controller.kd_pid);
		}

		if (kiFlag) {
			controller.setIntegral((value / 1000.00));
			PRINTF("Controller new ki value %f \r\n", controller.ki_pi);
		}

		if (kpFlag) {
			controller.setProportional((value / 1000.00));
			PRINTF("Controller new kp value %f \r\n", controller.kp_pi);
		}

		if (setPointFlag) {
			this->setPoint = value;
			PRINTF("New setpoint %f \r\n", this->setPoint);
		}

		flag = magCalFlag = gyrCalFlag = accCalFlag = changeAlphaFlag = setPointFlag = kpFlag = kdFlag = kiFlag = epsilonFlag = false;

		// Inner read loop
		while (1) {
			// Read the Sensors
			imu.accRead();
			imu.gyrRead();
			imu.magReadLSM303DLH();

			// Filter the data
			ahrs->filterUpdate2(&imu.acc, &imu.gyr, &imu.mag);

			// Soft iron calibration value Print
			if (softIrFlag) {
				sprintf(printOutput, "%d,%d,%d\r", imu.mag.xRAW, imu.mag.yRAW, imu.mag.zRAW);
				PRINTF(printOutput);

				// Disables other print methods
				cnt = 0;
			}

			// Print
			if ((cnt++ > 16) && (send)) {
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

				// Print Gyrometer data
				if (gyrPrint) {
					sprintf(printOutput, "G dx%.1f dy%.1f dz%.1f\r\n", imu.gyr.dx, imu.gyr.dy, imu.gyr.dz);
					PRINTF(printOutput);
				}

				// Print Magnetometer data
				if (magPrint) {
					sprintf(printOutput, "%Raw x%d y%d z%d\r\n", imu.mag.xRAW, imu.mag.yRAW, imu.mag.zRAW);
					PRINTF(printOutput);

					sprintf(printOutput, "CAL x%.1f y%.1f z%.1f\r\n", imu.mag.x, imu.mag.y, imu.mag.z);
					PRINTF(printOutput);
				}

				// Print fuison data
				if (filPrint) {
					PRINTF("----------------------------\r\n");

					sprintf(printOutput, "Y %.1f\r\n", ahrs->mY);
					PRINTF(printOutput);

//					sprintf(printOutput, "G r%.1f p %.1f y%.1f\r\n", imu.gyr.r, imu.gyr.p, imu.gyr.y);
//					PRINTF(printOutput);

					sprintf(printOutput, "G y%.1f\r\n", imu.gyr.y);
					PRINTF(printOutput);

					sprintf(printOutput, "F y%.1f\r\n", ahrs->yFus);
					PRINTF(printOutput);
				}

				// Print current sensor
				if (currePrint) {
					bat.read();
					//sprintf(printOutput, "Current ", bat.Current())

				}

				// Print solar pannel data
				if (solarPrint) {
					sprintf(printOutput, "Solar vol: %.2f\r\n", solPan.getVoltage());
					PRINTF(printOutput);

					sprintf(printOutput, "Solar cur: %.1f\r\n", solPan.getCurrent());
					PRINTF(printOutput);
				}

				// Print ir sensor data
				if (irPrint) {
					sprintf(printOutput, "IR1 %f\r\n", ir1.read());
					PRINTF(printOutput);

					sprintf(printOutput, "IR2 %f\r\n", ir2.read());
					PRINTF(printOutput);
				}

				if (motorPrintPos) {
					PRINTF("----------POSITION---------------\r\n\n\n");
					PRINTF("Duty %d set %.1f acu %.1f \r\n", duty, setPoint, ahrs->yFus);
					PRINTF("p: %.7f i:%.7f d%.7f \r\n", controller.kp_pid, controller.ki_pid, controller.kd_pid);
					PRINTF("p: %.1f i:%.1f d%.1f \r\n", (controller.error * controller.kd_pid), (controller.integral * controller.ki_pid), (controller.derivative * controller.kd_pid));

					if (motorCtrl) {
						PRINTF("---->ON!\r\n");
					} else {
						PRINTF("OFF<-----\r\n");
					}
				}
				if (motorPrintVel) {
					PRINTF("----------VELOCITY---------------\r\n\n\n");
					PRINTF("Duty %d set %.1f acual %.1f \r\n", duty, setPoint, imu.gyr.dz);
					PRINTF("p: %.7f i:%.7f\r\n", controller.kp_pi, controller.ki_pi);
					PRINTF("p: %.1f i:%.1f\r\n", (controller.error * controller.kp_pi), (controller.integral * controller.ki_pi));

					if (motorCtrl) {
						PRINTF("---->ON!\r\n");
					} else {
						PRINTF("OFF<-----\r\n");
					}
				}

			}

			// input value is just the gyro dz value!
			//duty = controller.pid(setPoint, ahrs.yFus);

			if(controlType == 1)
			{
				// input value is just the gyro dz value!
				duty = controller.pi(setPoint, imu.gyr.dz);
			}
			else if(controlType == 2)
				duty = controller.pid(setPoint, ahrs->yFus);
			else
				duty = 0;

			// Enables/disables motor controller
			if (motorCtrl) {
				flWheel->setDuty(duty);
			} else {
				flWheel->setDuty(0);
			}

			// For more performance just one flag to break out of the inner loop
			if (flag)
				break;

			suspendCallerUntil(NOW()+15*MILLISECONDS);


		}
	}
}

