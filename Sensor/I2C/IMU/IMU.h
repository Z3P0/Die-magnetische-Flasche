/*
 * IMU.h
 *
 *  Created on: 22.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_IMU_H_
#define SENSOR_IMU_H_
#include "../../../Extern/Include.h"
#include "Register.h"

/*-----------------Accelerometer----------------*/
#define ACC_NBR_SCALE_SAMPLES		50

#define ACC_X_OFFSET				-103.0
#define ACC_Y_OFFSET				9.0
#define ACC_Z_OFFSET				-6.0

/*------------------Gyroscope-------------------*/
#define GYR_CALIBATION_SAMPLES 		500

/*------------------Magnetometer----------------*/
#define MAG_NBR_SCALE_SAMPLES		1500
//     without motors
#define MAG_X_MIN					-579 // | -579 | -553 | -547 |
#define MAG_Y_MIN					-233 // | -206 | -243 | -218 |
#define MAG_Z_MIN					-325 // | -313 | -318 | -304 |

#define MAG_X_DIFF					755  //  |  710 | 691  |  696 |
#define MAG_Y_DIFF					716  //  |  604 | 735  |  706 |
#define MAG_Z_DIFF					732  //  |  715 | 698  |  701 |

struct IMU_Acc {
	float x;
	float y;
	float z;

	float xOffset;
	float yOffset;
	float zOffset;
};

struct IMU_Gyro {
	float dx;
	float dy;
	float dz;

	// Average of the standstill values
	float dxOffset;
	float dyOffset;
	float dzOffset;
};

struct IMU_Mag {
	float x;
	float y;
	float z;

	float xMin;
	float xDiff;     //xMax - xMin

	float yMin;
	float yDiff;     //xMax - xMin

	float zMin;
	float zDiff;     //xMax - xMin
};

class IMU {

public:
	IMU_Acc acc;
	IMU_Gyro gyr;
	IMU_Mag mag;

	IMU(Thread* caller);
	virtual ~IMU();
	void errorDetection(int16_t nbrOfReceivedBytes, int8_t expectedNumber);
	void resetValues();

	// Accelerometer
	void accCalibrate();
	void accRead();
	void accSetDefaultValues();

	// Gyroscope
	void gyrRead();
	void gyrCalibrate(int warningTimer);

	// Magnetometer
	void magRead();
	void magCalibrate();
	void magSetDefaultValues();

private:
	Thread *caller;

	void configurateIMU();
	void resetI2C();

	void accAxisOffset(float &axisValue, float &axisOffset, char name);

	/* Gyroscope*/
	const uint8_t gyrAdress = GYRO_ADDRESS;
	const uint8_t gyrOutAllAxis[1] = { GYRO_OUT_ALL_AXIS };

	const uint8_t gyrCtrlReg1[2] = { CTRL_REG1_G_ADDRESS, CTRL_REG1_G_VALUE };
	const uint8_t gyrCtrlReg4[2] = { CTRL_REG4_G_ADDRESS, CTRL_REG4_G_VALUE };
	const uint8_t gyrCtrlReg5[2] = { CTRL_REG5_G_ADDRESS, CTRL_REG5_G_VALUE };

	/* Magnetometer Accelerometer*/
	const uint8_t accMagAdress = ACC_MAG_ADDRESS;

	const uint8_t accOutAllAxis[1] = { ACC_OUT_ALL_AXIS };
	const uint8_t magOutAllAxis[1] = { MAG_OUT_ALL_AXIS };

	const uint8_t accMagCtrlReg1[2] = { CTRL_REG1_XM_ADDRESS, CTRL_REG1_XM_VALUE };
	const uint8_t accMagCtrlReg2[2] = { CTRL_REG2_XM_ADDRESS, CTRL_REG2_XM_VALUE };
	const uint8_t accMagCtrlReg5[2] = { CTRL_REG5_XM_ADDRESS, CTRL_REG5_XM_VALUE };
	const uint8_t accMagCtrlReg6[2] = { CTRL_REG6_XM_ADDRESS, CTRL_REG6_XM_VALUE };
	const uint8_t accMagCtrlReg7[2] = { CTRL_REG7_XM_ADDRESS, CTRL_REG7_XM_VALUE };

};

#endif /* SENSOR_IMU_H_ */
