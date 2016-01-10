/*
 * IMU.cpp
 *
 *  Created on: 22.12.2015
 *      Author: pinker
 */

#include "IMU.h"
#include "../../../Define/Define.h"
#include "../../../Extern/Extern.h"

/*HAL initialization*/
HAL_GPIO CS_XM(GPIO_032);     // Chip select Accelerometer and Magnetometer
HAL_GPIO CS_G(GPIO_018);	  // Chip select Gyroscope
HAL_I2C IMU_HAL(I2C_IDX2);	  // Integrated IMU is on I2C 2

IMU::IMU(Thread *caller) {
	// Reference to the caller thread to suspend it
	this->caller = caller;
	
	// Set values to zero
	acc.x = acc.y = acc.z = acc.xOffset = acc.yOffset = acc.zOffset = 0;
	gyr.dx = gyr.dy = gyr.dz = gyr.dxOffset = gyr.dyOffset = gyr.dzOffset = 0;
	mag.x = mag.xMin = mag.xDiff = mag.y = mag.yMin = mag.yDiff = mag.z = mag.zMin = mag.zDiff = 0;

	configurateIMU();
}

IMU::~IMU() {
}

void IMU::configurateIMU() {
	
	// Chip select
	CS_XM.init(true, 1, 1);
	CS_G.init(true, 1, 1);

	// Enable IMU
	I2C_EN.init(true, 1, 1);
	IMU_HAL.init(400000);

	// Initialize the gyroscope
	errorDetection(IMU_HAL.write(gyrAdress, gyrCtrlReg1, 2), 2);
	errorDetection(IMU_HAL.write(gyrAdress, gyrCtrlReg4, 2), 2);
	errorDetection(IMU_HAL.write(gyrAdress, gyrCtrlReg5, 2), 2);

	// Initialize the accelerometer and the magnetometer
	errorDetection(IMU_HAL.write(accMagAdress, accMagCtrlReg1, 2), 2);
	errorDetection(IMU_HAL.write(accMagAdress, accMagCtrlReg2, 2), 2);
	errorDetection(IMU_HAL.write(accMagAdress, accMagCtrlReg5, 2), 2);
	errorDetection(IMU_HAL.write(accMagAdress, accMagCtrlReg6, 2), 2);
	errorDetection(IMU_HAL.write(accMagAdress, accMagCtrlReg7, 2), 2);
}

/* Checks if if there is a IMU read error by comparing the written with  the expected number of bytes.*/
void IMU::errorDetection(int16_t nbrOfReceivedBytes, int8_t expectedNumber) {
	if (nbrOfReceivedBytes != expectedNumber) {
		PRINTF("ERROR - IMU\n number of bytes: %d expected %d\r\n", expectedNumber, nbrOfReceivedBytes);
		resetI2C();
	}
}

/* Resets the I2C if there is an error.*/
void IMU::resetI2C() {
	IMU_HAL.reset();
	caller->suspendCallerUntil(NOW()+ 5*MILLISECONDS);
	IMU_HAL.init(400000);
	I2C_EN.setPins(0);
	PRINTF("RESET I2C!\r\n");
	caller->suspendCallerUntil(NOW()+ 5*MILLISECONDS);
	I2C_EN.setPins(1);
	I2C_EN.init(true, 1, 1);
	configurateIMU();
}

void IMU::accRead() {
	uint8_t data[6];
	// Read value and write data to data[].
	errorDetection(IMU_HAL.writeRead(accMagAdress, accOutAllAxis, 1, data, 6), 6);

	// Shift, merge, scale( factor, offset) the values.
	acc.x = ((int16_t) ((data[1] << 8) | data[0])) * ACC_SCALING_FACTOR - acc.xOffset;
	acc.y = ((int16_t) ((data[3] << 8) | data[2])) * ACC_SCALING_FACTOR - acc.yOffset;
	acc.z = ((int16_t) ((data[5] << 8) | data[4])) * ACC_SCALING_FACTOR - acc.zOffset;
}

void IMU::accSetDefaultValues() {
	acc.xOffset = ACC_X_OFFSET;
	acc.yOffset = ACC_Y_OFFSET;
	acc.zOffset = ACC_Z_OFFSET;
}

/*Finds the offset value for every axis by the distance to the maxim values to 1000.*/
void IMU::accCalibrate() {
	PRINTF("Accelerometer calibration started...\r\n");
	accAxisOffset(acc.x, acc.xOffset, 'X');
	accAxisOffset(acc.y, acc.yOffset, 'Y');
	accAxisOffset(acc.z, acc.zOffset, 'Z');

	PRINTF("Accelerometer calibration finished. xOffset: %f yOffset: %f zOffset: %f\r\n", acc.xOffset, acc.yOffset, acc.zOffset);
}

/*
 * Calibration of an axis. n samples for every axis / n - 1000 = Offset
 */
void IMU::accAxisOffset(float &axisValue, float &axisOffset, char name) {

	axisOffset = 0;
	uint16_t cnt = 0;
	
	//Display the value until the axis value is bigger than the value (850)
	while (1) {
		PRINTF("Put the board in the direction so that the %c-axis is MAX.\r\n", name);
		accRead();
		PRINTF("ACC x:%f y:%f z:%f\r\n", acc.x, acc.y, acc.z);
		caller->suspendCallerUntil(NOW()+ 400*MILLISECONDS);

		cnt = (axisValue > 850) ? (cnt + 1) : 0;

		if (cnt > 8)
			break;
	}

	axisOffset = 0;
	int16_t offset = 0;
	cnt = 0;

	//Taking samples to calculate the offset
	while (cnt++ < ACC_NBR_SCALE_SAMPLES) {
		accRead();
		offset += (axisValue - 1000);
		PRINTF("value %f\r\n", axisValue);
		caller->suspendCallerUntil(NOW()+ 10*MILLISECONDS);
	}

	axisOffset = offset / ACC_NBR_SCALE_SAMPLES;

	PRINTF("%s-axis offset %f\r\n", name, axisOffset);
}

void IMU::gyrRead() {
	uint8_t data[6];
	// Read value and write data to data[].
	errorDetection(IMU_HAL.writeRead(gyrAdress, gyrOutAllAxis, 1, data, 6), 6);

	// Shift, merge, scale( factor, offset) the values.
	gyr.dx = (((int16_t) ((data[1] << 8) | data[0])) * GYR_SCALING_FACTOR - gyr.dxOffset);
	gyr.dy = (((int16_t) ((data[3] << 8) | data[2])) * GYR_SCALING_FACTOR - gyr.dyOffset);
	gyr.dz = (((int16_t) ((data[5] << 8) | data[4])) * GYR_SCALING_FACTOR - gyr.dzOffset);
}

/* Standstill calibration - Taking N-samples to get a offset for every axis.*/
void IMU::gyrCalibrate(int warningTimer = 3) {
	PRINTF("Gyroscope standstill calibration...starts in\r\n");
	int cnt = 0;

	// Shows the user a counter so that the s/c has standstill.
	while (cnt++ < warningTimer) {
		PRINTF("%d\r\n", warningTimer - cnt);
		caller->suspendCallerUntil(NOW()+ 1*SECONDS);
	}

	cnt = 0;
	gyr.dxOffset = gyr.dyOffset = gyr.dzOffset = 0;

	float gyrX = 0;
	float gyrY = 0;
	float gyrZ = 0;

	while (cnt++ < GYR_CALIBATION_SAMPLES) {
		gyrRead();

		gyrX += gyr.dx;
		gyrY += gyr.dy;
		gyrZ += gyr.dz;

		caller->suspendCallerUntil(NOW()+ 10*MILLISECONDS);
	}
	gyr.dxOffset = gyrX / GYR_CALIBATION_SAMPLES;
	gyr.dyOffset = gyrY / GYR_CALIBATION_SAMPLES;
	gyr.dzOffset = gyrZ / GYR_CALIBATION_SAMPLES;

	PRINTF("Gyroscope offset\r\n  dx %f\r\n  dy %f\r\n  dz %f\r\n", gyr.dxOffset, gyr.dyOffset, gyr.dzOffset);
}

/*
 * IMU data calibrated by the hard iron calibration values.
 * M[i]_s = (M[i] - M[i]_min)/(M[i]_max - M[i]_min)* 2 - 1
 */
void IMU::magRead() {
	uint8_t data[6];
	errorDetection(IMU_HAL.writeRead(accMagAdress, magOutAllAxis, 1, data, 6), 6);

	mag.x = ((((int16_t) ((data[1] << 8) | data[0])) * MAG_SCALING_FACTOR - mag.xMin) / mag.xDiff) * 2 - 1;
	mag.y = ((((int16_t) ((data[3] << 8) | data[2])) * MAG_SCALING_FACTOR - mag.yMin) / mag.yDiff) * 2 - 1;
	mag.z = ((((int16_t) ((data[5] << 8) | data[4])) * MAG_SCALING_FACTOR - mag.zMin) / mag.zDiff) * 2 - 1;
}

/*
 * Hard iron calibration. Calculation of a koeff = 2 *(M[i]max - M[i]min) and a offset = (2 * M[i]min)/(M[i]max -M[i]min) -1
 * These values are used in the magRead()
 */
void IMU::magCalibrate() {
	uint8_t data[6];
	uint16_t cnt = 0;

	int16_t xMax, yMax, zMax, xMin, yMin, zMin, tmpX, tmpY, tmpZ;

	PRINTF("Magnetometer calibration...\r\nRotate the board around all axis.\r\n");
	errorDetection(IMU_HAL.writeRead(accMagAdress, magOutAllAxis, 1, data, 6), 6);
	tmpX = xMax = xMin = ((int16_t) ((data[1] << 8) | data[0]));
	tmpY = yMax = yMin = ((int16_t) ((data[3] << 8) | data[2]));
	tmpZ = zMax = zMin = ((int16_t) ((data[5] << 8) | data[4]));

	while (cnt++ < MAG_NBR_SCALE_SAMPLES) {
		errorDetection(IMU_HAL.writeRead(accMagAdress, magOutAllAxis, 1, data, 6), 6);
		tmpX = ((int16_t) ((data[1] << 8) | data[0]));
		tmpY = ((int16_t) ((data[3] << 8) | data[2]));
		tmpZ = ((int16_t) ((data[5] << 8) | data[4]));

		xMax = (tmpX > xMax) ? tmpX : xMax;
		xMin = (tmpX < xMin) ? tmpX : xMin;

		yMax = (tmpY > yMax) ? tmpY : yMax;
		yMin = (tmpY < yMin) ? tmpY : yMin;

		zMax = (tmpZ > zMax) ? tmpZ : zMax;
		zMin = (tmpZ < zMin) ? tmpZ : zMin;

		if (cnt % 100 == 0)
			PRINTF("-------------\r\nxMin %d xMax %d\r\nyMin %d yMax %d\r\nzMin %d zMax %d\r\n", (int)(xMin*MAG_SCALING_FACTOR), (int)(xMax*MAG_SCALING_FACTOR), (int)(yMin*MAG_SCALING_FACTOR), (int)(yMax*MAG_SCALING_FACTOR), (int)(zMin*MAG_SCALING_FACTOR), (int)(zMax*MAG_SCALING_FACTOR));

		caller->suspendCallerUntil(NOW()+ 10*MILLISECONDS);
	}

	mag.xMin = xMin * MAG_SCALING_FACTOR;
	mag.yMin = yMin * MAG_SCALING_FACTOR;
	mag.zMin = zMin * MAG_SCALING_FACTOR;

	mag.xDiff = (xMax - xMin) * MAG_SCALING_FACTOR;
	mag.yDiff = (yMax - yMin) * MAG_SCALING_FACTOR;
	mag.zDiff = (zMax - zMin) * MAG_SCALING_FACTOR;

	PRINTF("xMin %f\r\nyMin %f\r\nzMax %f\r\n", mag.xMin, mag.yMin, mag.zMin);
	PRINTF("xDiff %f yDiff: %f zDiff: %f\r\n", mag.xDiff, mag.yDiff, mag.zDiff);
}

