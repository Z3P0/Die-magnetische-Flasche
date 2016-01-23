/*
 * IMU.cpp
 *
 *  Created on: 22.12.2015
 *      Author: pinker
 */

#include "IMU.h"
#include "../../../Define/Define.h"
#include "../../../Extern/Extern.h"

/*HAL*/
HAL_GPIO CS_XM(GPIO_032);     // Chip select Accelerometer and Magnetometer
HAL_GPIO CS_G(GPIO_018);	  // Chip select Gyroscope
HAL_I2C I2C_2(I2C_IDX2);

IMU::IMU(Thread *caller, float sampleRate) {
	// Reference to the caller thread to suspend it
	this->caller = caller;

	// Sample rate for integration of the values from the Gyroscope
	this->sampleRate = sampleRate;

	//set values to zero
	acc.x = acc.y = acc.z = 0;
	accXOff = accYOff = accZOff = 0;
	gyr.dx = gyr.dy = gyr.dz = 0;
	gyr.r = gyr.p = gyr.y = 0;
	gyrDxOff = gyrDyOff = gyrDzOff = 0;
	mag.x = mag.y = mag.z = 0;
	magXMin = magXDiff = 0;
	magYMin = magYDiff = 0;
	magZMin = magZDiff = 0;

	configurateIMU();
}

IMU::~IMU() {
}

void IMU::configurateIMU() {

	//Chip select
	CS_XM.init(true, 1, 1);
	CS_G.init(true, 1, 1);

	//enable IMU
	I2C_EN.init(true, 1, 1);
	I2C_2.init(400000);

	//Initialize the gyroscope
	errorDetection(I2C_2.write(gyrAdress, gyrCtrlReg1, 2), 2);
	errorDetection(I2C_2.write(gyrAdress, gyrCtrlReg4, 2), 2);
	errorDetection(I2C_2.write(gyrAdress, gyrCtrlReg5, 2), 2);

	//Initialize the accelerometer and the magnetometer LSM9DS0
	errorDetection(I2C_2.write(accMagAdress, accMagCtrlReg1, 2), 2);
	errorDetection(I2C_2.write(accMagAdress, accMagCtrlReg2, 2), 2);
	errorDetection(I2C_2.write(accMagAdress, accMagCtrlReg5, 2), 2);
	errorDetection(I2C_2.write(accMagAdress, accMagCtrlReg6, 2), 2);
	errorDetection(I2C_2.write(accMagAdress, accMagCtrlReg7, 2), 2);

	//Initialize the accelerometer and the magnetometer LSM303DLH
	errorDetection(I2C_2.write(magAdressLSM303, crAReg, 2), 2);
	errorDetection(I2C_2.write(magAdressLSM303, crBReg, 2), 2);
	errorDetection(I2C_2.write(magAdressLSM303, mrReg, 2), 2);
}

/*Checks if if there is a IMU read error by comparing the written with  the expected number of bytes.*/
void IMU::errorDetection(int16_t nbrOfReceivedBytes, int8_t expectedNumber) {
	if (nbrOfReceivedBytes != expectedNumber) {
		PRINTF("ERROR - IMU\n number of bytes: %d expected %d\r\n", expectedNumber, nbrOfReceivedBytes);
		resetI2C();
	}
}

/*Resets the I2C if there is an error.*/
void IMU::resetI2C() {
	I2C_2.reset();
	caller->suspendCallerUntil(NOW()+ 5*MILLISECONDS);
	I2C_2.init(400000);
	I2C_EN.setPins(0);
	PRINTF("RESET I2C!\r\n");
	caller->suspendCallerUntil(NOW()+ 5*MILLISECONDS);
	I2C_EN.setPins(1);
	I2C_EN.init(true, 1, 1);
	configurateIMU();
}

void IMU::accRead() {
	uint8_t data[6];
	errorDetection(I2C_2.writeRead(accMagAdress, accOutAllAxis, 1, data, 6), 6);

	acc.x = ((int16_t) ((data[1] << 8) | data[0])) * ACC_SCALING_FACTOR - accXOff;
	acc.y = ((int16_t) ((data[3] << 8) | data[2])) * ACC_SCALING_FACTOR - accYOff;
	acc.z = ((int16_t) ((data[5] << 8) | data[4])) * ACC_SCALING_FACTOR - accZOff;

	//Accelerometer: Roll, pitch angle
	acc.r = -atan2((double) acc.x, sqrt((double) (acc.y * acc.y + acc.z * acc.z))) * RAD_TO_DEG;
	acc.p = atan2((double) acc.y, sqrt((double) (acc.x * acc.x + acc.z * acc.z))) * RAD_TO_DEG;

}

void IMU::accSetDefaultValues() {
	accXOff = ACC_X_OFFSET;
	accYOff = ACC_Y_OFFSET;
	accZOff = ACC_Z_OFFSET;
}

/*Finds the offset value for every axis by the distance to the maxim values to 1000.*/
void IMU::accCalibrate() {
	PRINTF("Accelerometer calibration started...\r\n");
	accAxisOffset(acc.x, accXOff, 'X');
	accAxisOffset(acc.y, accYOff, 'Y');
	accAxisOffset(acc.z, accZOff, 'Z');

	PRINTF("Accelerometer calibration finished. xOffset: %f yOffset: %f zOffset: %f\r\n", accXOff, accYOff, accZOff);
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
	errorDetection(I2C_2.writeRead(gyrAdress, gyrOutAllAxis, 1, data, 6), 6);

	gyr.dx = (((int16_t) ((data[1] << 8) | data[0])) * GYR_SCALING_FACTOR - gyrDxOff);
	gyr.dy = (((int16_t) ((data[3] << 8) | data[2])) * GYR_SCALING_FACTOR - gyrDyOff);
	gyr.dz = (((int16_t) ((data[5] << 8) | data[4])) * GYR_SCALING_FACTOR - gyrDzOff);

	// Roll, pitch, yaw
	gyr.r += (gyr.dy * sampleRate);
	gyr.p += (gyr.dx * sampleRate);
	gyr.y += (gyr.dz * sampleRate);
}

/* Standstill calibration - Taking N-samples to get a offset for every axis.*/
void IMU::gyrCalibrate() {
	PRINTF("Gyroscope standstill calibration...starts in\r\n");
	int cnt = 0;
	int timer = 3;

	while (cnt++ < timer) {
		PRINTF("%d\r\n", timer - cnt);
		caller->suspendCallerUntil(NOW()+ 1*SECONDS);
	}

	cnt = 0;
	gyrDxOff = gyrDyOff = gyrDzOff = 0;

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
	gyrDxOff = gyrX / GYR_CALIBATION_SAMPLES;
	gyrDyOff = gyrY / GYR_CALIBATION_SAMPLES;
	gyrDzOff = gyrZ / GYR_CALIBATION_SAMPLES;

	PRINTF("Gyroscope offset\r\n  dx %f\r\n  dy %f\r\n  dz %f\r\n", gyrDxOff, gyrDyOff, gyrDzOff);

	gyr.r = gyr.p = gyr.y = 0;
}

/*
 * LSM303DLH
 */
void IMU::magReadLSM303DLH() {
	uint8_t data[6];
	errorDetection(I2C_2.writeRead(magAdressLSM303, outAllAxisLSM303, 1, data, 6), 6);

	mag.x = ((int16_t) (((data[0] << 8) | data[1]) - magXMin / magXDiff) * 2 - 1);
	mag.y = ((int16_t) (((data[2] << 8) | data[3]) - magYMin / magYDiff) * 2 - 1);
	mag.z = ((int16_t) (((data[4] << 8) | data[5]) - magZMin / magZDiff) * 2 - 1);
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

	// LSM303DHL
	errorDetection(I2C_2.writeRead(magAdressLSM303, outAllAxisLSM303, 1, data, 6), 6);

	tmpX = xMax = xMin = ((int16_t) ((data[0] << 8) | data[1]));
	tmpY = yMax = yMin = ((int16_t) ((data[2] << 8) | data[3]));
	tmpZ = zMax = zMin = ((int16_t) ((data[4] << 8) | data[5]));

	while (cnt++ < MAG_NBR_SCALE_SAMPLES) {
		errorDetection(I2C_2.writeRead(magAdressLSM303, outAllAxisLSM303, 1, data, 6), 6);
		tmpX = ((int16_t) ((data[0] << 8) | data[1]));
		tmpY = ((int16_t) ((data[2] << 8) | data[3]));
		tmpZ = ((int16_t) ((data[4] << 8) | data[5]));

		xMax = (tmpX > xMax) ? tmpX : xMax;
		xMin = (tmpX < xMin) ? tmpX : xMin;

		yMax = (tmpY > yMax) ? tmpY : yMax;
		yMin = (tmpY < yMin) ? tmpY : yMin;

		zMax = (tmpZ > zMax) ? tmpZ : zMax;
		zMin = (tmpZ < zMin) ? tmpZ : zMin;

		if (cnt % 100 == 0) {
			PRINTF("-------------\r\nxMin %d xMax %d\r\nyMin %d yMax %d\r\nzMin %d zMax %d\r\n", (int) (xMin * LSM303_MAG_GAIN_XY), (int) (xMax * LSM303_MAG_GAIN_XY),
					(int) (yMin * LSM303_MAG_GAIN_XY), (int) (yMax * LSM303_MAG_GAIN_XY), (int) (zMin * LSM303_MAG_GAIN_Z), (int) (zMax * LSM303_MAG_GAIN_Z));
			//((PRINTF("x:%d\r\ny:%d\r\nz:%d\r\n", tmpX, tmpY, tmpZ);
			PRINTF("-----------------------\r\n");
		}
		caller->suspendCallerUntil(NOW()+ 10*MILLISECONDS);
	}

	magXMin = xMin;
	magYMin = yMin;
	magZMin = zMin;


	magXDiff = (xMax - xMin);
	magYDiff = (yMax - yMin);
	magZDiff = (zMax - zMin);

	PRINTF("xMin %f\r\nyMin %f\r\nzMax %f\r\n", magXMin, magYMin, magZMin);
	PRINTF("magXDiff %f yDiff: %f zDiff: %f\r\n", magXDiff, magYDiff, magZDiff);
}

