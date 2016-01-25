/*
 * Current.cpp
 *
 *  Created on: 11.01.2016
 *      Author: pinker
 */

#include "Current.h"

//TODO 16 V

Current::Current(uint8_t adress) {
	this->adress = adress;

	raw_calReg = 0;
	raw_shunt_v = 0;
	raw_bus_v = 0;
	raw_power_v = 0;
	raw_current = 0;
}

Current::~Current() {
}


/*
 *
 * Error check by the number of the returned bytes
 * return 0 no error
 * return -1 error
 */
int8_t Current::read() {
	uint8_t data[2];

	// Read the calibration register
	if (I2C_1.writeRead(adress, CALIBRATION_REG, 2, data, 2) != 2)
		return -1;
	raw_calReg = (int16_t) (data[0] << 8 | data[1]);

	// Read the shunt register
	if (I2C_1.writeRead(adress, SHUNT_REG, 2, data, 2) != 2)
		return -1;
	raw_shunt_v = (int16_t) (data[0] << 8 | data[1]);

	// Read the bus voltage register
	if (I2C_1.writeRead(adress, BUS_V_REG, 2, data, 2) != 2)
		return -1;
	raw_bus_v = (int16_t) (data[0] << 8 | data[1]);

	// Read the power voltage register
	if (I2C_1.writeRead(adress, POWER_V_REG, 2, data, 2) != 2)
		return -1;
	raw_power_v = (int16_t) (data[0] << 8 | data[1]);

	// Read the current register
	if (I2C_1.writeRead(adress, CURRRENT_REG, 2, data, 2) != 2)
		return -1;
	raw_current = (int16_t) (data[0] << 8 | data[1]);

	return 0;
}

// values [0] power; values [1] current
void Current::getPowerCurrent(float *values) {
	if (read() == -1)
		PRINTF("Current sensor read error\r\n");

	// Power = Current * BusVoltage / 5000
	values[0] = (raw_current * raw_bus_v) / 5000.0;

	// Current = shuntVoltage * calibrationRegister / 4096;
	values[1] = (raw_shunt_v * RAW_CAL_REGISTER) / 4096.0;

}

