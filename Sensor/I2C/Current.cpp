/*
 * Current.cpp
 *
 *  Created on: 11.01.2016
 *      Author: pinker
 */

#include "Current.h"

Current::Current(uint8_t adress) {
	this->adress = adress;
}

Current::~Current() {
}

void Current::init() {
	//I2C 1 enable and initialize
	I2C_EN.init(true, 1, 1);
	I2C_1.init(i2c_1_speed);
}

/*
 * Error check by the number of the returned bytes
 * return 0 no error
 * return -1 error
 */
int8_t Current::read(CurrentValues *values) {
	uint8_t data[2];

	// Read the power register
	//if(I2C_1.writeRead(adress, POWER_REG, 2, data, 2) != 2)
		return -1;
	values->power = (int16_t)(data[0] <<8 | data[1]);

	// Read the current register
	//if(I2C_1.writeRead(adress, CURRENT_REG, 2, data, 2) != 2)
		return -1;
	values->current = (int16_t)(data[0] <<8 | data[1]);

	return 0;
}

