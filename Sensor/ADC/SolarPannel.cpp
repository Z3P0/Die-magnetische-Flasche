/*
 * SolarPannel.cpp
 *
 *  Created on: 25.01.2016
 *      Author: pinker
 */

#include "SolarPannel.h"

SolarPannel::SolarPannel(HAL_ADC * adc, ADC_CHANNEL chVoltage, ADC_CHANNEL chCurrent) {
	this->chVoltage = chVoltage;
	this->chCurrent = chCurrent;
	sensor = adc;
}

SolarPannel::~SolarPannel() {}

// TODO Check the magic value 819
int32_t SolarPannel::read(ADC_CHANNEL *channel) {
	int32_t val = sensor->read(chVoltage);
	val = val & 0xFFFFFFF8;     //Removing noise on LSBs
	return val;
}

float SolarPannel::getVoltage() {
	return ((float)read(&chVoltage)/ 819);
}

float SolarPannel::getCurrent() {
	return (((float)read(&chVoltage)/ 819) / RESISTANCE);
}
