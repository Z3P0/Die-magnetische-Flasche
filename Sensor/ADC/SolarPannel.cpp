/*
 * SolarPannel.cpp
 *
 *  Created on: 25.01.2016
 *      Author: pinker
 */

#include "SolarPannel.h"

// 2.4125 Voltage calibration value
// 2.8875 Current calibration value

SolarPannel::SolarPannel(HAL_ADC * adc, ADC_CHANNEL chVoltage, ADC_CHANNEL chCurrent) {
	this->chVoltage = chVoltage;
	this->chCurrent = chCurrent;
	sensor = adc;
}

void SolarPannel::init() {
	sensor->config(ADC_PARAMETER_RESOLUTION, 12);
	sensor->init(chVoltage);
	sensor->init(chCurrent);
}

SolarPannel::~SolarPannel() {}

int32_t SolarPannel::read(ADC_CHANNEL *channel) {
	int32_t val = sensor->read(chVoltage);
	return val;
}

float SolarPannel::getVoltage() {
	// Resolution 12 bit = 4096
	// Max Voltage = 3.3 V
	// v_res = (Max Voltage * Result)/4095
	return (((float)read(&chCurrent)*2.8875)/4095);
	//return (((float)read(&chVoltage)*2.4125)/4095);
}

float SolarPannel::getCurrent() {
	return (((float)read(&chVoltage)/ 1240) / RESISTANCE);
}
