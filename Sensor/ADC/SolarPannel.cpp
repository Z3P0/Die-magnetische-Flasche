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

	volVal = (R1 + R2)/ R1;
	curVal = (R4 + R5) / (R5 * R3);
}

void SolarPannel::init() {
	sensor->config(ADC_PARAMETER_RESOLUTION, 12);
	sensor->init(chVoltage);
	sensor->init(chCurrent);
}

SolarPannel::~SolarPannel() {}

/* Read function for both ADC
 * 1) Voltage
 * 2) Currency
 * @param readChannel
 * */
int32_t SolarPannel::read(ADC_CHANNEL *channel) {
	int32_t val = sensor->read(chVoltage);
	return val;
}

/*
 * Voltage divider resistor formula implemented
 */
float SolarPannel::getVoltage() {
	// Resolution 12 bit = 4096
	// Max Voltage = 3.3 V
	// v_res = (Max Voltage * Result)/4095
	float mes = (float)read(&chVoltage) * VOL_CAL;
	return mes * volVal;
}

/*
 * Voltage and Current divider resistor formula implemented
 */
float SolarPannel::getCurrent() {

	//TODO check that!
	float cur = (float)read(&chCurrent) * CUR_CAL;

	return (getVoltage() /R1 + cur * curVal + cur/R5);
}

/*
 * P = U * I
 */
float SolarPannel::getPower(){
	return (getCurrent() * getVoltage());
}
