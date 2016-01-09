/*
 * IR.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "IR.h"

HAL_ADC sensor(ADC_IDX1);

IR::IR() {
	sensor.config(ADC_PARAMETER_RESOLUTION, 12);
	sensor.init(ADC_CH_1);
}

IR::~IR() {}

int32_t IR::read(){
	return sensor.read(ADC_CH_1);
}

void IR::scale(){
	//TODO: Find scaling function or a polynom to scale the values
}
