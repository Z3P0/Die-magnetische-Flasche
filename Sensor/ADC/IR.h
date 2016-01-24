/*
 * IR.h
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_ADC_IR_H_
#define SENSOR_ADC_IR_H_
#include "../../Extern/Include.h"
#include "../../Define/Define.h"
#include "../../Extern/Extern.h"

#define ADC_CH 	ADC_CH_011 // PC1

class IR {
public:
	IR(HAL_ADC * adc);
	virtual ~IR();
	float read();
	void sample(float distance);
	void init();

private:
	void scale();
	float readInternal();
	HAL_ADC * sensor;
	int totalSamples;
	int maxSamples;
	float dist[20];
	float volt[20];
	float a, b, c;
};

#endif /* SENSOR_ADC_IR_H_ */
