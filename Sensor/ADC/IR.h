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

class IR {
public:
	IR(HAL_ADC * adc, ADC_CHANNEL cha);
	virtual ~IR();
	float read();
	void sample(float distance);
	void init();
	void simpleCheck();
	float readInternal();

private:
	void scale();
	HAL_ADC * sensor;
	ADC_CHANNEL  channel;
	int totalSamples;
	int maxSamples;
	float dist[20];
	float volt[20];
	float a, b, c;
};

#endif /* SENSOR_ADC_IR_H_ */
