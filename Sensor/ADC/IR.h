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

#define ADC_CH_1 	ADC_CH_001 // PA1

class IR {
public:
	IR();
	virtual ~IR();
	int32_t read();

private:
	void scale();
};

#endif /* SENSOR_ADC_IR_H_ */
