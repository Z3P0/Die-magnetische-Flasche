/*
 * SolarPannel.h
 *
 *  Created on: 25.01.2016
 *      Author: pinker
 */

#ifndef SENSOR_ADC_SOLARPANNEL_H_
#define SENSOR_ADC_SOLARPANNEL_H_
#include "../../Extern/Include.h"

#define RESISTANCE 	1.0

class SolarPannel {
public:
	/*
	 * @param ADC
	 * @param Channel for voltage measurements
	 * @param Channel for current measurements
	 */
	SolarPannel(HAL_ADC * adc, ADC_CHANNEL chVoltage, ADC_CHANNEL chCurrent);
	virtual ~SolarPannel();
	// Converts the ADC value to [V]
	float getCurrent();
	// Converts the ADC value to [A]
	float getVoltage();
	void init();

private:
	/* Internal function to read the ADC channel*/
	int32_t read(ADC_CHANNEL *channel);
	HAL_ADC * sensor;
	ADC_CHANNEL  chVoltage;
	ADC_CHANNEL  chCurrent;
};

#endif /* SENSOR_ADC_SOLARPANNEL_H_ */
