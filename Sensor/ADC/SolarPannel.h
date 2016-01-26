/*
 * SolarPannel.h
 *
 *  Created on: 25.01.2016
 *      Author: pinker
 */

#ifndef SENSOR_ADC_SOLARPANNEL_H_
#define SENSOR_ADC_SOLARPANNEL_H_
#include "../../Extern/Include.h"

#define R1		1.98E3 //[Ohm]
#define R2		8.0E3  //[Ohm]
#define R3		150.0  //[Ohm]
#define R4		2.2E3  //[Ohm]
#define R5		1.0E3  //[Ohm]

#define VOL_CAL	(2.4125/4095) // Voltage calibration value
#define CUR_CAL	(2.8875/4095) //Current calibration value


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
	// Returns the power
	float getPower();

	void init();

private:
	/* Internal function to read the ADC channel*/
	int32_t read(ADC_CHANNEL *channel);
	HAL_ADC * sensor;
	ADC_CHANNEL  chVoltage;
	ADC_CHANNEL  chCurrent;


	float volVal; 	// (R1+R2)/ R1
	float curVal;	// (R4+R5)/ (R5 * R3)
};

#endif /* SENSOR_ADC_SOLARPANNEL_H_ */
