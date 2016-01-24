/*
 * Light.h
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_LIGHT_H_
#define SENSOR_LIGHT_H_
#include "../../../Extern/Include.h"

/**
 *  Light sensor TAOS TSL2561 - https://www.adafruit.com/datasheets/TSL2561.pdf)
 */

// 				Wiring
//--------------------------------------------
// 		 LIGHT SENSOR			    BOARD(I2C_1)          Power
//	GND 	black  ------------------------------- black   GND
//	ADDR
//	INT
//	SCL  	blue   ---------------	blue 	SCL
//	SDA	 	green  ---------------	green 	SDA
//	VCC		red    ------------------------------- red 	   VCC
//

/*--Light sensor--*/
#define LIGHT_ADDRESS			0x39
#define CTRL_REG		    	0x00
#define POWER_UP				0x03

#define CH0_DATA_LOW          	0x0C  // Channel 0: Visible + IR light
#define CH1_DATA_LOW          	0x0E  // Channel 1: IR light

class Light {
public:
	Light();
	virtual ~Light();
	/*
	 * The silicon detectors respond strongly to the IR light that is invisible for the human eye.
	 * The sensor has two different IR-diodes to compensate this error. The first thing is to calculate the
	 * coefficient. After that a formula as described in the data sheet is used.
	 */
	float getLuxValue();

private:
	// Reference to the i2c
	HAL_I2C *i2c;

	int16_t ch0;  // Channel 0: Visible + IR light
	int16_t ch1;  // Channel 1: IR light

	/*Light sensor*/
	const uint8_t lightAdress = LIGHT_ADDRESS;
	const uint8_t lightCtrlReg[2] = { CTRL_REG, POWER_UP };
	const uint8_t CHANNEL_0_LOW[1] = { CH0_DATA_LOW | 0xA0 };
	const uint8_t CHANNEL_1_LOW[1] = { CH1_DATA_LOW | 0xA0 };

	void init();
	uint8_t read();
};

#endif /* SENSOR_LIGHT_H_ */
