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

// Addresses
#define LIGHT_ADDRESS			0x39
#define CTRL_REG		    	(0x00| 0x80)
#define POWER_UP				0x03
//---------------------------
#define CTR_REG1				(0x01| 0x80)
// Low gain simple
// Integration time 101 ms


#define CH0_DATA_LOW          	(0x0C| 0xA0)	// Channel 0: Visible + IR light
#define CH1_DATA_LOW          	(0x0E| 0xA0) 	// Channel 1: IR light



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
	int16_t read();
	int16_t ch0;     // Channel 0: Visible + IR light
	int16_t ch1;     // Channel 1: IR light
	void init();

private:
//	int16_t read();
//	int16_t ch0;     // Channel 0: Visible + IR light
//	int16_t ch1;     // Channel 1: IR light

	// Light sensor
	const uint8_t lightAdress = LIGHT_ADDRESS;
	const uint8_t lightCtrlReg[2] = { CTRL_REG, POWER_UP };
	const uint8_t CHANNEL_0_LOW[1] = { CH0_DATA_LOW };
	const uint8_t CHANNEL_1_LOW[1] = { CH1_DATA_LOW };



};

#endif /* SENSOR_LIGHT_H_ */
