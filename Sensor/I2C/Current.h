/*
 * Current.h
 *
 *  Created on: 11.01.2016
 *      Author: pinker
 */

#ifndef SENSOR_I2C_CURRENT_H_
#define SENSOR_I2C_CURRENT_H_
#include "../../Define/Define.h"
#include "../../Extern/Include.h"
#include "../../Extern/Extern.h"

/* This class is build for INA219AIDCNR current sensors. */
#define CONFIGURATION_REG_ADDR	0x00
#define SHUNT_VOLTAGE_REG_ADDR	0x01
#define BUS_VOLTAGE_REG_ADDR	0x02
#define POWER_REG_ADDR			0x03
#define CURRENT_REG_ADDR		0x04
#define CALIBRATION_REG_ADDR	0x05

// The address of the first current sensor, which gives the total current from the battery
#define ADDR_BATT   		0b1000000 	// Battery current
#define ADDR_HB_A			0b1000001	// H-bridge A
#define ADDR_HB_B			0b1000010	// H-bridge B
#define ADDR_HB_C			0b1000011	// H-bridge C
#define ADDR_HB_D			0b1000100	// H-bridge D

// CONFIGURATION_REG_ADDR
// ----------------------------------------
// Reset Bit: 1=generate reset 					0111
// Power down: 			normal mode				1
// Z: 					enabled					1
// Y: 					enabled					1
// X: 					enabled					1

#define CTRL_REG4_G_ADDRESS 	0x23
#define CTRL_REG4_G_VALUE 		0b10000000


struct CurrentValues {
	int16_t power;
	int16_t current;
};

class Current {
public:
	Current(uint8_t adress);
	// Initialize current sensor
	void init();
	int8_t read(CurrentValues *values);
	uint8_t current_sensor_read(int16_t* values);
	virtual ~Current();
private:
	//The register addresses of the INA219 current sensor
	//Every register has 2 bytes of data, so two bytes need to be sent over i2c
	const uint8_t CONFIG_REG[1] = { CONFIGURATION_REG_ADDR };
	const uint8_t SHUNT_REG[1] = { SHUNT_VOLTAGE_REG_ADDR };
	const uint8_t BUS_V_REG[1] = { BUS_VOLTAGE_REG_ADDR };
	const uint8_t POWER_V_REG[1] = { POWER_REG_ADDR };
	const uint8_t CURRRENT_REG[1] = { CURRENT_REG_ADDR };
	const uint8_t CALIBRATION_REG[1] = { CALIBRATION_REG_ADDR };

private:
	uint8_t adress;
};

#endif /* SENSOR_I2C_CURRENT_H_ */

