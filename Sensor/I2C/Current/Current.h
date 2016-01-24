/*
 * Current.h
 *
 *  Created on: 11.01.2016
 *      Author: pinker
 */

#ifndef SENSOR_I2C_CURRENT_H_
#define SENSOR_I2C_CURRENT_H_
#include "../../../Define/Define.h"
#include "../../../Extern/Include.h"
#include "../../../Extern/Extern.h"

/* This class is build for INA219AIDCNR current sensors. */
#define CONFIGURATION_REG_ADDR	0x00
#define SHUNT_VOLTAGE_REG_ADDR	0x01
#define BUS_VOLTAGE_REG_ADDR	0x02
#define POWER_REG_ADDR			0x03
#define CURRENT_REG_ADDR		0x04
#define CALIBRATION_REG_ADDR	0x05

// CONFIGURATION_REG_ADDR
// ----------------------------------------------
// Reset Bit: 1=generate reset 					1
// --- 											-
// Bus Voltage Range							1
// -----------------------------------------------
// Shunt Voltage +-320 V
// PG1 											1
// PG0											1
// -----------------------------------------------
// BADC Bus ADC Resolution/Averaging (12-bits)
// ADC 4										1
// ADC 3										X
// ADC 2										1
// ADC 1										1
// -----------------------------------------------
// SADC Shunt ADC Resolution/Averaging (12-bits)
// ADC 4										1
// ADC 3										X
// ADC 2										1
// ADC 1										1
// -----------------------------------------------
// Operation Mode (Shunt and Bus, Continuous)
// MODE 3										1
// MODE 2										1
// MODE 1										1
//-----------------------------------------------
#define CALIBRATION_REG_VAL	0b11111111


// The address of the first current sensor, which gives the total current from the battery
#define ADDR_BATT   		0b1000000 	// Battery current
#define ADDR_HB_A			0b1000001	// H-bridge A
#define ADDR_HB_B			0b1000010	// H-bridge B
#define ADDR_HB_C			0b1000011	// H-bridge C
#define ADDR_HB_D			0b1000100	// H-bridge D

struct CurrentValues {
	float 	power;		// [W]
	float 	current;	// [A]
};

class Current {
public:
	Current(uint8_t adress, HAL_I2C *i2c);
	// return [0] power; return [1] current
	void getPowerCurrent(float *values);
	virtual ~Current();

private:
	// Reference to the i2c
	HAL_I2C *i2c;

	// Read raw vales
	int8_t read();

	// The register addresses of the INA219 current sensor
	// Every register has 2 bytes of data, so two bytes need to be sent over i2c
	const uint8_t CONFIG_REG[1] = { CONFIGURATION_REG_ADDR };
	const uint8_t SHUNT_REG[1] = { SHUNT_VOLTAGE_REG_ADDR };
	const uint8_t BUS_V_REG[1] = { BUS_VOLTAGE_REG_ADDR };
	const uint8_t POWER_V_REG[1] = { POWER_REG_ADDR };
	const uint8_t CURRRENT_REG[1] = { CURRENT_REG_ADDR };
	const uint8_t CALIBRATION_REG[1] = { CALIBRATION_REG_ADDR };

	int16_t raw_calReg;
	int16_t raw_shunt_v;
	int16_t raw_bus_v;
	int16_t raw_power_v;
	int16_t raw_current;

private:
	uint8_t adress;
};

#endif /* SENSOR_I2C_CURRENT_H_ */

