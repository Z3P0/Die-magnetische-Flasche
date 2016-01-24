/*
 * HALSCCB.h
 *
 *  Created on: Jan 10, 2016
 *      Author: Usama Tariq
 */

#ifndef SENSOR_CAMERA_HALSCCB_H_
#define SENSOR_CAMERA_HALSCCB_H_

#include "../../Define/Define.h"
#include "../../Extern/Extern.h"

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dcmi.h>
#include <stm32f4xx_dma.h>

class HAL_SCCB {
public:
	HAL_SCCB();
	virtual ~HAL_SCCB();
	void init();
	uint8_t read_reg(uint8_t address);
	uint8_t write_reg(uint8_t address, uint8_t data);
private:
	void init_I2C2(void);
	void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
	void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
	uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
	uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
	void I2C_stop(I2C_TypeDef* I2Cx);
	uint8_t I2C_writereg(uint8_t reg, uint8_t data);
	uint8_t I2C_readreg(uint8_t reg);
	void Delay(uint32_t ms);

};

#endif /* SENSOR_CAMERA_HALSCCB_H_ */
