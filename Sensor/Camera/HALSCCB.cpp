/*
 * HALSCCB.cpp
 *
 *  Created on: Jan 10, 2016
 *      Author: Usama Tariq
 */

#include "HALSCCB.h"

HAL_I2C i2c_sccb(I2C_IDX1);

HAL_SCCB::HAL_SCCB() {
	// TODO Auto-generated constructor stub
}

HAL_SCCB::~HAL_SCCB() {
	// TODO Auto-generated destructor stub
}

void HAL_SCCB::init()
{
	i2c_sccb.init(400000);
}

uint8_t HAL_SCCB::read_reg(uint8_t address)
{
	uint8_t data = 0;
	//delay
	//i2c_sccb.init(100000);
	//delay
	i2c_sccb.write(0x42, &address, 1);
	//delay
	//for(int i=0;i<1000;i++) asm("nop");
	i2c_sccb.read(0x42, &data, 1);
	return data;
}

uint8_t HAL_SCCB::write_reg(uint8_t address, uint8_t data)
{
	uint8_t buff[2];
	buff[0] = address;
	buff[1] = data;
	int res = i2c_sccb.write(0x42, buff, 2);
	PRINTF("Write result: %d\r\n", res);
	return 0;
}
