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


class HAL_SCCB {
public:
	HAL_SCCB();
	virtual ~HAL_SCCB();
	void init();
	uint8_t read_reg(uint8_t address);
	uint8_t write_reg(uint8_t address, uint8_t data);
};

#endif /* SENSOR_CAMERA_HALSCCB_H_ */
