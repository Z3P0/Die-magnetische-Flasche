/*
 * Dcmi.h
 *
 *  Created on: 12.12.2015
 *      Author: akynos
 */

#ifndef PERIPHERAL_DCMI_H_
#define PERIPHERAL_DCMI_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"

#include "stm_misc.h"

#include "../../Define/Define.h"


class Dcmi {
private:
	uint32_t imgSize;
	uint32_t dmaMemAddr;
	uint16_t captRate;
	uint16_t captMode;
public:
	Dcmi(uint32_t imageSize, uint32_t dmaMemoryAddress, uint16_t captureRate, uint16_t captureMode);
	void InitDCMI();
	void InitDCMI2();
	void InitGPIO();
	void EnableDCMI();
	void EnableDCMI2();
	void DisableDCMI();
	void delay(int ns);
};

#endif /* PERIPHERAL_DCMI_H_ */
