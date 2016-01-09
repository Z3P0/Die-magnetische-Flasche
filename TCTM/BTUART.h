/*
 * BTUART.h
 *
 *  Created on: Dec 9, 2015
 *      Author: Usama Tariq
 */

#ifndef STM32F4_RODOS_TEMPLATE_COMMUNICATION_BTUART_H_
#define STM32F4_RODOS_TEMPLATE_COMMUNICATION_BTUART_H_

#include "rodos.h"
#include "hal/hal_uart.h"


class BT_UART {
public:
	void write(const char *buf, int size);
	void read(char *buff, int index, int size);
	BT_UART();
};
#endif /* STM32F4_RODOS_TEMPLATE_COMMUNICATION_BTUART_H_ */
