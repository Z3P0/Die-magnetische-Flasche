/*
 * Extern.h
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#ifndef EXTERN_H_
#define EXTERN_H_
#include "Include.h"
#include "../Define/Define.h"

/* Communication UART*/
extern HAL_UART bluetooth_uart;

/* Blue LED - the last that is not effected by the H-bridge*/
extern HAL_GPIO BlueLED;

/* H-Bridges*/
extern HAL_GPIO HBRIDGE_EN;

/* H Bridge A*/
extern HAL_PWM  HBRIDGE_A;
//---Inputs
extern HAL_GPIO HBRIDGE_A_INA;
extern HAL_GPIO HBRIDGE_A_INB;

/* H Bridge B*/
extern HAL_PWM HBRIDGE_B;
//---Inputs
extern HAL_GPIO HBRIDGE_B_INA;
extern HAL_GPIO HBRIDGE_B_INB;

/* H Bridge C*/
extern HAL_PWM HBRIDGE_C;
//---Inputs
extern HAL_GPIO HBRIDGE_C_INA;
extern HAL_GPIO HBRIDGE_C_INB;

extern HAL_ADC ADC_1;
extern HAL_ADC ADC_2;

extern HAL_UART BT;

extern HAL_I2C I2C_1;

extern uint8_t DCMI_Buffer[HEIGHT*WIDTH * 2];


#endif /* EXTERN_H_ */
