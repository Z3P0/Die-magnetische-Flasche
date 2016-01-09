/*
 * Extern.h
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#ifndef EXTERN_H_
#define EXTERN_H_
#include "Include.h"

extern HAL_GPIO I2C_EN;

extern HAL_GPIO BlueLED;
extern HAL_UART bluetooth_uart;

/* H Bridge A*/
extern HAL_PWM  HBRIDGE_A;
/* Inputs*/
extern HAL_GPIO HBRIDGE_A_INA;
extern HAL_GPIO HBRIDGE_A_INB;

/* H Bridge B*/
extern HAL_PWM HBRIDGE_B;
/* Inputs*/
extern HAL_GPIO HBRIDGE_B_INA;
extern HAL_GPIO HBRIDGE_B_INB;

/* H Bridge C*/
extern HAL_PWM HBRIDGE_C;
/* Inputs*/
extern HAL_GPIO HBRIDGE_C_INA;
extern HAL_GPIO HBRIDGE_C_INB;

extern HAL_GPIO HBRIDGE_EN;


#endif /* EXTERN_H_ */
