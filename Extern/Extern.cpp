/*
 * Extern.cpp
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#include "Extern.h"

/* I2C 1 - user for the light and the current sensors.*/
HAL_GPIO I2C_EN(GPIO_055);
HAL_I2C I2C_1(I2C_IDX1);

/* Communication UART*/
HAL_UART bluetooth_uart(UART_IDX2);

/* Blue LED - the last that is not effected by the H-bridge*/
HAL_GPIO BlueLED(GPIO_063);

/* H-Bridges*/
HAL_GPIO HBRIDGE_EN(GPIO_066);

/* H Bridge A*/
HAL_PWM  HBRIDGE_A(PWM_IDX12);
/* Inputs*/
HAL_GPIO HBRIDGE_A_INA(GPIO_036);
HAL_GPIO HBRIDGE_A_INB(GPIO_017);

/* H Bridge B*/
HAL_PWM HBRIDGE_B(PWM_IDX13);
/* Inputs*/
HAL_GPIO HBRIDGE_B_INA(GPIO_016);
HAL_GPIO HBRIDGE_B_INB(GPIO_071);

/* H Bridge C*/
HAL_PWM HBRIDGE_C(PWM_IDX14);
/* Inputs*/
HAL_GPIO HBRIDGE_C_INA(GPIO_072);
HAL_GPIO HBRIDGE_C_INB(GPIO_074);


