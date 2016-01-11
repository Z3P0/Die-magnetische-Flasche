/*
 * PiController.cpp
 *
 *  Created on: 06.01.2016
 *      Author: pinker
 */

#include "PiController.h"
#include <stdlib.h>    //needed for abs()

PiController::PiController() {
	dt = 0.01;     				// Sampling time of the control function
	kp = KP;
	ki = KI;
	kd = KD;

	epsilon = EPSILON;
	factor = (PWM_RES / V_MAX);     // PWM Resolution /V_MAX

	derivative = 0;
	integral = 0;
	error = 0;
	preError = 0;
}

PiController::~PiController() {
}

int32_t PiController::velocityPI(float setpoint, float actual) {

	// Calculate error value
	error = setpoint - actual;

	// Reduces the steady state error.
	//In case error is too small then it stops the integration
	if (ABS(error) > epsilon) {
		integral += (error * dt);
	}

	float output = kp * error + ki * integral;

	//Saturation filter
	if (output > V_MAX_HBrdg)
		output = V_MAX_HBrdg;
	else if (output < 0)
		output = 0;

	// Transform the error value to a duty cycle
	int32_t dutyCycle = (output * factor);     //factor = PWM resolution / Vmax
	
	if(setpoint < 0)
	   	return -dutyCycle;

	return dutyCycle;
}




int32_t PiController::velocityPI2(float setpoint, float actual) {

	// Calculate error value
	error = setpoint - actual;

	// Reduces the steady state error.
	//In case error is too small then it stops the integration
	if (ABS(error) > epsilon) {
		integral += (error * dt);
	}

	float output = kp * error + ki * integral;

	//Saturation filter
	if (output > V_MAX_HBrdg)
		output = V_MAX_HBrdg;
	else if (output < V_MAX_HBrdg)
		output = - V_MAX_HBrdg;

	// Transform the error value to a duty cycle
	int32_t dutyCycle = (output * factor);     //factor = PWM resolution / Vmax

	return dutyCycle;
}

int32_t PiController::velocityPID(float setpoint, float actual) {

	// Calculate error value
	error = setpoint - actual;

	// Reduces the steady state error.
	//In case error is too small then it stops the integration
	if (ABS(error) > epsilon) {
		integral += (error * dt);
	}

	// Derivative value calculation.
	derivative = (error - preError) / dt;

	float output = kp * error + ki * integral + kd * derivative;

	//Saturation filter
	if (output > V_MAX_HBrdg)
		output = V_MAX_HBrdg;
	else if (output < V_MAX_HBrdg)
		output = - V_MAX_HBrdg;

	// Transform the error value to a duty cycle
	int32_t dutyCycle = (output * factor);     //factor = PWM resolution / Vmax

	return dutyCycle;
}

