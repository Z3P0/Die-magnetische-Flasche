/*
 * PiController.cpp
 *
 *  Created on: 06.01.2016
 *      Author: pinker
 */

#include "PiController.h"

PiController::PiController() {
	dt = SAMPLING_TIME;     	// Sampling time of the control function
	//PI set values

	kp_pi = KP_PI;
	ki_pi = KI_PI;
	kd_pi = KD_PI;

	//PID set values
	kp_pid = KP_PID;
	ki_pid = KI_PID;
	kd_pid = KD_PID;

	epsilon = EPSILON;
	factor = (PWM_RES / V_MAX_HBrdg);     // PWM Resolution /V_MAX
	output = 0;

	duty = 0;
	derivative = 0;
	integral = 0;
	error = 0;
	preError = 0;
}

PiController::~PiController() {
}

// This function implements the PI-control required to control the velocity of
// the satellite it outputs the duty cycle to be applied to the PWM
// @parameters setpoint =desired speed
// @parameters actual= speed from Gyroscope
unsigned int PiController::pi(float setpoint, float actual) {

	//Calculate PID
	error = setpoint - actual;

	//In case error is too small then stop integration
	if (ABS(error) > epsilon)
		integral = integral + error * dt;

	//derivative= (error - pre_error)/dt;
	//output= Kp*error + Ki*integral + Kd*derivative;
	output = kp_pi * error + ki_pi * integral;

	//Scale PID output for PWM  (it has to be a value between 0-8v)
	//output= (output*Vmax)/(226000*setpoint);

	//from angular velocity to voltage   v=w*0.01175
	output = ABS(output * 0.01175);

	//Saturation filter
	if (output > V_MAX)
		output = V_MAX;
	else if (output < MIN)
		output = MIN;

	//duty cycle
	return ((output / V_MAX) * PWM_RES);
}

// This function implements the PI control required to control the velocity of
// the satellite it outputs the duty cycle to be applied to the PWM
// @parameters setpoint =desired speed
// @parameters actual= speed from Gyroscope
int32_t PiController::pid(float setpoint, float actual) {

//	if(actual > 360)
//		actual -= 360;
//	else if(actual < -360)
//		actual += 360;

	//Calculate PID
	error = setpoint - actual;

	// In case error is too small then stop integration
	if (ABS(error) > epsilon)
		integral = integral + error * dt;

	derivative = (error - preError) / dt;

	preError = error;

	//Change all ki values to pid
	output = kp_pid * error + ki_pid * integral + kd_pid * derivative;
	//output = kp_pid * error + ki_pid * integral;

	// From angular velocity to voltage
	// output = ABS(output);

	// Saturation filter
	if (output > V_MAX)
		output = V_MAX;
	else if (output < -V_MAX)
		output = -V_MAX;

	// Duty cycle
	return (-1*output *factor);
}
