/*
 * PiController.cpp
 *
 *  Created on: 06.01.2016
 *      Author: pinker
 */

#include "PiController.h"

PiController::PiController() {
	dt = 0.01;     				// Sampling time of the control function
	kp = KP;
	ki = KI;
	kd = KD;

	epsilon = EPSILON;
	factor = (PWM_RES / V_MAX_HBrdg);     // PWM Resolution /V_MAX
	output = 0;

	derivative = 0;
	integral = 0;
	error = 0;
	preError = 0;
}

PiController::~PiController() {
}

//this function implements the PI control required to control the velocity of the satellite
//it outputs the duty cycle to be applied to the pwm
//parameters= 1)setpoint=desired speed 2)actual= speed from gyro
unsigned int PiController::pi(float setpoint, float actual) {

	//Calculate PID
	error = setpoint - actual;

	//In case error is too small then stop integration
	if (ABS(error) > epsilon)
		integral = integral + error * dt;

	//derivative= (error - pre_error)/dt;
	//output= Kp*error + Ki*integral + Kd*derivative;
	output = kp * error + ki * integral;

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

//this function implements the PI control required to control the velocity of the satellite
//it outputs the duty cycle to be applied to the pwm
//parameters= 1)setpoint=desired speed 2)actual= speed from gyro
int32_t PiController::pid(float setpoint, float actual) {

	//Calculate PID
	error = setpoint - actual;

	//In case error is too small then stop integration
	if (ABS(error) > epsilon)
		integral = integral + error * dt;

	derivative = (error - preError) / dt;

	preError = error;

	output = kp * error + ki * integral + kd * derivative;


	//from angular velocity to voltage
	//output = ABS(output);

	//Saturation filter
	if (output > V_MAX)
		output = V_MAX;
	else if (output < -V_MAX)
		output = -V_MAX;

	//duty cycle
	return (output *factor);
}
