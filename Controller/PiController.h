/*
 * PiController.h
 *
 *  Created on: 06.01.2016
 *      Author: pinker
 */

#ifndef CONTROLLER_PICONTROLLER_H_
#define CONTROLLER_PICONTROLLER_H_
#include "../Define/Define.h"
#include "../Extern/Include.h"
#include "../Extern/Extern.h"

// Controller values
#define KP 			0.07 		// Proportional gain
#define KI 			0.01		// Integral gain
#define KD 			0.01		// Derivative gain

#define PWM_RES		1000     	// PWM resolution
#define V_MAX 		9      	 	// [Volts] V max output of PWM
#define V_MAX_HBrdg	7       	// [Volts] V max that should be applied to the H-bridge.
#define EPSILON 	0.5    		// [Degree] Steady state error margin

class PiController {

public:
	PiController();
	virtual ~PiController();

	/*
	 * These functions implements the PI or PID control required to control the
	 * velocity of the satellite it outputs the duty cycle to be applied
	 * to the PWM parameters:
	 * 1) setpoint = desired speed
	 * 2) actual = speed from gyroscope
	 * The unit of setpoint, actual and EPSILON has to be the same [rad/s] or [deg/s] or [RPM].
	 */
	int32_t velocityPI(float setpoint, float actual);
	int32_t velocityPID(float setpoint, float actual);

	/* Sets the proportional part of the controller*/
	void setProportional(float kp){
		this->kp = kp;
	}

	/* Sets the integral part of the controller*/
	void setIntegral(float ki){
		this->ki = ki;
	}

	/* Sets the derivative part of the controller*/
	void setDerivative(float kp){
		this->kp = kp;
	}

	/* Sets the error margin part of the controller*/
	void setEpsion(float epsilon){
		this->epsilon = epsilon;
	}

	float kp;
	float ki;
	float kd;

	float dt;
	float epsilon;

	float error;
	float integral;
	float derivative;
	float preError;

	float factor;
};

#endif /* CONTROLLER_PICONTROLLER_H_ */
