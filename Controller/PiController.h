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

//// Controller values for PI
#define KP_PI 		0.1		// Proportional gain
#define KI_PI 		0.018		// Integral gain

// Controller values for PID
#define KP_PID 		0.1			// Proportional gain  0.08
#define KI_PID 		0.03 		// Integral gain  0.03
#define KD_PID 		0.05		// Derivative gain 0.0

#define PWM_RES		1000     	// PWM resolution
#define V_MAX 		5      	 	// [Volts] V max output of PWM
#define V_MIN		-5			// [Volts] V max output of PWM

#define V_MAX_HBrdg	9     		// [Volts] V max that should be applied to the H-bridge.
#define EPSILON_I	0.3


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
	 * The unit of setpoint, actual and EPSILON1 has to be the same [rad/s] or [deg/s] or [RPM].
	 */
	int32_t pi(float setpoint, float actual);
	int32_t pid(float setpoint, float actual);
	/* Sets the proportional part of the controller*/
	void setProportional(float kp) {
		this->kp_pid = kp;
	}

	/* Sets the integral part of the controller*/
	void setIntegral(float ki) {
		this->ki_pid = ki;
	}

	/* Sets the derivative part of the controller*/
	void setDerivative(float kd) {
		this->kd_pid = kd;
	}

	/* Sets the error margin part of the controller*/
	void setEpsion(float epsilon) {
		this->epsilonI = epsilon;
	}

	// PI - Velocity control
	float kp_pi;
	float ki_pi;

	//PID - Position control
	float kp_pid;
	float ki_pid;
	float kd_pid;

	float dt;
	float epsilonI;

	float error;
	float integral;
	float derivative;
	float preError;
	float output;
	uint32_t duty;
	float factor;
};

#endif /* CONTROLLER_PICONTROLLER_H_ */
