/*
 * Hbridge.h
 *
 *  Created on: 28.12.2015
 *      Author: pinker
 */

#ifndef ACTUATORS_HBRIDGE_H_
#define ACTUATORS_HBRIDGE_H_
#include "../Extern/Include.h"
#include "../Define/Define.h"

//Max duty cycle that can be applied to the H-bridge.
#define MAX_DUTY_CYCLE 9.0


/* Selector which of the H-bridges should be changed.*/
enum Select {
	TH_KNIFE = 0, IR_MOTOR, FL_MOTOR
};

class Hbridge {

public:
	Hbridge(int frequency, int increments, HAL_PWM *socket, HAL_GPIO *inA, HAL_GPIO *inB);
	virtual ~Hbridge();

	/* Initializes the the ports for the h-bridges.*/
	void init();

	/*Change of the PWM value range 0 - 1000 -> 0%-100%
	 selection of the Hbridge by the enum 'Select'*/
	void setDuty(int value);

	/* Special functions for the IR Motor*/
	void moveUp(){
		setDuty(increments);
	}

	/* Special functions for the IR Motor*/
	void moveDown(){
		setDuty(-increments);
	}

	/* Special functions for the IR Motor*/
	void stop(){
		setDuty(0);
	}

	/*Change of the frequency and th increment value
	 selection of the Hbridge by the enum 'Select'*/
	void setParams(int frequency, int increments, Select s);

private:
	int frequency;
	int increments;
	HAL_PWM *socket;
	HAL_GPIO *inA;
	HAL_GPIO *inB;
	bool clockwise; //changes the spinning direction respectively the polarity of the ports

};

#endif /* ACTUATORS_HBRIDGE_H_ */
