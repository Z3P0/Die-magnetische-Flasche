/*
 * Hbridge.cpp
 *
 *  Created on: 28.12.2015
 *      Author: pinker
 */

#include "Hbridge.h"
#include "../Extern/Extern.h"
#include "../Define/Define.h"

Hbridge::Hbridge(int frequency, int increments, HAL_PWM *socket, HAL_GPIO *inA, HAL_GPIO *inB) {
	this->frequency = frequency;
	this->increments = increments;
	this->socket = socket;
	this->inA = inA;
	this->inB = inB;

	this->clockwise = true;
}

Hbridge::~Hbridge() {
}

void Hbridge::init() {
	socket->init(frequency, increments);
	HBRIDGE_EN.init(true, 1, 1);

	inA->init(true, 1, 1);
	inB->init(true, 1, 0);

	socket->write(0);
}

void Hbridge::moveUp()
{
	setDuty(-increments);
}

void Hbridge::moveDown(){
	setDuty(increments);
}

void Hbridge::stop(){
	setDuty(0);
}

void Hbridge::setDuty(int duty) {

	/*Changes the polarity of the pins in case:
	 *  (1):
	 *  value < 0 and the spinning direction is clockwise
	 * 	(2)
	 * 	value > 0 and the spinning direction is not clockwise
	 * */
	if (((duty < 0) && (clockwise)) || ((duty > 0) && (!clockwise))) {
		//toggle direction
		clockwise = !clockwise;

		inA->setPins(~inA->readPins());
		inB->setPins(~inB->readPins());
	}

	// sets the value positive
	duty = ABS(duty);
	//set value in the range form 0 to increments
	duty = (duty > increments) ? increments : duty;

	//TODO check for the max duty cycle!
	//MAX_DUTY_CYCLE < increments/value!

	socket->write(duty);
}
