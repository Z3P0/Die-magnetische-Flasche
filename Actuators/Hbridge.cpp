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

	// Enable the H-bridge three times! (Not sure it will cause a problem, but till now it seems ok.)
	HBRIDGE_EN.init(true, 1, 1);

	/* Enable the outputs form the H-bridge. This includes the initial direction. (clockwise)
	* The polarity of the wiring has to be the same all the time
	* -> A == Red wire
	* -> B == Black wire
	*/
	inA->init(true, 1, 0);
	inB->init(true, 1, 1);

	this->clockwise = true;

	socket->write(0);
}

void Hbridge::setDuty(int duty) {

	/* Changes the polarity of the pins in case:
	 *  (1):
	 *  value < 0 and the spinning direction is clockwise
	 * 	(2)
	 * 	value > 0 and the spinning direction is not clockwise
	 */
	if (((duty < 0) && (clockwise)) || ((duty > 0) && (!clockwise))) {
		// Toggle direction
		clockwise = !clockwise;

		// Toggle the direction in RODOS
		inA->setPins(~inA->readPins());
		inB->setPins(~inB->readPins());
	}

	// Sets the value positive
	duty = ABS(duty);

	// Per definition the duty can not be higher hand the increments so there is a saturation to this value
	duty = (duty > increments) ? increments : duty;

	socket->write(duty);
}
