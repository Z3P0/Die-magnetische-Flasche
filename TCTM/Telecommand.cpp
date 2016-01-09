/*
 * Telecommand.cpp
 *
 *  Created on: 07.01.2016
 *      Author: pinker
 */

#include "Telecommand.h"
#include <stdio.h>

Telecommand::Telecommand(char* cmdStr, char* cmdBin, char* description) {
	this->cmdStr = cmdStr;
	this->cmdBin = cmdBin;
	this->description = description;

	this->min = 0;
	this->max = 0;

	this->hasValue = false;
}

Telecommand::Telecommand(char* cmdStr, char* cmdBin,int min, int max,  char* description) {
	this->cmdStr = cmdStr;
	this->cmdBin = cmdBin;
	this->description = description;

	this->min = min;
	this->max = max;

	this->hasValue = true;
}

Telecommand::~Telecommand() {
}

/* Getter*/
char* Telecommand::getCmdStr(){
	return cmdStr;
}

char* Telecommand::getBinStr(){
	return cmdStr;
}

bool Telecommand::isInRange(int value){
	if((value > max)||(value < min))
		return false;

	return true;
}

char* Telecommand::toString(){
	char str[80];
	sprintf(str, "%s - %s\r\n", this->cmdStr, this->description);
	return str;
}
