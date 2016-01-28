/*
 * ThSolar.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThSolar.h"

ThSolar::ThSolar(const char* name, Hbridge *bridge, ThImuRead* controlLoop, AHRS* ahrs, Light* lightSensor) {
	this->bridge = bridge;
	this->flyWheel = flyWheel;
	this->ahrs = ahrs;
	this->lightSensor = lightSensor;
	this->controlLoop = controlLoop;
}

ThSolar::~ThSolar() {
}

void ThSolar::init() {
}


void ThSolar::run() {
	// This is a suspended thread that gets resumed for the
	// deployment time and than it sleeps again.
	suspendCallerUntil(END_OF_TIME);

	float startpos = ahrs->yFinal;
	float degrees = 0;

	// Stores the values for the position the the max light
	float maxLux = 0;
	float mxPos = 0;

	float dx = 100;

	PRINTF("Starting controller \r\n");

	controlLoop->setValue(60);
	controlLoop->setSetPointFlag();
	controlLoop->setFlag();
	controlLoop->setControlType(1);


	while(dx>0 || degrees < 340)//loop
	{
		suspendCallerUntil(NOW()+20*MILLISECONDS);
		float currLux = lightSensor->getLuxValue();
		float currpos = ahrs->yFinal;
		if(currLux>maxLux)
		{
			maxLux = currLux;
			mxPos = ahrs->yFinal;
		}

		float rotation = currpos - startpos;
		if(rotation < -10)
			rotation += 360;
		dx = degrees;
		degrees = rotation>degrees ? rotation:degrees;
		dx =degrees - dx;
		PRINTF("FY: %f   DX: %f  Degrees: %f  StartPos: %f rotation: %f \r\n", currpos, dx, degrees, startpos, rotation);
	}


	controlLoop->setControlType(0);
	controlLoop->setValue(mxPos);
	controlLoop->setSetPointFlag();
	controlLoop->setFlag();
	controlLoop->setControlType(2);

	while(mxPos - ahrs->yFinal > 3)
	{
		suspendCallerUntil(NOW()+250*MILLISECONDS);
	}
#ifndef PROTOCOL_BINARY
	PRINTF("Solar: start deployment\r\n");
	PRINTF("HBridge function is commend out\r\n");
#endif
	suspendCallerUntil(NOW()+2500*MILLISECONDS);
	bridge->setDuty(800);

	//counter for the deployment time
	int cnt = 0;
	while (cnt++ < DEPLOY_TIME) {
		suspendCallerUntil(NOW()+ 1*SECONDS);
		PRINTF("Solar %d\r\n", (DEPLOY_TIME - cnt));
	}
	PRINTF("Solar array deployed.\r\n");
	bridge->setDuty(0);

	controlLoop->setControlType(0); //Stop controller
	suspendCallerUntil(END_OF_TIME);

	//its just possible to burn the wire once
	while (1) {
#ifndef PROTOCOL_BINARY
		PRINTF("The solar panel is already deployed\r\n");
#endif
		suspendCallerUntil(END_OF_TIME);
	}


}

void ThSolar::start() {
	resume();
}



