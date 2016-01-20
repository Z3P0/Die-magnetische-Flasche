/*
 * ThMission.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThMission.h"
//#include "../Sensor/Camera/Camera.h"
//Camera cam;
ThMission::ThMission(const char* name, IR* ir) {
	irSensor = ir;
	missionMode = false;
}

ThMission::~ThMission() {
}

void ThMission::init(){

}
void ThMission::run(){
	irSensor->init();
	//cam.init();
	while(1)
	{
		while(missionMode)
		{

			//float v = ((float)ir)/839;
			//float dist = 2.435 *v*v + 29.75 - 14.98*v;
			//irSensor->read();
			PRINTF("Distance (cm): %f\r\n", irSensor->read());
			suspendCallerUntil(NOW() + 1*SECONDS);
		}
		suspendCallerUntil(NOW() + 2*SECONDS);
	}

}

void ThMission::toggleMission()
{
	missionMode = !missionMode;
}
