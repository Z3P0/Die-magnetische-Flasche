/*
 * ThMission.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThMission.h"
#include "../Sensor/Camera/Camera.h"

//Camera cam;
ThMission::ThMission(const char* name, IR* ir, Hbridge* irmtr) {
	irSensor = ir;
	missionMode = false;
	irMotor = irmtr;
}

ThMission::~ThMission() {
}

void ThMission::init(){

}
void ThMission::run(){
	static bool sensorOnTop = false;
	irSensor->init();
	while(1)
	{
		suspendCallerUntil();
		//cam.init();
		int64_t mission_start = NOW();
		if(sensorOnTop)
			irMotor->moveDown();
		else
			irMotor->moveUp();

		//while(missionMode)
		while(true)
		{
			PRINTF("Distance (cm): %f\r\n", irSensor->read());

			//Measurement every 50ms
			suspendCallerUntil(NOW() + 50*MILLISECONDS);
			if(NOW() - mission_start > SECONDS * 55)
				break;
		}
		irMotor->stop();
		sensorOnTop = !sensorOnTop;
		suspendCallerUntil(NOW() + 2*SECONDS);
		PRINTF("MISSION COMPLETED\r\n");
	}
}

void ThMission::toggleMission()
{
	PRINTF("Starting MISSION\r\n");
	resume();
}
