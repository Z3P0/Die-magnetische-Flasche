/*
 * ThMission.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThMission.h"
#include "../Sensor/Camera/Camera.h"
#include "../TCTM/SerializationUtil.h"

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


		while(true)
		{


			float h = (NOW() - mission_start) / (MILLISECONDS * 60);
			if(sensorOnTop)
				h = 1000 - h;
			float ir = irSensor->read();
#ifdef PROTOCOL_BINARY
			//Lock bluetooth
			BT_Semaphore.enter();

			//Mission packet header
			PRINTF("%c%c%c%c",0xAA,0xAA,0xAA,0x03);
			//int32_t x = *(int32_t*)(&h);
			char buff[2];

			//Timestamp for height
			SerializationUtil::WriteInt((int16_t)h,buff,0);
			PRINTF(buff);

			//roll
			SerializationUtil::WriteInt((int16_t)h,buff,0);
			PRINTF(buff);

			//pitch
			SerializationUtil::WriteInt((int16_t)h,buff,0);
			PRINTF(buff);

			//yaw
			SerializationUtil::WriteInt((int16_t)h,buff,0);
			PRINTF(buff);

			//IR
			SerializationUtil::WriteInt((int16_t)ir,buff,0);
			PRINTF(buff);

			//Unlock bluetooth
			BT_Semaphore.leave();
#else
			PRINTF("Distance (cm): %f\r\n", ir);
			PRINTF("Height (time ratio): %f\r\n", h);
#endif

			//Measurement every 50ms
			suspendCallerUntil(NOW() + 2000*MILLISECONDS);
			if(NOW() - mission_start > SECONDS * 60)
				break;
		}
		irMotor->stop();
		sensorOnTop = !sensorOnTop;
		//PRINTF("MISSION COMPLETED\r\n");
	}
}

void ThMission::toggleMission()
{
	//PRINTF("Starting MISSION\r\n");
	resume();
}
