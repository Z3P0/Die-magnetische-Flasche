/*
 * ThMission.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThMission.h"
#include "../TCTM/SerializationUtil.h"

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
			char buff[16];

			//Mission packet Header
			buff[0] = 0xAA;
			buff[1] = 0xAA;
			buff[2] = 0xAA;
			buff[3] = 0x03;

			//Timestamp for height
			SerializationUtil::WriteInt((int16_t)h,buff,4);

			//roll
			SerializationUtil::WriteInt((int16_t)10,buff,6);

			//pitch
			SerializationUtil::WriteInt((int16_t)h,buff,8);

			//yaw
			SerializationUtil::WriteInt((int16_t)h,buff,10);

			ir = h / 100;
			//IR-1 value
			SerializationUtil::WriteInt((int16_t)ir,buff,12);

			//IR-2 value
			SerializationUtil::WriteInt((int16_t)ir,buff,14);


			//Lock bluetooth
			BT_Semaphore.enter();
			writeToBT(buff, 16);
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
