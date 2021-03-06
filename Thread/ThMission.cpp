/*
 * ThMission.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThMission.h"
#include "../TCTM/SerializationUtil.h"

ThMission::ThMission(const char* name, IR* ir1, IR* ir2, Hbridge* irmtr, AHRS *ahrs, ThImuRead *controlLoop) {
	irSensor1 = ir1;
	irSensor2 = ir2;
	missionMode = false;
	irMotor = irmtr;
	this->ahrs = ahrs;
	this->controlLoop = controlLoop;
}

ThMission::~ThMission() {
}

void ThMission::init(){

}
void ThMission::run(){
	suspendCallerUntil(NOW() + 5*SECONDS);//Wait until hardware initialized
	static bool sensorOnTop = false;
	irSensor1->init();
	irSensor2->init();

	while(1)
	{
		suspendCallerUntil();
		controlLoop->setValue(80);
		controlLoop->setSetPointFlag();
		controlLoop->setFlag();
		controlLoop->setControlType(1);
		suspendCallerUntil(NOW() + 5*SECONDS);

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
			float ir1 = irSensor1->read();
			float ir2 = irSensor2->read();
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
			SerializationUtil::WriteInt((int16_t)ahrs->rFus,buff,6);

			//pitch
			SerializationUtil::WriteInt((int16_t)ahrs->pFus,buff,8);

			//yaw
			SerializationUtil::WriteInt((int16_t)ahrs->yFus,buff,10);

			//IR-1 value
			SerializationUtil::WriteInt((int16_t)ir1,buff,12);

			//IR-2 value
			SerializationUtil::WriteInt((int16_t)ir2,buff,14);

			//Lock bluetooth
			BT_Semaphore.enter();
			writeToBT(buff, 16);
			//Unlock bluetooth
			BT_Semaphore.leave();
#else
			PRINTF("Distance (cm): %f\r\n", ir1);
			PRINTF("Height (time ratio): %f\r\n", h);
#endif

			//Measurement every 50ms
			suspendCallerUntil(NOW() + 20*MILLISECONDS);
			if(NOW() - mission_start > SECONDS * 60)
				break;
		}
		irMotor->stop();
		sensorOnTop = !sensorOnTop;
		controlLoop->setControlType(0);

		//PRINTF("MISSION COMPLETED\r\n");
	}
}

void ThMission::toggleMission()
{
	//PRINTF("Starting MISSION\r\n");
	resume();
}
