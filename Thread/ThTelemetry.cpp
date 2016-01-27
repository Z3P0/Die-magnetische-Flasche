/*
 * ThTelemetry.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "ThTelemetry.h"
#include "../Extern/Extern.h"
#include "../Sensor/I2C/Current/Current.h"

ThTelemetry::ThTelemetry(const char* name, AHRS *ahrs, SolarPannel *solPan, Light *lightSensor) {
	this->ahrs = ahrs;
	this->solPan = solPan;
	this->lightSensor = lightSensor;
}

ThTelemetry::~ThTelemetry() {
}

void ThTelemetry::init() {
	//bluetooth_uart.init(115200);
	BlueLED.init(true, 1, 1);
}

void ThTelemetry::run() {

	// Current
	//Current batteries(ADDR_BATT);
	char printftOut[80];

	suspendCallerUntil(NOW() + 8*SECONDS);

	TIME_LOOP(NOW(), MILLISECONDS*1000)
	{
		BlueLED.setPins(~BlueLED.readPins());
#ifdef PROTOCOL_BINARY

		char buff[22];

		//Telemetry Header
		buff[0] = 0xAA;
		buff[1] = 0xAA;
		buff[2] = 0xAA;
		buff[3] = 0x01;

		// Roll
		SerializationUtil::WriteInt((int16_t) (ahrs->rFus), buff, 4);

		// Pitch
		SerializationUtil::WriteInt((int16_t) (ahrs->pFus), buff, 6);

		// Yaw
		SerializationUtil::WriteInt((int16_t) (ahrs->yFus), buff, 8);

		// Batteries voltage
		//SerializationUtil::WriteInt((int16_t) (batteries.getVoltage() * 1000), buff, 10);

		// Batteries current
		//SerializationUtil::WriteInt((int16_t) (batteries.getCurrent() * 1000), buff, 12);

		// Panal voltage
		SerializationUtil::WriteInt((int16_t) (solPan->getVoltage() * 1000), buff, 14);

		// Panal current
		SerializationUtil::WriteInt((int16_t) (solPan->getVoltage() * 1000), buff, 16);

		// Light sensor value
		SerializationUtil::WriteInt((int16_t) (lightSensor->getLuxValue() * 1000), buff, 18);

		//operation mode
		buff[20] = 0x00;
		buff[21] = 0x01;

		BT_Semaphore.enter();
		writeToBT(buff, 22);
		while (!uart_stdout.isWriteFinished())
			;
		BT_Semaphore.leave();
#else

		// Filtered angels Roll, Pitch, Yaw
		sprintf(printftOut, "r: %.1f deg p: %.1fdeg y: %.1fdeg\r\n", ahrs->rFus, ahrs->pFus ,ahrs->yFus);
		PRINTF(printftOut);

		// Batteries voltage & current
		//sprintf(printftOut, "Batteries %.1f[V] %.1f[A]\r\n", batteries.getVoltage(), batteries.getCurrent());
		//PRINTF(printftOut);

		//Panal voltage & current
		sprintf(printftOut, "Solar pannels %.1f[V]  %.1f[V]\r\n", solPan->getCurrent(), solPan->getVoltage());
		PRINTF(printftOut);

		//Light sensor value
		sprintf(printftOut, "Light Sensor %.1f[Lux]\r\n", lightSensor->getLuxValue());
		PRINTF(printftOut);

		//operation mode
#endif

	}

}

