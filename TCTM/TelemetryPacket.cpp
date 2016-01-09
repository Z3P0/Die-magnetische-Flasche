/*
 * TelemetryPacket.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: Usama Tariq
 */

#include "TelemetryPacket.h"
#include "../Extern/Extern.h"

TelemetryPacket::TelemetryPacket() {}

TelemetryPacket::~TelemetryPacket() {}

void TelemetryPacket::writeToBt()
{
	char buffer[2];

	//Telemetry packet
	buffer[0]=0x00;
	buffer[1]=0x01;
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(roll,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(pitch,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(yaw,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(battery_v,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(battery_i,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(panel_v,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(panel_i,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(light_sensor_value,buffer,0);
	bluetooth_uart.write(buffer,2);

	SerializationUtil::WriteInt(mode,buffer,0);
	bluetooth_uart.write(buffer,2);
}

