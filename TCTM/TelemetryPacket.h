/*
 * TelemetryPacket.h
 *
 *  Created on: Dec 14, 2015
 *      Author: Usama Tariq
 */

#ifndef COMMUNICATION_TELEMETRY_H_
#define COMMUNICATION_TELEMETRY_H_
#include "BTUART.h"
#include "SerializationUtil.h"

class TelemetryPacket {
public:
	TelemetryPacket();
	virtual ~TelemetryPacket();

	int16_t roll = 0;
	int16_t pitch = 0;
	int16_t yaw = 0;
	int16_t battery_v = 0;
	int16_t battery_i = 0;
	int16_t panel_v = 0;
	int16_t panel_i = 0;
	int16_t light_sensor_value = 0;
	int16_t mode = 0;

	void writeToBt();
};

#endif /* COMMUNICATION_TELEMETRY_H_ */
