/*
 * SerializationUtil.cpp
 *
 *  Created on: Dec 9, 2015
 *      Author: Usama Tariq
 */

#include "SerializationUtil.h"

SerializationUtil::SerializationUtil() {}

void SerializationUtil::WriteInt(int32_t data, char* buffer, int index = 0) {
	for (int i = 3; i >= 0; i--)
		buffer[index++] = (data >> (i * 8)) & 0xFF;
}

void SerializationUtil::WriteInt(int16_t data, char* buffer, int index = 0) {
	for (int i = 1; i >= 0; i--)
		buffer[index++] = (data >> (i * 8)) & 0xFF;
}

int32_t SerializationUtil::ReadInt32(char* buffer, int index) {
	int32_t number = 0;
	for (int i = 3; i >= 0; i--)
		number |= ((int32_t) buffer[index++] & 0xFF) << (i * 8);
	return number;
}

int16_t SerializationUtil::ReadInt16(char* buffer, int index) {
	int16_t number = 0;
	for (int i = 1; i >= 0; i--) {
		number |= ((int16_t) buffer[index++] & 0xFF) << (i * 8);
	}
	return number;
}

