/*
 * SerializationUtil.h
 *
 *  Created on: Dec 9, 2015
 *      Author: Usama Tariq
 */

#ifndef STM32F4_RODOS_TEMPLATE_COMMUNICATION_SERIALIZATIONUTIL_H_
#define STM32F4_RODOS_TEMPLATE_COMMUNICATION_SERIALIZATIONUTIL_H_
#include "../Extern/Include.h"

class SerializationUtil {
public:
	SerializationUtil();
	static void WriteInt(int32_t data, char* buffer, int index);
	static void WriteInt(int16_t data, char* buffer, int index);
	static int32_t ReadInt32(char* buffer, int index);
	static int16_t ReadInt16(char* buffer, int index);
};

#endif /* STM32F4_RODOS_TEMPLATE_COMMUNICATION_SERIALIZATIONUTIL_H_ */
