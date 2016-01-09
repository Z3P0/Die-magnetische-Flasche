/*
 * ThTelemetry.h
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#ifndef THREAD_THTELEMETRY_H_
#define THREAD_THTELEMETRY_H_
#include "../Extern/Include.h"

class ThTelemetry: public Thread{
public:
	ThTelemetry(const char* name);
	virtual ~ThTelemetry();
	void init();
	void run();
};

#endif /* THREAD_THTELEMETRY_H_ */
