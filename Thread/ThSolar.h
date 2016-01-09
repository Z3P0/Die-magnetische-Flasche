/*
 * ThSolar.h
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#ifndef THREAD_THSOLAR_H_
#define THREAD_THSOLAR_H_
#include "../Extern/Include.h"
#include "../Extern/Extern.h"
#include "../Actuators/Hbridge.h"

#define DEPLOY_TIME 		18	 //Seconds

class ThSolar: public Thread {
public:
	ThSolar(const char* name, Hbridge *bridge);
	virtual ~ThSolar();
	void init();
	void run();
private:
	Hbridge *bridge;
};

#endif /* THREAD_THSOLAR_H_ */
