/*
 * ThMission.h
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#ifndef THREAD_THMISSION_H_
#define THREAD_THMISSION_H_
#include "../Extern/Include.h"
#include "../Sensor/ADC/IR.h"

class ThMission: public Thread{
public:
	ThMission(const char* name, IR* irSensor);
	virtual ~ThMission();
	void init();
	void run();
	void toggleMission();

private:
	bool missionMode;
	IR* irSensor;
};

#endif /* THREAD_THMISSION_H_ */
