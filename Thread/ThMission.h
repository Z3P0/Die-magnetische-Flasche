/*
 * ThMission.h
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#ifndef THREAD_THMISSION_H_
#define THREAD_THMISSION_H_
#include "../Extern/Include.h"

class ThMission: public Thread{
public:
	ThMission(const char* name);
	virtual ~ThMission();
	void init();
	void run();
};

#endif /* THREAD_THMISSION_H_ */
