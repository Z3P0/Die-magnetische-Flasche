/*
 * ThCamera.h
 *
 *  Created on: Jan 26, 2016
 *      Author: Avioniklabor
 */

#ifndef THREAD_THCAMERA_H_
#define THREAD_THCAMERA_H_

#include "../Extern/Include.h"

class ThCamera : public Thread{
public:
	ThCamera();
	virtual ~ThCamera();
	void run();
	void captureAndSend();
};

#endif /* THREAD_THCAMERA_H_ */
