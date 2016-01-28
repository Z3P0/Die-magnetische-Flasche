/*
 * ThCamera.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: Avioniklabor
 */

#include "ThCamera.h"
#include "../Sensor/OV7670/camera.h"
#include "../TCTM/SerializationUtil.h"

namespace RODOS {
extern HAL_UART uart_stdout;
}


Camera cam;
ThCamera::ThCamera() {
	// TODO Auto-generated constructor stub

}

ThCamera::~ThCamera() {
	// TODO Auto-generated destructor stub
}

void ThCamera::run()
{
	suspendCallerUntil(NOW() + 5*SECONDS);
	cam.init();
	while(1)
	{
		//Suspend until request received
		suspendCallerUntil();
		cam.takePicture();

#ifdef PROTOCOL_BINARY


		char buff[8];

		//Header for image packet
		buff[0] = 0xAA;
		buff[1] = 0xAA;
		buff[2] = 0xAA;
		buff[3] = 0x02;


		SerializationUtil::WriteInt((int16_t)HEIGHT,buff,4);
		SerializationUtil::WriteInt((int16_t)WIDTH,buff,6);

		//Lock bluetooth
		BT_Semaphore.enter();

		uart_stdout.write(buff, 8);
#else
	PRINTF("I");
#endif
		writeToBT((char *)DCMI_Buffer, IMAGESIZE);
		//Release bluetooth
		BT_Semaphore.leave();
	}
}

void ThCamera::captureAndSend()
{
	resume();
}

