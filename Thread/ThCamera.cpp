/*
 * ThCamera.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: Avioniklabor
 */

#include "ThCamera.h"
#include "../Sensor/OV7670/camera.h"
#include "../TCTM/SerializationUtil.h"

Camera cam;
ThCamera::ThCamera() {
	// TODO Auto-generated constructor stub

}

ThCamera::~ThCamera() {
	// TODO Auto-generated destructor stub
}

void ThCamera::run()
{
	cam.init();
	while(1)
	{
		//Suspend until request received
		suspendCallerUntil();

		cam.takePicture();

#ifdef PROTOCOL_BINARY
		//Lock bluetooth
		BT_Semaphore.enter();

		//Header for Image packet
		PRINTF("%c%c%c%c", 0xAA, 0xAA, 0xAA, 0x02);
		char len[2];

		//Height of image
		SerializationUtil::WriteInt((int16_t)HEIGHT,len,0);
		PRINTF("%c%c", len[0], len[1]);

		//Width of image
		SerializationUtil::WriteInt((int16_t)WIDTH,len,0);
		PRINTF("%c%c", len[0], len[1]);
#else
	PRINTF("I");
#endif
		//Sending image
		for(int j=0; j<HEIGHT*2;j++){
			for(int i=0; i<WIDTH; i++)
				PRINTF("%c",DCMI_Buffer[i+j*WIDTH]);
			suspendCallerUntil(NOW()+5*MILLISECONDS);
		}

		//Release bluetooth
		BT_Semaphore.leave();
	}
}

void ThCamera::captureAndSend()
{
	resume();
}

