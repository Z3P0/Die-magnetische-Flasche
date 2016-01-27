/*
 * ThTelemetry.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "ThTelemetry.h"
#include "../Extern/Extern.h"
#include "../Sensor/I2C/IMU/IMU.h"
#include "../Sensor/Filter/MadgwickFilter.h"


ThTelemetry::ThTelemetry(const char* name) {}

ThTelemetry::~ThTelemetry() {}

void ThTelemetry::init(){
	//bluetooth_uart.init(115200);
	BlueLED.init(true,1,1);
}

void ThTelemetry:: run(){
	/*
	IMU imu(this);
	imu.accSetDefaultValues();
	imu.gyrCalibrate();
	imu.magCalibrate();
	int cnt = 0;
	int cnt2 = 0;
	float gain = 10;
	float eulAng[3];
	MadgwickFilter madf(0.01, gain);
	madf.resetFilter();
	char sprintOut[100];
	TelemetryPacket tmp;*/
	TIME_LOOP(NOW(), MILLISECONDS*100){
#ifdef PROTOCOL_BINARY

		char buff[22];

		//Telemetry Header
		buff[0] = 0xAA;
		buff[1] = 0xAA;
		buff[2] = 0xAA;
		buff[3] = 0x01;

		//Roll
		buff[4] = 0x00;
		buff[5] = 0x02;

		//pitch
		buff[6] = 0x00;
		buff[7] = 0x04;

		//yaw
		buff[8] = 0x00;
		buff[9] = 0x06;

		//Battery volt
		buff[10] = 0x00;
		buff[11] = 0x08;

		//Battery current
		buff[12] = 0x00;
		buff[13] = 0x02;

		//Panal volt
		buff[14] = 0x00;
		buff[15] = 0x0A;

		//Panal current
		buff[16] = 0x00;
		buff[17] = 0x01;

		//Light sensor value
		buff[18] = 0x00;
		buff[19] = 0x10;

		//operation mode
		buff[20] = 0x00;
		buff[21] = 0x01;

		BT_Semaphore.enter();
		writeToBT(buff, 22);
		while(!uart_stdout.isWriteFinished());
		BT_Semaphore.leave();
#else
#endif
		/*
		imu.accRead();
		imu.gyrRead();
		imu.magRead();
		char *buffer;
		madf.filterUpdate(imu.gyr.dx, imu.gyr.dy, imu.gyr.dz, imu.acc.x, imu.acc.y, imu.acc.z, imu.mag.x, imu.mag.y, imu.mag.z);


		if ((gain > 2.5) && (cnt2++ > 200)) {
			cnt2 = 0;
			gain = (gain - 0.5);
			madf.setGain(gain);
			PRINTF("Set gain %f \r\n", gain);
		}
		if (cnt++ > 40) {
			cnt = 0;
			madf.getEulerAnglesDeg(eulAng);

			//tmp.roll = (int16_t)eulAng[0];
			//tmp.pitch = (int16_t)eulAng[1];
			//tmp.yaw = (int16_t)eulAng[2];

			//tmp.writeToBt();

			//sprintf(buffer, "F R:%d P%d Y%d\r\n", (int16_t)eulAng[0], (int16_t)eulAng[1], (int16_t)eulAng[2]);



		}*/

		BlueLED.setPins(~BlueLED.readPins());
		//bluetooth_uart.write("Alive\r\n", sizeof("Alive\r\n"));

	}
}
