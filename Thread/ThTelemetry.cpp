/*
 * ThTelemetry.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "ThTelemetry.h"
#include "../Extern/Extern.h"
#include "../TCTM/TelemetryPacket.h"
#include "../Sensor/I2C/IMU/IMU.h"
#include "../Sensor/Filter/MadgwickFilter.h"

#define LED_BLUE GPIO_063

HAL_GPIO BlueLED(LED_BLUE);

ThTelemetry::ThTelemetry(const char* name) {}

ThTelemetry::~ThTelemetry() {}

void ThTelemetry::init(){
	bluetooth_uart.init(115200);
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

	TIME_LOOP(NOW(), 1*SECONDS){
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
