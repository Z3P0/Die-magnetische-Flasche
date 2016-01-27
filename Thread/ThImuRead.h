/*
 * ThImuRead.h
 *
 *  Created on: 24.12.2015
 *      Author: pinker
 */

#ifndef SENSOR_THIMUREAD_H_
#define SENSOR_THIMUREAD_H_
#include "../Extern/Include.h"
#include "../Controller/PiController.h"
#include "../Actuators/Hbridge.h"

#define SAMPLES_SOFTCAL 1500

class ThImuRead: public Thread {
public:
	ThImuRead(const char* name, Hbridge *flWheel);
	virtual ~ThImuRead();
	void init();
	void run();
	void gyrCalibrate();
	void setAlpha(int alpha);

	// Setter for the private values
	void setFlag() {
		flag = true;
	}

	void setMagCalFlag() {
		magCalFlag = true;
	}

	void setGyrCalFlag() {
		gyrCalFlag = true;
	}

	void setAccCalFlag() {
		accCalFlag = true;
	}

	void setAlphaFlag() {
		changeAlphaFlag = true;
	}

	void setSetPointFlag(){
		setPointFlag = true;
	}

	void setKdFlag(){
		kdFlag = true;
	}

	void setKpFlag(){
		kpFlag = true;
	}

	void setKiFlag(){
		kiFlag = true;
	}

	void setEpsilonFlag(){
		epsilonFlag = true;
	}

	void setValue(int value) {
		this->value = value;
	}

	void toggleSpeedCtrl() {
		this->motorCtrl = !motorCtrl;
	}

	bool toggleSend() {
		return (send = !send);
	}

private:
	bool send;

	bool flag;	 // Flag to leave the main loop

	// Flags for sensor read
	bool lightPrint;
	bool accPrint;
	bool gyrPrint;
	bool magPrint;
	bool filPrint;
	bool currePrint;
	bool solarPrint;
	bool irPrint;
	bool motorPrint;

	// Flags for calibration
	bool magCalFlag;
	bool gyrCalFlag;
	bool accCalFlag;
	bool softIrFlag;

	// AHRS change value
	bool changeAlphaFlag;

	// Controller change values
	bool setPointFlag;
	bool kpFlag;
	bool kiFlag;
	bool kdFlag;
	bool epsilonFlag;

	bool motorCtrl;

	int value;
	float setPoint;

	// Reference to the motor
	Hbridge *flWheel;
};

#endif /* SENSOR_THIMUREAD_H_ */
