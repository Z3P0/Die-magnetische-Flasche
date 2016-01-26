/*

I * ThTelecommand.cpp
 *
 *  Created on: 02.01.2016
 *      Author: pinker
 */

#include "ThTelecommand.h"
#include "ThTelemetry.h"
#include "ThTelecommand.h"
#include "ThMission.h"
#include "ThImuRead.h"
#include "ThSolar.h"
#include "ThCamera.h"
#include "../TCTM/Telecommand.h"
#include "../Sensor/ADC/IR.h"
#include "../TCTM/SerializationUtil.h"
#include <stdio.h>
#include "../Sensor/OV7670/camera.h"
#include "../Define/Define.h"

// Threads objects
ThTelemetry tm("Telemetry");
ThTelecommand tc("Telecommand");
/* Definition of the H-bridges*/
Hbridge thKnife(1000, 1000, &HBRIDGE_A, &HBRIDGE_A_INA, &HBRIDGE_A_INB);
Hbridge flWheel(1000, 1000, &HBRIDGE_B, &HBRIDGE_B_INA, &HBRIDGE_B_INB);
Hbridge irMotor(1000, 1000, &HBRIDGE_C, &HBRIDGE_C_INA, &HBRIDGE_C_INB);

/* The solar thread gets a reverence to the thermal knife bridge*/
//ThSolar thSolar("Solar", &thKnife);
/* The IMU thread gets the reference to flywheel*/
ThImuRead imuRead("IMURead", &flWheel);

//IR sensor: gets reference to ADC
//IR irSensor(&ADC_1);
//Mission thread: gets reference to IR sensor
//ThMission thMission("Mission", &irSensor, &irMotor);
ThCamera thCamera;

ThTelecommand::ThTelecommand(const char* name) {
	cmd = value = 0;
}

ThTelecommand::~ThTelecommand() {
}

void ThTelecommand::init() {
	thKnife.init();
	flWheel.init();
	irMotor.init();
}

//Run function for binary protocol
void ThTelecommand::run_binary() {
	char out[6];
	char tmp;
	int i = -1;
	bool nbr = false;
	cmd = value = 0;

	while (1) {
		i = -1;
		nbr = false;
		for (i = 0; i < 6; i++) {
			uart_stdout.suspendUntilDataReady();
			out[i] = uart_stdout.getcharNoWait();
		}
		cmd = SerializationUtil::ReadInt32(out, 2);
		//Commands with extra parameter

// TODO What is that for!?
		switch (cmd) {
		case TRAC:
		case PONT:
		case CTEP:
		case CTKD:
		case CTKI:
		case CTKP:
		case CTSP:
		case FWTO:
			for (i = 0; i < 2; i++) {
				uart_stdout.suspendUntilDataReady();
				out[i] = uart_stdout.getcharNoWait();
			}
			value = SerializationUtil::ReadInt16(out, 0);
			break;
		}
		exectue();
	}
}

//Run function for text based protocol (terminal)
void ThTelecommand::run_text() {
	char out[50];
	char tmp;
	int i = -1;
	bool nbr = false;
	cmd = value = 0;

	while (1) {
		i = -1;
		nbr = false;
		do {
			i++;
			uart_stdout.suspendUntilDataReady();
			tmp = toUpperCase(uart_stdout.getcharNoWait());
			out[i] = tmp;
			if (tmp == '\n' || tmp == '\r') {
				if (!nbr) {
					out[i] = '\0';
					cmd = utilization(out);
					break;
				} else {
					out[i] = '\0';
					value = getNumber(out);
					break;
				}
			}
			if (tmp == ' ') {
				out[i] = '\0';
				cmd = utilization(out);
				nbr = true;
				i = -1;
			}
		} while ('\n' != (char) tmp);
		exectue();
	}
}
void delayx(int x)
{
	while(x-->0)
	{asm("nop");}
}

void ThTelecommand::run()
{
	I2C_1.init();

#ifdef PROTOCOL_BINARY
		run_binary();
#else
		run_text();
#endif

}

//Execute telecommand
void ThTelecommand::exectue() {

	switch (cmd) {

	//Controller
	case (CTEP):
		//No need for a extra print command!
		imuRead.setValue(value);
		imuRead.setFlag();
		imuRead.setEpsilonFlag();
		break;

	case (CTKD):
		//No need for a extra print command!
		imuRead.setValue(value);
		imuRead.setFlag();
		imuRead.setKdFlag();
		break;

	case (CTKI):
		//No need for a extra print command!
		imuRead.setValue(value);
		imuRead.setFlag();
		imuRead.setKiFlag();
		break;

	case (CTKP):
		//No need for a extra print command!
		imuRead.setValue(value);
		imuRead.setFlag();
		imuRead.setKpFlag();
		break;

	case (CTSP):
		//No need for a extra print command!
		imuRead.setValue(value);
		imuRead.setFlag();
		imuRead.setSetPointFlag();
		break;

	case (CTTO):
		//No need for a extra print command!
		imuRead.toggleSpeedCtrl();
		break;

		// Attitude Sensors
	case (ACCC):
		//No need for a extra print command!
		imuRead.setFlag();
		imuRead.setAccCalFlag();
		break;

	case (GYRC):
		//No need for a extra print command!
		imuRead.setFlag();
		imuRead.setGyrCalFlag();
		break;

	case (MAGC):
		//No need for a extra print command!
		imuRead.setFlag();
		imuRead.setMagCalFlag();
		break;

		// Other
	case (HELP):
		char out[80];
		for (int i = 0; i < (sizeof(telecmds) / sizeof(Telecommand)); i++) {
			sprintf(out, "%s - %s\r\n", telecmds[i].cmdStr, telecmds[i].description);
			PRINTF(out);
		}
		break;

	case (RESE):
		PRINTF("TC: TODO find a method to reset the S/C\r\n");
		hwResetAndReboot();
		break;

	case (SEND):
		if (imuRead.toggleSend())
			PRINTF("TC: SEND OFF%c\r\n");
		else
			PRINTF("TC: SEND ON%c\r\n");
		break;

	case (FWTO):
		PRINTF("TC: Motor %d\r\n", value);
		flWheel.setDuty(value);
		break;

	case (IRDO):
		PRINTF("TC : IR DOWN\r\n");
		irMotor.moveDown();
		break;

	case (IRST):
		PRINTF("TC : IR STOP\r\n");
		irMotor.stop();
		break;

	case (IRUP):
		PRINTF("TC : IR UP\r\n");
		irMotor.moveUp();
		break;

	case (KNIF):
		PRINTF("TC : KNIFE %d\r\n");
		thKnife.setDuty(value);
		break;

	case (DEPL):
		//No need for a extra print command!
		//thSolar.resume();
		break;

	case (FSUN):
		PRINTF("TC: FSUN TODO find sun mode");
		break;

	case (PICT):
		//PRINTF("TC: TODO picture mode");
                thCamera.captureAndSend();
		break;

	case (PONT):
		PRINTF("TC: TODO point mode");
		break;

	case (TRAC):
		PRINTF("TC: TODO point mode");
		break;

	case (MISSION):
//		thMission.toggleMission();
		break;

	case (IRSM):
		//irSensor.sample(value);
		break;

	default:
		PRINTF("No Execution %d\r\n", cmd);
		break;
	}
}

//Decides which telecommand fits to the send one
int ThTelecommand::utilization(char *tc) {
	for (int i = 0; i < (sizeof(telecmds) / sizeof(Telecommand)); i++) {
		if (strcmp(telecmds[i].getCmdStr(), tc) == 0)
			return telecmds[i].getBinOpcode();
	}
	return -1;
}

// Reads the char value and changes it to an int
int ThTelecommand::getNumber(char *nbr) {
	bool negative = (nbr[0] == '-') ? true : false;
	int i = 0;
	int sign = 1;

	int returnNbr = 0;

	if (negative) {
		i = 1;
		sign = -1;
	}

	while (nbr[i] != '\0') {
		returnNbr = (returnNbr * 10);
		returnNbr += (int) (nbr[i] - '0');
		i++;
	}

	returnNbr = sign * returnNbr;

	return returnNbr;
}

// Small letters ASCI 97 to 122 are set to upper case
char ThTelecommand::toUpperCase(int c) {
	if ((c > 96) && (c < 123))
		return (char) (c - 32);

	return (char) c;
}
