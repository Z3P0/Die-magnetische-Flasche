/*
 * ThTelecommand.cpp
 *
 *  Created on: 02.01.2016
 *      Author: pinker
 */

#include "ThTelecommand.h"
#include "ThTelemetry.h"
#include "ThTelecommand.h"
#include "ThImuRead.h"
#include "ThSolar.h"
#include "../TCTM/Telecommand.h"
#include "ThMission.h"
#include "../Sensor/ADC/IR.h"

// Threads objects
ThTelemetry tm("Telemetry");
ThTelecommand tc("Telecommand");

// Definition of the H-bridges
Hbridge thKnife(1000, 1000, &HBRIDGE_A, &HBRIDGE_A_INA, &HBRIDGE_A_INB);
Hbridge flWheel(1000, 1000, &HBRIDGE_B, &HBRIDGE_B_INA, &HBRIDGE_B_INB);
Hbridge irMotor(1000, 1000, &HBRIDGE_C, &HBRIDGE_C_INA, &HBRIDGE_C_INB);

// The solar thread gets a reverence to the thermal knife bridge
ThSolar thSolar("Solar", &thKnife);

// The IMU thread gets the reference to flywheel
ThImuRead imuRead("IMURead", &flWheel);

//IR sensor: gets reference to ADC
IR irSensor(&ADC_1);
//Mission thread: gets reference to IR sensor
ThMission thMission("Mission", &irSensor);

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

void ThTelecommand::run() {

	char out[50];
	char tmp;
	int i = -1;
	bool nbr = false;
	cmd = value = 0;

	// endless loop
	while (1) {
		i = -1;
		nbr = false;
		value = 0;

		do {
			i++;
			// Suspend until there are data on the bus
			uart_stdout.suspendUntilDataReady();
			// read char
			tmp = toUpperCase(uart_stdout.getcharNoWait());
			out[i] = tmp;
			// Stop bit is either a line ending
			if (tmp == '\n') {

				// If there is a command without a number the do while loop (putting words together) is left
				// to execute the command.
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
			// Command and value has to be separated by space.
			if (tmp == ' ') {
				out[i] = '\0';
				// Starts reading the command.
				cmd = utilization(out);
				nbr = true;
				i = -1;
			}
		} while ('\n' != (char) tmp);

		// Executes finally the command with or without the value.
		exectue();
	}
}

void ThTelecommand::exectue() {

	switch (cmd) {

	// Group: Controller
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

	//Group: Attitude Sensors
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

	//Group:  Other
	case (HELP):
		for (int i = 0; i < (sizeof(telecmds) / sizeof(Telecommand)); i++)
			PRINTF(telecmds[i].toString());
		break;

	case (RESE):
		PRINTF("TC: TODO find a method to reset the S/C\r\n");
		break;

	case (SEND):
		if (imuRead.toggleSend())
			PRINTF("TC: SEND OFF%c\r\n");
		else
			PRINTF("TC: SEND ON%c\r\n");
		break;

	case (FWTO):
		PRINTF("TC: Motor %d\r\n", value);
		//flWheel.setDuty(value);
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
		thSolar.resume();
		break;

	case (FSUN):
		PRINTF("TC: FSUN TODO find sun mode");
		break;

	case (PICT):
		PRINTF("TC: TODO picture mode");
		break;

	case (PONT):
		PRINTF("TC: TODO point mode");
		break;

	case (TRAC):
		PRINTF("TC: TODO point mode");
		break;

	case (MISSION):
		thMission.toggleMission();
		break;

	case (IRSMP):
		irSensor.sample(value);
		break;

	default:
		PRINTF("No Execution %d\r\n", cmd);
		break;
	}
}

// Decides which telecommand fits to the send one
int ThTelecommand::utilization(char *tc) {
	for (int i = 0; i < (sizeof(telecmds) / sizeof(Telecommand)); i++) {
		if (strcmp(telecmds[i].getCmdStr(), tc) == 0)
			return i;
	}
	return -1;
}

// Reads the char value and changes it to an int
// TODO improvements to read float values!
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

// Small letters ASCI 97 to 122 are set to upper case.
char ThTelecommand::toUpperCase(int c) {
	if ((c > 96) && (c < 123))
		return (char) (c - 32);

	return (char) c;
}
