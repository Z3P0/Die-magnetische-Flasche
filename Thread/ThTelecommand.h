/*
 * ThTelecommand.h
 *
 *  Created on: 02.01.2016
 *      Author: pinker
 */

#ifndef THREAD_THTELECOMMAND_H_
#define THREAD_THTELECOMMAND_H_
#include "../Extern/Include.h"
#include "../Actuators/Hbridge.h"
#include "../TCTM/Telecommand.h"

namespace RODOS {
extern HAL_UART uart_stdout;
}

Telecommand telecmds[] = {     // Controller
		{ "CTEP", "00000000", 0, 360, "Controller epsilon value" },
		{ "CTKD", "00000000", 0, 1000, "Controller set kd value" },
		{ "CTKI", "00000000", 0, 1000, "Controller set ki value" },
		{ "CTKP", "00000000", 0, 1000, "Controller set kp value" },
		{ "CTSP", "00000000", 0, 360, "Controller set setpoint" },
		{ "CTTO", "00000000", 0, 360, "Stops/ starts the controller motor" },
		// Attitude Sensors
		{ "ACCC", "00000000", "Accelerometer calibration" },
		{ "GYRC", "00000000", "Gyroscope calibration" },
		{ "MAGC", "00000000", "Magnetometer calibration" },
		// Other
		{ "HELP", "00000000", "Prints out all Telecommands" },
		{ "RESE", "00000000", "Reset the hole s/c" },
		{ "SEND", "00000000", "Stops/ starts the continuous sending of the data" },

		// Basic function
		{ "FWTO", "00000000", -1000, -1000, "Sets the flywheel motor to a value" },
		{ "IRDO", "00000000", "Infrared sensor move down" },
		{ "IRST", "00000000", "Infrared sensor move stop" },
		{ "IRUP", "00000000", "Infrared sensor move up" },
		// Modes
		{ "KNIF", "00000000", "Sets the thermal knife motor to a value" },
		{ "DEPL", "00000000", "Start the deployment of the solar panels by hand" },
		{ "FSUN", "00000000", "Find the sun and point to it and deploy the solar panels" },
		{ "PICT", "00000000", 0, 360, "Take picture at a setpoint" },
		{ "PONT", "00000000", 0, 360, "Point the s/c to a degree value." },
		{ "TRAC", "00000000", 0, 360, "Turn the s/c with a constant velocity" } };

enum tc {
	//Controller
	CTEP = 0,
	CTKD,
	CTKI,
	CTKP,
	CTSP,
	CTTO,
	// Attitude Sensors
	ACCC,
	GYRC,
	MAGC,
	// Other
	HELP,
	RESE,
	SEND,

	// Basic function
	FWTO,
	IRDO,
	IRST,
	IRUP,
	// Modes
	KNIF,
	DEPL,
	FSUN,
	PICT,
	PONT,
	TRAC
};

class ThTelecommand: public Thread {
public:
	ThTelecommand(const char* name);
	virtual ~ThTelecommand();
	void init();
	void run();
	char toUpperCase(int c);
	int utilization(char *tc);
	int getNumber(char *nbr);
	void exectue();
	void read();
	int cmd;
	int value;
};

#endif /* THREAD_THTELECOMMAND_H_ */
