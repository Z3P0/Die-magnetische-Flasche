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

Telecommand telecmds[] = {
		// Controller
		// { TEXT_CMD, BIN_OPCODE, MIN_VAL, MAX_VAL, COMMENTS }
		{ "CTEP", 0x00020001, 0, 360, "Controller epsilon value" },
		{ "CTKD", 0x00020002, 0, 1000, "Controller set kd value" },
		{ "CTKI", 0x00020003, 0, 1000, "Controller set ki value" },
		{ "CTKP", 0x00020004, 0, 1000, "Controller set kp value" },
		{ "CTSP", 0x00020005, 0, 360, "Controller set setpoint" },
		{ "CTTO", 0x00020006, 0, 360, "Stops/ starts the controller motor" },
		// Attitude Sensors
		{ "ACCC", 0x00020007, "Accelerometer calibration" },
		{ "GYRC", 0x00020008, "Gyroscope calibration" },
		{ "MAGC", 0x00020009, "Magnetometer calibration" },
		// Other
		{ "HELP", 0x0002000A, "Prints out all Telecommands" },
		{ "RESE", 0x0002000B, "Reset the hole s/c" },
		{ "SEND", 0x0002000C, "Stops/ starts the continuous sending of the data" },

		// Basic function
		{ "FWTO", 0x0002000D, -1000, -1000, "Sets the flywheel motor to a value" },
		{ "IRDO", 0x0002000E, "Infrared sensor move down" },
		{ "IRST", 0x0002000F, "Infrared sensor move stop" },
		{ "IRUP", 0x00020010, "Infrared sensor move up" },

		// Modes
		{ "KNIF", 0x00020011, "Sets the thermal knife motor to a value" },
		{ "DEPL", 0x00020012, "Start the deployment of the solar panels by hand" },
		{ "FSUN", 0x00010002, "Find the sun and point to it and deploy the solar panels" },
		{ "PICT", 0x00010004, 0, 360, "Take picture at a setpoint" },
		{ "PONT", 0x00010003, 0, 360, "Point the s/c to a degree value." },
		{ "TRAC", 0x00010001, 0, 360, "Turn the s/c with a constant velocity" },
		{ "3DSC", 0x00010005, "Start mission" },
		{ "IRSM", 0x00020014, 0, 50, "Sample IR for calibration" } };



enum tc {
	//Controller
	CTEP = 0x00020001,
	CTKD = 0x00020002,
	CTKI = 0x00020003,
	CTKP = 0x00020004,
	CTSP = 0x00020005,
	CTTO = 0x00020006,

	// Attitude Sensors
	ACCC = 0x00020007,
	GYRC = 0x00020008,
	MAGC = 0x00020009,

	// Other
	HELP = 0x0002000A,
	RESE = 0x0002000B,
	SEND = 0x0002000C,

	// Basic function
	FWTO = 0x0002000D,
	IRDO = 0x0002000E,
	IRST = 0x0002000F,
	IRUP = 0x00020010,
	// Modes
	KNIF = 0x00020011,
	DEPL = 0x00020012,
	FSUN = 0x00010002,
	PICT = 0x00010004,
	PONT = 0x00010003,
	TRAC = 0x00010001,

	MISSION = 0x00010005,
	IRSM = 0x00020014
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
	void run_binary();

	int cmd;
	int value;
};

#endif /* THREAD_THTELECOMMAND_H_ */
