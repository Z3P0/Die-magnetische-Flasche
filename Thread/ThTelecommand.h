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


Telecommand telecmds[] = {
		// Controller
		// { TEXT_CMD, BIN_OPCODE, MIN_VAL, MAX_VAL, COMMENTS }
		{ "CTEP", 0xAA020001, 0, 360, "Controller epsilon value" },
		{ "CTKD", 0xAA020002, 0, 1000, "Controller set kd value" },
		{ "CTKI", 0xAA020003, 0, 1000, "Controller set ki value" },
		{ "CTKP", 0xAA020004, 0, 1000, "Controller set kp value" },
		{ "CTSP", 0xAA020005, 0, 360, "Controller set setpoint" },
		{ "CTTO", 0xAA020006, 0, 360, "Stops/ starts the controller motor" },
		// Attitude Sensors
		{ "ACCC", 0xAA020007, "Accelerometer calibration" },
		{ "GYRC", 0xAA020008, "Gyroscope calibration" },
		{ "MAGC", 0xAA020009, "Magnetometer calibration" },
		// Other
		{ "HELP", 0xAA02000A, "Prints out all Telecommands" },
		{ "RESE", 0xAA02000B, "Reset the hole s/c" },
		{ "SEND", 0xAA02000C, "Stops/ starts the continuous sending of the data" },

		// Basic function
		{ "FWTO", 0xAA02000D, -1000, -1000, "Sets the flywheel motor to a value" },
		{ "IRDO", 0xAA02000E, "Infrared sensor move down" },
		{ "IRST", 0xAA02000F, "Infrared sensor move stop" },
		{ "IRUP", 0xAA020010, "Infrared sensor move up" },

		// Modes
		{ "KNIF", 0xAA020011, "Sets the thermal knife motor to a value" },
		{ "DEPL", 0xAA020012, "Start the deployment of the solar panels by hand" },
		{ "FSUN", 0xAA010002, "Find the sun and point to it and deploy the solar panels" },
		{ "PICT", 0xAA010004, 0, 360, "Take picture at a setpoint" },
		{ "PONT", 0xAA010003, 0, 360, "Point the s/c to a degree value." },
		{ "TRAC", 0xAA010001, 0, 360, "Turn the s/c with a constant velocity" },
		{ "3DSC", 0xAA010005, "Start mission" },
		{ "IRSM", 0xAA020014, 0, 50, "Sample IR for calibration" } };



enum tc {
	//Controller
	CTEP = 0xAA020001,
	CTKD = 0xAA020002,
	CTKI = 0xAA020003,
	CTKP = 0xAA020004,
	CTSP = 0xAA020005,
	CTTO = 0xAA020006,

	// Attitude Sensors
	ACCC = 0xAA020007,
	GYRC = 0xAA020008,
	MAGC = 0xAA020009,

	// Other
	HELP = 0xAA02000A,
	RESE = 0xAA02000B,
	SEND = 0xAA02000C,

	// Basic function
	FWTO = 0xAA02000D,
	IRDO = 0xAA02000E,
	IRST = 0xAA02000F,
	IRUP = 0xAA020010,
	// Modes
	KNIF = 0xAA020011,
	DEPL = 0xAA020012,
	FSUN = 0xAA010002,
	PICT = 0xAA010004,
	PONT = 0xAA010003,
	TRAC = 0xAA010001,

	MISSION = 0xAA010005,
	IRSM = 0xAA020014
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
	void run_text();

	int cmd;
	int value;
};

#endif /* THREAD_THTELECOMMAND_H_ */
