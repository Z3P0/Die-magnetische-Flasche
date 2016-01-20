/*
 * Telecommand.h
 *
 *  Created on: 07.01.2016
 *      Author: pinker
 */

#ifndef TCTM_TELECOMMAND_H_
#define TCTM_TELECOMMAND_H_

class Telecommand {

public:
	Telecommand(char* cmdStr, int cmdBin, char* description);
	Telecommand(char* cmdStr, int cmdBin, int min, int max, char* description);
	virtual ~Telecommand();
	char* getCmdStr();
	char* getBinStr();
	bool getValue(int *returnValue);
	bool isInRange(int value);
	char* toString();
	char* cmdStr;
	char* description;

private:
	int cmdBin;
	int min;
	int max;
	bool hasValue;
};


#endif /* TCTM_TELECOMMAND_H_ */
