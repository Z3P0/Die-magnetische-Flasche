/*
 * ThSolar.cpp
 *
 *  Created on: 03.01.2016
 *      Author: pinker
 */

#include "ThSolar.h"

ThSolar::ThSolar(const char* name, Hbridge *bridge) {
	this->bridge = bridge;
}

ThSolar::~ThSolar() {
}

void ThSolar::init() {
}

// This is a suspended thread that gets resumed for the
// deployment time and than it sleeps again.
void ThSolar::run() {

	suspendCallerUntil(END_OF_TIME);
	PRINTF("Solar: start deployment\r\n");
	PRINTF("HBridge function is commend out\r\n");
	bridge->setDuty(1000);

	// Counter for the deployment time
	int cnt = 0;
	while (cnt++ < DEPLOY_TIME) {
		suspendCallerUntil(NOW()+ 1*SECONDS);
		PRINTF("%d\r\n", (DEPLOY_TIME - cnt));
	}
	PRINTF("Solar array deployed.\r\n");
	bridge->setDuty(0);
	suspendCallerUntil(END_OF_TIME);

	// Its just possible to burn the wire once
	while (1) {
		PRINTF("The solar panel is already deployed\r\n");
		suspendCallerUntil(END_OF_TIME);
	}

}
