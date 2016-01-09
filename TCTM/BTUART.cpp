/*
 * BTUART.cpp
 *
 *  Created on: Dec 9, 2015
 *      Author: Usama Tariq
 */

#include "BTUART.h"
#include "../Extern/Extern.h"

BT_UART::BT_UART() {
	bluetooth_uart.init(115200);

}

void BT_UART::write(const char *buf, int size) {
	int sentBytes = 0;
	int retVal;
	while (sentBytes < size) {
		retVal = bluetooth_uart.write(&buf[sentBytes], size - sentBytes);
		bluetooth_uart.suspendUntilWriteFinished();
		if (retVal < 0) {
			PRINTF("UART sent error\n");
		} else {
			sentBytes += retVal;
		}
	}
}

void BT_UART::read(char *buff, int index, int size) {

	bluetooth_uart.suspendUntilDataReady();
	int bytesRead = 0;
	int totalBytesRead = 0;
	char temp[size];
	while (totalBytesRead < size) {
		bluetooth_uart.suspendUntilDataReady();
		int c = bluetooth_uart.getcharNoWait();
		if (c == -1)
			continue;
		char ch = c;
		buff[index++] = ch;
		totalBytesRead++;
	}
}
