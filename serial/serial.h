/*
 * serial.h
 *
 *  Created on: Feb 3, 2014
 *      Author: thaus
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <termio.h>

class Serial {
public:


	Serial(char *port, unsigned long baudRate);
	virtual ~Serial();
	int Open();
	int Close();
	uint32_t availableBytes();
	uint32_t readBytes(uint8_t *buff, uint32_t buffSize);
	uint32_t writeBytes(uint8_t *buff, uint32_t buffSize);

private:
	char *port;
	unsigned long baudRate;
	int fp;

};

#endif /* SERIAL_H_ */
