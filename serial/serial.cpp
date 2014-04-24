/*
 * serial.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: thaus
 */

#include "serial.h"

using namespace std;

Serial::Serial(char *port, unsigned long baudRate) {

	this->port = new char(strlen(port) + 1);
	memcpy(this->port, port, strlen(port) + 1);
	this->baudRate = baudRate;
}

Serial::~Serial() {
	delete[] this->port;
}

int Serial::Open() {

	fp = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fp >= 0) {

		fcntl(fp, F_SETFL, 0);

		struct termios options;

		/*
		 * Get the current options for the port...
		 */
		tcgetattr(fp, &options);

		/*
		 * Set the baud rates...
		 */

		switch(baudRate) {
			case 9600: cfsetispeed(&options, B9600);
						break;
			case 19200:	cfsetispeed(&options, B19200);
						break;
			case 38400: cfsetispeed(&options, B38400);
						break;
			case 57600: cfsetispeed(&options, B57600);
						break;
			case 115200:cfsetispeed(&options, B115200);
						break;
			default: cerr << "Unsupported baud rate" << endl;
					 return 0;
		}

		/*
		 * Enable the receiver and set local mode...
		 */
		options.c_cflag |= (CLOCAL | CREAD );

		/*
		 * Set the new options for the port...
		 */
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		//options.c_cflag &= ~CNEW_RTSCTS;  //CNEW_RTSCTS; //Turn off flow control

		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Make sure that Canonical input is off (raw input data)

		options.c_iflag &= ~(IXON | IXOFF | IXANY); //Turn off software control flow

		options.c_oflag &= ~OPOST; //Raw output data

		tcsetattr(fp, TCSANOW, &options);

		return 1;
	}
	else
		return 0;
}

int Serial::Close() {

	close(fp);

	return 1;
}


uint32_t Serial::availableBytes() {
	uint32_t bytesNum;
	ioctl(fp, FIONREAD, &bytesNum);
	return bytesNum;
}

uint32_t Serial::readBytes(uint8_t *buff, uint32_t buffSize) {

	uint32_t bytes;
	uint32_t bytesRec = 0;

	ioctl(fp, FIONREAD, &bytes);
	//printf("Available %d bytes\n", bytes);

	// Read serial port
	if (bytes > 0) {
		//fcntl(fd, F_SETFL, FNDELAY); //Return read immediatly

		if (bytes < buffSize)
			bytesRec = read(fp, buff, bytes);
		else
			bytesRec = read(fp, buff, buffSize);
	}

	return bytesRec;
}

uint32_t Serial::writeBytes(uint8_t *buff, uint32_t buffSize) {
	uint32_t bytesWriten = 0;

	bytesWriten = write(fp, buff, buffSize);
	return bytesWriten;
}


