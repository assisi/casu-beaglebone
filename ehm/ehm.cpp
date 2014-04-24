/*
 * ehm.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: thaus
 */

#include "ehm.h"

ehm::ehm() : Serial("/dev/ttyUSB0", 9600) {

}

ehm::ehm(char *port, int baudRate) : Serial(port, baudRate) {

}

int ehm::initEField() {


	return 1;

}

int ehm::initMField() {

	return 1;
}

int ehm::initHeating() {

	unsigned char data = 0;
	char outBuff[200] = {0};

	if (this->Open() <= 0) {
		printf("Cannot open com port \n");
		return -1;
	}

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)MODULE_ON, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)V40_ON, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)LFM_OFF, 4);
	usleep(DELAY);

	data = data | L_OUTPUT;
	sprintf(outBuff, "%03db", data);
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 4);
	usleep(DELAY);

	sprintf(outBuff, "%05de", L_PWM_PERIOD);  // period 100 us, freq 10 Khz
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);
	usleep(DELAY);

	sprintf(outBuff, "%05dn", 0);  // high duration, 0 us
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);
	usleep(DELAY);

	this->Close();
	return 1;
}
int ehm::readParams() {
	int status;
	unsigned char inBuff[200];
	if (this->Open() <= 0){
		printf("Cannot open com port \n");
		return -1;
	}

	this->writeBytes((unsigned char*) INIT_COMM, 4);
	this->writeBytes((unsigned char*) READ_PARAMS, 4);

	usleep(500000);
	if (this->availableBytes() > 0)
		status = this->readBytes(inBuff, 200);

	inBuff[status] = '\0';
	printf("Params = %s \n", inBuff);
	fflush(stdout);
	this->Close();
	return 1;

}

int ehm::setHeaterPwm(int dutyCycle)  {

	if (this->Open() <= 0){
			printf("Cannot open com port \n");
			return -1;
	}
	char outBuff[100];
	int highPeriod = dutyCycle * L_PWM_PERIOD / 100;
	sprintf(outBuff, "%05dn", highPeriod);
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);
	usleep(DELAY);
	this->Close();
	return 1;
}

ehm::~ehm() {
	// TODO Auto-generated destructor stub
}

