/*
 * ehm.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: thaus
 */

#include "ehm.h"


ehm::ehm(char *port, int baudRate) : Serial(port, baudRate) {

	this->moduleOff();
}

int ehm::initEField() {

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

	// turn on low frequency modulator (lfm)
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)LFM_ON, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"50000i", 6);		// low pulse lfm 500 ms
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"50000q", 6);		// high pulse lfm 500 ms
	usleep(DELAY);

	data = data | E_OUTPUT | E_LFM;
	sprintf(outBuff, "%03db", data);			// set E output and LFM on E outputs
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 4);
	usleep(DELAY);

	this->Close();

	return 1;

}

int ehm::setEFieldFreq(int freq) {

	if (this->Open() <= 0) {
			printf("Cannot open com port \n");
			return -1;
	}

	float period_half = 1.0 / freq * 1000 / 2; // in ms
	if (freq >= 1000) {
		printf("Warning - frequency >= 1000 Hz, setting freq = 1000 Hz\n");
		period_half = 0.5;
	}
	else if (freq < 1) {
		printf("Warning - frequency < 1 Hz, setting freq = 1 Hz\n");
		period_half = 500;
	}

	char outBuff[200] = {0};

	sprintf(outBuff, "%05di", (int)(period_half * 100));
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);		// low pulse lfm
	usleep(DELAY);

	sprintf(outBuff, "%05dq", (int)(period_half * 100));
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);		// high pulse lfm
	usleep(DELAY);

	this->Close();

	return 1;
}

int ehm::initMField() {

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

	// turn on low frequency modulator (lfm)
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)LFM_ON, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"00200e", 6);		// period for high freqeuncy modulator (pwm) - 200 us
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"00010n", 6);		// high duration for high freqeuncy modulator (pwm) - 10 us
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"50000i", 6);		// low pulse lfm 500 ms
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"00050q", 6);		// high pulse lfm	0.5 ms  - this is the smallest value, with low pulse 500 it means it is practically turned off
	usleep(DELAY);

	data = data | L_OUTPUT | L_LFM;
	sprintf(outBuff, "%03db", data);			// set L output and LFM on L outputs
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 4);
	usleep(DELAY);

	this->Close();

	return 1;
}

int ehm::setMFieldFreq(int freq) {

	if (this->Open() <= 0) {
			printf("Cannot open com port \n");
			return -1;
	}

	float period = 1.0 / freq * 1000; // in ms
	if (freq >= 1000) {
		printf("Warning - frequency >= 1000 Hz, setting freq = 1000 Hz\n");
		period = 1;
	}
	else if (freq < 1) {
		printf("Warning - frequency < 1 Hz, setting freq = 1 Hz\n");
		period = 1000;
	}

	char outBuff[200] = {0};

	sprintf(outBuff, "%05di", (int)(0.6 * period * 100));		// 60% low duration ,40 high
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);		// low pulse lfm
	usleep(DELAY);

	sprintf(outBuff, "%05dq", (int)(0.4 * period * 100)); // 60% low duration ,40 high
	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)outBuff, 6);		// high pulse lfm
	usleep(DELAY);

	this->Close();

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

int ehm::moduleOff() {

	if (this->Open() <= 0) {
			printf("Cannot open com port \n");
			return -1;
		}

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)V40_OFF, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)"000b", 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)LFM_OFF, 4);
	usleep(DELAY);

	writeBytes((unsigned char*)INIT_COMM, 4);
	writeBytes((unsigned char*)MODULE_OFF, 4);
	usleep(DELAY);

	this->Close();

	return 1;
}

ehm::~ehm() {
	// TODO Auto-generated destructor stub
}

