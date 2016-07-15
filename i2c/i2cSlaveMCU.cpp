/*
 * I2CSlavePIC.cpp
 *
 *  Created on: Jan 20, 2014
 *      Author: thaus
 */

#include "i2cSlaveMCU.h"

using namespace std;

I2C_SlaveMCU::I2C_SlaveMCU(int i2cBus, int picAddress) : I2C_Device(i2cBus, picAddress) {

}

I2C_SlaveMCU::~I2C_SlaveMCU() {
}

int I2C_SlaveMCU::sendData(char *buff, int bytesNum) {
	
	writeByte(i2cBusSaved, muxAddressSaved);
	int status = -1;
	int count = 0;
	while (status <= 0) {
		status = writeBuff(buff, bytesNum);
		count++;
		if (count >= 10) {
			cerr << "Waiting for outgoing data" << endl;
			break;
		}
	}
	return status;
}

int I2C_SlaveMCU::receiveData(char *buff, int bytesNum) {

	int status = -1;
	int count  = 0;
	while (status <= 0) {
		status = readBuff(buff, bytesNum);
		count++;
		if (count >= 10) {
			cerr << "Waiting for incoming data" << endl;
			break;
		}
	}
	return status;
}

