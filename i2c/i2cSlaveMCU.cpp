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

void I2C_SlaveMCU::initI2C(int i2cBus, int devAddress, int i2c_connector) {
	this->i2cBus = i2cBus;
	this->i2cAddress = devAddress;
	this->i2cConnector = i2c_connector;
}

int I2C_SlaveMCU::sendData(char *buff, int bytesNum) {
	
	int file = openBus();
	if (file < 0) {
		cerr << "Did not open file!" << endl;
		close(file);
		return -1;
	}

    int lockMode = F_LOCK;
    int lockStatus = lockf(file, lockMode, 0);

	mux.initI2C(i2cBus, 112);
	usleep(1000);

	int muxStatus = -100;
	int countMux = 0;
	int muxMask = 1 << (i2cConnector - 1);
	while (muxStatus <= 0) {
		muxStatus = mux.writeByte(0, muxMask, file);	
		if (muxStatus <= 0) {
			usleep(1000);
			cerr << "Did not set mask to mux, sleep for 1ms " << endl;
		}
		countMux++;
		if (countMux >= 10) {
			if (muxStatus <= 0) {
				cerr << "Could not write mask to mux"  << endl;
				lockMode = F_ULOCK;
    			lockStatus = lockf(file, lockMode, 0);
				close(file);
				return -2;
			}
			break;
		}
	}

//	mux.writeByte(0, 0x01, file);
//	writeByte(i2cBusSaved, muxAddressSaved, file);
	int status = -1;
	int count = 0;
	while (status <= 0) {
		status = writeBuff(buff, bytesNum, file);
		count++;
		if (count >= 10) {
			cerr << "Waiting for outgoing data" << endl;
			if (status <= 0) {
				cerr << "Did not write to device" << endl;
				lockMode = F_ULOCK;
                lockStatus = lockf(file, lockMode, 0);
				close(file);
				return -3;
			}
			break;
		}
	}
	lockMode = F_ULOCK;
    lockStatus = lockf(file, lockMode, 0);
	close(file);
	return status;
}

int I2C_SlaveMCU::receiveData(char *buff, int bytesNum) {

	int file = openBus();
	if (file < 0) {
		cerr << "Did not open file!" << endl;
		close(file);
		return -1;
	}

	int lockMode = F_LOCK;
    int lockStatus = lockf(file, lockMode, 0);

	mux.initI2C(i2cBus, 112);
	usleep(1000);

	int muxStatus = -100;
	int countMux = 0;
	int muxMask = 1 << (i2cConnector - 1);
	while (muxStatus <= 0) {
		muxStatus = mux.writeByte(0, muxMask, file);	
		if (muxStatus <= 0) {
			usleep(1000);
			cerr << "Did not set mask to mux, sleep for 1ms " << endl;
		}
		countMux++;
		if (countMux >= 10) {
			if (muxStatus <= 0) {
				cerr << "Could not write mask to mux"  << endl;
				lockMode = F_ULOCK;
    			lockStatus = lockf(file, lockMode, 0);
				close(file);
				return -2;
			}
			break;
		}
	}

	int status = -1;
	int count  = 0;
	while (status <= 0) {
		status = readBuff(buff, bytesNum, file);
		count++;
		if (count >= 10) {
			cerr << "Waiting for incoming data" << endl;
			if (status <= 0) {
				cerr << "Did not write to device" << endl;
				lockMode = F_ULOCK;
    			lockStatus = lockf(file, lockMode, 0);
    			close(file);
				return -3;
			}
			break;
		}
	}
	lockMode = F_ULOCK;
    lockStatus = lockf(file, lockMode, 0);
	close(file);
	return status;
}

