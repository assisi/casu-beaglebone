/*
 * I2CSlavePIC.h
 *
 *  Created on: Jan 20, 2014
 *      Author: thaus
 */

#ifndef I2CSLAVEPIC_H_
#define I2CSLAVEPIC_H_

#include "i2cDevice.h"


class I2CSlavePIC : public I2CDevice {
public:
	I2CSlavePIC(int i2cBus = 0, int picAddress = 0);
	int sendData(char *buff, int bytesNum);
	int receiveData(char *buff, int bytesNum);
	virtual ~I2CSlavePIC();
};

#endif /* I2CSLAVEPIC_H_ */
