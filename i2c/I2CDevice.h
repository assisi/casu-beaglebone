/*
 * I2CDevice.h
 *
 *  Created on: Nov 24, 2013
 *      Author: thaus
 */

#ifndef I2CDevice_H_
#define I2CDevice_H_

#define bufferSize 64

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
//#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*cmake_find_library
 * Class for communicating with devices on i2c bus. Only master mode is supported, i.e. all other devices should be configured as slaves (multi-master mode !?)
 * Specific device is meant to inherit this class and implement methods such as reading/writing different registers
 */
class I2CDevice {
protected:
	int i2cBus, i2cAddress;
	int writeByte(char regAddress, char data);
	int writeBytes(char regAddress, char *buff, int bytesNum);
	int readByte(char regAddress, char *data);
	int readBytes(char regAddress, char *buff, int bytesNum);
	int writeBuff(char *buff, int bytesNum);
	int readBuff(char *buff, int bytesNum);
public:
	explicit I2CDevice(unsigned int i2cBus = 0, unsigned int devAddress = 0);
	void initI2C(int i2cBus, int devAddress);
	virtual ~I2CDevice();
};

#endif /* I2CDevice_H_ */
