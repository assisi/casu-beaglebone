/*
 * I2CDevice.cpp
 *
 *  Created on: Nov 24, 2013
 *      Author: thaus
 */

#include "I2CDevice.h"

using namespace std;

/*
 * Constructor
 * inputs: i2cbus - number of the i2c file, corresponding to the i2c bus devices are connected to, default 0 (e.g. number 0 translates to file /dev/i2c-2)
 * 		   devAddress - address of the device existing on the selectedi2c bus, default 0x00
 */
I2CDevice::I2CDevice(unsigned int i2cBus, unsigned int devAddress) {
	this->i2cBus = i2cBus;
	this->i2cAddress = devAddress;
}

I2CDevice::~I2CDevice() {
}


void I2CDevice::initI2C(int i2cBus, int devAddress) {
	this->i2cBus = i2cBus;
	this->i2cAddress = devAddress;
}


/*
 * The following methods(writeByte, readByte, readBytes) are meant to be used with typical I2C devices which use numbered registers for storing data.
 * First the address of the device is sent, second the device's internal register address is sent and finally data byte(s) is read or written
 */

/*
 * Method writes byte to a given register of the device connected to the i2c bus
 * inputs: regAddress - address of the register on the i2c device
 * 		   data - byte to be written
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -2 - failed to write byte to i2c slave device
 * 		   1 - byte successfully sent
 */
int I2CDevice::writeByte(char regAddress, char data) {

		// open the i2c address
		char busName[bufferSize];
		sprintf(busName, "/dev/i2c-%d", i2cBus);
		int file;
		if ((file = open(busName, O_RDWR)) < 0){
			cout << "Failed to open " << busName << " I2C Bus" << endl;
			return 0;
		}
		// initialize communication
		if (ioctl(file, I2C_SLAVE, i2cAddress) < 0){
			cout << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
			close(file);
			return -1;
		}

		char buffer[2];
		buffer[0] = regAddress;
		buffer[1] = data;

		 if (write(file, buffer, 2) != 2) {
			cout << "Failure to write values to I2C Device address." << endl;
			close(file);
			return -2;
		}

		close(file);

		return 0;
}

/*
 * Method writes bytes to a given register of the device connected to the i2c bus
 * inputs: regAddress - address of the register on the i2c device
 * 		   buff - pointer to the memory location where from bytes are to be written
 * 		   byteNum - number of byte to write
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -2 - failed to write byte to i2c slave device
 * 		   1 - byte successfully sent
 */
int I2CDevice::writeBytes(char regAddress, char *buff, int bytesNum) {

	// open the i2c address
	char busName[bufferSize];
	sprintf(busName, "/dev/i2c-%d", i2cBus);
	int file;
	if ((file = open(busName, O_RDWR)) < 0){
		cout << "Failed to open " << busName << " I2C Bus" << endl;
		return 0;
	}
	// initialize communication
	if (ioctl(file, I2C_SLAVE, i2cAddress) < 0){
		cout << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
		close(file);
		return -1;
	}

	char buffer[bytesNum + 1];
	buffer[0] = regAddress;
	int i;
	for (i = 1; i < bytesNum + 1; i++)
		buffer[i] = buff[i];

	if (write(file, buffer, bytesNum + 1) != (bytesNum + 1)) {
		cout << "Failure to write bytes to I2C Device address." << endl;
		close(file);
		return -2;
	}

	close(file);

	return 0;
}

/*
 * Method reads byte from a given register of the device connected to the i2c bus
 * inputs: regAddress - address of the register on the i2c device
 * 		   data - pointer to a memory location where incoming byte is to be saved
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -3 - failed to read byte to i2c slave device
 * 		   1 - byte successfully read
 */
int I2CDevice::readByte(char regAddress, char *data) {
	char buff[1];
	int status = readBytes(regAddress, buff, 1);
	if ( status > 0){
		*data = buff[0];
	}
	return status;
}

/*
 * Method reads number of bytes from a given register of the device connected to the i2c bus. Typical usage is with 16 bit registers on the slave device.
 * inputs: regAddress - address of the register on the i2c device
 * 		   buff - pointer to a memory where incoming bytes are to be saved
 * 		   bytesNum - number of bytes to read
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -3 - failed to read byte to i2c slave device
 * 		   1 - byte successfully read
 */
int I2CDevice::readBytes(char regAddress, char *buff, int bytesNum) {

	// open the i2c address
	char busName[bufferSize];
	sprintf(busName, "/dev/i2c-%d", i2cBus);
	int file;
	if ((file = open(busName, O_RDWR)) < 0){
		cout << "Failed to open " << busName << " I2C Bus" << endl;
		return 1;
	}
	// initialize communication
	if (ioctl(file, I2C_SLAVE, i2cAddress) < 0){
		cout << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
		close(file);
		return 2;
	}

	char buffer[1];
	buffer[0] = regAddress;
	if (write(file, buffer, 1) != 1) {
		cout << "Failure to write values to I2C Device address." << endl;
		close(file);
		return 3;
	}

	if (read(file, buff, bytesNum) != bytesNum) {
		cout << "Failure to read value from I2C Device address." << endl;
		close(file);
		return 4;
	}

	close(file);

	return 0;
}

/*
 * The following functions are meant to be used with any kind of device, both those using register addresses in communication and those with custom protocols
 *
 */

/*
 * Method writes bytes to a given to the device connected to the i2c bus
 * inputs: buff - pointer to the memory location where from bytes are to be written
 * 		   byteNum - number of byte to write
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -2 - failed to write byte to i2c slave device
 * 		   1 - bytes successfully sent
 */
int I2CDevice::writeBuff(char *buff, int bytesNum) {

	// open the i2c address
	char busName[bufferSize];
	sprintf(busName, "/dev/i2c-%d", i2cBus);
	int file;
	if ((file = open(busName, O_RDWR)) < 0){
		cout << "Failed to open " << busName << " I2C Bus" << endl;
		return 0;
	}
	// initialize communication
	if (ioctl(file, I2C_SLAVE, i2cAddress) < 0){
		cout << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
		close(file);
		return -1;
	}

	if (write(file, buff, bytesNum) != bytesNum) {
		cout << "Failure to write values to I2C Device address." << endl;
		close(file);
		return -2;
	}

	close(file);

	return 1;
}

/*
 * Method reads bytes from a given to the device connected to the i2c bus
 * inputs: buff - pointer to the memory location where bytes are to be stored
 * 		   byteNum - number of byte to write
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -3 - failed to read bytes from i2c slave device
 * 		   1 - bytes successfully sent
 */
int I2CDevice::readBuff(char *buff, int bytesNum) {

	// open the i2c address
	char busName[bufferSize];
	sprintf(busName, "/dev/i2c-%d", i2cBus);
	int file;
	if ((file = open(busName, O_RDWR)) < 0){
		cout << "Failed to open " << busName << " I2C Bus" << endl;
		return 0;
	}
	// initialize communication
	if (ioctl(file, I2C_SLAVE, i2cAddress) < 0){
		cout << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
		close(file);
		return -1;
	}

	if (read(file, buff, bytesNum) != bytesNum) {
		cout << "Failure to read value from I2C Device address." << endl;
		close(file);
		return -3;
	}

	close(file);

	return 1;
}

