/*
 * I2CDevice.cpp
 *
 *  Created on: Nov 24, 2013
 *      Author: thaus
 */

#include "i2cDevice.h"

using namespace std;

I2C_Device::I2C_Device(unsigned int i2cBus, unsigned int devAddress) {
    this->i2cBus = i2cBus;
    this->i2cAddress = devAddress;
}

I2C_Device::~I2C_Device() {
}

void I2C_Device::initI2C(int i2cBus, int devAddress) {
    this->i2cBus = i2cBus;
    this->i2cAddress = devAddress;
}

/*
* The following method is meant to be used with typical I2C devices operating on Linux. It opens I2C bus stream.
* The opened bus has to be closed after writing/reading.
*/

int I2C_Device::openBus() {

    // open the i2c address
    char busName[bufferSize];
    sprintf(busName, "/dev/i2c-%d", i2cBus);
    int file;
    if ((file = open(busName, O_RDWR)) < 0) {
        cerr << "Failed to open " << busName << " I2C Bus" << endl;
    }

    return file;
}

/*
 * The following methods(writeByte, readByte, readBytes) are meant to be used with typical I2C devices which use numbered registers for storing data.
 * First the address of the device is sent, second the device's internal register address is sent and finally data byte(s) is read or written
 */

int I2C_Device::writeByte(char regAddress, char data, int file) {

    // initialize communication
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        cerr << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
        return -1;
    }

    char buffer[2];
    buffer[0] = regAddress;
    buffer[1] = data;

     if (write(file, buffer, 2) != 2) {
        cerr << "Failure to write values to I2C Device address." << endl;
        return -2;
    }

    return 1;
}

int I2C_Device::writeBytes(char regAddress, char *buff, int bytesNum, int file) {

    // initialize communication
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        cerr << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
        return -1;
    }

    char buffer[bytesNum + 1];
    buffer[0] = regAddress;
    int i;
    for (i = 1; i < bytesNum + 1; i++) {
        buffer[i] = buff[i];
    }

    if (write(file, buffer, bytesNum + 1) != (bytesNum + 1)) {
        cerr << "Failure to write bytes to I2C Device address." << endl;
        return -2;
    }

    return 1;
}

int I2C_Device::readByte(char regAddress, char *data, int file) {
    char buff[1];
    int status = readBytes(regAddress, buff, 1, file);
    if ( status > 0) {
        *data = buff[0];
    }
    return status;
}

int I2C_Device::readBytes(char regAddress, char *buff, int bytesNum, int file) {

    // initialize communication
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        cerr << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
        return -1;
    }

    char buffer[1];
    buffer[0] = regAddress;
    if (write(file, buffer, 1) != 1) {
        cerr << "Failure to write values to I2C Device address." << endl;
        return -2;
    }

    if (read(file, buff, bytesNum) != bytesNum) {
        cerr << "Failure to read value from I2C Device address1." << endl;
        return -3;
    }

    return 1;
}

/*
 * The following functions are meant to be used with any kind of device, both those using register addresses in communication and those with custom protocols
 *
 */

int I2C_Device::writeBuff(char *buff, int bytesNum, int file) {

    // initialize communication
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        cerr << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
        return -1;
    }

    if (write(file, buff, bytesNum) != bytesNum) {
        //cerr << "Failure to write values to I2C Device address." << endl;
        return -2;
    }

    return 1;
}

int I2C_Device::readBuff(char *buff, int bytesNum, int file) {

    // initialize communication
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        cerr << "i2c_SLAVE address " << i2cAddress << " failed..." << endl;
        return -1;
    }

    if (read(file, buff, bytesNum) != bytesNum) {
        cerr << "Failure to read value from I2C Device address2." << endl;
        return -3;
    }

    return 1;
}
