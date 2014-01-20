/*
 * I2CSlavePIC.cpp
 *
 *  Created on: Jan 20, 2014
 *      Author: thaus
 */

bla

#include "I2CSlavePIC.h"

/*
 * Constructor
 * inputs: i2cbus - number of the i2c file, corresponding to the i2c bus pic is connected to (e.g. number 0 translates to file /dev/i2c-0
 * 		   devAddress - address of the pic device existing on the selectedi2c bus
 */
I2CSlavePIC::I2CSlavePIC(int i2cBus, int picAddress) : I2CDevice(i2cBus, picAddress) {

}

I2CSlavePIC::~I2CSlavePIC() {
}

/*
 * Method sends bytes to the pic device connected to the i2c bus
 * inputs: buff - pointer to the memory location from where bytes are to be written
 * 		   byteNum - number of byte to write
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -2 - failed to write byte to i2c slave device
 * 		   1 - byte successfully sent
 */
int I2CSlavePIC::sendData(char *buff, int bytesNum) {

	int status;
	status = writeBuff(buff, bytesNum);
	return status;
}

/*
 * Method reads number of bytes from a pic device connected to the i2c bus.
 * inputs: buff - pointer to a memory where incoming bytes are to be saved
 * 		   bytesNum - number of bytes to read
 * return: 0 - failed to open i2c bus
 * 		   -1 - failed to initialize communication with i2c slave device
 * 		   -3 - failed to read byte to i2c slave device
 * 		   1 - byte successfully read
 */
int I2CSlavePIC::receiveData(char *buff, int bytesNum) {

	int status;
	status = readBuff(buff, bytesNum);
	return status;
}

