/*! \file i2cDevice.h
 * \brief Definition of I2CDevice class.
 */

#ifndef I2CDevice_H
#define I2CDevice_H

/*! Internal buffer size in bytes.
 */
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

/*! \brief Implements i2c communication of a Linux based computer and a generic i2c device.
 *
 * Implementation is based on reading and writing specific number of bytes.
 * Only master mode is supported, i.e. connected devices should be configured as slaves.
 * It should be typically used on a single-board computer such as Beaglebone, RaspberryPI, Odroid etc.
 */
class I2C_Device {

public:

	/*! \brief
	 * Constructor.
	 *
	 * Initializes i2c params - number of an i2c bus and i2c address of a slave device.
	 * @param i2cbus Number of an i2c file, default 0 which translates to the file /dev/i2c-0)
	 * @param i2cAddress Address of a device existing on the selected i2c bus, default 0x00
	 */
	explicit I2C_Device(unsigned int i2cBus = 0, unsigned int i2cAddress = 0);

	/*! Destructor.
	 */
	virtual ~I2C_Device();

	/*! \brief Initializes an i2c device bus and address.
	 *
	 * @param i2cbus Number of an i2c file, default 0 which translates to the file /dev/i2c-0
	 * @param i2cAddress Address of a device existing on the selected i2c bus, default 0x00
	 */
	void initI2C(int i2cBus, int i2cAddress);

protected:

	/*! \brief Method writes byte to the given register of an i2c device.
	 *
	 * @param regAddress Address of an i2c device register.
	 * @param data Data byte to be written.
	 * @return  1 - byte successfully sent \n
	 * 			0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -2 - failed to write byte to the i2c slave device
	 */
	int writeByte(char regAddress, char data);

	/*! \brief Method writes bytes to the given register of an i2c device.
	 *
	 * @param regAddress Register address of an i2c device.
	 * @param buff Pointer to a memory location containing data.
	 * @param byteNum Number of bytes to write.
	 * @return 1 - bytes successfully sent \n
	 * 		   0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -2 - failed to write byte to the i2c slave device
	 */
	int writeBytes(char regAddress, char *buff, int bytesNum);

	/*! \brief Method reads byte from the given register of an i2c device.
	 *
	 * @param regAddress Address of an i2c device register.
	 * @param data Pointer to a memory location where the incoming byte should be saved.
	 * @return  1 - byte successfully read \n
	 * 			0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -2 - failed to write byte to the i2c slave device (register address) \n
	 * 		   -3 - failed to read byte from the i2c slave device
	 */
	int readByte(char regAddress, char *data);

	/*! \brief Method reads bytes from the given register of an i2c device.
	 *
	 * @param regAddress Address of an register on the i2c device.
	 * @param buff Pointer to a memory where the incoming bytes should be saved.
	 * @param bytesNum Number of bytes to read.
	 * @return  1 - byte successfully read \n
	 * 			0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -2 - failed to write byte to the i2c slave device (register address) \n
	 *		   -3 - failed to read bytes from the i2c slave device
	 */
	int readBytes(char regAddress, char *buff, int bytesNum);

	/*! \brief Method writes bytes to an i2c device without using register address.
	 *
	 * @param buff Pointer to a memory location containing data.
	 * @param byteNum Number of bytes to write.
	 * @return  1 - bytes successfully sent \n
	 * 			0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -2 - failed to write byte to the i2c slave device
	 */
	int writeBuff(char *buff, int bytesNum);

	/*! \brief Method reads bytes from an i2c device without using register address.
	 *
	 * @param buff Pointer to a memory location where the incoming bytes should be saved.
	 * @param byteNum Number of bytes to read.
	 * @return  1 - bytes successfully sent \n
	 * 			0 - failed to open the i2c bus \n
	 * 		   -1 - failed to initialize communication with the i2c slave device \n
	 * 		   -3 - failed to read bytes from the i2c slave device
	 */
	int readBuff(char *buff, int bytesNum);

	int i2cBus; /*!< Number of an i2c bus, e.g. 1 translates to the file /dev/i2c-1 */
	int i2cAddress; /*!< Address of an i2c device */
};

#endif /* I2CDevice_H */
