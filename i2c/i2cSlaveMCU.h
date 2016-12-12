/*! \file i2cSlaveMCU.h
 * \brief Definition of I2CSlaveMCU class.
 */

#ifndef I2CSLAVEMCU_H
#define I2CSLAVEMCU_H

#include "i2cDevice.h"

/*! \brief Implements i2c communication with MCU device configured as i2c slave.
 *
 * Implementation is based on receiving and sending data buffer.
 * Only master mode is supported, which means that MCU device should be configured as slave.
 * Typical usage with single-board computers such as Beaglebone, RaspberryPI, Odroid etc.
 */
class I2C_SlaveMCU : public I2C_Device {

public:
	/*! \brief Constructor.
	 *
	 * @param i2cbus Number of i2c bus, number 0 translates to file /dev/i2c-0.
	 * @param mcuAddress Address of MCU device connected to specified i2c bus.
	 */
	I2C_SlaveMCU(int i2cBus = 0, int mcuAddress = 0);

	/* \Brief Method sends data to MCU device connected to the i2c bus.
	 * @param buff Pointer to the memory location where data is stored.
	 * @param bytesNum Number of bytes to send.
	 * @return: 1 - data successfully sent \n
	 * 			0 - failed to open i2c bus \n
	 * 		   -1 - failed to initialize communication with i2c slave device \n
	 * 		   -2 - failed to write data to i2c slave device
	 */

	void initI2C(int i2cBus, int i2cAddress, int i2c_connector);
	 	/*! \brief Initializes an i2c device bus and address.
	 *
	 * @param i2cbus Number of an i2c file, default 0 which translates to the file /dev/i2c-0
	 * @param i2cAddress Address of a device existing on the selected i2c bus, default 0x00
	 */
	
	int sendData(char *buff, int bytesNum);

	/*
	 * Method reads number data from MCU device connected to the i2c bus.
	 * @param buff Pointer to a memory used for saving incoming data.
	 * @param bytesNum Number of bytes to read
	 * @return: 1 - data successfully read \n
	 * 			0 - failed to open i2c bus \n
	 * 		   -1 - failed to initialize communication with i2c slave device \n
	 * 		   -3 - failed to read byte to i2c slave device
	 *
	 */
	int receiveData(char *buff, int bytesNum);
	virtual ~I2C_SlaveMCU();

	private:
		char i2cBusSaved;
		char muxAddressSaved;
		I2C_Device mux;
		int i2cConnector;
		
};

#endif /* I2CSLAVEMCU_H */
