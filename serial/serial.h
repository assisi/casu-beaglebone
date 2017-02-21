/*! \file serial.h
 * \brief Definition of Serial class.
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <termio.h>

/*! \brief Implements standard serial communication on a Linux based computer.
 *
 * Reading/writing to a serial port is based on Linux filesystem.
 * Serial communication is fixed to type 8-n-1 (8 data bits, no parity, 1 stop bits) and serial port name and baud rate are parameters.
 */

class Serial {
public:

    /*! \brief Constructor. Initializes port name and baud rate.
     *
     * @param port Serial port name.
     * @param baudRate Serial communication baud rate.
     */
    Serial(char *port, unsigned long baudRate);

    /*! \brief Destructor.
     */
    virtual ~Serial();

    /*! \brief Method opens and sets a serial port.
     *
     * Port name and baud rate are given as class members.
     * Port settings are limited to 8-n-1 communication (8 data bits, no parity, 1 stop bits).
     *
     * @return 1 - port successfully set and open \n
     *         0 - failed to open port
     */
    int Open();

    /*! \brief Method closes a serial port.
     *
     * @return 1 - port successfully closed \n
     *         0 - failed to close port
     */
    int Close();

    /*! \brief Method prompts for the number of bytes available on a serial port.
     *
     * @return Number of bytes available for reading on the serial port.
     */
    uint32_t availableBytes();

    /*! \brief Method reads bytes on a serial port.
     *
     * @param buff  Pointer to a  memory location where data be stored.
     * @param buffSize Maximum number of bytes to read on the serial port.
     *
     * @return Actual number of bytes read on the serial port.
     */
    uint32_t readBytes(uint8_t *buff, uint32_t buffSize);

    /*! \brief Method writes data on a serial port.
     *
     * @param buff Pointer to a memory location where data is stored.
     * @param buffSize Number of bytes to write on the serial port.
     *
     * @return Actual number of bytes written on the serial port.
     */
    uint32_t writeBytes(uint8_t *buff, uint32_t buffSize);

private:
    char *port; /*!< Pointer to a memory location where the port name is stored. */
    unsigned long baudRate; /*!< Serial communication baud rate. */
    int fp; /*!< File pointer of a serial port. */

};

#endif /* SERIAL_H */
