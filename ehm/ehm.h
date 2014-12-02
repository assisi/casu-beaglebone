/*! \file ehm.h
 *  \brief Definition of EHM class used for communicating with electro-magnetic emmitters by Cybertronica.
 */

#ifndef EHM_H
#define EHM_H

#include "../serial/serial.h"

/*! \brief Command for initializing communication with EHM control board.
 */
#define INIT_COMM	"1h2h"

/*! \brief Command for reading current parameters of EHM control board.
 */
#define READ_PARAMS "005a"

/*! \brief Command for turning EHM control board on.
 */
#define MODULE_ON "001a"

/*! \brief Command for turning EHM control board off.
 */
#define MODULE_OFF "000a"

/*! \brief Command for turning voltage regulator (40 V) on.
 */
#define V40_ON	"001c"

/*! \brief Command for turning voltage regulator (40 V) off.
 */
#define V40_OFF	"000c"

/*! \brief Command for turning low frequency modulator on.
 */
#define LFM_ON "009a"

/*! \brief Command for turning low frequency modulator off.
 */
#define LFM_OFF "008a"

/*! \brief Command for turning timer on.
 */
#define TIMER_ON "001d"

/*! \brief Command for turning timer off.
 */
#define TIMER_OFF "000d"

/*! \brief Command for turning sound on.
 */
#define SOUND_ON "001f"

/*! \brief Command for turning sound off.
 */
#define SOUND_OFF "000f"

/*! \brief Mask for turning electric field output on.
 */
#define E_OUTPUT 0b00000001

/*! \brief Mask for turning magnetic field output on.
 */
#define H_OUTPUT 0b00000010

/*! \brief Mask for turning high current output on.
 */
#define L_OUTPUT 0b00000100

/*! \brief Mask for turning reverse PWM on magnetic field output on.
 */
#define H_PWM_REV 0b00001000

/*! \brief Mask for turning on low frequency modulator on electric field.
 */
#define E_LFM 0b00010000

/*! \brief Mask for turning on low frequency modulator on magnetic field.
 */
#define H_LFM 0b00100000

/*! \brief Mask for turning on low frequency modulator on high current output.
 */
#define L_LFM 0b01000000

/*! \brief Mask for turning on low frequency modulator.
 */
#define TOGGLE_LFM 0b10000000

/*! \brief PWM period in microseconds for high current output.
 */
#define L_PWM_PERIOD 100 // us, f = 10 kHz

/*! \brief Delay value in microseconds between sending two consecutive commands.
 */
#define DELAY 25000 // us

/*! \brief Implements protocol for communicating with electro-magnetic (EHM) emitters control board by Cybertronica.
 *
 * EHM control board should be attached to the USB port of the single-board computers, such as Beaglebone, RaspberryPi, Odroid, etc.
 * Implemented protocol is based on EHM control board documentation (<a href="http://cybertronica.co/sites/datasheets/EHM-C-v14-E.pdf">EHM control board</a>)
 */
class EHM : Serial {
public:

	/*! \brief Constructor. Initializes serial port and baud rate.
	 *
	 *@param port Serial port name.
	 *@param baudRate Serial communication baud rate.
	 */
	EHM(char *port, int baudRate);

	/*!\brief Destructor.
	 */
	virtual ~EHM();

	/*! \brief Method reads and prints on standard output current parameter values of the EHM control board.
	 *
	 * @return 1 - parameters successfully read \n
	 * 		  -1 - failed to open serial port, parameters not read
	 */
	int readParams();

	/*! \brief Method initializes EHM control board for heat generation.
	 *
	 *@return 1 - EHM board successfully initialized \n
	 * 		  -1 - failed to open serial port, board not initialized
	 */
	int initHeating();

	/*! \brief Method initializes EHM control board for electric field generation.
	 *
	 *@return 1 - EHM board successfully initialized \n
	 * 		  -1 - failed to open serial port, board not initialized
	 */
	int initEField();

	/*! \brief Method initializes EHM control board for magnetic field generation.
	 *
	 *@return 1 - EHM board successfully initialized \n
	 * 		  -1 - failed to open serial port, board not initialized
	 */
	int initMField();

	/*! \brief Method sets PWM duty cycle value for heating magnetic actuator.
	 *
	 *@param dutyCycle Duty cycle (0-100) of PWM signal for heater actuator
	 *@return 1 - parameter successfully set \n
	 * 		 -1 - failed to open serial port, parameter not set
	 */
	int setHeaterPwm(int dutyCycle);

	/*! \brief Method sets magnetic field frequency.
	 *
	 *@param freq Magnetic field frequency.
	 *@return 1 - parameter successfully set \n
	 * 		 -1 - failed to open serial port, parameter not set
	 */
	int setMFieldFreq(int freq);

	/*! \brief Method sets electric field frequency.
	 *
	 *@param freq Electric field frequency.
	 *@return 1 - parameter successfully set \n
	 * 		 -1 - failed to open serial port, parameter not set
	 */
	int setEFieldFreq(int freq);

	/*! \brief Method turns off all EHM board devices.
	 *
	 * Turns off voltage regulators and driver circuits.
	 *@return 1 - parameter successfully set \n
	 * 		 -1 - failed to open serial port, parameter not set
	 */
	int moduleOff();
};

#endif /* EHM_H */
