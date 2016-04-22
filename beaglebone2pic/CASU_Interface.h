/*! \file  CASU_Interface.h
    \brief Definition of CASU_Interface class and enums used in class implementations.
 */

#ifndef CASU_INTERFACE_H
#define CASU_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "i2cSlaveMCU.h"
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <zmq.hpp>
#include "dev_msgs.pb.h"
#include "zmq_helpers.hpp"
#include "ehm.h"
#include <time.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

/*! Number of bytes (reference data) sent to CASU MCU through i2c communication.
 */
#define OUT_REF_DATA_NUM 12

/*! Number of bytes (calibration data) sent to CASU MCU through i2c communication.
 */
#define OUT_CAL_DATA_NUM 13

/*! Number of bytes received from CASU MCU through i2c communication.
 */
#define IN_DATA_NUM 61

/*! \brief Implements communication with CASU microcontroller (MCU), communication with a user code (CASU controller) and data logging.
 *
 * Class serves as an interface between CASU MCU and a user code and it should be run on a single-board computer (SBC) such as Beaglebone, RaspberyPI, Odroid etc.
 * Communication with MCU is based on an i2c protocol with SBC acting as a master and CASU MCU acting as a slave.
 * Communication with user code is based on messages generated with Google Protobuf and transmitted using ZMQ protocol.
 * Data is logged in a txt file compatible for Matlab import (first row - header containing data info, following rows - data values separated by space delimiter).
 */
class CASU_Interface {

public:

	/*! \brief Constructor.
	 *
     * @param fbc_file Firmware board configuration file
	 */
    CASU_Interface(char *fbc_file);

	/*! Destructor.
	 */
	virtual ~CASU_Interface();

	/*! \brief Used for enumerating CASU infra-red (IR) sensors.
	 */
	enum IR_ID {
		IR_F = 0,  /*!< IR sensor on forward CASU side */
		IR_FR = 1, /*!< IR sensor on forward-right CASU side  */
		IR_BR = 2, /*!< IR sensor on back-right CASU side */
		IR_B = 3,  /*!< IR sensor on back CASU side */
		IR_BL = 4, /*!< IR sensor on back-left CASU side */
		IR_FL = 5, /*!< IR sensor on forward-left CASU side */
		IR_T = 6   /*!< IR sensor on top CASU side */
	};

	/*! \brief Used for enumerating CASU temperature sensors.
	 */
	enum T_ID {
		T_F = 0, /*!< Temperature sensor on forward CASU side */
		T_R = 1, /*!< Temperature sensor on right CASU side */
		T_B = 2, /*!< Temperature sensor on back CASU side */
		T_L = 3, /*!< Temperature sensor on left CASU side */
        T_flexPCB = 4,  /*!< Temperature sensor on flex PCB */
        T_PCB = 5 /*!< Temperature sensor on main pcb */

	};

	/*! \brief Used for enumerating CASU accelerometer sensors.
	 */
	enum ACC_ID {
		A_F = 0, /*!< Accelerometer sensor on forward CASU side */
		A_R = 1, /*!< Accelerometer sensor on right CASU side */
		A_B = 2, /*!< Accelerometer sensor on back CASU side */
		A_L = 3, /*!< Accelerometer sensor on left CASU side */
	};

	/*! \brief Used for enumerating light-emitting diode (LED) components
	 */
	enum LED {
		L_R = 0, /*!< Red LED component */
		L_G = 1, /*!< Green LED component */
		L_B = 2  /*!< Blue LED component */
	};

	/*! Thread safe method that implements i2c communication with CASU MCU slave.
	*/
	void i2cComm();

	/*! Thread safe method that sends messages using ZMQ protocol.
	 *
	 * Function sends data received from CASU MCU (such as temperature value, vibration frequency and etc.).
	 * Data is formed in protobuf messages and sent using ZMQ protocol.
	 */
    void zmqPub();

	/*! Thread safe method that receives messages using ZMQ protocol.
	 *
	 * Function receives data from user code and/or GUI (such as temperature reference, vibration frequency reference and etc.).
	 * Data is received through ZMQ midlleware in a form of protobuf messages.
	 */
    void zmqSub();

private:

	zmq::context_t *zmqContext; /*!< ZMQ context variable.  */
	boost::mutex mtxPub_; /*!< Mutex used for locking outgoing data. */
	boost::mutex mtxSub_; /*!< Mutex used for locking incoming data. */
        I2C_Device mux;
	I2C_SlaveMCU i2cPIC; /*!< Used for i2c communication with CASU MCU. */
	
EHM *ehm_device;	 /*!< Used for serial communication with electro-magnetic emitter control board. */

	char outBuff[20]; /*!< Buffer for i2c outgoing data.  */
        char inBuff[61]; /*!< Buffer for i2c incoming data. */
	unsigned int dummy; /*!< Variable used for storing temporarily byte of incoming data.*/
	char status; /*!< Status variable. */
    int calRec; /*!< Status variable for receive notification of calibration data. 1 - data received, 0 - data not yet received */
    int calSend; /*!< Status variable for sending calibration data. 1 - data sent, 0 - data not yet sent */
	float temp[6]; /*!< Array containing latest temperature values from five sensors. */
    float tempWax; /*!< Estimated wax temperature. */
    float tempCasu; /*!< Estimated casu ring temperature. */
	float vAmp[4]; /*!< Array containing latest vibration amplitude values from four sensors. */
	float vFreq[4]; /*!< Array containing latest vibration frequency values from four sensors. */
	int irRawVals[7]; /*!< Array containing latest infra-red proximity values from seven sensors. */
	int ledCtl_s[3]; /*!< Array containing latest red, green and blue PWM values (0-100) of LED used as bee stimulus. */
	int ledDiag_s[3]; /*!< Array containing latest red, green and blue PWM values (0-100) of LED used as diagnostic light. */
	int ctlPeltier_s; /*!< Latest PWM value (-100,100) set to Peltier device. */
	int pwmMotor_s; /*!< Latest PWM value (0,100) set to vibration motor. */
    int airflow_s; /*!< Latest PWM value (0,100) set to the actuator producing airflow. */
    int fanCooler; /*!< Latest PWM value (0,100) set to the fan which cools the PCB and aluminium cooler. */

	float temp_ref; /*!< Actual reference value for CASU temperature. */
    float temp_ref_rec; /*!< Setted feference value for CASU temperature received from dsPIC. */
    float temp_ref_cur; /*!< Actual reference value for CASU temperature received from dsPIC. */
	int ledCtl_r[3]; /*!< Actual reference values (RGB) for control LED. */
	int ledDiag_r[3]; /*!< Actual reference values (RGB) for diagnostic LED. */
	int pwmMotor_r; /*!< Actual reference value for vibration motor. */
	int vibeAmp_r;
	int vibeFreq_r;

    int airflow_r; /*!< Actual reference value for actuator producing airflow. */

    float Kp; /*!< Proportional gain of PI controller */
    float Ki; /*!< Integral gain of PI controller */
    float Kf1; /*!< Weight of current input value of discrete PT1 filter for wax temperature */
    float Kf2; /*!< Weight of old input value of discrete PT1 filter for wax temperature */
    float Kf3; /*!< Weight of old output value of discrete PT1 filter for wax temperature */
    int tempCtlOn; /*!< Temperature control on/off flag */
    int fanCtlOn;  /*!< Fan control on/off flag */

    std::string pub_addr; /*!< Address for publising zmq messages */
    std::string sub_addr; /*!< Address for subscribing to zmq messages */
    std::string pub_addr_af;
    
    int i2c_bus;      /*!< The number of i2c bus being used for communication with dsPIC */
    int picAddress;  /*!< I2C address of dsPIC MCU */

	int proxyThresh; /*!< Proximity sensor threshold. */
    std::string casuName; /*!< Used for storing CASU name. */

	float vibeMotorConst; /*!< Vibration motor constant - ratio of motor frequency and motor voltage. */

    int ehm_freq_electric;	/*!< Latest frequency of the electric field. */
    int ehm_freq_magnetic; /*!< Latest frequency of the magnetic field. */
    int ehm_temp; /*!< Latest reference temperature value for magnetic heater (when used as a heater). */

    std::ofstream log_file; /*!< Data stream used for logging data in txt file. */
	timeval start_time; /*!< Stores program start time and used for logging data. */

};

#endif
