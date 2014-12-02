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
#include <zmq.hpp>
#include "dev_msgs.pb.h"
#include "zmq_helpers.hpp"
#include "ehm.h"
#include <time.h>
#include <fstream>
#include <sstream>

/*! Number of bytes sent to CASU MCU through i2c communication.
 */
#define OUT_DATA_NUM 10

/*! Number of bytes received from CASU MCU through i2c communication.
 */
#define IN_DATA_NUM 48

/*! \brief Implements communication with CASU microcontroller (MCU), communication with CASU controller and data logging.
 *
 * Class serves as interface between CASU MCU and CASU controller and it should be run on a single-board computer (SBC) such as Beaglebone, RaspberyPI, Odroid etc.
 * Communication with MCU is based on i2c protocol where SBC acts as a master and CASU MCU acts as a slave.
 * Communication with CASU controller is based on messages generated with Google Protobuf and transmitted using ZMQ protocol.
 * Data is logged in a txt file compatible for Matlab import (first row - header containing data info, following rows - data values separated by space delimiter).
 */
class CASU_Interface {

public:

	/*! \brief Constructor.
	 *
	 * @param i2c_bus number of i2c bus used for MCU communication
	 * @param i2c_address i2c address of a CASU MCU slave
	 */
	CASU_Interface(int i2c_bus, int i2c_address);

	/*! Destructor.
	 */
	virtual ~CASU_Interface();

	/*! Used for enumerating CASU infra-red (IR) sensors.
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

	/*! Used for enumerating CASU temperature sensors.
	 */
	enum T_ID {
		T_F = 0, /*!< Temperature sensor on forward CASU side */
		T_R = 1, /*!< Temperature sensor on right CASU side */
		T_B = 2, /*!< Temperature sensor on back CASU side */
		T_L = 3, /*!< Temperature sensor on left CASU side */
		T_T = 4 /*!< Temperature sensor on top CASU side */
	};

	/*! Used for enumerating CASU accelerometer sensors.
	 */
	enum ACC_ID {
		A_F = 0, /*!< Accelerometer sensor on forward CASU side */
		A_R = 1, /*!< Accelerometer sensor on right CASU side */
		A_B = 2, /*!< Accelerometer sensor on back CASU side */
		A_L = 3, /*!< Accelerometer sensor on left CASU side */
	};

	/*! Used for enumerating light-emitting diode (LED) components
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
	 * Function receives data from CASU controller(s) and/or GUI (such as temperature reference, vibration frequency reference and etc.).
	 * Data is received through ZMQ protocol in form of protobuf messages.
	 */
	void zmqSub();

private:

	zmq::context_t *zmqContext;
	boost::mutex mtxPub_, mtxSub_;

	char outBuff[20];
	char inBuff[60];
	unsigned int dummy;
	char status;;
	float temp[5];
	float vAmp[4];
	float vFreq[4];
	int irRawVals[7];
	int ledCtl_s[3];
	int ledDiag_s[3];
	int ctlPeltier_s;
	int pwmMotor_s;

	float temp_r, temp_ref, temp_ref_old;
	float temp_rate;
	int ledCtl_r[3];
	int ledDiag_r[3];
	int ctlPeltier_r;
	int pwmMotor_r;

	int proxyThresh;
	int nameLen;
	char casuName[15];
	I2C_SlaveMCU i2cPIC;

	float vibeMotorConst;

    // EHM device
	ehm *ehm_device;
    int ehm_freq_electric;
    int ehm_freq_magnetic;
    int ehm_temp;

    std::ofstream log_file;

	time_t ctlTime, time_a;

	// temp controller params
	float uOld_t, eOld_t, deOld_t, deadZone_t, uiOld;
	float Kp_t, Ki_t, Kd_t;
	int ctlFlag;
	float temp_old;
	int startFlag;

	timeval start_time;

	float PIDcontroller_t(float temp);
	void temp_rate_filter();

};

#endif
