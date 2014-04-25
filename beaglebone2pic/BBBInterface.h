/*
 * BBBRouter.h
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#ifndef BBBINTERFACE_H_
#define BBBRINTERFACE_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "../i2c/I2CSlavePIC.h"
#include <boost/thread/mutex.hpp>
#include <zmq.hpp>
#include "../msg/dev_msgs.pb.h"
#include "zmq_helpers.hpp"
#include "../ehm/ehm.h"

enum IR_ID {
	IR_F = 0,
	IR_FR = 1,
	IR_BR = 2,
	IR_B = 3,
	IR_BL = 4,
	IR_FL = 5,
	IR_T = 6
};

enum T_ID {
	T_F = 0,
	T_R = 1,
	T_B = 2,
	T_L = 3,
	T_T = 4
};

enum ACC_ID {
	A_F = 0,
	A_R = 1,
	A_B = 2,
	A_L = 3,
};

enum LED {
	L_R = 0,
	L_G = 1,
	L_B = 2
};

#define OUT_DATA_NUM 10
#define IN_DATA_NUM 48


class BBBInterface {

public:
	BBBInterface(int bus, int picAddress);
	virtual ~BBBInterface();
	void i2cComm();
	void zmqPub();
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
	int vFreq[4];
	int irRawVals[7];
	int ledCtl_s[3];
	int ledDiag_s[3];
	int ctlPeltier_s;
	int pwmMotor_s;

	float temp_r;
	int ledCtl_r[3];
	int ledDiag_r[3];
	int ctlPeltier_r;
	int pwmMotor_r;

	int proxyThresh;
	int nameLen;
	char casuName[15];
	I2CSlavePIC i2cPIC;

	float vibeMotorConst;

	ehm *ehm_device;

};

 /* namespace Assisi */
#endif /* BBBROUTER_H_ */
