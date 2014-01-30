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
#include "I2CSlavePIC.h"
#include <boost/thread/recursive_mutex.hpp>
#include <zmq.hpp>
#include "dev_msgs.pb.h"
#include "zmq_helpers.hpp"

enum IR_ID {
	IR_F = 0,
	IR_FR,
	IR_BR,
	IR_B,
	IR_BL,
	IR_FL,
	IR_T
};

enum T_ID {
	T_F = 0,
	T_R,
	T_B,
	T_L,
	T_T
};

enum ACC_ID {
	A_F = 0,
	A_R,
	A_B,
	A_L,
};

enum LED {
	L_R = 0,
	L_G,
	L_B
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
	boost::recursive_mutex mtxPub_, mtxRec_;

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

	I2CSlavePIC i2cPIC;

};

 /* namespace Assisi */
#endif /* BBBROUTER_H_ */
