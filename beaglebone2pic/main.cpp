/*
 * main.c
 *
 *  Created on: Jan 19, 2014
 *      Author: thaus
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "BBBInterface.h"
#include <boost/thread.hpp>

#define OUT_DATA_NUM 7
#define IN_DATA_NUM 48

using namespace std;

int main(int argc, char **argv) {

//	char outBuff[20] = {0};
//	char inBuff[60] = {0};
//	unsigned int dummy;
//	char status = 0;
//	float temp_f, temp_b, temp_r, temp_l, temp_t;
//	float vAmp_f, vAmp_r, vAmp_b, vAmp_l;
//	int vFreq_f, vFreq_r, vFreq_b, vFreq_l;
//	int proxy_f, proxy_fr, proxy_br, proxy_b, proxy_bl, proxy_fl, proxy_t;
//	int ctlPeltier, pwmMotor, pwmR1, pwmG1, pwmB1, pwmR2, pwmG2, pwmB2;
//
//
//	/*
//	 * uint16 temp = temp_f * 10;
//	 * if (temp < 0) temp = temp + 65636
//	 * outBuff[0] = (temp & 0x00FF)
//	 * outBuff[1] = (temp & 0xFF00) >> 8
//	 */
//	outBuff[0] = 255;
//	outBuff[1] = 0;
//	/*
//	 * uint16 vibeFreq
//	 * outBuff[2] = vibeFreq & 0x00FF
//	 * outBuff[3] = (vibeFreq & 0xFF00) >> 8
//	 */
//
//	outBuff[2] = 10;
//	outBuff[3] = 0;
//
//	/*
//	 * uint8 pwmR, pwmG, pwmB
//	 */
//	outBuff[4] = 40;
//	outBuff[5] = 70;
//	outBuff[6] = 100;
//
//	I2CSlavePIC i2cPIC(2, 0x0b);


	BBBInterface BBBintf(2, 0x0b);

	boost::thread_group threads;
	threads.create_thread(boost::bind(&BBBInterface::i2cComm, &BBBintf));
	threads.create_thread(boost::bind(&BBBInterface::zmqPub, &BBBintf));
	threads.create_thread(boost::bind(&BBBInterface::zmqSub, &BBBintf));
	threads.join_all();

	return 0;
}
