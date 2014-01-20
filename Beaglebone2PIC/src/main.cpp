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
#include "I2CSlavePIC.h"

#define OUT_DATA_NUM 7
#define IN_DATA_NUM 48

using namespace std;

int main(int argc, char **argv) {

	char outBuff[20] = {0};
	char inBuff[60] = {0};
	unsigned int dummy;
	char status = 0;
	float temp_f, temp_b, temp_r, temp_l, temp_t;
	float vAmp_f, vAmp_r, vAmp_b, vAmp_l;
	int vFreq_f, vFreq_r, vFreq_b, vFreq_l;
	int proxy_f, proxy_fr, proxy_br, proxy_b, proxy_bl, proxy_fl, proxy_t;
	int ctlPeltier, pwmMotor, pwmR1, pwmG1, pwmB1, pwmR2, pwmG2, pwmB2;


	/*
	 * uint16 temp = temp_f * 10;
	 * if (temp < 0) temp = temp + 65636
	 * outBuff[0] = (temp & 0x00FF)
	 * outBuff[1] = (temp & 0xFF00) >> 8
	 */
	outBuff[0] = 255;
	outBuff[1] = 0;
	/*
	 * uint16 vibeFreq
	 * outBuff[2] = vibeFreq & 0x00FF
	 * outBuff[3] = (vibeFreq & 0xFF00) >> 8
	 */

	outBuff[2] = 10;
	outBuff[3] = 0;

	/*
	 * uint8 pwmR, pwmG, pwmB
	 */
	outBuff[4] = 40;
	outBuff[5] = 70;
	outBuff[6] = 100;

	I2CSlavePIC i2cPIC(1, 0x0b);

	while (1) {

		status = i2cPIC.receiveData(inBuff, IN_DATA_NUM);

		if (status == 0) {
			printf("Failed to open device\n");
		}
		else if (status == -1) {
			printf("Communication init failed\n");
		}
		else if (status == -3) {
			printf("Failed to read %d bytes n", IN_DATA_NUM);
		}
		else {
			printf("Read %d bytes\n", IN_DATA_NUM);
		}

		dummy = inBuff[0] | (inBuff[1] << 8);
		if (dummy > 32767)
			temp_f = (dummy - 65536.0) / 10.0;
		else
			temp_f = dummy / 10.0;

		dummy = inBuff[2] | (inBuff[3] << 8);
		if (dummy > 32767)
			temp_r = (dummy - 65536.0) / 10.0;
		else
			temp_r = dummy / 10.0;

		dummy = inBuff[4] | (inBuff[5] << 8);
		if (dummy > 32767)
			temp_b = (dummy - 65536.0) / 10.0;
		else
			temp_b = dummy / 10.0;

		dummy = inBuff[6] | (inBuff[7] << 8);
		if (dummy > 32767)
			temp_l = (dummy - 65536.0) / 10.0;
		else
			temp_l = dummy / 10.0;

		dummy = inBuff[8] | (inBuff[9] << 8);
		if (dummy > 32767)
			temp_t = (dummy - 65536.0) / 10.0;
		else
			temp_t = dummy / 10.0;

		vAmp_f = (inBuff[10] | (inBuff[11] << 8)) / 10.0;
		vAmp_r = (inBuff[12] | (inBuff[13] << 8)) / 10.0;
		vAmp_b = (inBuff[14] | (inBuff[15] << 8)) / 10.0;
		vAmp_l = (inBuff[16] | (inBuff[17] << 8)) / 10.0;

		vFreq_f = inBuff[18] | (inBuff[19] << 8);
		vFreq_r = inBuff[20] | (inBuff[21] << 8);
		vFreq_b = inBuff[22] | (inBuff[23] << 8);
		vFreq_l = inBuff[24] | (inBuff[25] << 8);

		proxy_f = inBuff[26] | (inBuff[27] << 8);
		proxy_fr = inBuff[28] | (inBuff[29] << 8);
		proxy_br = inBuff[30] | (inBuff[31] << 8);
		proxy_b = inBuff[32] | (inBuff[33] << 8);
		proxy_bl = inBuff[34] | (inBuff[35] << 8);
		proxy_fl = inBuff[36] | (inBuff[37] << 8);
		proxy_t = inBuff[38] | (inBuff[39] << 8);

		ctlPeltier = inBuff[40];
		pwmMotor = inBuff[41];
		pwmR1 = inBuff[42];
		pwmG1 = inBuff[43];
		pwmB1 = inBuff[44];
		pwmR2 = inBuff[45];
		pwmG2 = inBuff[46];
		pwmB2 = inBuff[47];

		printf("temp = %.1f %.1f %.1f %.1f %.1f \n", temp_f, temp_r, temp_b, temp_l, temp_t);
		printf("vibe amp = %.1f %.1f %.1f %.1f \n", vAmp_f, vAmp_r, vAmp_b, vAmp_l);
		printf("vibe freq = %d %d %d %d \n", vFreq_f, vFreq_r, vFreq_b, vFreq_l);
		printf("proxy =  %d %d %d %d %d %d %d \n", proxy_f, proxy_fr, proxy_br, proxy_b, proxy_bl, proxy_fl, proxy_t);
		printf("peltier, motor, led1, led2 = %d %d %d %d %d %d %d %d \n", ctlPeltier, pwmMotor, pwmR1, pwmG1, pwmB1, pwmR2, pwmG2, pwmB2);
		printf("_________________________________________________________________\n\n");

		usleep(500000);
		i2cPIC.sendData(outBuff, OUT_DATA_NUM);
	}

	return 0;
}
