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

using namespace std;

int main(int argc, char **argv) {

	BBBInterface BBBintf(2, 0x0b);

	boost::thread_group threads;
	threads.create_thread(boost::bind(&BBBInterface::i2cComm, &BBBintf));
	threads.create_thread(boost::bind(&BBBInterface::zmqPub, &BBBintf));
	threads.create_thread(boost::bind(&BBBInterface::zmqSub, &BBBintf));
	threads.join_all();

	return 0;
}
