/*! \file  mainApp.cpp
    \brief Example of CASU_Interface class usage.
 */


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
//#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "CASU_Interface.h"
#include <boost/thread.hpp>

using namespace std;

/*! \brief Main function of the CASU interface between MCU and user controller.
 *
 *  Instantiates CASU_Interface and creates three threads for:
 *  - i2c communication
 *  - publishing ZMQ messages
 *  - receiving ZMQ messages.
 */
int main(int argc, char **argv) {

	CASU_Interface BBBintf(2, 0x0b);

	boost::thread_group threads;
	threads.create_thread(boost::bind(&CASU_Interface::i2cComm, &BBBintf));
	threads.create_thread(boost::bind(&CASU_Interface::zmqPub, &BBBintf));
	threads.create_thread(boost::bind(&CASU_Interface::zmqSub, &BBBintf));
	threads.join_all();

	return 0;
}
