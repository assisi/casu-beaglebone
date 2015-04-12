/*! \file  mainApp.cpp
    \brief Example of CASU_Interface class usage.
 */


#include "CASU_Interface.h"

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>

using namespace std;

/*! \brief Main function of the CASU interface between MCU and user controller.
 *
 *  An .fbc file *MUST* be provided as the only argument when invoking the program:
 *  
 *  casu-fw <firmware board config>.fbc
 *
 *  Instantiates CASU_Interface and creates three threads for:
 *  - i2c communication
 *  - publishing ZMQ messages
 *  - receiving ZMQ messages.
 */
int main(int argc, char **argv) {

    if (argc < 2)
    {
        cout << "Please provide the .fbc file name as the program argument." << endl;
        exit(1);
    }

    YAML::Node fbc = YAML::LoadFile(argv[1]);
    string name = fbc["name"].as<string>();
    string pub_addr = fbc["pub_addr"].as<string>();
    string sub_addr = fbc["sub_addr"].as<string>();
    int i2c_addr = fbc["i2c_addr"].as<int>();
    string cal_file = fbc["cal_file"].as<string>();

    CASU_Interface BBBintf(name, 2, i2c_addr, cal_file);

	boost::thread_group threads;
	threads.create_thread(boost::bind(&CASU_Interface::i2cComm, &BBBintf));
	threads.create_thread(boost::bind(&CASU_Interface::zmqPub, &BBBintf, pub_addr));
	threads.create_thread(boost::bind(&CASU_Interface::zmqSub, &BBBintf, sub_addr));
	threads.join_all();

	return 0;
}
