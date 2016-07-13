/*! \file  mainApp.cpp
    \brief Example of CASU_Interface class usage.
 */


#include "CASU_Interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <serial.h>

using namespace std;

/*! \brief Main function of the CASU interface between MCU and user controller.
 *
 *
 *  Instantiates CASU_Interface and creates three threads for:
 *  - i2c communication
 *  - publishing ZMQ messages
 *  - receiving ZMQ messages.
 */
int main(int argc, char **argv) {

    if (argc < 3)
    {
        cout << "Please provide the serial port name and baud rate as the program arguments." << endl;
        exit(1);
    }

    std::ofstream log_file; /*!< Data stream used for logging data in txt file. */
    timeval start_time; /*!< Stores program start time and used for logging data. */
    timeval current_time;
    float time_sec;
    char str_buff[256] = {0};

    serial_port Serial(argv[1], argv[2]);
    serial_port.Open();

    // write command to init board

    log_file.open((std::string"airsense_log.txt")).c_str(), ios::out);
    gettimeofday(&start_time, NULL);

    while (1) {
        //write command to read measurement

        //read line

        gettimeofday(&current_time, NULL);
        time_sec = current_time.tv_sec + current_time.tv_usec /1000.0;
        //write line to log file
        sprintf(str_buff, "%.3f ... \n", time_sec,  )
        log_file.write(str_buff, strlen(str_buff));
        log_file.flush();

        usleep(90000);

    }

    log_file.flush();
    log_file.close();
    serial_port.Close();

	return 0;
}
