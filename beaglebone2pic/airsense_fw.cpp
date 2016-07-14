/*! \file  mainApp.cpp
    \brief Example of CASU_Interface class usage.
 */


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
//#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <serial.h>
#include <csignal>
#include <time.h>
//#include <<inttypes.h>

#define INIT_CMD "bzp05*"
#define MEAS_CMD "bzq*"
#define STOP_CMD "bzk*"

using namespace std;

Serial *serial_port;
std::ofstream log_file;

void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    serial_port->writeBytes((unsigned char*) STOP_CMD, 4);
    usleep(50000);
    log_file.flush();
    log_file.close();
    serial_port->Close();
    delete serial_port;
    exit(signum);

}

int main(int argc, char **argv) {

    if (argc < 2)
    {
        cout << "Please provide the cfg file as the argument." << endl;
        exit(1);
    }

    signal(SIGINT, signalHandler);

    timeval start_time; /*!< Stores program start time and used for logging data. */
    timeval current_time;
    double time_sec;
    char str_buff[256] = {'\0'};
    int buff_size = 256;
    uint32_t send_bytes = 0;
    char chr_buff[256] = {'\0'};
    char start[24] = {'\0'};
    char end[24] = {'\0'};
    int rec_var = 0;
    uint32_t left_rms;
    uint32_t left_mean;
    uint32_t right_rms;
    uint32_t right_mean;
    char date_airsense[50];
    std::string port, baud, log_file_name;


    struct tm *time_struct;

    long            ms; // Milliseconds
    time_t          s;  // Seconds

    YAML::Node cfg = YAML::LoadFile(argv[1]);
    port = cfg["port"].as<string>();
    baud = cfg["baud_rate"].as<string>();
    log_file_name = cfg["log_file"].as<string>();

    std::cout << argv[1] << std::endl;

    std::cout << "Opening log file" << std::endl;

    time_t t = time(NULL);
    time_struct = localtime(&t);
    std::cout << time_struct->tm_year + 1900 << " "<< time_struct->tm_mon + 1<< " "<< time_struct->tm_mday<< std::endl;
    std::cout << time_struct->tm_hour << " "<< time_struct->tm_min << " "<< time_struct->tm_sec<< std::endl;
    sprintf(str_buff, "%s_%d_%d_%d_%d_%0d.txt", log_file_name.c_str(), (int)time_struct->tm_year + 1900, (int)time_struct->tm_mon + 1, (int)time_struct->tm_mday, (int)time_struct->tm_hour, (int)time_struct->tm_min);
    log_file.open(str_buff, ios::out);
    //gettimeofday(&start_time, NULL);

    serial_port = new Serial((char *)port.c_str(), atoi((char *)baud.c_str()));
    serial_port->Open();

    std::cout << "Opened serial port" << std::endl;

    usleep(100000);

    // write command to init board
    send_bytes = serial_port->writeBytes((unsigned char*) INIT_CMD, 6);
    //std::cout << "Sent " << send_bytes << "bytes." << std::endl;

    for (int i = 0; i<3; i++) {
        usleep(200000);
        serial_port->readBytes((uint8_t *)chr_buff, buff_size);
        std::cout << chr_buff << std::endl;
    }

    std::cout << "Entering while loop" << std::endl;



    while (1) {
        //write command to read measurement
        send_bytes = serial_port->writeBytes((unsigned char*) MEAS_CMD, 4);
        //std::cout << "Sent " << send_bytes << "bytes." << std::endl;
        //read line
        //while (serial_port->availableBytes() <= 0) {
        //    std::cout << "Waiting for data " << std::endl;
        //    usleep(10000);
        //}
        usleep(150000);
        rec_var = serial_port->readBytes((uint8_t *)chr_buff, buff_size);
        chr_buff[rec_var] = '\0';
        std::cout << chr_buff << std::endl;

        rec_var = sscanf(chr_buff, "%s %s %d %d %d %d %s", start, date_airsense,
                            &left_rms, &left_mean, &right_rms, &right_mean, end);

        if (rec_var == 7) {
            std::cout << "Received 7 values " << left_rms << " "<< left_mean << " "<< right_rms << " "<< right_mean << std::endl;
            //gettimeofday(&current_time, NULL);
            struct timespec time_spec;
            clock_gettime(CLOCK_REALTIME, &time_spec);
            std::cout << time_spec.tv_sec << std::endl;
            time_sec = time_spec.tv_sec + time_spec.tv_nsec / 1000000000.0;
            //write line to log file
            sprintf(str_buff, "%.3f, %s, %d, %d, %d, %d\r\n", time_sec, date_airsense, left_rms, left_mean, right_rms, right_mean);
            log_file.write(str_buff, strlen(str_buff));
            log_file.flush();
        }
        else {
            std::cout << "Received "<< rec_var <<"values "<<std::endl;
        }

        usleep(50000);
    }

    serial_port->writeBytes((unsigned char*) STOP_CMD, 4);
    usleep(50000);
    log_file.flush();
    log_file.close();
    serial_port->Close();
    delete serial_port;
    return 0;
}
