/*! \file  CASU_Interface.h
    \brief Definition of CASU_Interface class and enums used in class implementations.
 */

#ifndef CASU_INTERFACE_H
#define CASU_INTERFACE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>
//#include <boost/interprocess/sync/named_mutex.hpp>
//#include <boost/interprocess/sync/scoped_lock.hpp>

#include <zmq.hpp>
#include <yaml-cpp/yaml.h>

#include "i2cSlaveMCU.h"
#include "zmq_helpers.hpp"
#include "ehm.h"

#include "base_msgs.pb.h"

/*! Number of bytes (reference data) sent to CASU MCU through i2c communication.
 */
#define OUT_REF_DATA_NUM 12

/*! Number of bytes (calibration data) sent to CASU MCU through i2c communication.
 */
#define OUT_CAL_DATA_NUM 26

/*! Number of bytes received from CASU MCU through i2c communication.
 */
#define IN_DATA_NUM 55

#define IN_DATA_NUM_SLOW 78
#define IN_DATA_NUM_FAST 59
#define IN_DATA_NUM_ACC 4

#define I2C_COMM_LOOP_TIME 100.0

/*! \brief Implements communication with CASU microcontroller (MCU), communication with a user code (CASU controller) and data logging.
 *
 * Class serves as an interface between CASU MCU and a user code and it should be run on a single-board computer (SBC) such as Beaglebone, RaspberyPI, Odroid etc.
 * Communication with MCU is based on an i2c protocol with SBC acting as a master and CASU MCU acting as a slave.
 * Communication with user code is based on messages generated with Google Protobuf and transmitted using ZMQ protocol.
 * Data is logged in a txt file compatible for Matlab import (first row - header containing data info, following rows - data values separated by space delimiter).
 */
class CASU_Interface {

public:

    /*! \brief Constructor.
     *
     * @param fbc_file Firmware board configuration file
*/
    CASU_Interface(char *fbc_file);

    /*! Destructor.
     */
    virtual ~CASU_Interface();

    /*! \brief Used for enumerating CASU infra-red (IR) sensors.
     */
    enum IR_ID {
        IR_F = 0,  /*!< IR sensor on forward CASU side */
        IR_FL = 1, /*!< IR sensor on forward-left CASU side */
        IR_BL = 2, /*!< IR sensor on back-left CASU side */
        IR_B = 3,  /*!< IR sensor on back CASU side */
        IR_BR = 4, /*!< IR sensor on back-right CASU side */
        IR_FR = 5, /*!< IR sensor on forward-right CASU side  */
    };

    /*! \brief Used for enumerating CASU temperature sensors.
     */
    enum T_ID {
        T_F = 0, /*!< Temperature sensor on forward CASU side */
        T_R = 1, /*!< Temperature sensor on right CASU side */
        T_B = 2, /*!< Temperature sensor on back CASU side */
        T_L = 3, /*!< Temperature sensor on left CASU side */
        T_TOP = 4,  /*!< Top temperature sensor (on flex PCB) */
        T_PCB = 5, /*!< Temperature sensor on main PCB */
        T_RING = 6, /*!< Estimated ring temperature (average of T_F, T_L, T_B, T_R) */
        T_WAX = 7 /*!< Estimated wax temeperature */

    };

    /*! \brief Used for enumerating CASU accelerometer sensors.
     */
    enum ACC_ID {
        A_F = 0, /*!< Accelerometer sensor on forward CASU side */
        A_L = 1, /*!< Accelerometer sensor on left CASU side */
        A_B = 2, /*!< Accelerometer sensor on back CASU side */
        A_R = 3, /*!< Accelerometer sensor on right CASU side */
    };

    /*! \brief Used for enumerating light-emitting diode (LED) components
     */
    enum LED {
        L_R = 0, /*!< Red LED component */
        L_G = 1, /*!< Green LED component */
        L_B = 2  /*!< Blue LED component */
    };

    /*! \brief Out messages id.
     */
    enum MSG_OUT_ID {
        MSG_RESET_ID = 1, /*!< Reset msg id */
        MSG_CAL_ID = 2,   /*!< Msg cal id */
        MSG_REF_VIBE_ID = 3, /*!< Vibration red id */
        MSG_REF_LED_ID = 4,  /*!< LED ref id */
        MSG_REF_TEMP_ID = 5,  /*!< Temp ref id */
        MSG_REF_PROXY_ID = 6,
        MSG_MEASUREMENT_FAST_ID = 11, /*!< Request sensor and reference reading EXCEPT temperature */
        MSG_MEASUREMENT_SLOW_ID = 12 /*!< Request sensor and reference reading for all */
    };

    /*! \brief Maximum vibration frequency */
    static const double VIBE_FREQ_MAX;

    /*! \brief Maximum vibration amplitude */
    static const unsigned VIBE_AMP_MAX;

    /*! \brief Minimm vibration pattern period, in milliseconds */
    static const unsigned VIBE_PATTERN_PERIOD_MIN;

    /*! Main CASU communication loop
      Starts all communication and periodic job threads.
    */
    void run();

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
     * Function receives data from user code and/or GUI (such as temperature reference, vibration frequency reference and etc.).
     * Data is received through ZMQ midlleware in a form of protobuf messages.
     */
    void zmqSub();

    /*!
      Method for controlling vibration amplitude and frequency
     */
    void set_vibration(double freq, double amp);

    /*!
      Method for stopping vibration.
     */
    void stop_vibration();

private:

    /*! Utiliy function for setting header timestamps
     */
    void set_msg_header(AssisiMsg::Header* header);

    /*! Function for handling periodic jobs
     */
    void periodic_jobs();

    /*! Function for updating the vibration pattern
     */
    //void update_vibration_pattern(const boost::system::error_code& e);
    void update_vibration_pattern();

    zmq::context_t *zmqContext; /*!< ZMQ context variable.  */
    boost::mutex mtxPub_; /*!< Mutex used for locking outgoing data. */
    boost::mutex mtxSub_; /*!< Mutex used for locking incoming data. */
    boost::mutex mtxi2c_; /*!< Mutex used for locking i2c bus. */
    I2C_Device mux;
    I2C_SlaveMCU i2cPIC; /*!< Used for i2c communication with CASU MCU. */

    EHM *ehm_device;     /*!< Used for serial communication with electro-magnetic emitter control board. */

    char outBuff[20]; /*!< Buffer for i2c outgoing data.  */
    char inBuff[IN_DATA_NUM_SLOW]; /*!< Buffer for i2c incoming data. */
    unsigned int dummy; /*!< Variable used for storing temporarily byte of incoming data.*/
    char status; /*!< Status variable. */
    int calRec; /*!< Status variable for receive notification of calibration data. 1 - data received, 0 - data not yet received */
    int calSend; /*!< Status variable for sending calibration data. 1 - data sent, 0 - data not yet sent */
    float temp[8]; /*!< Array containing latest temperature values 6 sensors + two estimated temperatures. */
    float temp_old[8];
    int index_filter[8];
    float vAmp[IN_DATA_NUM_ACC]; /*!< Array containing latest vibration amplitude values from four sensors. */
    float vFreq[IN_DATA_NUM_ACC]; /*!< Array containing latest vibration frequency values from four sensors. */
    int irRawVals[6]; /*!< Array containing latest infra-red proximity values from seven sensors. */
    int ledDiag_s[3]; /*!< Array containing latest red, green and blue PWM values (0-100) of LED used as diagnostic light. */
    float ctlPeltier_s; /*!< Latest PWM value (-100,100) set to Peltier device. */
    int airflow_s; /*!< Latest PWM value (0,100) set to the actuator producing airflow. */
    int fanCooler; /*!< Latest PWM value (0,100) set to the fan which cools the PCB and aluminium cooler. */

    float temp_ref; /*!< Actual reference value for CASU temperature. */
    float temp_ref_rec; /*!< Set reference value for CASU temperature received from dsPIC. */
    float temp_ref_ramp; /*!< Temperature setpoint limited through ramp as received from dsPIC. */
    float ramp_slope; /*!<Slope of the ramp limiter for temperature reference. If step reference ramp_slope = 0. */
    float alpha; /*!< Current value of SMC adaptive temperature controller parameter alpha. */
    int filtered_glitch; /*!< Flag - received temp_wax value was filtered on PIC before sending. */
    int ledDiag_r[3];  /*!< Actual reference values (RGB) for diagnostic LED. */

    int vibeAmp_s; /*!< Latest reference value for speaker amplitude. */
    int vibeFreq_s; /*!< Latest reference value for speaker frequency. */
    int vibeAmp_r; /*!< Actual reference value for speaker amplitude. */
    int vibeFreq_r; /*!< Actual reference value for speaker frequency. */
    bool vibration_on;

    /* Vibration pattern parameters */
    std::vector<unsigned> vibe_periods;
    std::vector<float> vibe_freqs;
    std::vector<unsigned> vibe_amps;
    unsigned vibe_pattern_idx;
    bool vibe_pattern_on;

    int airflow_r; /*!< Actual reference value for actuator producing airflow. */

    float Kp; /*!< Proportional gain of PI controller */
    float Ki; /*!< Integral gain of PI controller */
    float Kf1; /*!< Weight of current input value of discrete PT1 filter for wax temperature */
    float Kf2; /*!< Weight of old input value of discrete PT1 filter for wax temperature */
    float Kf3; /*!< Weight of old output value of discrete PT1 filter for wax temperature */
    int tempCtlOn; /*!< Temperature control on/off flag */
    int fanCtlOn;  /*!< Fan control on/off flag */
    int i2c_connector;

    //SMC parameters
    int controller_type;
    float C1_sigma;
    float C2_sigma_m;
    float K1_alpha;
    float K2_beta;
    float epsilon;
    float alphak1;

    std::string pub_addr; /*!< Address for publising zmq messages */
    std::string sub_addr; /*!< Address for subscribing to zmq messages */
    std::string pub_addr_af;

    int i2c_bus;      /*!< The number of i2c bus being used for communication with dsPIC */
    int picAddress;  /*!< I2C address of dsPIC MCU */

    int proxyThresh; /*!< Proximity sensor threshold. */
    std::string casuName; /*!< Used for storing CASU name. */

    std::ofstream log_file; /*!< Data stream used for logging data in txt file. */
    timeval start_time; /*!< Stores program start time and used for logging data. */

    // Boost.Asio utilities for scheduling periodic jobs
    boost::asio::io_service io;
    boost::scoped_ptr<boost::asio::deadline_timer> timer_vp;

    struct timeval tLoopStart, tLoopCurrent;
    struct timeval tOutputStart, tOutputCurrent;
    double elapsedTime;

};

template <typename T>
T clamp(const T& n, const T& lower, const T& upper)
{
    return std::max(lower, std::min(n, upper));
}
#endif
