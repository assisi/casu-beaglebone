/*
 * BBBRouter.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#include "CASU_Interface.h"
#include <sys/time.h>

using namespace std;
using namespace AssisiMsg;

CASU_Interface::CASU_Interface(char *fbc_file)
    {


    YAML::Node fbc = YAML::LoadFile(fbc_file);
    casuName = fbc["name"].as<string>();
    pub_addr = fbc["pub_addr"].as<string>();
    sub_addr = fbc["sub_addr"].as<string>();
    pub_addr_af.assign("tcp://10.42.0.253:1555");

    i2c_bus = fbc["i2c_bus"].as<int>();
    picAddress = fbc["i2c_addr"].as<int>();
    tempCtlOn = fbc["tempCtlOn"].as<int>();
    Kp = fbc["Kp"].as<float>();
    Ki = fbc["Ki"].as<float>();
    Kf1 = fbc["Kf1"].as<float>();
    Kf2 = fbc["Kf2"].as<float>();
    Kf3 = fbc["Kf3"].as<float>();
    fanCtlOn = fbc["fanCtlOn"].as<int>();

    mux.initI2C(2, 112);
    mux.writeByte(0, 0xFF);
    
    i2cPIC.initI2C(i2c_bus, picAddress);
    zmqContext = new zmq::context_t(3);
    ledDiag_r[L_R] = 0;
    ledDiag_r[L_G] = 0;
    ledDiag_r[L_B] = 0;

    irRawVals[IR_F] = 0;
    irRawVals[IR_FR] = 0;
    irRawVals[IR_BR] = 0;
    irRawVals[IR_B] = 0;
    irRawVals[IR_BL] = 0;
    irRawVals[IR_FL] = 0;

    temp[0] = 0.0;
    temp[1] = 0.0;
    temp[2] = 0.0;
    temp[3] = 0.0;
    temp[4] = 0.0;
    temp[5] = 0.0;
    tempWax = -1;
    tempCasu = -1;

    vAmp[0] = 0.0;
    vAmp[1] = 0.0;
    vAmp[2] = 0.0;
    vAmp[3] = 0.0;

    vFreq[0] = 0.0;
    vFreq[1] = 0.0;
    vFreq[2] = 0.0;
    vFreq[3] = 0.0;

    vibeAmp_r = 0;
    vibeFreq_r = 100;

    vibeAmp_s = 0;
    vibeFreq_s = 100;

    airflow_r = 0;
    airflow_s = 0;

    fanCooler = 0;

    temp_ref = 0.0;
    temp_ref_rec = 0.0;

    calRec = 0;
    calSend = 0;

    log_file.open((std::string("/home/assisi/firmware/log/") + casuName + std::string(".txt")).c_str(), ios::out);

/*
    char out_i2c_buff[20];
    // send initial references
    out_i2c_buff[0] = MSG_REF_VIBE_ID;
    out_i2c_buff[1] =  vibeAmp_r;
    out_i2c_buff[2] = vibeFreq_r & 0x00FF;
    out_i2c_buff[3] = (vibeFreq_r & 0xFF00) >> 8;
    this->mtxi2c_.lock();
    status = i2cPIC.sendData(out_i2c_buff, 4);
    this->mtxi2c_.unlock();
    usleep(1000);

    out_i2c_buff[0] = MSG_REF_TEMP_ID;
    int tmp = temp_ref * 10;
    if (tmp < 0) tmp = tmp + 65536;
    out_i2c_buff[1] = (tmp & 0x00FF);   
    out_i2c_buff[2] = (tmp & 0xFF00) >> 8;
    this->mtxi2c_.lock();
    status = i2cPIC.sendData(out_i2c_buff, 3);
    this->mtxi2c_.unlock();
    usleep(1000);

    out_i2c_buff[0] = MSG_REF_LED_ID;
    out_i2c_buff[1] =  ledDiag_r[0];
    out_i2c_buff[2] =  ledDiag_r[1];
    out_i2c_buff[3] =  ledDiag_r[2];
    this->mtxi2c_.lock();
    status = i2cPIC.sendData(out_i2c_buff, 4);
    this->mtxi2c_.unlock();
    usleep(1000);

*/



}

CASU_Interface::~CASU_Interface() {
    log_file.close();
    //delete ehm_device;
    delete zmqContext;
        //boost::interprocess::named_mutex i2cMuxLock(boost::interprocess::open_or_create, "i2cMuxLock" );
    //i2cMuxLock.unlock();
}

void CASU_Interface::i2cComm() {
        //boost::interprocess::named_mutex i2cMuxLock(boost::interprocess::open_or_create, "i2cMuxLock" );
        //i2cMuxLock.unlock();
	int print_counter = 0 ;
	char str_buff[256] = {0};
	std::stringstream ss;
	gettimeofday(&start_time, NULL);
	double t_msec;
	timeval current_time;
    sprintf(str_buff, "time temp_f temp_r temp_b temp_l temp_pcb temp_top temp_casu temp_wax temp_ref pelt vibeAmp_s vibeFreq_s airfolow_s fanCool \
            proxi_f proxi_fr proxi_br proxi_b proxi_bl proxi_fl \n");
    log_file.write(str_buff, strlen(str_buff));
    log_file.flush();

    while(1) {

                       
        this->mtxi2c_.lock();
        status = i2cPIC.receiveData(inBuff, IN_DATA_NUM);
        this->mtxi2c_.unlock();

        if (status <= 0) {
            cerr << "I2C initialization unsuccessful, exiting thread" << std::endl;
            break;
        }
        else {

            //cout << "Read bytes: " << IN_DATA_NUM << std::endl;

            this->mtxPub_.lock();
            dummy = inBuff[0] | (inBuff[1] << 8);
            if (dummy > 32767)
                temp[T_F] = (dummy - 65536.0) / 10.0;
            else
                temp[T_F] = dummy / 10.0;

            dummy = inBuff[2] | (inBuff[3] << 8);
            if (dummy > 32767)
                temp[T_R] = (dummy - 65536.0) / 10.0;
            else
                temp[T_R] = dummy / 10.0;

            dummy = inBuff[4] | (inBuff[5] << 8);
            if (dummy > 32767)
                temp[T_B] = (dummy - 65536.0) / 10.0;
            else
                temp[T_B] = dummy / 10.0;

            dummy = inBuff[6] | (inBuff[7] << 8);
            if (dummy > 32767)
                temp[T_L] = (dummy - 65536.0) / 10.0;
            else
                temp[T_L] = dummy / 10.0;

            dummy = inBuff[8] | (inBuff[9] << 8);
            if (dummy > 32767)
                temp[T_flexPCB] = (dummy - 65536.0) / 10.0;
            else
                temp[T_flexPCB] = dummy / 10.0;

            dummy = inBuff[10] | (inBuff[11] << 8);
            if (dummy > 32767)
                temp[T_PCB] = (dummy - 65536.0) / 10.0;
            else
                temp[T_PCB] = dummy / 10.0;

            dummy = inBuff[12] | (inBuff[13] << 8);
            if (dummy > 32767)
                tempCasu= (dummy - 65536.0) / 10.0;
            else
                tempCasu = dummy / 10.0;

            dummy = inBuff[14] | (inBuff[15] << 8);
            if (dummy > 32767)
                tempWax = (dummy - 65536.0) / 10.0;
            else
                tempWax = dummy / 10.0;

            dummy = inBuff[16] | (inBuff[17] << 8);
            if (dummy > 32767)
                temp_ref_rec = (dummy - 65536.0) / 10.0;
            else
                temp_ref_rec = dummy / 10.0;

            vAmp[A_F] = (inBuff[18] | (inBuff[19] << 8)) / 10.0;
            vAmp[A_R] = (inBuff[20] | (inBuff[21] << 8)) / 10.0;
            vAmp[A_B] = (inBuff[22] | (inBuff[23] << 8)) / 10.0;
            vAmp[A_L] = (inBuff[24] | (inBuff[25] << 8)) / 10.0;

            vFreq[A_F] = (inBuff[26] | (inBuff[27] << 8)) / 10.0;
            vFreq[A_R] = (inBuff[28] | (inBuff[29] << 8)) / 10.0;
            vFreq[A_B] = (inBuff[30] | (inBuff[31] << 8)) / 10.0;
            vFreq[A_L] = (inBuff[32] | (inBuff[33] << 8)) / 10.0;

            vibeAmp_s = inBuff[34];
            vibeFreq_s = inBuff[35] | (inBuff[36] << 8);

            irRawVals[IR_F] = inBuff[37] | (inBuff[38] << 8);
            irRawVals[IR_FR] = inBuff[39] | (inBuff[40] << 8);
            irRawVals[IR_BR] = inBuff[41] | (inBuff[42] << 8);
            irRawVals[IR_B] = inBuff[43] | (inBuff[44] << 8);
            irRawVals[IR_BL] = inBuff[45] | (inBuff[46] << 8);
            irRawVals[IR_FL] = inBuff[47] | (inBuff[48] << 8);

            ctlPeltier_s = inBuff[49];
            if (ctlPeltier_s > 100) ctlPeltier_s = ctlPeltier_s - 201;

            ledDiag_s[L_R] = inBuff[50];
            ledDiag_s[L_G] = inBuff[51];
            ledDiag_s[L_B] = inBuff[52];

            fanCooler = inBuff[53];
            calRec = inBuff[54];

            this->mtxPub_.unlock();
            gettimeofday(&current_time, NULL);
            t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
            //printf("I2C data processing time stamp %.3f \n", t_msec);
            sprintf(str_buff, "%.2f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d %d %d %d %d %d %d %d %d %d %d \n",
                    t_msec / 1000, temp[T_F], temp[T_R], temp[T_B], temp[T_L], temp[T_PCB], temp[T_flexPCB], tempCasu, tempWax, temp_ref_rec,
                    ctlPeltier_s, vibeAmp_s, vibeFreq_s, airflow_r, fanCooler,
                    irRawVals[IR_F], irRawVals[IR_FR], irRawVals[IR_BR], irRawVals[IR_B], irRawVals[IR_BL], irRawVals[IR_FL]);
            log_file.write(str_buff, strlen(str_buff));
            log_file.flush();
            //gettimeofday(&current_time, NULL);
            //t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
            //printf("I2C flashin to file time stamp %.3f \n", t_msec);
            print_counter++;
            if (print_counter == 13) {
                printf("temp F R B L PCB TOP = ");
                for (int i = 0; i < 6; i++) {
                    printf("%.1f ", temp[i]);
                }
                printf("\n");
                printf("temp Casu Wax Ref = %.1f %.1f %.1f\n", tempCasu, tempWax, temp_ref_rec);

                printf("vibeAmp meas ref = %.1f %d", vAmp[0], vibeAmp_s);
                printf("\n");
                printf("vibeFreq meas ref = %.1f %d", vFreq[0], vibeFreq_s);
                printf("\n");

                printf("ir raw = ");
                for (int i = 0; i < 6; i++) {
                    printf("%d ", irRawVals[i]);
                }
                printf("\n");

                printf("ledDiag = ");
                for (int i = 0; i < 3; i++) {
                    printf("%d ", ledDiag_s[i]);
                }
                printf("\n");

                printf("peltier, airflow, fanCooler = %d %d %d\n", ctlPeltier_s, airflow_r, fanCooler);

                printf("calibration data rec = %d \n", calRec);
                printf("_________________________________________________________________\n\n");
                print_counter = 0;
            }
        }
        usleep(40000); // total loop time 40 + 10 sec (execution of the above code) = 50 sec
        
        if (calRec == 0 || calSend == 0) {
            outBuff[0] = 2;
            outBuff[1] = tempCtlOn;
            int tmp = Kp * 10;
            //printf("Sending temperature %d \n", tmp);
            outBuff[2] = (tmp & 0x00FF);
            outBuff[3] = (tmp & 0xFF00) >> 8;
            tmp = Ki * 1000;
            outBuff[4] = (tmp & 0x00FF);
            outBuff[5] = (tmp & 0xFF00) >> 8;
            tmp = Kf1 * 10000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[6] = (tmp & 0x00FF);
            outBuff[7] = (tmp & 0xFF00) >> 8;
            tmp = Kf2 * 10000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[8] = (tmp & 0x00FF);
            outBuff[9] = (tmp & 0xFF00) >> 8;
            tmp = Kf3 * 10000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[10] = (tmp & 0x00FF);
            outBuff[11] = (tmp & 0xFF00) >> 8;
            outBuff[12] = fanCtlOn;
            
            this->mtxi2c_.lock();
            status = i2cPIC.sendData(outBuff, OUT_CAL_DATA_NUM);
            this->mtxi2c_.unlock();
            
            usleep(20000);
            calSend = 1;
        }
        
        //gettimeofday(&current_time, NULL);
        //t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
        //printf("I2C finishing loop time stamp %.3f \n", t_msec);
    }
}

void CASU_Interface::zmqPub() {

    zmq::socket_t zmqPub(*zmqContext, ZMQ_PUB);
    zmqPub.bind(pub_addr.c_str());
    int range;
    cout << "Starting pub server: " << casuName << endl;

    int temp_clock = 0;
    std::string data;
    AssisiMsg::RangeArray ranges;
    for(int i = 0; i < 7; i++) {
        ranges.add_range(0);
        ranges.add_raw_value(0);
    }
    AssisiMsg::TemperatureArray temps;

    for(int i = 0; i < 8; i++) {
        temps.add_temp(0);
    }

    /* Data publishing loop */
    while (1) {

    timespec actual_time;

        /* Proximity sensor values */
        this->mtxPub_.lock();
        for(int i = 0; i < 7; i++) {
            if (irRawVals[i] > proxyThresh)
                range = 1;
            else
                range = 10;
            ranges.set_range(i, range);
            ranges.set_raw_value(i, irRawVals[i]);

        }
        this->mtxPub_.unlock();
        clock_gettime(CLOCK_REALTIME, &actual_time);
        ranges.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
        ranges.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
        ranges.SerializeToString(&data);
        zmq::send_multipart(zmqPub, casuName.c_str(), "IR", "Ranges", data);
        
        std::string act_state("On");

        /* Temperature and vibration measurements */
        temp_clock++;
        if (temp_clock == 10) {
            // Temperature is published 4 times less often
            this->mtxPub_.lock();
            for(int i = 0; i < 6; i++) {
                temps.set_temp(i, temp[i]);
            }
            temps.set_temp(6, tempCasu);
            temps.set_temp(7, tempWax);
            this->mtxPub_.unlock();
            clock_gettime(CLOCK_REALTIME, &actual_time);
            temps.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
            temps.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
            temps.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Temp", "Temperatures", data);

            /* Vibration measurements */
            // Why is vibration inside the temperature loop?
            AssisiMsg::VibrationReadingArray vibes;
            AssisiMsg::VibrationReading *vibe;

            this->mtxPub_.lock();
            for(int i = 0; i < 4; i++) {
                vibe = vibes.add_reading();
                vibe->add_amplitude(vAmp[i]);
                vibe->add_freq(vFreq[i]);
            }

            this->mtxPub_.unlock();

            clock_gettime(CLOCK_REALTIME, &actual_time);
            vibes.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
            vibes.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
            vibes.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Acc", "Measurements", data);

             /* Peltier actuator setpoint */
            Temperature temp_ref;
            this->mtxPub_.lock();
            temp_ref.set_temp(this->temp_ref_rec);
            //temp_ref.set_temp(ctlPeltier_s);
            if (this->temp_ref_rec < 26) // What is the meaning of 26?
            { 
                act_state = "Off";
            }
            else
            {
                act_state = "On";
            }
            this->mtxPub_.unlock();
        clock_gettime(CLOCK_REALTIME, &actual_time);
        temp_ref.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
        temp_ref.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
            temp_ref.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Peltier", act_state.c_str(), data);

            temp_clock = 0;
        }

        
        /* Speaker actuator setpoint */
        VibrationSetpoint vib_ref;
        this->mtxPub_.lock();
        vib_ref.set_freq(vibeFreq_r);
        vib_ref.set_amplitude(vibeAmp_r);
        if (vibeAmp_r == 0)
        {
            act_state = "Off";
        }
        else
        {
            act_state = "On";
        }
        this->mtxPub_.unlock();
    clock_gettime(CLOCK_REALTIME, &actual_time);
    vib_ref.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
    vib_ref.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
    vib_ref.SerializeToString(&data);
    zmq::send_multipart(zmqPub, casuName.c_str(), "Speaker", act_state.c_str(), data);

    /* Airflow actuator setpoint */
    Airflow air_ref;
    this->mtxPub_.lock();
    air_ref.set_intensity(1);
    if (airflow_r)
    {
        act_state = "On";
    }
    else
    {
        act_state = "Off";
    }
    this->mtxPub_.unlock();
    clock_gettime(CLOCK_REALTIME, &actual_time);
    air_ref.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
    air_ref.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
    air_ref.SerializeToString(&data);
    zmq::send_multipart(zmqPub, casuName.c_str(), "Airflow", act_state.c_str(), data);
    
    /* Diagnostic LED setpoint */
    ColorStamped color_ref;
    this->mtxPub_.lock();        
    color_ref.mutable_color()->set_red(ledDiag_r[L_R]/100.0);
    color_ref.mutable_color()->set_green(ledDiag_r[L_G]/100.0);
    color_ref.mutable_color()->set_blue(ledDiag_r[L_B]/100.0);
    if (ledDiag_r[L_R] || ledDiag_r[L_G] || ledDiag_r[L_B])            
    {
        act_state = "On";
    }
    else
    {
        act_state = "Off";
    }
    this->mtxPub_.unlock();
    clock_gettime(CLOCK_REALTIME, &actual_time);
    color_ref.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
    color_ref.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
    color_ref.SerializeToString(&data);
    zmq::send_multipart(zmqPub, casuName.c_str(), "DiagnosticLed", act_state.c_str(), data);

    usleep(100000);
    }
}

void CASU_Interface::zmqSub()
{


    zmq::socket_t zmqPub_af(*zmqContext, ZMQ_PUB);
    zmqPub_af.connect(pub_addr_af.c_str());

    zmq::socket_t zmqSub(*zmqContext, ZMQ_SUB);
    zmqSub.bind(sub_addr.c_str());

    //zmqSub.connect("tcp://127.0.0.1:5556");
    zmqSub.setsockopt(ZMQ_SUBSCRIBE, casuName.c_str(), 4);

    string name;
    string device;
    string command;
    string data;
    int len;
    char out_i2c_buff[20]; /*!< Buffer for i2c outgoing data.  */
    cout << "Starting sub server: " << casuName << endl;

    while (1) {
        len = zmq::recv_multipart(zmqSub, name, device, command, data);
        
        if (len >= 0) {

            if (device == "DiagnosticLed") {

                if (command == "On")
                {
                    AssisiMsg::ColorStamped color_msg;
                    assert(color_msg.ParseFromString(data));
                    mtxSub_.lock();
                    ledDiag_r[L_R] = 100 * color_msg.color().red();
                    ledDiag_r[L_G] = 100 * color_msg.color().green();
                    ledDiag_r[L_B] = 100 * color_msg.color().blue();
                    mtxSub_.unlock();
                }
                else if (command == "Off")
                {
                    mtxSub_.lock();
                    ledDiag_r[L_R] = 0;
                    ledDiag_r[L_G] = 0;
                    ledDiag_r[L_B] = 0;
                    mtxSub_.unlock();
                }
                else
                {
                    cerr << "Unknown command for " << name << "/" << device << endl;
                }

                //if ((ledDiag_s[0] != ledDiag_r[0]) || (ledDiag_s[1] != ledDiag_r[1]) || (ledDiag_s[2] != ledDiag_r[2])) {
                out_i2c_buff[0] = MSG_REF_LED_ID;
                out_i2c_buff[1] =  ledDiag_r[0];
                out_i2c_buff[2] =  ledDiag_r[1];
                out_i2c_buff[3] =  ledDiag_r[2];
                this->mtxi2c_.lock();
                status = i2cPIC.sendData(out_i2c_buff, 4);
                this->mtxi2c_.unlock();
                //}
            }

            else if (device == "Light") {

                printf("Received Light device message: %s\n \
                       ...Discarding message as we are now longer using light as an actuator", command.data());

            }

            else if (device == "Speaker") {

                printf("Received speaker command: %s", command.data());

                if (command == "On") {

                    AssisiMsg::VibrationSetpoint vibe;
                    assert(vibe.ParseFromString(data));
                    mtxSub_.lock();
                    vibeAmp_r = vibe.amplitude();
                    if (vibeAmp_r > 100)
                        vibeAmp_r = 100;
                    else if (vibeAmp_r < 0)
                        vibeAmp_r = 0;

                    vibeFreq_r = vibe.freq();
                    if (vibeFreq_r > 1500)
                        vibeFreq_r = 1500;
                    else if (vibeFreq_r < 1)
                        vibeFreq_r = 1;
                    mtxSub_.unlock();
                    printf(" Vibe amp freq %d %d \n", vibeAmp_r, vibeFreq_r);

                }
                else if (command == "Off") {
                    mtxSub_.lock();
                    vibeAmp_r = 0;
                    mtxSub_.unlock();
                    printf(" Vibe amp freq %d %d \n", vibeAmp_r, vibeFreq_r);

                }
                else {
                    cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
                }

                out_i2c_buff[0] = MSG_REF_VIBE_ID;
                out_i2c_buff[1] =  vibeAmp_r;
                out_i2c_buff[2] = vibeFreq_r & 0x00FF;
                out_i2c_buff[3] = (vibeFreq_r & 0xFF00) >> 8;
                this->mtxi2c_.lock();
                status = i2cPIC.sendData(out_i2c_buff, 4);
                this->mtxi2c_.unlock();

            }
            else if (device == "EM") {

                printf("Received EM device message: %s\n \
                       ...Discarding message as we are now longer using electro-magnetic emitters", command.data());
            }
            else if (device == "Peltier") {
                printf("Received Peltier device message: %s\n", command.data());
                if (command == "On") {
                    AssisiMsg::Temperature temp_msg;
                    assert(temp_msg.ParseFromString(data));
                    mtxSub_.lock();
                    temp_ref = temp_msg.temp();
                    if (temp_ref < 26)
                        temp_ref = 0;
                    mtxSub_.unlock();
                    printf("Reference temperature %.1f \n", temp_ref);
                }
                else if (command == "Off") {
                    mtxSub_.lock();
                    temp_ref = 0;
                    mtxSub_.unlock();
                }
                else printf("Received unknown temperature command");

                if (abs(temp_ref - temp_ref_rec) > 0.1) {
                    out_i2c_buff[0] = MSG_REF_TEMP_ID;
                    int tmp = temp_ref * 10;
                    if (tmp < 0) tmp = tmp + 65536;
                    out_i2c_buff[1] = (tmp & 0x00FF);   
                    out_i2c_buff[2] = (tmp & 0xFF00) >> 8;
                    this->mtxi2c_.lock();
                    status = i2cPIC.sendData(out_i2c_buff, 3);
                    this->mtxi2c_.unlock();
                }
            }
            else if (device == "Airflow") {
                printf("Received Airflow message: %s\n", command.data());
                string valveCommand = casuName.substr(6,2);;
                if (command == "On") {
                    //AssisiMsg::Airflow air_msg;
                    //assert(air_msg.ParseFromString(data));
                    mtxSub_.lock();
                    airflow_r = 1;
                    mtxSub_.unlock();
                    valveCommand.insert(0, "1");
                    //printf("Airflow ON");
                }
                else if (command == "Off") {
                    mtxSub_.lock();
                    airflow_r = 0;
                    mtxSub_.unlock();
                    valveCommand.insert(0, "0");
                    //printf("Airflow OFF");
                }
                zmq::message_t valve_msg;
                str_to_msg(valveCommand.c_str(), valve_msg);
                //zmqPub_af.send(valve_msg);
                zmq::send_multipart(zmqPub_af, casuName.c_str(), device, command, data);
            }
            
            else if (device == "IR") {
                out_i2c_buff[0] = MSG_REF_PROXY_ID;
                
                if (command == "Standby") {
                    out_i2c_buff[1] = 1;
                }
                else if (command == "Activate") {
                    out_i2c_buff[1] = 0;                    
                }

                this->mtxi2c_.lock();
                status = i2cPIC.sendData(out_i2c_buff, 2);
                this->mtxi2c_.unlock();             
            }
            else
            {
                cerr << "Unknown device " << device << endl;
            }

        }

        fflush(stdout);
    }
}
