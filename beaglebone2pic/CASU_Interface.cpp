/*
 * BBBRouter.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#include "CASU_Interface.h"
#include <sys/time.h>
#include <climits>
#include <dirent.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

// Protobuf autogenerated message files
#include "dev_msgs.pb.h"

using namespace std;
using namespace AssisiMsg;
using namespace boost::posix_time;
using namespace boost::asio;

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
    i2c_connector = fbc["i2c_connector"].as<int>();
    Kp = fbc["Kp"].as<float>();
    Ki = fbc["Ki"].as<float>();
    Kf1 = fbc["Kf1"].as<float>();
    Kf2 = fbc["Kf2"].as<float>();
    Kf3 = fbc["Kf3"].as<float>();
    fanCtlOn = fbc["fanCtlOn"].as<int>();
    // SMC parameters
    controller_type = fbc["controller_type"].as<int>(); // 0 = PID, 1 = SMC
    C1_sigma = fbc["C1_sigma"].as<float>();
    C2_sigma_m = fbc["C2_sigma_m"].as<float>();
    K1_alpha = fbc["K1_alpha"].as<float>();
    K2_beta = fbc["K2_beta"].as<float>();
    epsilon = fbc["epsilon"].as<float>();
    alphak1 = fbc["alphak1"].as<float>();
    //mux.initI2C(2, 112);
    //mux.writeByte(0, 0xFF);

    i2cPIC.initI2C(i2c_bus, picAddress, i2c_connector);
    zmqContext = new zmq::context_t(3);
    ledDiag_r[L_R] = 0;
    ledDiag_r[L_G] = 0;
    ledDiag_r[L_B] = 0;

    // Initialize raw IR reading values
    for (int i = 0; i <= IR_FR; i++)
    {
        irRawVals[i] = 0;
    }

    // Initialize Temp reading and estimate values
    for (int i = 0; i <= T_WAX; i++)
    {
        temp[i] = 0.0;
    }

    for (int i = 0; i <= A_R; i++)
    {
        vAmp[i] = 0.0;
        vFreq[i] = 0.0;
    }

    vibeAmp_r = 0;
    vibeFreq_r = 1;

    vibeAmp_s = 0;
    vibeFreq_s = 1;

    vibration_on = false;

    vibe_periods.push_back(1000);
    vibe_freqs.push_back(1.0);
    vibe_amps.push_back(0);
    vibe_pattern_idx = 0;
    vibe_pattern_on = false;

    airflow_r = 0;
    airflow_s = 0;

    fanCooler = 0;

    temp_ref = 0.0;
    temp_ref_rec = 0.0;

    calRec = 0;
    calSend = 0;

    // Setup logging
    // Move old logs to arhive folder ~/firmware/old-log
    DIR *dir;
    class dirent *ent;
    class stat st;
    const string casuName_begin = "casu-0";
    int count_log_files = 0;
    int count_old_log_files = 0;
    string old_date_time;

    dir = opendir("/home/assisi/firmware/log");
    while ((ent = readdir(dir)) != NULL) {
        const string file_name = ent->d_name;
        const string full_file_name = "/home/assisi/firmware/log/" + file_name;
        // not the right file
        if (file_name[0] == '.')
            continue;
        if (stat(full_file_name.c_str(), &st) == -1)
            continue;
        const bool is_directory = (st.st_mode & S_IFDIR) != 0;
        if (is_directory)
            continue;
        // found the log file
        if (file_name.compare(0, casuName.length(), casuName) == 0) {
            // move to ~/firmware/old-log by renaming
            if (rename (full_file_name.c_str(), (string("/home/assisi/firmware/old-log/") + file_name).c_str())) {
                cerr << "Could not move file to old-log folder" << endl;
                if (errno == EXDEV) {
                    cerr << "Invalid path for moving log file" << endl;
               } else {
                   cerr << "Error renaming (moving) old log files" << endl;
               }
            }
            else {
                // log file moved to ~/firmware/old-log
                count_log_files++;
                size_t begin_datetime, len_datetime;
                begin_datetime = casuName.length();
                len_datetime = file_name.length() - string(".txt").length() - begin_datetime;
                old_date_time = file_name.substr(begin_datetime, len_datetime);
            }
        }
        else {
            if (file_name.compare(0, casuName_begin.length(), casuName_begin) == 0) {
                count_log_files++;
            }
            continue;
        }
    }
    closedir(dir);

    // check if all files moved
    // if yes --> archive

    dir = opendir("/home/assisi/firmware/old-log");
    while ((ent = readdir(dir)) != NULL) {
        const string file_name = ent->d_name;
        const string full_file_name = "/home/assisi/firmware/old-log/" + file_name;
        // not the right file
        if (file_name[0] == '.')
            continue;
        if (stat(full_file_name.c_str(), &st) == -1)
            continue;
        const bool is_directory = (st.st_mode & S_IFDIR) != 0;
        if (is_directory)
            continue;
        // found the log file
        if (file_name.compare(0, casuName_begin.length(), casuName_begin) == 0) {
            count_old_log_files++;
        }
        else {
            continue;
        }
    }

    closedir(dir);

    if (count_old_log_files == count_log_files) {
        // tar
        string systemtar = "bash -c 'tar -C /home/assisi/firmware/old-log/ -czvf /home/assisi/firmware/archives/old-log" + old_date_time + ".tar.gz $(ls /home/assisi/firmware/old-log/) &>> /home/assisi/errorlog.txt'";
        //string systemtar = "bash -c 'zip -r /home/assisi/firmware/archives/old-log-" + old_date_time + ".zip /home/assisi/firmware/old-log &>> /home/assisi/errorlog.txt'";
        system(systemtar.c_str());
        systemtar = "bash -c 'chmod 666 /home/assisi/firmware/old-log/casu*'";
        system(systemtar.c_str());
        systemtar = "bash -c 'rm /home/assisi/firmware/old-log/casu*'";
        system(systemtar.c_str());
    }
    // Open new log file with date time in name
    ptime t_now(second_clock::local_time());
    std::string date_time_now = to_iso_string(t_now.date()) + "_" + to_iso_string(t_now.time_of_day());
    string log_file_name = "/home/assisi/firmware/log/" + casuName + "-" + date_time_now + ".txt";

    log_file.open(log_file_name.c_str(), ios::out);


    // Default period for checking for vibration pattern setpoints
    // is 1 second.
    timer_vp.reset(new deadline_timer(io, seconds(1)));

}

CASU_Interface::~CASU_Interface() {
    log_file.close();
    //delete ehm_device;
    delete zmqContext;
        //boost::interprocess::named_mutex i2cMuxLock(boost::interprocess::open_or_create, "i2cMuxLock" );
    //i2cMuxLock.unlock();
}

void CASU_Interface::run()
{
    boost::thread_group threads;
    threads.create_thread(boost::bind(&CASU_Interface::periodic_jobs, this));
    threads.create_thread(boost::bind(&CASU_Interface::i2cComm, this));
    threads.create_thread(boost::bind(&CASU_Interface::zmqPub, this));
    threads.create_thread(boost::bind(&CASU_Interface::zmqSub, this));
    threads.join_all();
}

void CASU_Interface::i2cComm() {
        //boost::interprocess::named_mutex i2cMuxLock(boost::interprocess::open_or_create, "i2cMuxLock" );
        //i2cMuxLock.unlock();
    int print_counter = 0;
    char str_buff[256] = {0};
    char msg_request_code[1] = {0};
    std::stringstream ss;
    gettimeofday(&start_time, NULL);
    double t_msec;
    timeval current_time;
    sprintf(str_buff, "time temp_f temp_r temp_b temp_l temp_top temp_pcb temp_casu temp_wax temp_ref pelt vibeAmp_s vibeFreq_s airfolow_s fanCool \
            proxi_f proxi_fr proxi_br proxi_b proxi_bl proxi_fl flag_error flag_count\n");
    log_file.write(str_buff, strlen(str_buff));
    log_file.flush();

    gettimeofday(&tOutputStart, NULL);

    // First loop slow to get temperature values
    int flag_first_loop_run = 1;

    while(1) {
        gettimeofday(&tLoopStart, NULL);

        // Print output every 1s; slow msg
        gettimeofday(&tOutputCurrent, NULL);
        elapsedTime = (tOutputCurrent.tv_sec - tOutputStart.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (tOutputCurrent.tv_usec - tOutputStart.tv_usec) / 1000.0;   // us to ms
        if ((elapsedTime > 1000) || (flag_first_loop_run == 1)) {

            //if (flag_first_loop_run == 1) cout << " first loop run!!!! " << std::endl;
            flag_first_loop_run = 0;
            // Request large message
            msg_request_code[0] = MSG_MEASUREMENT_SLOW_ID;
            this->mtxi2c_.lock();
            status = i2cPIC.sendData(msg_request_code, 2);
            this->mtxi2c_.unlock();

            this->mtxi2c_.lock();
            status = i2cPIC.receiveData(inBuff, IN_DATA_NUM_SLOW);
            this->mtxi2c_.unlock();

            if (status <= 0) {
                cerr << "I2C initialization unsuccessful, exiting thread" << std::endl;
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
                    temp[T_TOP] = (dummy - 65536.0) / 10.0;
                else
                    temp[T_TOP] = dummy / 10.0;

                dummy = inBuff[10] | (inBuff[11] << 8);
                if (dummy > 32767)
                    temp[T_PCB] = (dummy - 65536.0) / 10.0;
                else
                    temp[T_PCB] = dummy / 10.0;

                dummy = inBuff[12] | (inBuff[13] << 8);
                if (dummy > 32767)
                    temp[T_RING]= (dummy - 65536.0) / 10.0;
                else
                    temp[T_RING] = dummy / 10.0;

                dummy = inBuff[14] | (inBuff[15] << 8);
                if (dummy > 32767)
                    temp[T_WAX] = (dummy - 65536.0) / 10.0;
                else
                    temp[T_WAX] = dummy / 10.0;

                dummy = inBuff[16] | (inBuff[17] << 8);
                if (dummy > 32767)
                    temp_ref_rec = (dummy - 65536.0) / 10.0;
                else
                    temp_ref_rec = dummy / 10.0;

                vAmp[A_F] = (inBuff[18] | (inBuff[19] << 8)) / 10.0;
                vAmp[A_R] = (inBuff[20] | (inBuff[21] << 8)) / 10.0;
                vAmp[A_B] = (inBuff[22] | (inBuff[23] << 8)) / 10.0;
                vAmp[A_L] = (inBuff[24] | (inBuff[25] << 8)) / 10.0;

                vFreq[A_F] = (inBuff[26] | (inBuff[27] << 8));
                vFreq[A_R] = (inBuff[28] | (inBuff[29] << 8));
                vFreq[A_B] = (inBuff[30] | (inBuff[31] << 8));
                vFreq[A_L] = (inBuff[32] | (inBuff[33] << 8));

                vibeAmp_s = inBuff[34];
                vibeFreq_s = inBuff[35] | (inBuff[36] << 8);

                irRawVals[IR_F] = inBuff[37] | (inBuff[38] << 8);
                irRawVals[IR_FR] = inBuff[39] | (inBuff[40] << 8);
                irRawVals[IR_BR] = inBuff[41] | (inBuff[42] << 8);
                irRawVals[IR_B] = inBuff[43] | (inBuff[44] << 8);
                irRawVals[IR_BL] = inBuff[45] | (inBuff[46] << 8);
                irRawVals[IR_FL] = inBuff[47] | (inBuff[48] << 8);

                ctlPeltier_s = (inBuff[49] | (inBuff[50] << 8)) / 100.0;
                if (ctlPeltier_s > 100) ctlPeltier_s = ctlPeltier_s - 201.0;

                ledDiag_s[L_R] = inBuff[51];
                ledDiag_s[L_G] = inBuff[52];
                ledDiag_s[L_B] = inBuff[53];

                fanCooler = inBuff[54];
                calRec = inBuff[55];
                this->mtxPub_.unlock();

                printf("temp model alpha sigma sigma_m TOP PCB RING WAX ref= ");
                for (int i = 0; i < 8; i++) {
                    printf("%.1f ", temp[i]);
                }
                printf("%.1f\n", temp_ref_rec);

                printf("vibeAmp meas_max ref = %.1f %.1f %.1f %.1f %d", vAmp[A_F], vAmp[A_R], vAmp[A_B], vAmp[A_L], vibeAmp_s);
                printf("\n");
                printf("vibeFreq meas_max ref = %.1f %.1f %.1f %.1f %d", vFreq[A_F], vFreq[A_R], vFreq[A_B], vFreq[A_L], vibeFreq_s);
                printf("\n");

                printf("ir raw F FL BL B BR FR = ");
                for (int i = 0; i < 6; i++) {
                    printf("%d ", irRawVals[i]);
                }
                printf("\n");

                printf("ledDiag = ");
                for (int i = 0; i < 3; i++) {
                    printf("%d ", ledDiag_s[i]);
                }
                printf("\n");

                printf("peltier, airflow, fanCooler = %.2f %d %d\n", ctlPeltier_s, airflow_r, fanCooler);

                printf("calibration data rec = %d \n", calRec);
                printf("_________________________________________________________________\n\n");

                gettimeofday(&tOutputStart, NULL);
            }
        }
        else {

            // Request smaller message
            msg_request_code[0] = MSG_MEASUREMENT_FAST_ID;
            this->mtxi2c_.lock();
            status = i2cPIC.sendData(msg_request_code, 2);
            this->mtxi2c_.unlock();

            this->mtxi2c_.lock();
            status = i2cPIC.receiveData(inBuff, IN_DATA_NUM_FAST);
            this->mtxi2c_.unlock();

            if (status <= 0) {
                cerr << "I2C initialization unsuccessful, exiting thread" << std::endl;
            }
            else {
                //cout << "Read bytes: " << IN_DATA_NUM_FAST << std::endl;

                this->mtxPub_.lock();

                vAmp[A_F] = (inBuff[0] | (inBuff[1] << 8)) / 10.0;
                vAmp[A_R] = (inBuff[2] | (inBuff[3] << 8)) / 10.0;
                vAmp[A_B] = (inBuff[4] | (inBuff[5] << 8)) / 10.0;
                vAmp[A_L] = (inBuff[6] | (inBuff[7] << 8)) / 10.0;

    //            vFreq[A_F] = (inBuff[26] | (inBuff[27] << 8)) / 10.0;
    //            vFreq[A_R] = (inBuff[28] | (inBuff[29] << 8)) / 10.0;
                vFreq[A_F] = (inBuff[8] | (inBuff[9] << 8));
                vFreq[A_R] = (inBuff[10] | (inBuff[11] << 8));
                vFreq[A_B] = (inBuff[12] | (inBuff[13] << 8));
                vFreq[A_L] = (inBuff[14] | (inBuff[15] << 8));;

                vibeAmp_s = inBuff[16];
                vibeFreq_s = inBuff[17] | (inBuff[18] << 8);

                irRawVals[IR_F] = inBuff[19] | (inBuff[20] << 8);
                irRawVals[IR_FR] = inBuff[21] | (inBuff[22] << 8);
                irRawVals[IR_BR] = inBuff[23] | (inBuff[24] << 8);
                irRawVals[IR_B] = inBuff[25] | (inBuff[26] << 8);
                irRawVals[IR_BL] = inBuff[27] | (inBuff[28] << 8);
                irRawVals[IR_FL] = inBuff[29] | (inBuff[30] << 8);

                ledDiag_s[L_R] = inBuff[31];
                ledDiag_s[L_G] = inBuff[32];
                ledDiag_s[L_B] = inBuff[33];

                fanCooler = inBuff[34];
                calRec = inBuff[35];
                this->mtxPub_.unlock();
            }
        }

        gettimeofday(&current_time, NULL);
        t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
        //printf("I2C data processing time stamp %.3f \n", t_msec);
        sprintf(str_buff, "%.2f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.2f %d %d %d %d %d %d %d %d %d %d %.1f %.1f\n",
                t_msec / 1000, temp[T_F], temp[T_L], temp[T_B], temp[T_R], temp[T_TOP], temp[T_PCB], temp[T_RING], temp[T_WAX], temp_ref_rec,
                ctlPeltier_s, vibeAmp_s, vibeFreq_s, airflow_r, fanCooler,
                irRawVals[IR_F], irRawVals[IR_FL], irRawVals[IR_BL], irRawVals[IR_B], irRawVals[IR_BR], irRawVals[IR_FR],
                vFreq[A_F], vAmp[A_F]);
        log_file.write(str_buff, strlen(str_buff));
        log_file.flush();

        if (calRec == 0 || calSend == 0) {
            outBuff[0] = MSG_CAL_ID;
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

            outBuff[13] = controller_type;

            tmp = C1_sigma * 1000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[14] = (tmp & 0x00FF);
            outBuff[15] = (tmp & 0xFF00) >> 8;
            tmp = C2_sigma_m * 1000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[16] = (tmp & 0x00FF);
            outBuff[17] = (tmp & 0xFF00) >> 8;
            tmp = K1_alpha * 10000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[18] = (tmp & 0x00FF);
            outBuff[19] = (tmp & 0xFF00) >> 8;
            tmp = K2_beta * 10000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[20] = (tmp & 0x00FF);
            outBuff[21] = (tmp & 0xFF00) >> 8;
            tmp = epsilon * 1000;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[22] = (tmp & 0x00FF);
            outBuff[23] = (tmp & 0xFF00) >> 8;
            tmp = alphak1 * 100;
            if (tmp < 0) tmp = tmp + 65536;
            outBuff[24] = (tmp & 0x00FF);
            outBuff[25] = (tmp & 0xFF00) >> 8;

            this->mtxi2c_.lock();
            status = i2cPIC.sendData(outBuff, OUT_CAL_DATA_NUM);
            this->mtxi2c_.unlock();

            calSend = 1;
        }

        gettimeofday(&tLoopCurrent, NULL);

        elapsedTime = (tLoopCurrent.tv_sec - tLoopStart.tv_sec) * 1000.0;       // sec to ms
        elapsedTime += (tLoopCurrent.tv_usec - tLoopStart.tv_usec) / 1000.0;   // us to ms

        //printf("Elapsed time: %f \n", elapsedTime);

        if (elapsedTime < I2C_COMM_LOOP_TIME)
            usleep((I2C_COMM_LOOP_TIME - elapsedTime) * 1000.0); //sleep in us for while loop to last I2C_COMM_LOOP_TIME ms

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
    int fft_clock = 0;
    std::string data;
    AssisiMsg::RangeArray ranges;
    for(int i = 0; i <= IR_FR; i++) {
        ranges.add_range(0);
        ranges.add_raw_value(0);
    }
    AssisiMsg::TemperatureArray temps;

    for(int i = 0; i <= T_WAX; i++) {
        temps.add_temp(0);
    }

    /* Data publishing loop */
    while (1) {

        timespec actual_time;

        /* Proximity sensor values */
        this->mtxPub_.lock();
        for(int i = 0; i <= IR_FR; i++) {
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
            // Temperature is published 10 times less often
            this->mtxPub_.lock();
            for(int i = 0; i <= T_WAX; i++) {
                temps.set_temp(i, temp[i]);
            }
            this->mtxPub_.unlock();
            clock_gettime(CLOCK_REALTIME, &actual_time);
            temps.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
            temps.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
            temps.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Temp", "Temperatures", data);

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

        /* FFt vibration measurements */
        fft_clock++;
        if (fft_clock == 5) {
            // FFT is published 5 times less often

            AssisiMsg::VibrationReadingArray vibes;
            AssisiMsg::VibrationReading *vibe;

            this->mtxPub_.lock();
            vibe = vibes.add_reading();
            for(int i = 0; i < IN_DATA_NUM_ACC; i++) {
                vibe->add_amplitude(vAmp[i]);
                vibe->add_freq(vFreq[i]);
            }

            this->mtxPub_.unlock();

            clock_gettime(CLOCK_REALTIME, &actual_time);
            vibes.mutable_header()->mutable_stamp()->set_sec(actual_time.tv_sec);
            vibes.mutable_header()->mutable_stamp()->set_nsec(actual_time.tv_nsec);
            vibes.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Fft", "Measurements", data);

            fft_clock = 0;
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

        /* Vibration pattern setpoint */
        VibrationPattern vibe_pattern;
        this->mtxPub_.lock();
        for (int i = 0; i < vibe_periods.size(); i++)
        {
            vibe_pattern.add_vibe_periods(vibe_periods[i]);
            vibe_pattern.add_vibe_freqs(vibe_freqs[i]);
            vibe_pattern.add_vibe_amps(vibe_amps[i]);
        }
        if (vibe_pattern_on)
        {
            act_state = "On";
        }
        else
        {
            act_state = "Off";
        }
        this->mtxPub_.unlock();
        this->set_msg_header(vibe_pattern.mutable_header());
        vibe_pattern.SerializeToString(&data);
        zmq::send_multipart(zmqPub, casuName.c_str(), "VibrationPattern",
                            act_state.c_str(), data);

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
            else if (device == "Speaker") {

                printf("Received speaker command: %s", command.data());

                if (command == "On")
                {
                    AssisiMsg::VibrationSetpoint vibe;
                    assert(vibe.ParseFromString(data));
                    // We've received a vibration setpoint
                    // This cancels the pattern that might be running
                    vibe_pattern_on = false;
                    this->set_vibration(vibe.freq(),vibe.amplitude());
                }
                else if (command == "Off") {
                    stop_vibration();
                }
                else {
                    cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
                }

            }
            else if (device == "VibrationPattern")
            {
                AssisiMsg::VibrationPattern vp;

                if (command == "On")
                {
                    assert(vp.ParseFromString(data));
                    // Check that the sizes of all fields match
                    assert((vp.vibe_periods_size() == vp.vibe_freqs_size())
                           && (vp.vibe_freqs_size() == vp.vibe_amps_size()));
                    // Unpack values
                    mtxSub_.lock();
                    vibe_periods.clear();
                    vibe_freqs.clear();
                    vibe_amps.clear();
                    for (int i = 0; i < vp.vibe_periods_size(); i++)
                    {
                        vibe_periods.push_back(clamp(vp.vibe_periods(i),VIBE_PATTERN_PERIOD_MIN,UINT_MAX));
                        vibe_freqs.push_back(vp.vibe_freqs(i));
                        vibe_amps.push_back(vp.vibe_amps(i));
                    }
                    vibe_pattern_idx = 0;
                    vibe_pattern_on = true;
                    mtxSub_.unlock();
                    /*
                      We're not explicitly starting the vibration pattern.
                      Instead, we rely on the update rate of timer_vp,
                      which is currently set to 1s.
                      This simplifies our code, but can be changed if necessary.
                    */
                }
                else
                {
                    cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
                }
                mtxSub_.unlock();
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

void CASU_Interface::set_vibration(double freq, double amp)
{
    mtxSub_.lock();
    vibeFreq_r = clamp(freq, 1.0, VIBE_FREQ_MAX);
    vibeAmp_r = clamp(static_cast<unsigned>(amp), 0U, VIBE_AMP_MAX);
    // This check is necessary because of a possible race condition
    // between update_vibration_pattern() and stop_vibration()
    if (vibeAmp_r > 0)
    {
        vibration_on = true;
    }
    else
    {
        vibration_on = false;
    }
    mtxSub_.unlock();

    // Send vibration command over i2c
    char out_i2c_buff[4];
    out_i2c_buff[0] = MSG_REF_VIBE_ID;
    out_i2c_buff[1] =  vibeAmp_r;
    out_i2c_buff[2] = vibeFreq_r & 0x00FF;
    out_i2c_buff[3] = (vibeFreq_r & 0xFF00) >> 8;
    this->mtxi2c_.lock();
    status = i2cPIC.sendData(out_i2c_buff, 4);
    this->mtxi2c_.unlock();
}

void CASU_Interface::stop_vibration()
{
    mtxSub_.lock();
    // Stop the vibrations
    vibeAmp_r = 0;
    vibeFreq_r = 1;
    vibration_on = false;
    // Also stop the vibration pattern.
    vibe_pattern_on = false;
    vibe_periods.clear(); vibe_periods.push_back(1000);
    vibe_freqs.clear(); vibe_freqs.push_back(1.0);
    vibe_amps.clear(); vibe_amps.push_back(0);
    vibe_pattern_idx = 0;
    mtxSub_.unlock();

    // Send vibration command over i2c
    char out_i2c_buff[4];
    out_i2c_buff[0] = MSG_REF_VIBE_ID;
    out_i2c_buff[1] =  vibeAmp_r;
    out_i2c_buff[2] = vibeFreq_r & 0x00FF;
    out_i2c_buff[3] = (vibeFreq_r & 0xFF00) >> 8;
    this->mtxi2c_.lock();
    status = i2cPIC.sendData(out_i2c_buff, 4);
    this->mtxi2c_.unlock();
}

void CASU_Interface::update_vibration_pattern()
{
    mtxSub_.lock();
    if (vibe_pattern_on)
    {
        vibeFreq_r = vibe_freqs[vibe_pattern_idx];
        vibeAmp_r = vibe_amps[vibe_pattern_idx];
        timer_vp->expires_from_now(milliseconds(vibe_periods[vibe_pattern_idx]));
        vibe_pattern_idx++;
        if (vibe_pattern_idx >= vibe_freqs.size())
        {
            vibe_pattern_idx = 0;
        }
    }
    else
    {

        timer_vp->expires_from_now(seconds(1));
    }
    mtxSub_.unlock();
    // This call is a bit inelegant, becase set_vibration already
    // has access to vibeFreq_r and vibeAmp_r
    // Should reconsider the design of set_vibration
    // Care should be taken to avoid possible deadlock/race conditions
    set_vibration(vibeFreq_r, vibeAmp_r);
    timer_vp->async_wait(boost::bind(&CASU_Interface::update_vibration_pattern,this));
}

void CASU_Interface::set_msg_header(AssisiMsg::Header* header)
{
    timespec actual_time;
    clock_gettime(CLOCK_REALTIME, &actual_time);
    header->mutable_stamp()->set_sec(actual_time.tv_sec);
    header->mutable_stamp()->set_nsec(actual_time.tv_nsec);
}

void CASU_Interface::periodic_jobs()
{
    // We'll just set up the necessary timers and
    // run the boost::asio::io_service loop
    timer_vp->async_wait(boost::bind(&CASU_Interface::update_vibration_pattern,this));
    io.run();
}

/* static */ const double CASU_Interface::VIBE_FREQ_MAX = 1500.0;
/* static */ const unsigned CASU_Interface::VIBE_AMP_MAX = 50;
/* static */ const unsigned CASU_Interface::VIBE_PATTERN_PERIOD_MIN = 100;
