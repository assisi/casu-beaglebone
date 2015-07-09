/*
 * BBBRouter.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#include "CASU_Interface.h"

using namespace std;

CASU_Interface::CASU_Interface(char *fbc_file)
	{

    YAML::Node fbc = YAML::LoadFile(fbc_file);
    casuName = fbc["name"].as<string>();
    pub_addr = fbc["pub_addr"].as<string>();
    sub_addr = fbc["sub_addr"].as<string>();
    i2c_bus = fbc["i2c_bus"].as<int>();
    pic1Address = fbc["pic1_addr"].as<int>();
    pic2Address = fbc["pic2_addr"].as<int>();
    tempCtlOn = fbc["tempCtlOn"].as<int>();
    Kp = fbc["Kp"].as<float>();
    Ki = fbc["Ki"].as<float>();
    Kf1 = fbc["Kf1"].as<float>();
    Kf2 = fbc["Kf2"].as<float>();
    Kf3 = fbc["Kf3"].as<float>();
    fanCtlOn = fbc["fanCtlOn"].as<int>();

    i2cPIC1.initI2C(i2c_bus, pic1Address);

    if (pic2Address != 0) {
        // we are using two dspic boards, init the second
        i2cPIC2.initI2C(i2c_bus, pic2Address);
    }
	zmqContext = new zmq::context_t(2);
	ledDiag_r[L_R] = 0;
	ledDiag_r[L_G] = 0;
	ledDiag_r[L_B] = 0;

	ledCtl_r[L_R] = 0;
	ledCtl_r[L_G] = 0;
	ledCtl_r[L_B] = 0;

	irRawVals[IR_F] = 0;
	irRawVals[IR_FR] = 0;
	irRawVals[IR_BR] = 0;
	irRawVals[IR_B] = 0;
	irRawVals[IR_BL] = 0;
	irRawVals[IR_FL] = 0;
	irRawVals[IR_T] = 0;

	temp[0] = 100;
	temp[1] = 101;
	temp[2] = 102;
	temp[3] = 103;
	temp[4] = 104;
    tempWax = -1;
    tempCasu = -1;

	vAmp[0] = 0.1;
	vAmp[1] = 0.2;
	vAmp[2] = 0.3;
	vAmp[3] = 0.4;

	vFreq[0] = 10.0;
	vFreq[1] = 20.0;
	vFreq[2] = 30.0;
	vFreq[3] = 40.0;

	pwmMotor_r = 0;

    speakerAmp_r = 0;
    speakerAmp_s = 0;
    speakerFreq_r = 0;
    speakerFreq_s = 0;

    airflow_r = 0;
    airflow_s = 0;

    fanCooler = 0;

    temp_ref = 25.0;
    temp_ref_rec = 0.0;
    temp_ref_cur = 0.0;

    calRec = 0;
    calSend = 0;

	proxyThresh = 4000;

    // EHM device configuration
    // We are no longer using electro-magnetic emitters
    //ehm_device = new EHM("/dev/ttyACM0", 9600);
    ehm_freq_electric = 0;
    ehm_freq_magnetic = 0;
    ehm_temp = 0;

	/*Scale freq to motor pwm
	 * From datasheet 3V > 516 Hz
	 * u = K * f; K = 3 / 516 = 0.0058
	 * pwm = u / 3.3 * 100 = 30.303
	 * pwm = k2 * f = 0.0058*30.303 = 0.1758
	* */
	vibeMotorConst = 0.1758;

    log_file.open((std::string("/home/assisi/firmware/log/") + casuName + std::string(".txt")).c_str(), ios::out);
}

CASU_Interface::~CASU_Interface() {
	log_file.close();
    //delete ehm_device;
	delete zmqContext;
}

void CASU_Interface::i2cComm() {
	int print_counter = 0 ;
    int ref;
    int ref_conf_main;
    int ref_conf_aux;
	char str_buff[256] = {0};
	std::stringstream ss;
	gettimeofday(&start_time, NULL);
	timeval current_time;
    sprintf(str_buff, "time temp_f temp_r temp_b temp_l temp_pcb temp_casu temp_wax temp_ref pelt mot fanAir fanCool \
            proxi_f proxi_fr proxi_br proxi_b proxi_bl proxi_fl \n");
	log_file.write(str_buff, strlen(str_buff));
	while(1) {

        for (int i = 0; i < 8; i++) {
            this->mtxSub_.lock();
            ref = newRef;
            newRef = 0;
            ref_conf_main = (pwmMotor_r != pwmMotor_s)
                    || (ledDiag_r[0] != ledDiag_s[0] || ledDiag_r[1] != ledDiag_s[1] || ledDiag_r[2] != ledDiag_s[2])
                    || (ledCtl_r[0] != ledCtl_s[0] || ledCtl_r[1] != ledCtl_s[1] || ledCtl_r[2] != ledCtl_s[2])
                    || (airflow_r != airflow_s)
                    || (abs(temp_ref - temp_ref_rec) < 0.1);
            ref_conf_aux = (speakerAmp_r != speakerAmp_s) || (speakerFreq_r != speakerFreq_s);
            mtxSub_.unlock();

            if (ref == 1 || ref_conf_main == 1) {
                this->mtxSub_.lock();
                outBuff[0] = 3;
                int tmp = temp_ref * 10;
                if (tmp < 0) tmp = tmp + 65536;
                //printf("Sending temperature %d \n", tmp);
                outBuff[1] = (tmp & 0x00FF);
                outBuff[2] = (tmp & 0xFF00) >> 8;

                outBuff[3] = pwmMotor_r & 0x00FF;
                outBuff[4] = (pwmMotor_r & 0xFF00) >> 8;

                outBuff[5] = ledCtl_r[0];
                outBuff[6] = ledCtl_r[1];
                outBuff[7] = ledCtl_r[2];

                outBuff[8] = ledDiag_r[0];
                outBuff[9] = ledDiag_r[1];
                outBuff[10] = ledDiag_r[2];

                outBuff[11] = airflow_r;
                this->mtxSub_.unlock();
                status = i2cPIC1.sendData(outBuff, OUT1_REF_DATA_NUM);
            }

            if (pic2Address != 0 && (ref == 1 || ref_conf_aux == 1)) {
                this->mtxSub_.lock();
                outBuff[0] = 3;
                outBuff[1] = speakerFreq_r & 0x00FF;
                outBuff[2] = (speakerFreq_r & 0xFF00) >> 8;
                outBuff[3] = speakerAmp_r & 0x00FF;
                outBuff[4] = (speakerAmp_r & 0xFF00) >> 8;
                this->mtxSub_.unlock();
                status = i2cPIC2.sendData(outBuff, OUT2_REF_DATA_NUM);
            }
            usleep(10000);
            // total 8 * 10 ms = 80 msec
        }

        status = i2cPIC1.receiveData(inBuff, IN1_DATA_NUM);

		if (status <= 0) {
            printf("Failed to receive data from the main MCU");
            cerr << "Main I2C initialization unsuccessful, exiting thread" << std::endl;
			break;
		}
		else {

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
				temp[T_T] = (dummy - 65536.0) / 10.0;
			else
				temp[T_T] = dummy / 10.0;

			vAmp[A_F] = (inBuff[10] | (inBuff[11] << 8)) / 10.0;
			vAmp[A_R] = (inBuff[12] | (inBuff[13] << 8)) / 10.0;
			vAmp[A_B] = (inBuff[14] | (inBuff[15] << 8)) / 10.0;
			vAmp[A_L] = (inBuff[16] | (inBuff[17] << 8)) / 10.0;

			vFreq[A_F] = (inBuff[18] | (inBuff[19] << 8)) / 10.0;
			vFreq[A_R] = (inBuff[20] | (inBuff[21] << 8)) / 10.0;
			vFreq[A_B] = (inBuff[22] | (inBuff[23] << 8)) / 10.0;
			vFreq[A_L] = (inBuff[24] | (inBuff[25] << 8)) / 10.0;

			irRawVals[IR_F] = inBuff[26] | (inBuff[27] << 8);
			irRawVals[IR_FR] = inBuff[28] | (inBuff[29] << 8);
			irRawVals[IR_BR] = inBuff[30] | (inBuff[31] << 8);
			irRawVals[IR_B] = inBuff[32] | (inBuff[33] << 8);
			irRawVals[IR_BL] = inBuff[34] | (inBuff[35] << 8);
			irRawVals[IR_FL] = inBuff[36] | (inBuff[37] << 8);
			irRawVals[IR_T]= inBuff[38] | (inBuff[39] << 8);

			ctlPeltier_s = inBuff[40];
            if (ctlPeltier_s > 100) ctlPeltier_s = ctlPeltier_s - 201;
            motPwm_s = inBuff[41];

			ledCtl_s[L_R] = inBuff[42];
			ledCtl_s[L_G] = inBuff[43];
			ledCtl_s[L_B] = inBuff[44];
			ledDiag_s[L_R] = inBuff[45];
			ledDiag_s[L_G] = inBuff[46];
			ledDiag_s[L_B] = inBuff[47];

            airflow_s = inBuff[48];
            fanCooler = inBuff[49];

            dummy = inBuff[50] | (inBuff[51] << 8);
            if (dummy > 32767)
                tempCasu= (dummy - 65536.0) / 10.0;
            else
                tempCasu = dummy / 10.0;

            dummy = inBuff[52] | (inBuff[53] << 8);
            if (dummy > 32767)
                tempWax = (dummy - 65536.0) / 10.0;
            else
                tempWax = dummy / 10.0;

            dummy = inBuff[54] | (inBuff[55] << 8);
            if (dummy > 32767)
                temp_ref_rec = (dummy - 65536.0) / 10.0;
            else
                temp_ref_rec = dummy / 10.0;

            dummy = inBuff[56] | (inBuff[57] << 8);
            if (dummy > 32767)
                temp_ref_cur = (dummy - 65536.0) / 10.0;
            else
                temp_ref_cur = dummy / 10.0;

            calRec = inBuff[58];

			this->mtxPub_.unlock();
			gettimeofday(&current_time, NULL);
			double t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
            sprintf(str_buff, "%.2f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d %d %d %d %d %d %d %d %d %d \n",
                    t_msec / 1000, temp[T_F], temp[T_R], temp[T_B], temp[T_L], temp[T_T], tempCasu, tempWax, temp_ref_rec, temp_ref_cur,
                    ctlPeltier_s, pwmMotor_s, airflow_s, fanCooler,
                    irRawVals[IR_F], irRawVals[IR_FR], irRawVals[IR_BR], irRawVals[IR_B], irRawVals[IR_BL], irRawVals[IR_FL]);
			log_file.write(str_buff, strlen(str_buff));
			log_file.flush();
			print_counter++;
            if (print_counter == 10) {
				printf("temp = ");
				for (int i = 0; i < 5; i++) {
					printf("%.1f ", temp[i]);
				}
                printf("%.1f %.1f %.1f %.1f", tempCasu, tempWax, temp_ref_rec, temp_ref_cur);
				printf("\n");

				printf("vibeAmp = ");
				for (int i = 0; i < 4; i++) {
					printf("%.1f ", vAmp[i]);
				}
				printf("\n");

				printf("vibeFreq = ");
				for (int i = 0; i < 4; i++) {
					printf("%.1f ", vFreq[i]);
				}
				printf("\n");

				printf("ir raw = ");
				for (int i = 0; i < 7; i++) {
					printf("%d ", irRawVals[i]);
				}
				printf("\n");

				printf("ledCtl = ");
				for (int i = 0; i < 3; i++) {
					printf("%d ", ledCtl_s[i]);
				}
				printf("\n");

				printf("ledDiag = ");
				for (int i = 0; i < 3; i++) {
					printf("%d ", ledDiag_s[i]);
				}
				printf("\n");

                printf("peltier, motPwm, speakFreq, speakAmp, fan, fanCooler = %d %d %d %d %d\n", ctlPeltier_s, motPwm_s, speakerFreq_s, speakerAmp_s, airflow_s, fanCooler);

                printf("EM electric, EM magnetic, EM heat = %d %d %d\n", ehm_freq_electric,
                       ehm_freq_magnetic,
                       ehm_temp);
                printf("calibration data rec = %d \n", calRec);
				printf("_________________________________________________________________\n\n");
				print_counter = 0;
			}
		}

        if (pic2Address != 0 ) {
            // using the second dspic for speaker control
            status = i2cPIC2.receiveData(inBuff, IN2_DATA_NUM);

            if (status <= 0) {
                printf("Failed to receive data from aux pic");
            }
            else {
                this->mtxPub_.lock();
                speakerFreq_s = inBuff[0] | (inBuff[1] << 8);
                speakerAmp_s = inBuff[2] | (inBuff[3] << 8);
                this->mtxPub_.unlock();
            }
        }

        // send calibration file if not already sent
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
            status = i2cPIC1.sendData(outBuff, OUT1_CAL_DATA_NUM);
            calSend = 1;
        }
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

    for(int i = 0; i < 7; i++) {
		temps.add_temp(0);
	}

    AssisiMsg::Temperature peltier;
    std::string peltMsg;

	while (1) {


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
		ranges.SerializeToString(&data);
		zmq::send_multipart(zmqPub, casuName.c_str(), "IR", "Ranges", data);

		temp_clock++;
        if (temp_clock == 10) {
			this->mtxPub_.lock();
			for(int i = 0; i < 5; i++) {
				temps.set_temp(i, temp[i]);
			}
            temps.set_temp(5, tempCasu);
            temps.set_temp(6, tempWax);
			this->mtxPub_.unlock();
			temps.SerializeToString(&data);
			zmq::send_multipart(zmqPub, casuName.c_str(), "Temp", "Temperatures", data);
			//printf("Sending temp data %.1f %.1f %.1f %.1f %.1f \n", temps.temp(0), temps.temp(1), temps.temp(2), temps.temp(3), temps.temp(4));

            this->mtxPub_.lock();
            peltier.set_temp(this->temp_ref_rec);
            if (this->temp_ref_rec < 26) {
                peltMsg = "Off";
            }
            else {
                peltMsg = "On";
            }
            this->mtxPub_.unlock();
            peltier.SerializeToString(&data);
            zmq::send_multipart(zmqPub, casuName.c_str(), "Peltier", peltMsg.c_str(), data);

			AssisiMsg::VibrationReadingArray vibes;
			AssisiMsg::VibrationReading *vibe;

            this->mtxPub_.lock();
			for(int i = 0; i < 4; i++) {
				vibe = vibes.add_reading();
				vibe->add_amplitude(vAmp[i]);
				vibe->add_freq(vFreq[i]);
			}

			this->mtxPub_.unlock();
			//printf("Sending acc data %.1f %.1f %.1f %.1f \n", vibes.amplitude(0),vibes.amplitude(1), vibes.amplitude(2), vibes.amplitude(3));
			//printf("Sending proxy data %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n", ranges.rif (abs(temp[3] - temp_old) > 5)) {

			vibes.SerializeToString(&data);
			zmq::send_multipart(zmqPub, casuName.c_str(), "Acc", "Measurements", data);

			temp_clock = 0;
		}
        usleep(100000);

	}
}

void CASU_Interface::zmqSub()
{

	zmq::socket_t zmqSub(*zmqContext, ZMQ_SUB);
	zmqSub.bind(sub_addr.c_str());
	//zmqSub.connect("tcp://127.0.0.1:5556");
	zmqSub.setsockopt(ZMQ_SUBSCRIBE, casuName.c_str(), 4);

	string name;
	string device;
	string command;
	string data;
	int len;
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
			}

			else if (device == "Light") {

				if (command == "On") {
					AssisiMsg::ColorStamped color_msg;
					assert(color_msg.ParseFromString(data));

					mtxSub_.lock();
					ledCtl_r[L_R] = 100 * color_msg.color().red();
					ledCtl_r[L_G] = 100 * color_msg.color().green();
					ledCtl_r[L_B] = 100 * color_msg.color().blue();
					mtxSub_.unlock();
				}
				else if (command == "Off") {
					mtxSub_.lock();
					ledCtl_r[L_R] = 0;
					ledCtl_r[L_G] = 0;
					ledCtl_r[L_B] = 0;
					mtxSub_.unlock();
				 }
				else {
					cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
				}
			}

			else if (device == "VibeMotor") {

                printf("Received vibe command: %s\n", command.data());

				if (command == "On") {

					AssisiMsg::VibrationSetpoint vibe;
					assert(vibe.ParseFromString(data));
					mtxSub_.lock();
                    pwmMotor_r = vibe.amplitude();
                    mtxSub_.unlock();
                    printf(" Motor pwm %d\n", pwmMotor_r);
				}
				else if (command == "Off") {
					mtxSub_.lock();
                    pwmMotor_r = 0;
					mtxSub_.unlock();
				}
				else {
					cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
				}
			}

            else if (device == "Speaker") {

                 printf("Received vibe command: %s\n", command.data());

                 if (command == "On") {

                     AssisiMsg::VibrationSetpoint vibe;
                     assert(vibe.ParseFromString(data));
                     mtxSub_.lock();
                     speakerAmp_r = vibe.amplitude();
                     speakerFreq_r = vibe.freq();
                     if (speakerAmp_r == 0 || speakerFreq_r == 0) {
                         speakerAmp_r = 0;
                         speakerFreq_r = 0;
                     }
                     mtxSub_.unlock();
                     printf(" Speaker freq, pwm  %d %d\n", speakerAmp_r, speakerFreq_r);
                 }
                 else if (command == "Off") {
                     mtxSub_.lock();
                     speakerAmp_r = 0;
                     speakerFreq_r = 0;
                     mtxSub_.unlock();
                 }
                 else {
                     cerr << "Unknown command " << command << " for " << name << "/" << device << endl;
                 }
            }

			else if (device == "EM") {

                printf("Received EM device message: %s\n \
                       ...Discarding message as we are now longer using electro-magnetic emitters", command.data());
                /*
                if (command == "config") {
					AssisiMsg::EMDeviceConfig config;
					assert(config.ParseFromString(data));
					if (config.mode() == AssisiMsg::EMDeviceConfig_DeviceMode_ELECTRIC) {
						ehm_device->initEField();
					}
					else if (config.mode() == AssisiMsg::EMDeviceConfig_DeviceMode_MAGNETIC) {
						ehm_device->initMField();
					}
					else if (config.mode() == AssisiMsg::EMDeviceConfig_DeviceMode_HEAT) {
						ehm_device->initHeating();
					}
				}
				else if (command == "temp") {
					AssisiMsg::Temperature temp_msg;
					assert(temp_msg.ParseFromString(data));

					// for now we use temperature as a pwm duty cycle, i.e. 36° is 36% duty
                    // TODO: Check that the device is in the right mode!
					ehm_temp = (int)temp_msg.temp();
                    ehm_device->setHeaterPwm(ehm_temp);
				}
				else if (command == "efield") {

					AssisiMsg::ElectricField efield_msg;
					assert(efield_msg.ParseFromString(data));

                    // TODO: Check that the device is in the right mode!
                    ehm_freq_electric = (int)efield_msg.freq();
					ehm_device->setEFieldFreq(ehm_freq_electric);
				}
				else if (command == "mfield") {

					AssisiMsg::MagneticField mfield_msg;
					assert(mfield_msg.ParseFromString(data));

                    // TODO: Check that the device is in the right mode!
                    ehm_freq_magnetic = (int)mfield_msg.freq();
					ehm_device->setMFieldFreq(ehm_freq_magnetic);
				}
				else if (command == "Off") {
					ehm_device->moduleOff();
                    ehm_temp = 0;
                    ehm_freq_electric = 0;
                    ehm_freq_magnetic = 0;
				}
                */

			}
			else if (device == "Peltier") {
				printf("Received Peltier device message: %s\n", command.data());
				if (command == "temp") {
					AssisiMsg::Temperature temp_msg;
					assert(temp_msg.ParseFromString(data));
					mtxSub_.lock();
					temp_ref = temp_msg.temp();
					mtxSub_.unlock();
					printf("Reference temperature %.1f \n", temp_ref);
				}
				else if (command == "Off") {
					mtxSub_.lock();
					temp_ref = 0;
					mtxSub_.unlock();
				}
			}
            else if (device == "Airflow") {
                printf("Received Airflow message: %s\n", command.data());
                if (command == "On") {
                    AssisiMsg::Airflow air_msg;
                    assert(air_msg.ParseFromString(data));
                    mtxSub_.lock();
                    airflow_r = air_msg.intensity();
                    mtxSub_.unlock();
                    printf("Reference airflow %d \n", airflow_r);
                }
                else if (command == "Off") {
                    mtxSub_.lock();
                    airflow_r = 0;
                    mtxSub_.unlock();
                }
            }
			else
			{
				cerr << "Unknown device " << device << endl;
			}

		}
        mtxSub_.lock();
        newRef = 1;
        mtxSub_.unlock();
		fflush(stdout);
	}
}
