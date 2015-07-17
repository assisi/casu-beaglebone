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
    picAddress = fbc["pic1_addr"].as<int>();
    tempCtlOn = fbc["tempCtlOn"].as<int>();
    Kp = fbc["Kp"].as<float>();
    Ki = fbc["Ki"].as<float>();
    Kf1 = fbc["Kf1"].as<float>();
    Kf2 = fbc["Kf2"].as<float>();
    Kf3 = fbc["Kf3"].as<float>();
    fanCtlOn = fbc["fanCtlOn"].as<int>();

    i2cPIC.initI2C(i2c_bus, picAddress);
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

    airflow_r = 0;
    airflow_s = 0;

    fanCooler = 0;

    temp_ref = 25.0;

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

    log_file.open((std::string("log/") + casuName + std::string(".txt")).c_str(), ios::out);
}

CASU_Interface::~CASU_Interface() {
	log_file.close();
    //delete ehm_device;
	delete zmqContext;
}

void CASU_Interface::i2cComm() {
    char str_buff[10000] = {0};
    char aux_buff[10] = {0};
	std::stringstream ss;
	gettimeofday(&start_time, NULL);
	timeval current_time;
    sprintf(str_buff, "Raw acc measurements, taken with 1 Khz every second \n");
	log_file.write(str_buff, strlen(str_buff));
	while(1) {
        status = i2cPIC.receiveData(inBuff, IN_DATA_NUM);

		if (status <= 0) {
			cerr << "I2C initialization unsuccessful, exiting thread" << std::endl;
			break;
		}
		else {

            cout << "Read bytes: " << IN_DATA_NUM << std::endl;
            gettimeofday(&current_time, NULL);
            double t_msec = (current_time.tv_sec - start_time.tv_sec) * 1000 + (current_time.tv_usec - start_time.tv_usec)/1000;
            sprintf(str_buff, "%.2f", t_msec / 1000);
            for (int i=0; i < IN_DATA_NUM; i = i + 2) {
                int dummy = int(inBuff[i] + (inBuff[i+1] << 8));
                if (dummy > 32767)
                    dummy -= 65536; // negative
                sprintf(aux_buff, " %d", dummy);
                strcat(str_buff, aux_buff);
            }
            strcat(str_buff, "\n");
            log_file.write(str_buff, strlen(str_buff));
            log_file.flush();
		}
        usleep(100000);
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

	for(int i = 0; i < 5; i++) {
		temps.add_temp(0);
	}


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
		if (temp_clock == 4) {
			this->mtxPub_.lock();
			for(int i = 0; i < 5; i++) {
				temps.set_temp(i, temp[i]);
			}
			this->mtxPub_.unlock();
			temps.SerializeToString(&data);
			zmq::send_multipart(zmqPub, casuName.c_str(), "Temp", "Temperatures", data);
			//printf("Sending temp data %.1f %.1f %.1f %.1f %.1f \n", temps.temp(0), temps.temp(1), temps.temp(2), temps.temp(3), temps.temp(4));

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
		usleep(250000);

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

				printf("Received vibe command: %s", command.data());

				if (command == "On") {

					AssisiMsg::VibrationSetpoint vibe;
					assert(vibe.ParseFromString(data));
					mtxSub_.lock();
					pwmMotor_r = vibe.freq() * vibeMotorConst;
					mtxSub_.unlock();
					printf(" Frequency, pwm %f, %d \n", vibe.freq(), pwmMotor_r);
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

		fflush(stdout);
	}
}
