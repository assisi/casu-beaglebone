/*
 * BBBRouter.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#include "BBBInterface.h"

using namespace std;

BBBInterface::BBBInterface(int bus, int picAddress)
	{

	i2cPIC.initI2C(bus, picAddress);
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

	vAmp[0] = 0.1;
	vAmp[1] = 0.2;
	vAmp[2] = 0.3;
	vAmp[3] = 0.4;

	vFreq[0] = 10;
	vFreq[1] = 20;
	vFreq[2] = 30;
	vFreq[3] = 40;

	pwmMotor_r = 0;

	temp_r = 27;
	temp_ref = 0;

	proxyThresh = 4000;

    // EHM device configuration
	ehm_device = new ehm("/dev/ttyACM0", 9600);
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
	ctlFlag = 0;
	startFlag = 1;

	Kp_t = 2;
	Ki_t = 0.075;
	//Kd_t = 10;
	uOld_t = 0;
	uiOld = 0;
	eOld_t = 0;
	deOld_t = 0;
	deadZone_t = 0.25;
	temp_old = 26;

	time(&time_a);
	time(&ctlTime);

	gethostname(casuName, 15);
}

BBBInterface::~BBBInterface() {
	delete ehm_device;
	delete zmqContext;
}

void BBBInterface::i2cComm() {
	int print_counter = 0 ;
	while(1) {
		status = i2cPIC.receiveData(inBuff, IN_DATA_NUM);

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
				temp[T_T] = (dummy - 65536.0) / 10.0;
			else
				temp[T_T] = dummy / 10.0;

			vAmp[A_F] = (inBuff[10] | (inBuff[11] << 8)) / 10.0;
			vAmp[A_R] = (inBuff[12] | (inBuff[13] << 8)) / 10.0;
			vAmp[A_B] = (inBuff[14] | (inBuff[15] << 8)) / 10.0;
			vAmp[A_L] = (inBuff[16] | (inBuff[17] << 8)) / 10.0;

			vFreq[A_F] = inBuff[18] | (inBuff[19] << 8);
			vFreq[A_R] = inBuff[20] | (inBuff[21] << 8);
			vFreq[A_B] = inBuff[22] | (inBuff[23] << 8);
			vFreq[A_L] = inBuff[24] | (inBuff[25] << 8);

			irRawVals[IR_F] = inBuff[26] | (inBuff[27] << 8);
			irRawVals[IR_FR] = inBuff[28] | (inBuff[29] << 8);
			irRawVals[IR_BR] = inBuff[30] | (inBuff[31] << 8);
			irRawVals[IR_B] = inBuff[32] | (inBuff[33] << 8);
			irRawVals[IR_BL] = inBuff[34] | (inBuff[35] << 8);
			irRawVals[IR_FL] = inBuff[36] | (inBuff[37] << 8);
			irRawVals[IR_T]= inBuff[38] | (inBuff[39] << 8);

			ctlPeltier_s = inBuff[40];
			if (ctlPeltier_s > 100) ctlPeltier_s = ctlPeltier_s - 200;
			pwmMotor_s = inBuff[41];

			ledCtl_s[L_R] = inBuff[42];
			ledCtl_s[L_G] = inBuff[43];
			ledCtl_s[L_B] = inBuff[44];
			ledDiag_s[L_R] = inBuff[45];
			ledDiag_s[L_G] = inBuff[46];
			ledDiag_s[L_B] = inBuff[47];

			this->mtxPub_.unlock();

			print_counter++;
			if (print_counter == 50) {
				printf("temp = ");
				for (int i = 0; i < 5; i++) {
					printf("%.1f ", temp[i]);
				}
				printf("\n");

				printf("vibeAmp = ");
				for (int i = 0; i < 4; i++) {
					printf("%.1f ", vAmp[i]);
				}
				printf("\n");

				printf("vibeFreq = ");
				for (int i = 0; i < 4; i++) {
					printf("%d ", vFreq[i]);
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

				printf("peltier, motor = %d %d\n", ctlPeltier_s, pwmMotor_s);

                printf("EM electric, EM magnetic, EM heat = %d %d %d\n", ehm_freq_electric,
                       ehm_freq_magnetic,
                       ehm_temp);
				printf("_________________________________________________________________\n\n");
				print_counter = 0;
			}
		}


		usleep(25000);
		this->mtxSub_.lock();
		//int tmp = temp_r * 10;
        int tmp = temp_ref * 10;
		if (tmp < 0) tmp = tmp + 65536;
		//printf("Sending temperature %d \n", tmp);
		outBuff[0] = (tmp & 0x00FF);
		outBuff[1] = (tmp & 0xFF00) >> 8;

		outBuff[2] = pwmMotor_r & 0x00FF;
		outBuff[3] = (pwmMotor_r & 0xFF00) >> 8;

		outBuff[4] = ledCtl_r[0];
		outBuff[5] = ledCtl_r[1];
		outBuff[6] = ledCtl_r[2];

		outBuff[7] = ledDiag_r[0];
		outBuff[8] = ledDiag_r[1];
		outBuff[9] = ledDiag_r[2];
		this->mtxSub_.unlock();
		status = i2cPIC.sendData(outBuff, OUT_DATA_NUM);
		usleep(25000);

		if (startFlag) {
			temp_old = temp[3];
			startFlag = 0;
		}
		if (abs(temp[3] - temp_old) > 5) {
			temp[3] = temp_old;
		}
		time(&time_a);

        /*
		double diff_t = difftime(time_a, ctlTime);
		if (diff_t > 0.95) {
			if (ctlFlag) {
				temp_r = PIDcontroller_t(temp[3]); //left sensor;
				printf("Ctl value = %.1f \n", temp_r);
			}
			else {
				temp_r = 0;
			}
			temp_old = temp[3];
			time(&ctlTime);
			//printf("Control loop period = %.1f\n", diff_t);
		}
        */

		temp_old = temp[3];

	}
}

void BBBInterface::zmqPub() {

	zmq::socket_t zmqPub(*zmqContext, ZMQ_PUB);
	zmqPub.bind("tcp://*:5555");
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
	AssisiMsg::VibrationArray vibes;

	for(int i = 0; i < 4; i++) {
		vibes.add_amplitude(0);
		vibes.add_freq(0);
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
		zmq::send_multipart(zmqPub, casuName, "IR", "Ranges", data);

		temp_clock++;
		if (temp_clock == 4) {
			this->mtxPub_.lock();
			for(int i = 0; i < 5; i++) {
				temps.set_temp(i, temp[i]);
			}
			this->mtxPub_.unlock();
			temps.SerializeToString(&data);
			zmq::send_multipart(zmqPub, casuName, "Temp", "Temperatures", data);
			//printf("Sending temp data %.1f %.1f %.1f %.1f %.1f \n", temps.temp(0), temps.temp(1), temps.temp(2), temps.temp(3), temps.temp(4));
			this->mtxPub_.lock();
			for(int i = 0; i < 4; i++) {
				vibes.set_amplitude(i, vAmp[i]);
				vibes.set_freq(i, vFreq[i]);
			}
			this->mtxPub_.unlock();
			//printf("Sending acc data %.1f %.1f %.1f %.1f \n", vibes.amplitude(0),vibes.amplitude(1), vibes.amplitude(2), vibes.amplitude(3));
			//printf("Sending proxy data %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n", ranges.rif (abs(temp[3] - temp_old) > 5)) {

			vibes.SerializeToString(&data);
			zmq::send_multipart(zmqPub, casuName, "Acc", "Measurements", data);

			temp_clock = 0;
		}
		usleep(250000);

	}
}

void BBBInterface::zmqSub() {

	zmq::socket_t zmqSub(*zmqContext, ZMQ_SUB);
	zmqSub.bind("tcp://*:5556");
	//zmqSub.connect("tcp://127.0.0.1:5556");
	zmqSub.setsockopt(ZMQ_SUBSCRIBE, casuName, 4);

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

					AssisiMsg::Vibration vibe;
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

				printf("Received EM device message: %s\n", command.data());
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

			}
			else if (device == "Peltier") {
				printf("Received Peltier device message: %s\n", command.data());
				if (command == "temp") {
					AssisiMsg::Temperature temp_msg;
					assert(temp_msg.ParseFromString(data));
					mtxSub_.lock();
					temp_ref = temp_msg.temp();
					ctlFlag = 1;
					mtxSub_.unlock();
					printf("Reference temperature %.1f \n", temp_ref);
				}
				else if (command == "Off") {
					mtxSub_.lock();
					ctlFlag = 0;
					temp_ref = 0;
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

/*
 * Function implements PI controller for Peltier device.
 * We use global variables for reference and measured temperature.
 * int i - row index
 * int j - column index
 */
float BBBInterface::PIDcontroller_t(float temp) {

	//printf("Controler temp_ref, temp_m = %.1f %.1f \n", temp_ref, temp);
    float e_t = temp_ref - temp;
    float de_t = e_t - eOld_t;
    float up = 0;
    float ui = 0;
    float ud = 0;

    if (e_t > deadZone_t)
        e_t = e_t - deadZone_t;
    else if (e_t < -deadZone_t)
        e_t = e_t + deadZone_t;
    else
        e_t = 0;


    //float u  = uOld_t + Kp_t * de_t + Ki_t * e_t + Kd_t * (de_t - deOld_t);
    if (abs(e_t) > 2) {
    	Kp_t = 10;
    	up = Kp_t * e_t;
    	ui = 0;
    	uiOld = up;
    }
    else {
    	Kp_t = 2;
    	up = Kp_t * e_t;
        ui = uiOld + Ki_t * e_t;
        uiOld = ui;
    }

    float u  = up + ui;

    // rate limiter
//    if (u - uOld_t  > 5)
//        u = uOld_t + 5;
//    else if (u - uOld_t < - 5)
//        u = uOld_t - 5;


    if (u > 100) {
        u = 100;
    	//ui = uiOld;
    }
    else if (u < -100) {
        u = -100;
        //ui = uiOld;
    }
    uOld_t = u;
    eOld_t = e_t;
    deOld_t = de_t;

    return u;
}


