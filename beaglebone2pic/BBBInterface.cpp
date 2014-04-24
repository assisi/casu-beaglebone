/*
 * BBBRouter.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: thaus
 */

#include "BBBInterface.h"

using namespace std;

BBBInterface::BBBInterface(int bus, int picAddress) {

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

	pwmMotor_r = 0;

	proxyThresh = 4000;

	/*Scale freq to motor pwm
	 * From datasheet 3V > 516 Hz
	 * u = K * f; K = 3 / 516 = 0.0058
	 * pwm = u / 3.3 * 100 = 30.303
	 * pwm = k2 * f = 0.0058*30.303 = 0.1758
	* */
	vibeMotorConst = 0.1758;

	gethostname(casuName, 15);
}

BBBInterface::~BBBInterface() {
	delete zmqContext;
}

void BBBInterface::i2cComm() {

	while(1) {
		status = i2cPIC.receiveData(inBuff, IN_DATA_NUM);

		if (status <= 0) {
			cerr << "I2C initialization unsuccessful, exiting thread" << std::endl;
			break;
		}
		else {

			cout << "Read bytes: " << IN_DATA_NUM << std::endl;

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
			pwmMotor_s = inBuff[41];

			ledCtl_s[L_R] = inBuff[42];
			ledCtl_s[L_G] = inBuff[43];
			ledCtl_s[L_B] = inBuff[44];
			ledDiag_s[L_R] = inBuff[45];
			ledDiag_s[L_G] = inBuff[46];
			ledDiag_s[L_B] = inBuff[47];

			this->mtxPub_.unlock();

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
			printf("_________________________________________________________________\n\n");
		}

		usleep(25000);
		temp_r = 26.0;
		this->mtxSub_.lock();
		int tmp = temp_r * 10;
		if (tmp < 0) tmp = tmp + 65636;
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
	}
}

void BBBInterface::zmqPub() {

	zmq::socket_t zmqPub(*zmqContext, ZMQ_PUB);
	zmqPub.bind("tcp://*:5555");
	int range;
	cout << "Starting pub server: " << casuName;
	while (1) {

		std::string data;
		AssisiMsg::RangeArray ranges;
		this->mtxPub_.lock();
		for(int i = 0; i < 6; i++) {
			if (irRawVals[i] > proxyThresh)
				range = 1;
			else
				range = 10;
			ranges.add_range(range);
			ranges.add_raw_value(irRawVals[i]);

		}
		this->mtxPub_.unlock();
		ranges.SerializeToString(&data);
		zmq::send_multipart(zmqPub, casuName, "IR", "Ranges", data);
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
	cout << "Starting sub server: " << casuName;

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
			else
			{
				cerr << "Unknown device " << device << endl;
			}

		}
	}

}

