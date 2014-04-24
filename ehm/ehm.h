/*
 * ehm.h
 *
 *  Created on: Apr 24, 2014
 *      Author: thaus
 */

#ifndef EHM_H_
#define EHM_H_

#include "../serial/serial.h"

#define INIT_COMM	"1h2h"
#define READ_PARAMS "005a"
#define MODULE_ON "001a"
#define MODULE_OFF "000a"
#define V40_ON	"001c"
#define V40_OFF	"000c"
#define LFM_ON "008a"
#define LFM_OFF "009a"
#define TIMER_ON "001d"
#define TIMER_OFF "000d"
#define SOUND_ON "001f"
#define SOUND_OFF "000f"
#define E_OUTPUT 0b00000001
#define H_OUTPUT 0b00000010
#define L_OUTPUT 0b00000100
#define H_PWM_REV 0b00001000
#define E_LFM 0b00010000
#define H_LFM 0b00100000
#define L_LFM 0b01000000
#define TOGGLE_LFM 0b10000000
#define L_PWM_PERIOD 100 // us, f = 10 kHz
#define DELAY 25000 // us

class ehm : Serial {
public:
	ehm();
	ehm(char *port, int baudRate);
	int readParams();
	int initHeating();
	int initEField();
	int initMField();
	int setHeaterPwm(int dutyCycle);
	virtual ~ehm();
};

#endif /* EHM_H_ */
