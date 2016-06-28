#ifndef COMMAND_H
#define COMMAND_H
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#define DEBUG_MODE		0
#define USER_MODE		!DEBUG_MODE

extern uint8_t Pi_Receive_String_Ready;

void receive_command();
void receive_pi_command();

static struct RECEIVE_CMD{
	volatile unsigned char _start;
	volatile unsigned char cmd_id;
	volatile unsigned char cmd_value;
	volatile unsigned char _end;
};

#endif
