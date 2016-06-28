#include "FreeRTOS.h"
#include "usart.h"
#include "command.h"
#include "main.h"
#include "stm32f4xx_conf.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "string.h"
#include "stdio.h"
#include "motor.h"
uint32_t inc = 1;
uint8_t i = 0;
uint32_t pts_x = 0;
uint32_t pts_y = 0;
char pos_received_x[3];
char pos_received_y[3];
struct RECEIVE_CMD *cmd_raw;

#if USER_MODE
void receive_command(){

	USART_puts(USART6, "user mode test\r\n");

}
void receive_pi_command(){
	for(i = 0; i < 3; i++){
		pos_received_x[i] = pi_received_string[i];
		pos_received_y[i] = pi_received_string[i+3];
	}
	pts_x = atoi(pos_received_x);
	pts_y = atoi(pos_received_y);
	//USART_puts(USART6, "\r\nPi receive test\n\r");
	USART_puts(USART6, "\r\nX:");
	USART_putd(USART6, pts_x);
	USART_puts(USART6, "  Y:");
	USART_putd(USART6, pts_y);

	// Motor moving
	if(pts_x < 300){
		// motorLeft(lValue, rValue)
		motorLeft(pts_x, 0);
		//motorForward();
		vTaskDelay(20);
	}
	else
	{
		motorRight(0, pts_x);
		//motorStop();
		vTaskDelay(20);
	}

	for(i = 0; i < 3; i++){
		pos_received_x[i] = 0;
		pos_received_y[i] = 0;
	}
	pts_x = 0;
	pts_y = 0;

	/*if(pi_received_string[0] == '1'){
		USART_puts(USART6, ":1\r\n");
		motorForward();
	}
	else if(pi_received_string[0] == '2'){
		USART_puts(USART6, ":2\r\n");
		motorStop();
	}*/
}


#endif