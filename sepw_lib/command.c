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
uint32_t lidar_Pos = 0;
uint32_t object_distance = 0;
char pos_received_x[3];
char pos_received_x1[2];
char pos_received_y[3];
char pos_received_y1[2];
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
	if(pos_received_x[0]=='0'){
		pos_received_x1[0] = pos_received_x[1];
		pos_received_x1[1] = pos_received_x[2];
		pts_x = atoi(pos_received_x1);
	}
	else
		pts_x = atoi(pos_received_x);
	if(pos_received_y[0]=='0'){
		pos_received_y1[0] = pos_received_y[1];
		pos_received_y1[1] = pos_received_y[2];
		pts_y = atoi(pos_received_y1);
	}
	else
		pts_y = atoi(pos_received_y);
	//USART_puts(USART6, "\r\nPi receive test\n\r");

	
	// pts_x is the position on image plane 
	// actual position of object
	// pts_x = XXX * pts_x

	// degree of lidar data
	lidar_Pos = pts_x*9/32;
	motorForward();
	
	object_distance = Lidar_distance[lidar_Pos];
#if	0
	USART_puts(USART6, "\r\nX:");
	USART_putd(USART6, pts_x);
	USART_puts(USART6, "  Y:");
	USART_putd(USART6, pts_y);
	USART_puts(USART6, "\r\nlidar_Pos:");
	USART_putd(USART6, lidar_Pos);
	USART_puts(USART6, "\r\nobject_distance:");
	USART_putd(USART6, object_distance);

#endif
	#if	0
	// Motor moving
	if(pts_x < 320){
		// motorLeft(lValue, rValue)
		motorLeft(pts_x, 0);
		vTaskDelay(300);
	}
	else
	{
		motorRight(0, pts_x);
		vTaskDelay(300);
	}
#endif
	for(i = 0; i < 3; i++){
		pos_received_x[i] = 0;
		pos_received_y[i] = 0;
	}
	pts_x = 0;
	pts_y = 0;
}


#endif