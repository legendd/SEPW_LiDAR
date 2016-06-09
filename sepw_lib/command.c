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
uint32_t inc = 1;

struct RECEIVE_CMD *cmd_raw;

#if USER_MODE
void receive_command(){

	USART_puts(USART6, "user mode test\r\n");

}
void receive_pi_command(){

	USART_puts(USART1, "Pi receive test\r\n");

}


#endif