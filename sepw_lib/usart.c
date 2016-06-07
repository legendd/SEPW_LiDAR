#include "FreeRTOS.h"
#include "usart.h"
#include "main.h"
#include "stm32f4xx_conf.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#if 0
uint8_t Receive_String_Ready=0;
volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL ; 
volatile xQueueHandle serial_str_queue = NULL ; 
/* Queue structure used for passing messages. */
typedef struct {
	char str[100];
} serial_str_msg;

/* Queue structure used for passing characters. */
typedef struct {
    char ch;
} serial_ch_msg;
#endif

volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL ; 
volatile xQueueHandle serial_str_queue = NULL ; 
/* Queue structure used for passing messages. */
typedef struct {
  char str[100];
} serial_str_msg;

/* Queue structure used for passing characters. */
typedef struct {
    char ch;
} serial_ch_msg;

//USING USART3 For LiDAR
void USART3_Config(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USARTx, ENABLE);  

  USART_InitStructure.USART_BaudRate = baudrate;//115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
}
//USING USART6 For BT
void USART6_Config(uint32_t baudrate1){

  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTy_TX_GPIO_CLK | USARTy_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTy_CLK_INIT(USARTy_CLK, ENABLE);
  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTy_TX_GPIO_PORT, USARTy_TX_SOURCE, USARTy_TX_AF);
  GPIO_PinAFConfig(USARTy_RX_GPIO_PORT, USARTy_RX_SOURCE, USARTy_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTy_TX_PIN;
  GPIO_Init(USARTy_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTy_RX_PIN;
  GPIO_Init(USARTy_RX_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USARTy, ENABLE);  

  USART_InitStructure.USART_BaudRate = baudrate1;//9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTy, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = USARTy_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USARTy, ENABLE);
}

void USART_puts(USART_TypeDef* USART, volatile uint8_t *s)
{
  while(*s){
    // wait until data register is empty
    //while( !(USARTx->SR & 0x00000040) );
    while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
    USART_SendData(USART, *s);
    *s++;
    //s++;
  }
}

void USART_putd(USART_TypeDef* USART, uint32_t number)
{
  static uint32_t temp;
  static uint8_t cnt = 0;
  volatile uint8_t tmp_num[10];
  volatile uint8_t num[10];

  if(number == 0){
    tmp_num[cnt++] = '0';
  }
  while(number != 0){
    temp = number % 10;
    number -= temp;
    number /= 10;
    tmp_num[cnt] = temp+'0';
    cnt++;
  }
  
  int j = 0;
  while(cnt){
    num[j++] = tmp_num[--cnt];
  }
  num[j] = '\0';

  USART_puts(USART6, num);
  cnt = 0;
  int i;
  for( i = 0 ; i< 10 ; i++){
    tmp_num[i]= 0;
    num[i]= 0;
  }
}

void send_byte(char ch)
{
  /* Wait until the RS232 port can receive another byte (this semaphore
   * is "given" by the RS232 port interrupt when the buffer has room for
   * another byte.
   */
  while (!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));

  /* Send the byte and enable the transmit interrupt (it is disabled by
   * the interrupt).
   */
  USART_SendData(USART3, ch);
  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}
char receive_byte()
{
    serial_ch_msg msg ; 

    /* Wait for a byte to be queued by the receive interrupts handler. */
    while (!xQueueReceive(serial_rx_queue, &msg, portMAX_DELAY));
    return msg.ch ; 

}
