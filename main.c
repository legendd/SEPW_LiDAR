/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "task.h"
//#include "usbd_hid_core.h"
//#include "usbd_usr.h"
//#include "usbd_desc.h"

#include "main.h"
#include "stm32f4xx_conf.h"
#include "usart.h"
#include "motor.h"
#include "queue.h"
#include "semphr.h"
#include "mpu6050.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555
#define ADC1_DR_Address   ((uint32_t)0x4001204C)

/* PWM Timer mapping */
//#define PWM1 (TIM2 -> CCR3)
#define PWM1 (TIM12 -> CCR1)  // right of the front
#define PWM2 (TIM12 -> CCR2)
#define PWM3 (TIM10 -> CCR1)
#define PWM4 (TIM11 -> CCR1)
#define PWM5 (TIM4 -> CCR1)  // center of the front
#define PWM6 (TIM4 -> CCR2)
#define PWM7 (TIM3 -> CCR3)
#define PWM8 (TIM3 -> CCR4)
#define PWM9 (TIM9 -> CCR2)  // left of the front
#define DegreeShift 0         // Use this value to set the angle of lidar

__IO uint32_t TimingDelay = 0;
__IO uint8_t UserButtonPressed = 0x00;

__IO uint64_t currentTime = 0;
__IO uint64_t oldTime = 0;
__IO uint64_t Timing = 0;
__IO uint32_t diffTime = 0;
__IO uint64_t Velocity = 0;
volatile int over_flag = 0;
volatile int btFlag = 0;

/* Lidar (Usart) -------------------------------------------------------------*/
//const int front_obstacle_region[90] = {500, 500, 500, 501, 501, 502, 503, 504, 505, 506, 508, 509, 511, 513, 515, 518, 520, 523, 526, 529, 532, 536, 539, 543, 547, 552, 556, 561, 566, 572, 577, 583, 590, 596, 603, 610, 618, 626, 635, 643, 653, 663, 673, 684, 695, 707, 720, 733, 747, 762, 778, 795, 812, 831, 851, 872, 894, 918, 944, 971, 1000, 1031, 1065, 1101, 1141, 1183, 1229, 1280, 1335, 1395, 1462, 1536, 1618, 1710, 1814, 1932, 2067, 2223, 2405, 2620, 2879, 3196, 3593, 4103, 4783, 5737, 7168, 9554, 14327, 28649};
const int sin_array[90] = {0, 17, 35, 53, 71, 89, 107, 124, 142, 160, 177, 195, 212, 230, 247, 265, 282, 299, 316, 333, 350, 366, 383, 400, 416, 432, 448, 464, 480, 496, 511, 527, 542, 557, 572, 587, 601, 616, 630, 644, 658, 671, 685, 698, 711, 724, 736, 748, 760, 772, 784, 795, 806, 817, 828, 838, 848, 858, 868, 877, 886, 895, 904, 912, 920, 928, 935, 942, 949, 955, 962, 968, 973, 979, 984, 989, 993, 997, 1001, 1005, 1008, 1011, 1014, 1016, 1018, 1020, 1021, 1022, 1023, 1023};
unsigned char LiDAR_RxBuffer = 0;
unsigned char BT_RxBuffer = 0;
unsigned char Pi_RxBuffer = 0;

uint8_t Receive_String_Ready=0;
uint8_t Pi_Receive_String_Ready=0;
int16_t degree_distance[180] = {0};    /* Use this array to store distance information of front 180 degree */
int min_distance[9] = {0};
int check_sum(void);
int check_index(unsigned int);
void reset_tmp(void);
volatile uint32_t v_tmp = 0;
uint16_t distance_convert(uint8_t data0, uint8_t data1);
struct lidarData {
  uint8_t data[16];     /* Store the data of 4 angles */
  uint64_t time_stamp[2];  /* Use this to check update time */
};
volatile struct lidarData lidarBuffer[LidarPacketNumber];
/* Lidar (Usart) -------------------------------------------------------------*/
int front_region_right[5] = {6000};
int front_region_left[5] = {6000};
int front_region[9] = {6000};
volatile int handler_count = 0;

typedef struct _kalman_state{
  int32_t q[180];  // process noise covariance
  int32_t r[180];  // measurement noise covariance
  int32_t p[180];  // estimation error covariance
  int32_t x[180];  // distance value
  int32_t k[180];    // kalman gain
} kalman_state;

static kalman_state kalman_s;

void kaman_init(kalman_state *k_s){
  int i = 0;
  for (i = 0; i < 180; ++i)
  {
    k_s -> q[i] = 100;
    k_s -> r[i] = 2500;
    k_s -> p[i] = 0.5;
    k_s -> x[i] = 0;
  }
}

uint16_t kalman_update(int16_t measure, int16_t i,kalman_state *k_s){
  k_s->p[i] = ((k_s -> q[i])*(k_s -> q[i])+(k_s -> r[i])*(k_s -> r[i]))^(1/2);
  k_s->p[i] = k_s->p[i] + k_s->q[i];
  k_s->k[i] = (k_s->p[i] * 1000) / (k_s->p[i] + k_s->r[i]);
  k_s->x[i] = (uint16_t)(k_s->x[i] + (k_s->k[i] * (measure - k_s->x[i])/1000 )); 
  k_s->p[i] = (1000 - k_s->k[i]) * k_s->p[i];
  return k_s->x[i];
}

/**
  * @brief  Use this task to read and check the data sent from Lidar.
  * @param  None
  * @retval None
  */
static void usart_receive_task(void *pvParameters){
  
  while(1){
    /* Enable the Rx interrupt */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USARTy, USART_IT_RXNE, ENABLE);
    
    /* Waiting the end of Data transfer  */ 
    while(current_received_byte < LidarPacketSize);
    //while(!Receive_String_Ready);
    
    /* disable USART interrupt */
      NVIC_DisableIRQ(USARTx_IRQn);
      //NVIC_DisableIRQ(USARTy_IRQn);
        
    if(check_index(current_packet[1]) & check_sum()){
      write_packet();
    }
    reset_tmp();
    vTaskDelay(10);
  }
}
/**
  * @brief  Use this task to read and check the data sent from Bluetooth(PC Tablet/Android APP).
  * @param  None
  * @retval None
  */
static void bt_receive_task(void *pvParameters){
  
  while(1){
    /* Enable the Rx interrupt */
    USART_ITConfig(USARTy, USART_IT_RXNE, ENABLE);
    
    /* Waiting the end of Data transfer  */ 
    while(!Receive_String_Ready);
    
    /* disable USART interrupt */
    NVIC_DisableIRQ(USARTy_IRQn);
    vTaskDelay(10);
  }
}
/**
  * @brief  Use this task to read and check the data sent from Bluetooth(PC Tablet/Android APP).
  * @param  None
  * @retval None
  */
static void pi_receive_task(void *pvParameters){
  
  while(1){
    /* Enable the Rx interrupt */
    USART_ITConfig(USARTz, USART_IT_RXNE, ENABLE);
    
    /* Waiting the end of Data transfer  */ 
    while(!Pi_Receive_String_Ready);
    
    /* disable USART interrupt */
    NVIC_DisableIRQ(USARTz_IRQn);
    vTaskDelay(10);
  }
}

/**
  * @brief  This task used to calculated how much time the data was changed in each index
  * @param  None
  * @retval None
  */
#if 0
static void lidar_display_task(void *pvParameters){
  //int tmp_index;
  int changed_count = 0;
  int index_count = 0;
  int tmp_array [200];
  int index_statistic [90] = {0};
  
  while(1){
    while(tmp_array[changed_count-1] == current_packet[1]);
      
    tmp_array[changed_count++] = current_packet[1];
    index_statistic[current_packet[1]-160]++; 
    
    if(changed_count == 199){
      while(index_count < 90){
        //printf("%d,", index_statistic[index_count]);
        index_count++;
      }
      index_count = 0;
      changed_count = 0;
    }
    vTaskDelay(1);
  }
}
#endif
/**
  * @brief  This task used to write distance data of each degree to the buffer
  * @param  None
  * @retval None
  */
static void Transfer_Distance_task(void *pvParameters){
  int16_t count = 0;
  uint16_t distance_angle1 = 0;
  uint16_t distance_angle2 = 0;
  uint16_t distance_angle3 = 0;
  uint16_t distance_angle4 = 0;

  while(1){
    for(count = 0; count < 45; count++){
      distance_angle1 = distance_convert(lidarBuffer[count].data[0], lidarBuffer[count].data[1]);
      distance_angle2 = distance_convert(lidarBuffer[count].data[4], lidarBuffer[count].data[5]);
      distance_angle3 = distance_convert(lidarBuffer[count].data[8], lidarBuffer[count].data[9]);
      distance_angle4 = distance_convert(lidarBuffer[count].data[12], lidarBuffer[count].data[13]);
      
      int16_t c_1 = count*4;

      /* Write data to distance array */
      degree_distance[c_1] = kalman_update(distance_angle1, c_1, &kalman_s);
      degree_distance[c_1 + 1] = kalman_update(distance_angle2, c_1 + 1, &kalman_s);
      degree_distance[c_1 + 2] = kalman_update(distance_angle3, c_1 + 2, &kalman_s);
      degree_distance[c_1 + 3] = kalman_update(distance_angle4, c_1 + 3, &kalman_s);

    }
    vTaskDelay(30);
  }
}

/**
  * @brief  Use this task to detect the front obstacle
  * @param  None
  * @retval None
  */
static void Front_Obstacle_task(void *pvParameters){
  char current_status ='F'; // F : safe 
  char safe_message[9] = {'o', 'n', 'm', 'l', 'k', 'j', 'i', 'h', 'g'};
  char alarm_message[9] = {'O', 'N', 'M', 'L', 'K', 'J', 'I', 'H', 'G'};
  int range_alarm_distance[9] = {500,600,800,1500,3200,1500,800,600,500};
  int range_size = 10; //19
  int range_number = 18;

  while(1){
    long range_distance[18] = {0};
    long valid_distance[18] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
    int tmp_count, tmp_count2 = 0;
    current_status = 'F';

    for(tmp_count = 0; tmp_count < range_size; tmp_count++){
      for(tmp_count2 = 0; tmp_count2 < range_number; tmp_count2++){
        if(degree_distance[tmp_count2*range_size + tmp_count] > 0){
          range_distance[tmp_count2] += degree_distance[tmp_count2*range_size + tmp_count];
        } else{
          valid_distance[tmp_count2] --;
        }
      }
    }

    // Get the average distance from 9 direction 
    int range_count = 0;
    for (range_count = 0; range_count < range_number; range_count++){
      range_distance[range_count] = range_distance[range_count]/valid_distance[range_count];
    }

    // There is a obstacle in front of the scooter. send 'A'
    for (range_count = 0; range_count < range_number/2; range_count++){
      if (((range_distance[range_count*2] < range_alarm_distance[range_count]) & valid_distance[range_count*2]) || ((range_distance[range_count*2 + 1] < range_alarm_distance[range_count]) & valid_distance[range_count*2 + 1])){
        current_status = 'A';
        sendDirectionMessage(alarm_message[range_count]);
      } else{
        sendDirectionMessage(safe_message[range_count]);
      }
    }

    if(btFlag == 0){
      while(USART_GetFlagStatus(USARTy, USART_FLAG_TXE) == RESET);
      USART_SendData(USARTy, current_status);
    }
    vTaskDelay(100);
  }
}

void sendDirectionMessage(char c){
  if(btFlag == 0){
    while(USART_GetFlagStatus(USARTy, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTy, c);
  }
  vTaskDelay(10);
}
#if 0
static void external_interrupt_task(void *pvParameters){
  PWM1 = 0;
  
  while(1){
    v_tmp = ((1000*1000/diffTime)*(3600/1000)) / 1000;
    if (v_tmp >= 0 && v_tmp < 30)
    {
      //put_int(v_tmp);
    }
    vTaskDelay(500);
  }
}
#endif

#if 0
void put_int(uint32_t number){
  btFlag = 1;
  unsigned char char_buff[10] = {0};
  int count = 0;
  
  /* Trnasfer integer to chars */
  itoa(number, char_buff, 10);
  
  while(USART_GetFlagStatus(USARTy, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTy, 'S');

  vTaskDelay(1);

  /* Send the char buffer to usart */
  while(count < 2){
    while(USART_GetFlagStatus(USARTy, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTy, char_buff[count]);
    count ++;
  }

  vTaskDelay(1);
  while(USART_GetFlagStatus(USARTy, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTy, 'E');

  vTaskDelay(1);
  btFlag = 0;
}
#endif
/**
  * @brief  Use this function to check index of receive data is in the range of correct index number.
  * @param  None
  * @retval None
  */
int check_index(unsigned int index){
  if(index >= 0xA0 && index < 0xFA){ 
    return 1;
  }
  else{ 
    return 0;
  }
}

/**
  * @brief  Use this function to check the packets are correct.
  * @param  None
  * @retval None
  */
int check_sum(){
  int count;
  int transfer;
  long check32 = 0;
  long checksum;
  long received_checksum;

  for (count = 0; count < 10; count++){
    transfer = current_packet[2 * count] + ((int)current_packet[2 * count + 1] << 8);
    check32 = (check32 << 1) + transfer;
  }

  checksum = (check32 & 0x7FFF) + (check32 >> 15);
  checksum = checksum & 0x7FFF;
  received_checksum = current_packet[20] + ((int)current_packet[21] << 8);
  
  if (checksum == received_checksum){
    return 1;
  }
  else{
    return 0;
  }
}

/**
  * @brief  This function write the correct data to lidarBuffer.
  * @param  None
  * @retval None
  */
void write_packet(){
  int count = 0;
  int index = current_packet[1] - 160/* index start from (A0)16 = (160)(10)*/;

  for(; count < 16; count++){
    lidarBuffer[index].data[count] = current_packet[count];
  }
  writed_packet_num ++;
  //PWM5 = (writed_packet_num/100);  // check for receive data
}

/**
  * @brief  This function reset the flags and counts, and reenable the Interrrupt of Usart.
  * @param  None
  * @retval None
  */
void reset_tmp(){
  /* enable USART interrupt */
  NVIC_EnableIRQ(USARTx_IRQn);
  //NVIC_EnableIRQ(USARTy_IRQn);
  
  current_received_byte = 0;
  bt_received_byte = 0;
  started_flag = 0;
}

/**
  * @brief  This function convert the distance data from byte format.
  * @param  None
  * @retval None
  */
uint16_t distance_convert(uint8_t data0, uint8_t data1){
  uint16_t distance_data = ((0x00FF & data1) << 8) | data0;
  
  if ((distance_data & 0x8000) == 0x8000){
    //return 6000;/*if the data is invalid, return the maximum value.*/
    return -1;
  }
  else{
    return distance_data & 0x3FFF;
  }
}
#if 0
/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  if(UserButtonPressed){
    UserButtonPressed = 0x00;
  }else
    UserButtonPressed = 0x01;

}
#endif
#if 1
// USART3 IRQ Handler. Receive LiDAR data.
void USARTx_IRQHandler(void)
{
  /* USART3 in Receiver mode */
  if ((USART_GetITStatus(USARTx, USART_IT_RXNE) == SET))
  {
    /* Receive the data */
    LiDAR_RxBuffer = USART_ReceiveData(USARTx);
    //printf("%d ", LiDAR_RxBuffer);
    
    if(LiDAR_RxBuffer == 250/* FA(HEX) = 250(DEC) */){
      started_flag = 1;
    }

    /* after received start byte, storing the data to current_packet[] */
    if(started_flag == 1 && (current_received_byte < LidarPacketSize)){
      current_packet[current_received_byte] = LiDAR_RxBuffer;
      current_received_byte ++;
    }
  }
}
#endif
#if 1
// USART6 IRQ Handler. Receive BT instructions.
void USARTy_IRQHandler(void)
{
  /* USART6 in Receive Mode */
  if(USART_GetITStatus(USARTy, USART_IT_RXNE))
  {
    
    static uint8_t cnt = 0; // this counter is used to determine the string length
    BT_RxBuffer = USART_ReceiveData(USARTy);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    //if(!Receive_String_Ready){
      if(cnt < 16){
          received_string[cnt] = BT_RxBuffer;
          if(BT_RxBuffer=='0') GPIO_ToggleBits(GPIOD,GPIO_Pin_15);

          /*start determine the period of command.*/
          if(BT_RxBuffer=='\r'){
              Receive_String_Ready = 1; /*Ready to parse the command */
              cnt=0; /*restart to accept next stream message.*/
          }
          else{
            cnt++;
          }
      }
      else{ // over the max string length, cnt return to zero.
        Receive_String_Ready=1;
        cnt = 0;  
      }
    //}
    #if 1
    if(Receive_String_Ready){
      USART_puts(USART6, received_string);
      USART_puts(USART6, "\r\n");
      //receive_command();
      /*clear the received string and the flag*/
      Receive_String_Ready = 0;
      int i;
      for( i = 0 ; i < 16 ; i++){
        received_string[i]= 0;
      }      
    }
    #endif
}
}
#endif

#if 1
// USART3 IRQ Handler. Receive LiDAR data.
void USARTz_IRQHandler(void)
{
  /* USART3 in Receiver mode */
  if ((USART_GetITStatus(USARTz, USART_IT_RXNE) == SET))
  {
    static uint8_t count = 0; // this counter is used to determine the string length
    /* Receive the data */
    Pi_RxBuffer = USART_ReceiveData(USARTz);
    
    if(count < 16){
      pi_received_string[count] = Pi_RxBuffer;
      if (Pi_RxBuffer=='\r'){
          Pi_Receive_String_Ready = 1;
          count = 0;
      }
      else{
        count++;
      }
    }
    else{
      Pi_Receive_String_Ready = 1;
      count = 0;
    }
    if(Pi_Receive_String_Ready){
      USART_puts(USART1, pi_received_string);
      USART_puts(USART1, "\r\n");
      /*clear the received string and the flag*/
      Pi_Receive_String_Ready = 0;
      int j;
      for( j = 0 ; j < 16 ; j++){
        pi_received_string[j]= 0;
      }
    }      
  }
}
#endif


int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Configure SysTick */
  RCC_GetClocksFreq(&RCC_Clocks);

  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  SystemInit();

  //PSRAM_Init();
  //Configure_PB12();
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize LEDs to be managed by GPIO */
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDToggle(LED5);
  STM_EVAL_LEDToggle(LED6);
  // LiDAR
  USART3_Config(115200);
  // Bluetooth
  USART6_Config(9600);
  // STM32 -> Pi
  USART1_Config(9600);

  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

  kaman_init(&kalman_s);

  init_motor();
  forward(0);

  //MPU6050_Initialize();
  //MPU6050_Initialize();

  GPIO_SetBits(GPIOD, GPIO_Pin_13);
  //Configure_PB12();
  xTaskCreate(usart_receive_task,(signed portCHAR *) "Implement USART",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(bt_receive_task,(signed portCHAR *) "BT Receive",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(pi_receive_task,(signed portCHAR *) "Pi USART",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(Transfer_Distance_task,(signed portCHAR *) "Implement Transfer distance.",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(Front_Obstacle_task,(signed portCHAR *) "Implement front obstacle detect.",512 /* stack size */, NULL, tskIDLE_PRIORITY + 3, NULL);
  
  /* Start running the tasks. */
  vTaskStartScheduler(); 

  return 0;
}
#if 0
void Configure_PB12(void) {
  /* Set variables used */
  GPIO_InitTypeDef GPIO_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  /* Enable clock for GPIOB */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Set pin as input */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Tell system that you will use PB12 for EXTI_Line12 */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
  
  /* PB12 is connected to EXTI_Line12 */
  EXTI_InitStruct.EXTI_Line = EXTI_Line12;
  /* Enable interrupt */
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  /* Interrupt mode */
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  /* Triggers on rising and falling edge */
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  /* Add to EXTI */
  EXTI_Init(&EXTI_InitStruct);

  /* Add IRQ vector to NVIC */
  /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
  NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
  /* Set priority */
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
  /* Set sub priority */
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
  /* Enable interrupt */
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  /* Add to NVIC */
  NVIC_Init(&NVIC_InitStruct);
}
#endif
void EXTI15_10_IRQHandler(void) {
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
    oldTime = currentTime;
    currentTime = Timing;

    if (currentTime > oldTime)
    {
      diffTime = currentTime - oldTime;
    }

    currentTime = Timing;    
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */ 
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
  
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    vTaskDelay(5);
  }
}

/*  Frequency = 116kHz 
    it can be config in FreeRTOSConfig.h
*/
void vApplicationTickHook()
{
    Timing ++;
}
