#include "usart.h"
#include "main.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "motor.h"


#define TimPeriod 1200
#define TimPrescaler 7
#define SpeedValue 595 //SpeedValue = TimPeriod * duty cycle (1200*0.5)
uint32_t SpeedValue_left = SpeedValue;
uint32_t SpeedValue_right = SpeedValue;


void motorForward(){
  GPIO_WriteBit(MOTOR_PWM_PORT ,MOTOR_LEFT_IN1_PIN,Bit_SET);    /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN2_PIN,Bit_RESET);   /* 0 */
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN3_PIN,Bit_SET);    /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN4_PIN,Bit_RESET);  /* 0 */
  TIM_SetCompare3(TIM4, 1200);    
  TIM_SetCompare4(TIM4, 1200);
}
void motorStop(){
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN1_PIN,Bit_RESET);   /* 0 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN2_PIN,Bit_RESET);   /* 0 */
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN3_PIN,Bit_RESET);  /* 0 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN4_PIN,Bit_RESET);  /* 0 */
  TIM_SetCompare3(TIM4, 0);
  TIM_SetCompare4(TIM4, 0);
}
void motorLeft(uint32_t lValue1, uint32_t rValue1){
  // Left Motor
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN1_PIN,Bit_SET);   /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN2_PIN,Bit_RESET);   /* 0 */
  TIM_SetCompare3(TIM4, lValue1);
  // Right Motor
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN3_PIN,Bit_SET);  /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN4_PIN,Bit_RESET);  /* 0 */
  TIM_SetCompare4(TIM4, rValue1);
}
void motorRight(uint32_t lValue2, uint32_t rValue2){
  // Left Motor
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN1_PIN,Bit_SET);   /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN2_PIN,Bit_RESET);   /* 0 */
  TIM_SetCompare3(TIM4, lValue2);
  // Right Motor
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN3_PIN,Bit_SET);  /* 1 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN4_PIN,Bit_RESET);  /* 0 */
  TIM_SetCompare4(TIM4, rValue2);
}

/* STOP: Reset PWM value to initial setting
 * EX:mStop(1,0) to stop left motor
 *    mStop(1,1) to stop both motors */
void mStop(uint8_t mstop){
  if((mstop == mLeft) || (mstop == mBoth)){
    SpeedValue_left = SpeedValue;
    TIM_SetCompare3(TIM4, SpeedValue);
  }
  if((mstop == mRight) || (mstop == mBoth)){
    SpeedValue_right = SpeedValue;
    TIM_SetCompare4(TIM4, SpeedValue);
  }
}

void mMove(uint32_t SpeedValue_left, uint32_t SpeedValue_right){
  TIM_SetCompare3(TIM4, SpeedValue_left);
  TIM_SetCompare4(TIM4, SpeedValue_right);
}

void init_motor(void){
  init_motorPWM();
  init_motor_driver();
}

void init_motor_driver(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Enable GPIO D clock. */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_IN1_PIN| MOTOR_LEFT_IN2_PIN | MOTOR_RIGHT_IN3_PIN | MOTOR_RIGHT_IN4_PIN ;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( MOTOR_PWM_PORT, &GPIO_InitStruct ); 
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN1_PIN,Bit_RESET);   /* 0 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_LEFT_IN2_PIN,Bit_RESET);   /* 0 */
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN3_PIN,Bit_RESET);  /* 0 */       
  GPIO_WriteBit(MOTOR_PWM_PORT,MOTOR_RIGHT_IN4_PIN,Bit_RESET);  /* 0 */
}

void init_motorPWM(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable TIM4 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* Enable PWM pins for the motors */
  GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_PWM_PIN | MOTOR_RIGHT_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);
  
  /*Connect TIM4 pins to AF*/
  GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource15, GPIO_AF_TIM4);

  /* -----------------------------------------------------------------------
    TIM4 Configuration: generate 4 PWM signals with 4 different duty cycles.
    
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM4CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM4 counter clock at 12 MHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM4 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /12 MHz) - 1
                 = 7 - 1
                                              
    To get TIM4 output clock at 10 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1
           = 1200
     If TIM4_CCR1 = 600, TIM4_CCR2 = 450, TIM4_CCR3 = 300, TIM4_CCR4 = 150
    then
    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 37.5%
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 25%
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 12.5%
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */ 
  /* this case : 10kHz 
   * 84MHz/10kHz = 8400
   * you can adjust the value of the two:
   * Prescaler = 7 - 1
   * Period = 1200 - 1
   */

  /* Time base configuration*/
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = TimPeriod - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = TimPrescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* PWM setting */
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  /* initial value */
  TIM_OCInitStructure.TIM_Pulse = SpeedValue;//in this case(duty cycle:50%)

  //PWM1 Mode Configuration: TIM4 Channel3 (MOTOR_LEFT_PWM_PIN)
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  //PWM1 Mode Configuration: TIM4 Channel4 (MOTOR_RIGHT_PWM_PIN)
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_Cmd(TIM4, ENABLE);
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