#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"

enum {
	mLeft,
	mRight,
	mBoth
};

/* Define all pins of the motor of EPW2 */
#define MOTOR_PWM_PORT				GPIOD

/* Constant Voltage for motor driver initialization 
#define MOTOR_LEFT_CV_PIN			GPIO_Pin_12 //Green  TIM4_CH1
#define MOTOR_RIGHT_CV_PIN			GPIO_Pin_13 //Orange TIM4_CH2
*/
/* L298N */
#define MOTOR_LEFT_IN1_PIN          GPIO_Pin_9                                        
#define MOTOR_LEFT_IN2_PIN          GPIO_Pin_10
#define MOTOR_RIGHT_IN3_PIN         GPIO_Pin_11
#define MOTOR_RIGHT_IN4_PIN         GPIO_Pin_12
/* Enable Pin */
#define MOTOR_LEFT_PWM_PIN			GPIO_Pin_14 //Red  TIM4_CH3    Green   PD12
#define MOTOR_RIGHT_PWM_PIN			GPIO_Pin_15 //Blue TIM4_CH4    Orange  PD13

void init_motorPWM(void);
void init_motor_driver(void);
void motorForward();
void motorStop();
void init_motor(void);
void mMove(uint32_t SpeedValue_left, uint32_t SpeedValue_right);
void mStop(uint8_t mstop);
void motorLeft(uint32_t lValue1, uint32_t rValue1);
void motorRight(uint32_t lValue2, uint32_t rValue2);
#endif