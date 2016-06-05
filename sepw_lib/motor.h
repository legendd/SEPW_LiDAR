#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"

/* Define all pins of the motor of EPW2 */
#define MOTOR_PWM_PORT				GPIOD

/* Constant Voltage for motor driver initialization 
#define MOTOR_LEFT_CV_PIN			GPIO_Pin_12 //Green  TIM4_CH1
#define MOTOR_RIGHT_CV_PIN			GPIO_Pin_13 //Orange TIM4_CH2
*/
/* Control Signal */
#define MOTOR_LEFT_PWM_PIN			GPIO_Pin_14 //Red  TIM4_CH3
#define MOTOR_RIGHT_PWM_PIN			GPIO_Pin_15 //Blue TIM4_CH4

void Configure_PB12(void);
void init_motorPWM(void);
void init_motor(void);
void forward(uint32_t SpeedValue_left);
#endif