#ifndef USART_H
#define USART_H
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

//usart6
//#define USARTx                           USART3
#define USARTx_CLK                       RCC_APB1Periph_USART3
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_10                
#define USARTx_TX_GPIO_PORT              GPIOB                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_TX_SOURCE                 GPIO_PinSource10
#define USARTx_TX_AF                     GPIO_AF_USART3

#define USARTx_RX_PIN                    GPIO_Pin_11                
#define USARTx_RX_GPIO_PORT              GPIOB                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define USARTx_RX_SOURCE                 GPIO_PinSource11
#define USARTx_RX_AF                     GPIO_AF_USART3

// usart6
#define USARTy                           USART6
#define USARTy_CLK                       RCC_APB2Periph_USART6
#define USARTy_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTy_IRQn                      USART6_IRQn
#define USARTy_IRQHandler                USART6_IRQHandler

#define USARTy_TX_PIN                    GPIO_Pin_6                
#define USARTy_TX_GPIO_PORT              GPIOC                       
#define USARTy_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_TX_SOURCE                 GPIO_PinSource6
#define USARTy_TX_AF                     GPIO_AF_USART6

#define USARTy_RX_PIN                    GPIO_Pin_7                
#define USARTy_RX_GPIO_PORT              GPIOC                    
#define USARTy_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_RX_SOURCE                 GPIO_PinSource7
#define USARTy_RX_AF                     GPIO_AF_USART6


/*the usart acept the command from RX when RX interrupt is trigger*/
unsigned char Receive_data ;

/*Setting the USART MAX string lenth */
#define MAX_STRLEN 16 // this is the maximum string length of our string in characters

volatile unsigned char received_string[MAX_STRLEN]; // this will hold the recieved string

// uint32_t is from "stm32f4_discovery.h"
extern void USART3_IRQHandler(void);
void USART1_Config(uint32_t baudrate1);
void USART_Config(uint32_t baudrate);
//void init_USART();
extern void USART_puts(USART_TypeDef* USARTx, volatile uint8_t *s);
extern void USART_putd(USART_TypeDef* USARTx, uint32_t number);
void send_byte(char ch);
char receive_byte();

#endif