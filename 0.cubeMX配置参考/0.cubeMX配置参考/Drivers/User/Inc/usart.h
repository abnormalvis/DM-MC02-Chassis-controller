#ifndef __USART_H
#define __USART_H

#include "stm32h7xx_hal.h"
#include "stdio.h"

/*-------------------------------------------- USART config ---------------------------------------*/

#define  USART1_BaudRate  115200

#define  USART2_BaudRate  115200

#define  USART1_TX_PIN								GPIO_PIN_9										// TX pin
#define	USART1_TX_PORT							GPIOA												// TX GPIO port
#define 	GPIO_USART1_TX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE()		// TX GPIO clock


#define  USART1_RX_PIN								GPIO_PIN_10             			// RX pin
#define	USART1_RX_PORT							GPIOA                  				// RX GPIO port
#define 	GPIO_USART1_RX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE()		// RX GPIO clock

#define  USART2_TX_PIN								GPIO_PIN_2										// TX pin
#define	USART2_TX_PORT							GPIOA												// TX GPIO port
#define 	GPIO_USART2_TX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE()		// TX GPIO clock


#define  USART2_RX_PIN								GPIO_PIN_3              			// RX pin
#define	USART2_RX_PORT							GPIOA                  				// RX GPIO port
#define 	GPIO_USART2_RX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE()		// RX GPIO clock


/*---------------------------------------------- Function API ---------------------------------------*/

void USART1_Init(void) ;	// Initialize USART1
void USART2_Init(void) ;	// Initialize USART2

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#endif //__USART_H





