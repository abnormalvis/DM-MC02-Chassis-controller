/***
	*************************************************************************************************
	*	@file  	led.c
	*	@version V1.0
	*	@author  Demo Project
	*	@brief   LED initialization and control
   *************************************************************************************************
   *  @description
	*
	*	Configure LED GPIO as push-pull output.
	*
	************************************************************************************************
***/

#include "stm32h7xx_hal.h"
#include "led.h"  


void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_LED1_CLK_ENABLE;		// Enable LED1 GPIO clock


	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);		// Default ON for active-low LED


	GPIO_InitStruct.Pin 		= LED1_PIN;					// LED1 pin
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;	// Push-pull output
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;				// No pull-up/down
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;	// Low speed
	HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);


}

