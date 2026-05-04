/***
	************************************************************************************************
	*	@file  	usart.c
	*	@version V1.0
  *	@author  Demo Project
  *	@brief   USART initialization and callbacks
   ************************************************************************************************
   *  @description
	*
   *	Initialize UART peripherals and provide printf retarget + VOFA RX callbacks.
	*
	************************************************************************************************
***/


#include "usart.h"
#include "stm32h7xx_hal.h"
#include "vofa.h"
#include "crsf_task.h"

/*************************************************************************************************
*	Function:	USART1_Init
*	Input:		None
*	Return:		None
*	Description:	Initialize USART1
*************************************************************************************************/

void USART1_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {

  }
}

void USART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = USART2_BaudRate;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {

  }
}

/*************************************************************************************************
 *	Function:	fputc
*	Input:		ch: output character, f: file stream (unused)
*	Return:		written character, or EOF on failure
 *	Description:	Retarget printf output to USART1
*************************************************************************************************/

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
	return (ch);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
	{
		VOFA_OnUartRxCplt(huart);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
	{
		VOFA_OnUartError(huart);
	}
  else if (huart->Instance == USART3)
  {
    CRSF_OnUartError(huart);
  }
}

