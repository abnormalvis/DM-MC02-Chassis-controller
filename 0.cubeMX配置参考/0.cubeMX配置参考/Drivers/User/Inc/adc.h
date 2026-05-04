/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   ADC configuration for current sensing
  *          Channel 1 (PA5, ADC_CHANNEL_19): Primary current
  *          Channel 2 (PC4, ADC_CHANNEL_4):  Secondary current
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ADC channel count for DMA buffer */
#define ADC1_CHANNEL_COUNT  2U

/* ADC handle */
extern ADC_HandleTypeDef hadc1;

/**
  * @brief  Initialize ADC1 for current sensing with DMA
  * @retval None
  */
void ADC_Init(void);

/**
  * @brief  Start ADC1 DMA transfer
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_StartDMA(void);

/**
  * @brief  Get ADC conversion results
  * @param  buffer Pointer to receive 2 uint16_t values (Channel 1 current, Channel 2 current)
  * @retval None
  */
void ADC_GetValues(uint16_t *buffer);

/**
  * @brief  Convert ADC raw value to current (mA)
  *         Assuming 3.3V reference, 12-bit resolution, 10mV/A sensor
  * @param  raw_value ADC raw value
  * @retval Current in mA
  */
float ADC_RawToCurrent_mA(uint16_t raw_value);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
