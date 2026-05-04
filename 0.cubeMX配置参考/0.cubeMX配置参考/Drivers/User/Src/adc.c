/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   ADC initialization and current sensing implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "adc.h"
#include "stm32h7xx_hal.h"

/* DMA buffer for ADC results - 2 channels.
 * Align to a 32-byte cache line because the H7 D-Cache maintenance helper
 * operates in cache-line granularity. */
static uint16_t s_adc_dma_buffer[ADC1_CHANNEL_COUNT] __attribute__((aligned(32))) = {0};

/**
  * @brief  Initialize ADC1 for current sensing
  *         PA5 (ADC_CHANNEL_19): Primary current sensor
  *         PC4 (ADC_CHANNEL_4):  Secondary current sensor
  * @retval None
  */
void ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Configure ADC1 instance */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;            /* 16-bit resolution */
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;             /* Enable scan mode for multiple channels */
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;          /* EOC after each conversion */
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;               /* Disable continuous mode, use trigger */
    hadc1.Init.NbrOfConversion = 2;                        /* 2 channels */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO; /* TIM1 trigger */
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;  /* DMA circular mode */
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure Channel 1: PA5 (ADC_CHANNEL_19) */
    sConfig.Channel = ADC_CHANNEL_19;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;     /* Long sampling time for accurate current measurement */
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure Channel 2: PC4 (ADC_CHANNEL_4) */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Start ADC DMA transfer
  * @retval HAL status
  */
HAL_StatusTypeDef ADC_StartDMA(void)
{
    /* Start ADC with DMA */
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_adc_dma_buffer, ADC1_CHANNEL_COUNT);
}

/**
  * @brief  Get current ADC values from DMA buffer
  * @param  buffer Pointer to receive 2 uint16_t values
  * @retval None
  */
void ADC_GetValues(uint16_t *buffer)
{
    if (buffer != NULL)
    {
        /* Invalidate D-Cache before reading DMA destination to get fresh data */
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_adc_dma_buffer,
                                     (int32_t)sizeof(s_adc_dma_buffer));
        buffer[0] = s_adc_dma_buffer[0];  /* PA5 current */
        buffer[1] = s_adc_dma_buffer[1];  /* PC4 current */
    }
}

/**
  * @brief  Convert ADC raw value to current (mA)
  *         Bidirectional Hall sensor: 0A = 1.65V (midpoint), raw midpoint = 32768.
  *         Formula: Current(mA) = ((ADC_Value - 32768) * 3.3V / 65536) / 0.01ohm
  * @param  raw_value ADC raw value (0-65535)
  * @retval Current in mA (positive = forward, negative = reverse)
  */
float ADC_RawToCurrent_mA(uint16_t raw_value)
{
    /* Bidirectional Hall sensor: 1.65V = 0A, subtract midpoint before scaling.
     * Negated to match physical motor current direction (FWD drive → positive current). */
    float voltage_v = ((float)raw_value - 32768.0f) * 3.3f / 65536.0f;
    float current_ma = -(voltage_v / 0.01f);
    return current_ma;
}
