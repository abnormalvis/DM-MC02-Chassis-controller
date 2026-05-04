#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*VOFA_ParamCallback)(uint16_t id, float value);

/* VOFA parameter IDs for command frames: #P<id>=<value>!
 * IDs must match the channel numbers configured in the VOFA host software. */
#define VOFA_PARAM_ID_CURRENT_FWD_KP 1U
#define VOFA_PARAM_ID_CURRENT_FWD_KI 2U
#define VOFA_PARAM_ID_CURRENT_FWD_KD 6U
#define VOFA_PARAM_ID_CURRENT_REV_KP 3U
#define VOFA_PARAM_ID_CURRENT_REV_KI 4U
#define VOFA_PARAM_ID_CURRENT_REV_KD 7U
#define VOFA_PARAM_ID_TARGET_MA      5U

/* VOFA waveform channel map used by VOFA_SendFloats */
#define VOFA_CH_TARGET_MA            0U
#define VOFA_CH_FEEDBACK_MA          1U
#define VOFA_CH_OUTPUT_MA            2U
#define VOFA_CH_ERROR_MA             3U
#define VOFA_CH_KP                   4U
#define VOFA_CH_KI                   5U
#define VOFA_CH_KD                   6U
#define VOFA_CH_AUX_CURRENT_MA       7U

#define VOFA_TELEMETRY_COUNT         8U

void VOFA_Init(void);
void VOFA_SetParamCallback(VOFA_ParamCallback callback);
HAL_StatusTypeDef VOFA_Begin(void);
void VOFA_ProcessRx(void);
void VOFA_OnUartRxCplt(UART_HandleTypeDef *huart);
void VOFA_OnUartError(UART_HandleTypeDef *huart);
HAL_StatusTypeDef VOFA_SendFloats(const float *data, uint8_t count);
HAL_StatusTypeDef VOFA_SendOneFloat(float value);
uint32_t VOFA_GetUartErrorCount(void);
uint32_t VOFA_GetUartLastError(void);
uint32_t VOFA_GetUartLastIsr(void);

#ifdef __cplusplus
}
#endif


#endif //__VOFA_H





