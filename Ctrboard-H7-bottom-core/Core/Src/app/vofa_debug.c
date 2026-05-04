/*
 * @file vofa_debug.c
 * @brief USART10 Vofa telemetry for bare-metal control loop variables.
 */

#include "vofa_debug.h"

#include "crsf_task.h"
#include "main.h"
#include "motor_control_task.h"
#include "usart.h"

static const uint8_t s_vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};
static uint32_t s_last_tx_tick = 0U;

#define VOFA_DEBUG_PERIOD_MS  20U
#define VOFA_DEBUG_CHANNELS    9U

void VofaDebug_Init(void)
{
    s_last_tx_tick = HAL_GetTick();
}

static uint8_t VofaDebug_ReadInputs(float *values, uint8_t count)
{
    motor_control_state_t motor_state;
    crsf_input_t crsf_input;

    if ((values == NULL) || (count < VOFA_DEBUG_CHANNELS))
    {
        return 0U;
    }

    MotorControlTask_GetState(&motor_state);
    CRSFTask_GetInput(&crsf_input);

    values[0] = (float)motor_state.current_a;
    values[1] = (float)motor_state.current_b;
    values[2] = (float)motor_state.target_rpm_a;
    values[3] = (float)motor_state.target_rpm_b;
    values[4] = (float)motor_state.brake_active;
    values[5] = (float)crsf_input.throttle;
    values[6] = (float)crsf_input.steering;
    values[7] = (float)crsf_input.estop;
    values[8] = (float)crsf_input.valid;

    return 1U;
}

uint8_t VofaDebug_SendFrame(void)
{
    float values[VOFA_DEBUG_CHANNELS];

    if (VofaDebug_ReadInputs(values, VOFA_DEBUG_CHANNELS) == 0U)
    {
        return 0U;
    }

    if (HAL_UART_Transmit(&huart10, (uint8_t *)values, sizeof(values), 10U) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_UART_Transmit(&huart10, (uint8_t *)s_vofa_tail, sizeof(s_vofa_tail), 10U) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void VofaDebug_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed;

    elapsed = now - s_last_tx_tick;

    /* Handle uint32_t overflow: if elapsed looks too large (> 1 hour), reset */
    if (elapsed > 3600000U)
    {
        s_last_tx_tick = now;
        return;
    }

    if (elapsed < VOFA_DEBUG_PERIOD_MS)
    {
        return;
    }

    s_last_tx_tick = now;
    (void)VofaDebug_SendFrame();
}