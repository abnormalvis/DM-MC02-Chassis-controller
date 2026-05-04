#include "motor.h"

/* TIM handles defined in main.c; MX_TIM1_Init()/MX_TIM2_Init() must be called
 * by main.c before Motor_Init() is called. */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void Motor_Init(void)
{
    /* Start all 4 PWM channels (TIM counter starts here → TIM1 TRGO fires → ADC triggered) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* Motor A FWD — PE9  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  /* Motor A REV — PE13 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  /* Motor B FWD — PA0  */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* Motor B REV — PA1  */

    Motor_BrakeAll();
}

void Motor_SetOutput(motor_id_t id, float output_mA, float max_output_mA)
{
    float abs_out = (output_mA < 0.0f) ? -output_mA : output_mA;
    float ratio = (max_output_mA > 0.0f) ? (abs_out / max_output_mA) : 0.0f;
    if (ratio > 1.0f) { ratio = 1.0f; }

    uint32_t cmp = (uint32_t)(ratio * (float)MOTOR_PWM_MAX_CMP);

    if (id == MOTOR_A) {
        if (output_mA > 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        } else if (output_mA < 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp);
        } else {
            Motor_Brake(MOTOR_A);
        }
    } else {
        if (output_mA > 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cmp);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
        } else if (output_mA < 0.0f) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, cmp);
        } else {
            Motor_Brake(MOTOR_B);
        }
    }
}

void Motor_Brake(motor_id_t id)
{
    if (id == MOTOR_A) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    }
}

void Motor_BrakeAll(void)
{
    Motor_Brake(MOTOR_A);
    Motor_Brake(MOTOR_B);
}
