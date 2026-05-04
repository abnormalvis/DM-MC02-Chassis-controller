#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx_hal.h"

/* TIM1/TIM2: ARR = 12000-1 = 11999, clock = 240 MHz → 20 kHz PWM */
#define MOTOR_PWM_ARR      11999U
/* Cap at 99% duty: driver requires a switching waveform, not constant level */
#define MOTOR_PWM_MAX_CMP  (MOTOR_PWM_ARR * 99U / 100U)

typedef enum {
    MOTOR_A = 0,    /* TIM1 CH1=FWD(PE9)  CH3=REV(PE13) */
    MOTOR_B = 1,    /* TIM2 CH1=FWD(PA0)  CH2=REV(PA1)  */
} motor_id_t;

void Motor_Init(void);
void Motor_SetOutput(motor_id_t id, float output_mA, float max_output_mA);
void Motor_Brake(motor_id_t id);
void Motor_BrakeAll(void);

#endif /* __MOTOR_H */
