/**
 * @file motor_control_task.h
 * @brief 电机控制任务
 * @description 双电机速度控制、PWM输出、电流环
 */

#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* 电机编号 */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1
} motor_id_t;

/* PWM 输出结构 */
typedef struct {
    float duty_a_fwd;     /* 电机A正转占空比 [0,1] */
    float duty_a_rev;     /* 电机A反转占空比 [0,1] */
    float duty_b_fwd;     /* 电机B正转占空比 [0,1] */
    float duty_b_rev;     /* 电机B反转占空比 [0,1] */
} pwm_output_t;

/* PID 参数结构 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float max_integral;
} pid_param_t;

/* 电机控制状态 */
typedef struct {
    int16_t current_a;        /* 当前电流 0.01A */
    int16_t current_b;        /* 当前电流 0.01A */
    int16_t target_rpm_a;    /* 目标转速 rpm */
    int16_t target_rpm_b;    /* 目标转速 rpm */
    int16_t actual_rpm_a;     /* 实际转速 rpm */
    int16_t actual_rpm_b;    /* 实际转速 rpm */
    bool brake_active;        /* 刹车激活 */
} motor_control_state_t;

/**
 * @brief 电机控制任务初始化
 */
void MotorControlTask_Init(void);

/**
 * @brief 设置电机目标速度
 * @param rpm_a 电机A目标转速
 * @param rpm_b 电机B目标转速
 */
void MotorControlTask_SetTarget(int16_t rpm_a, int16_t rpm_b);

/**
 * @brief 触发立即刹车
 */
void MotorControlTask_Brake(void);

/**
 * @brief 释放刹车
 */
void MotorControlTask_ReleaseBrake(void);

/**
 * @brief 设置 PWM 上限
 * @param max_duty 最大占空比 [0,1]
 */
void MotorControlTask_SetMaxDuty(float max_duty);

/**
 * @brief 设置 PID 参数
 * @param motor 电机编号
 * @param param PID 参数
 */
void MotorControlTask_SetPID(motor_id_t motor, const pid_param_t *param);

/**
 * @brief 电机控制任务处理 (在 1kHz 任务中调用)
 */
void MotorControlTask_Process(void);

/**
 * @brief 获取电机状态
 */
void MotorControlTask_GetState(motor_control_state_t *state);

#endif /* MOTOR_CONTROL_TASK_H */
