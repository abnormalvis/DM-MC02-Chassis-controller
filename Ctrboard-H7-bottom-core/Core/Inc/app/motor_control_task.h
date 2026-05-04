/**
 * @file motor_control_task.h
 * @brief 电机控制任务
 * @description 双电机目标控制与本地 PWM 输出
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
    float deadzone;
} pid_param_t;

/* 电机控制状态 */
typedef struct {
    int16_t current_a;         /* 当前电流 0.01A (本地ADC估算) */
    int16_t current_b;         /* 当前电流 0.01A (本地ADC估算) */
    int16_t target_rpm_a;      /* 兼容保留，当前用于目标电流A(0.01A) */
    int16_t target_rpm_b;      /* 兼容保留，当前用于目标电流B(0.01A) */
    int16_t actual_rpm_a;      /* 兼容保留，当前用于PWM命令A */
    int16_t actual_rpm_b;      /* 兼容保留，当前用于PWM命令B */
    bool brake_active;        /* 刹车激活 */
} motor_control_state_t;

/* PID 控制模式 */
typedef enum {
    PID_MODE_DISABLED = 0,
    PID_MODE_SPEED,         /* 速度环 PID */
    PID_MODE_CURRENT        /* 电流环 PID (预留) */
} pid_mode_t;

/**
 * @brief 电机控制任务初始化
 */
void MotorControlTask_Init(void);

/**
 * @brief 设置电机目标速度
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
 * @brief 设置控制标志位 (兼容保留)
 */
void MotorControlTask_SetControlFlags(uint8_t flags);

/**
 * @brief 获取控制标志位
 */
uint8_t MotorControlTask_GetControlFlags(void);

/**
 * @brief 设置 PWM 上限
 */
void MotorControlTask_SetMaxDuty(float max_duty);

/**
 * @brief 设置 PID 参数
 */
void MotorControlTask_SetPID(motor_id_t motor, const pid_param_t *param);

/**
 * @brief 设置 PID 模式
 */
void MotorControlTask_SetPIDMode(pid_mode_t mode);

/**
 * @brief 电机控制任务处理 (1kHz)
 */
void MotorControlTask_Process(void *argument);

/**
 * @brief 获取电机状态
 */
void MotorControlTask_GetState(motor_control_state_t *state);

#endif /* MOTOR_CONTROL_TASK_H */
