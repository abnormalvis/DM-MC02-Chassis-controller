/**
 * @file motor_control_task.c
 * @brief 电机控制任务实现
 * @description 双电机 PWM 输出、速度控制、电流环内环
 */

#include "motor_control_task.h"
#include "main.h"
#include "tim.h"
#include "can_task.h"
#include <string.h>
#include <math.h>

/* ============================================
 * 配置参数
 * ============================================ */
#define DEFAULT_MAX_DUTY       0.90f       /* 默认最大占空比 90% */
#define DUTY_RAMP_RATE        0.02f       /* 每 1ms 最大变化 2% */
#define BRAKE_DUTY           0.0f       /* 刹车时占空比 */

/* PID 计算周期 1ms */
#define PID_DT               0.001f

/* 电流环采样比例 (根据实际标定) */
#define CURRENT_TO_DUTY    0.001f   /* 电流 0.01A → 占空比 */

/* ============================================
 * 私有变量
 * ============================================ */
static pwm_output_t s_pwm_output = {0};
static motor_control_state_t s_motor_state = {0};
static pid_param_t s_pid_a;
static pid_param_t s_pid_b;
static float s_max_duty = DEFAULT_MAX_DUTY;
static uint8_t s_brake_active = 0;
static pid_mode_t s_pid_mode = PID_MODE_SPEED;

/* PID 积分项 */
static float s_integral_a = 0.0f;
static float s_integral_b = 0.0f;
static float s_last_error_a = 0.0f;
static float s_last_error_b = 0.0f;

/* ============================================
 * PID 计算 (速度环)
 * ============================================ */
static float pid_calculate(float target, float actual, float *integral, float *last_error, const pid_param_t *pid)
{
    /* 计算误差 */
    float error = target - actual;

    /* 死区处理 */
    if (pid->deadzone > 0 && fabsf(error) < pid->deadzone)
    {
        error = 0;
    }

    /* 积分项 */
    *integral += error * PID_DT;

    /* 积分限幅 */
    if (pid->max_integral > 0)
    {
        if (*integral > pid->max_integral) *integral = pid->max_integral;
        if (*integral < -pid->max_integral) *integral = -pid->max_integral;
    }

    /* 微分项 */
    float derivative = (error - *last_error) / PID_DT;
    *last_error = error;

    /* PID 输出 */
    float output = pid->kp * error + pid->ki * *integral + pid->kd * derivative;

    /* 输出限幅 */
    if (pid->max_output > 0)
    {
        if (output > pid->max_output) output = pid->max_output;
        if (output < -pid->max_output) output = -pid->max_output;
    }

    return output;
}

/* ============================================
 * PWM 应用函数
 * ============================================ */
static void pwm_apply_output(void)
{
    /* 获取周期 */
    uint32_t period_a = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t period_b = __HAL_TIM_GET_AUTORELOAD(&htim2);

    /* 斜坡限幅 */
    static pwm_output_t last_pwm = {0};
    if (!s_brake_active)
    {
        float delta;

        delta = s_pwm_output.duty_a_fwd - last_pwm.duty_a_fwd;
        if (delta > DUTY_RAMP_RATE) s_pwm_output.duty_a_fwd = last_pwm.duty_a_fwd + DUTY_RAMP_RATE;
        if (delta < -DUTY_RAMP_RATE) s_pwm_output.duty_a_fwd = last_pwm.duty_a_fwd - DUTY_RAMP_RATE;

        delta = s_pwm_output.duty_a_rev - last_pwm.duty_a_rev;
        if (delta > DUTY_RAMP_RATE) s_pwm_output.duty_a_rev = last_pwm.duty_a_rev + DUTY_RAMP_RATE;
        if (delta < -DUTY_RAMP_RATE) s_pwm_output.duty_a_rev = last_pwm.duty_a_rev - DUTY_RAMP_RATE;

        delta = s_pwm_output.duty_b_fwd - last_pwm.duty_b_fwd;
        if (delta > DUTY_RAMP_RATE) s_pwm_output.duty_b_fwd = last_pwm.duty_b_fwd + DUTY_RAMP_RATE;
        if (delta < -DUTY_RAMP_RATE) s_pwm_output.duty_b_fwd = last_pwm.duty_b_fwd - DUTY_RAMP_RATE;

        delta = s_pwm_output.duty_b_rev - last_pwm.duty_b_rev;
        if (delta > DUTY_RAMP_RATE) s_pwm_output.duty_b_rev = last_pwm.duty_b_rev + DUTY_RAMP_RATE;
        if (delta < -DUTY_RAMP_RATE) s_pwm_output.duty_b_rev = last_pwm.duty_b_rev - DUTY_RAMP_RATE;
    }
    last_pwm = s_pwm_output;

    /* 限制最大值 */
    float duty_a_fwd = s_pwm_output.duty_a_fwd * s_max_duty;
    float duty_a_rev = s_pwm_output.duty_a_rev * s_max_duty;
    float duty_b_fwd = s_pwm_output.duty_b_fwd * s_max_duty;
    float duty_b_rev = s_pwm_output.duty_b_rev * s_max_duty;

    /* 应用到 TIM1 (电机A) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a_fwd * period_a));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_a_rev * period_a));

    /* 应用到 TIM2 (电机B) */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty_b_fwd * period_b));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(duty_b_rev * period_b));
}

/* ============================================
 * 任务初始化
 * ============================================ */
void MotorControlTask_Init(void)
{
    /* 初始化 PID 参数默认 (速度环) */
    s_pid_a.kp = 0.1f;
    s_pid_a.ki = 0.01f;
    s_pid_a.kd = 0.0f;
    s_pid_a.max_output = 1.0f;
    s_pid_a.max_integral = 0.5f;
    s_pid_a.deadzone = 5.0f;  /* 5 RPM 死区 */

    s_pid_b = s_pid_a;

    /* 确保 PWM 停止 */
    s_pwm_output.duty_a_fwd = 0;
    s_pwm_output.duty_a_rev = 0;
    s_pwm_output.duty_b_fwd = 0;
    s_pwm_output.duty_b_rev = 0;
    pwm_apply_output();
}

/* ============================================
 * 设置目标速度
 * ============================================ */
void MotorControlTask_SetTarget(int16_t rpm_a, int16_t rpm_b)
{
    if (s_brake_active) return;

    s_motor_state.target_rpm_a = rpm_a;
    s_motor_state.target_rpm_b = rpm_b;
}

/* ============================================
 * 刹车
 * ============================================ */
void MotorControlTask_Brake(void)
{
    s_brake_active = 1;
    s_motor_state.brake_active = 1;
    s_pwm_output.duty_a_fwd = BRAKE_DUTY;
    s_pwm_output.duty_a_rev = BRAKE_DUTY;
    s_pwm_output.duty_b_fwd = BRAKE_DUTY;
    s_pwm_output.duty_b_rev = BRAKE_DUTY;
    pwm_apply_output();
}

/* ============================================
 * 释放刹车
 * ============================================ */
void MotorControlTask_ReleaseBrake(void)
{
    s_brake_active = 0;
    s_motor_state.brake_active = 0;
}

/* ============================================
 * 设置 PWM 上限
 * ============================================ */
void MotorControlTask_SetMaxDuty(float max_duty)
{
    if (max_duty > 0.0f && max_duty <= 1.0f)
    {
        s_max_duty = max_duty;
    }
}

/* ============================================
 * 设置 PID 参数
 * ============================================ */
void MotorControlTask_SetPID(motor_id_t motor, const pid_param_t *param)
{
    if (param == NULL) return;

    if (motor == MOTOR_A)
    {
        memcpy(&s_pid_a, param, sizeof(pid_param_t));
    }
    else
    {
        memcpy(&s_pid_b, param, sizeof(pid_param_t));
    }
}

/* ============================================
 * 设置 PID 模式
 * ============================================ */
void MotorControlTask_SetPIDMode(pid_mode_t mode)
{
    s_pid_mode = mode;
}

/* ============================================
 * 任务处理 (1kHz)
 * ============================================ */
void MotorControlTask_Process(void *argument)
{
    (void)argument;

    /* 获取 CAN 电流数据 (C8T6 采集) */
    int16_t current_a = 0, current_b = 0;
    CANTask_GetCurrent(&current_a, &current_b);
    s_motor_state.current_a = current_a;
    s_motor_state.current_b = current_b;

    /* 处理刹车 */
    if (s_brake_active)
    {
        s_motor_state.target_rpm_a = 0;
        s_motor_state.target_rpm_b = 0;
        s_integral_a = 0;
        s_integral_b = 0;
        return;
    }

    /* 速度环 PID 控制 */
    float output_a = 0;
    float output_b = 0;

    if (s_pid_mode == PID_MODE_SPEED)
    {
        /* 速度环 PID: 根据目标转速输出占空比 */
        /* 注意: 这里暂时用目标 RPM 做开环，因为没有编码器反馈 */
        /* 后续可以通过电流反馈形成闭环 */
        output_a = (float)s_motor_state.target_rpm_a / 1000.0f;
        output_b = (float)s_motor_state.target_rpm_b / 1000.0f;

        /* 可选: 加入 PID 修正 (如果后续添加了速度反馈) */
        output_a = pid_calculate(
            (float)s_motor_state.target_rpm_a,
            0.0f, /* 暂时没有实际速度反馈 */
            &s_integral_a,
            &s_last_error_a,
            &s_pid_a
        );
        output_b = pid_calculate(
            (float)s_motor_state.target_rpm_b,
            0.0f,
            &s_integral_b,
            &s_last_error_b,
            &s_pid_b
        );
    }
    else
    {
        /* 开环 - 简单比例映射 */
        output_a = (float)s_motor_state.target_rpm_a / 1000.0f;
        output_b = (float)s_motor_state.target_rpm_b / 1000.0f;
    }

    /* 转换 PWM 占空比 (正/反转) */
    if (output_a >= 0)
    {
        s_pwm_output.duty_a_fwd = output_a;
        s_pwm_output.duty_a_rev = 0;
    }
    else
    {
        s_pwm_output.duty_a_fwd = 0;
        s_pwm_output.duty_a_rev = -output_a;
    }

    if (output_b >= 0)
    {
        s_pwm_output.duty_b_fwd = output_b;
        s_pwm_output.duty_b_rev = 0;
    }
    else
    {
        s_pwm_output.duty_b_fwd = 0;
        s_pwm_output.duty_b_rev = -output_b;
    }

    /* 应用 PWM */
    pwm_apply_output();
}

/* ============================================
 * 获取电机状态
 * ============================================ */
void MotorControlTask_GetState(motor_control_state_t *state)
{
    if (state != NULL)
    {
        *state = s_motor_state;
    }
}