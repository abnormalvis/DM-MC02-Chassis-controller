/**
 * @file motor_control_task.c
 * @brief 电机控制任务实现
 * @description 双电机目标计算与本地PWM输出
 */

#include "motor_control_task.h"
#include "adc.h"
#include "crsf_task.h"
#include "main.h"
#include "tim.h"
#include <string.h>

/* ============================================
 * 配置参数
 * ============================================ */
#define DRIVE_CMD_LIMIT       1000
#define PWM_ARR_VALUE         999U
#define ADC_TO_CURRENT_SCALE   64U
#define ADC_MIDPOINT        32768U
#define CURRENT_TARGET_LIMIT  300
#define CRSF_DEADBAND         0.05f
#define CONTROL_DT_S          0.0001f

/* ============================================
 * 私有变量
 * ============================================ */
static motor_control_state_t s_motor_state = {0};
static uint8_t s_brake_active = 0;
static pid_mode_t s_pid_mode = PID_MODE_CURRENT;
static uint8_t s_control_flags = 0U;
static uint16_t s_pwm_a = 0U;
static uint16_t s_pwm_b = 0U;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_output;
    float max_integral;
} current_pid_t;

static current_pid_t s_pid_a = {1.2f, 35.0f, 0.0f, 0.0f, 0.0f, (float)DRIVE_CMD_LIMIT, 800.0f};
static current_pid_t s_pid_b = {1.2f, 35.0f, 0.0f, 0.0f, 0.0f, (float)DRIVE_CMD_LIMIT, 800.0f};

static uint16_t clamp_pwm(uint16_t value)
{
    if (value > PWM_ARR_VALUE)
    {
        return PWM_ARR_VALUE;
    }

    return value;
}

static void read_current_samples(int16_t *current_a, int16_t *current_b)
{
    const uint16_t *adc_samples;
    int32_t raw_a;
    int32_t raw_b;

    if ((current_a == NULL) || (current_b == NULL))
    {
        return;
    }

    adc_samples = ADC1_GetControlDmaBuffer();
    raw_a = (int32_t)adc_samples[0] - (int32_t)ADC_MIDPOINT;
    raw_b = (int32_t)adc_samples[1] - (int32_t)ADC_MIDPOINT;

    *current_a = (int16_t)(raw_a / (int32_t)ADC_TO_CURRENT_SCALE);
    *current_b = (int16_t)(raw_b / (int32_t)ADC_TO_CURRENT_SCALE);
}

static int16_t float_to_cmd(float value)
{
    float constrained = value;

    if (constrained > 1.0f)
    {
        constrained = 1.0f;
    }
    else if (constrained < -1.0f)
    {
        constrained = -1.0f;
    }

    if ((constrained > -CRSF_DEADBAND) && (constrained < CRSF_DEADBAND))
    {
        constrained = 0.0f;
    }

    return (int16_t)(constrained * (float)CURRENT_TARGET_LIMIT);
}

static int16_t run_current_pid(current_pid_t *pid, int16_t target_current, int16_t measured_current)
{
    float error;
    float deriv;
    float output;

    if (pid == NULL)
    {
        return 0;
    }

    error = (float)(target_current - measured_current);
    pid->integral += error * pid->ki * CONTROL_DT_S;
    if (pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }
    else if (pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }

    deriv = (error - pid->prev_error) / CONTROL_DT_S;
    output = (pid->kp * error) + pid->integral + (pid->kd * deriv);
    pid->prev_error = error;

    if (output > pid->max_output)
    {
        output = pid->max_output;
    }
    else if (output < -pid->max_output)
    {
        output = -pid->max_output;
    }

    return (int16_t)output;
}

static void pid_reset(current_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void apply_motor_pwm(int16_t left_cmd, int16_t right_cmd)
{
    uint16_t left_abs = (uint16_t)((left_cmd >= 0) ? left_cmd : -left_cmd);
    uint16_t right_abs = (uint16_t)((right_cmd >= 0) ? right_cmd : -right_cmd);

    s_pwm_a = clamp_pwm((uint16_t)(((uint32_t)left_abs * PWM_ARR_VALUE) / DRIVE_CMD_LIMIT));
    s_pwm_b = clamp_pwm((uint16_t)(((uint32_t)right_abs * PWM_ARR_VALUE) / DRIVE_CMD_LIMIT));

    if (left_cmd >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s_pwm_a);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s_pwm_a);
    }

    if (right_cmd >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, s_pwm_b);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_pwm_b);
    }
}

static int16_t clamp_drive_cmd(int16_t value)
{
    if (value > DRIVE_CMD_LIMIT)
    {
        value = DRIVE_CMD_LIMIT;
    }
    else if (value < -DRIVE_CMD_LIMIT)
    {
        value = -DRIVE_CMD_LIMIT;
    }

    return value;
}

/* ============================================
 * 任务初始化
 * ============================================ */
void MotorControlTask_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    ADC1_StartControlDma();

    memset(&s_motor_state, 0, sizeof(s_motor_state));
    s_brake_active = 0U;
    s_control_flags = 0U;
    s_pwm_a = 0U;
    s_pwm_b = 0U;
    pid_reset(&s_pid_a);
    pid_reset(&s_pid_b);

    apply_motor_pwm(0, 0);
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
    pid_reset(&s_pid_a);
    pid_reset(&s_pid_b);
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
 * 设置控制标志位
 * ============================================ */
void MotorControlTask_SetControlFlags(uint8_t flags)
{
    s_control_flags = flags;
}

/* ============================================
 * 获取控制标志位
 * ============================================ */
uint8_t MotorControlTask_GetControlFlags(void)
{
    return s_control_flags;
}

/* ============================================
 * 设置 PWM 上限 (兼容保留)
 * ============================================ */
void MotorControlTask_SetMaxDuty(float max_duty)
{
    (void)max_duty;
}

/* ============================================
 * 设置 PID 参数 (兼容保留)
 * ============================================ */
void MotorControlTask_SetPID(motor_id_t motor, const pid_param_t *param)
{
    current_pid_t *pid_target = NULL;

    if (param == NULL)
    {
        return;
    }

    if (motor == MOTOR_A)
    {
        pid_target = &s_pid_a;
    }
    else if (motor == MOTOR_B)
    {
        pid_target = &s_pid_b;
    }

    if (pid_target == NULL)
    {
        return;
    }

    if (param->kp > 0.0f)
    {
        pid_target->kp = param->kp;
    }

    if (param->ki >= 0.0f)
    {
        pid_target->ki = param->ki;
    }

    if (param->kd >= 0.0f)
    {
        pid_target->kd = param->kd;
    }

    if (param->max_output > 0.0f)
    {
        pid_target->max_output = param->max_output;
    }

    if (param->max_integral > 0.0f)
    {
        pid_target->max_integral = param->max_integral;
    }
}

/* ============================================
 * 设置 PID 模式 (兼容保留)
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
    crsf_input_t rc_input;
    int16_t cmd_left;
    int16_t cmd_right;
    int16_t target_current_a;
    int16_t target_current_b;

    (void)argument;

    read_current_samples(&s_motor_state.current_a, &s_motor_state.current_b);

    CRSFTask_GetInput(&rc_input);
    if (rc_input.valid)
    {
        target_current_a = float_to_cmd(rc_input.throttle + rc_input.steering);
        target_current_b = float_to_cmd(rc_input.throttle - rc_input.steering);
        s_motor_state.target_rpm_a = target_current_a;
        s_motor_state.target_rpm_b = target_current_b;
    }
    else
    {
        s_motor_state.target_rpm_a = 0;
        s_motor_state.target_rpm_b = 0;
    }

    /* 处理刹车 */
    if (s_brake_active)
    {
        s_motor_state.target_rpm_a = 0;
        s_motor_state.target_rpm_b = 0;
    }

    if (s_pid_mode == PID_MODE_CURRENT)
    {
        cmd_left = run_current_pid(&s_pid_a, s_motor_state.target_rpm_a, s_motor_state.current_a);
        cmd_right = run_current_pid(&s_pid_b, s_motor_state.target_rpm_b, s_motor_state.current_b);
    }
    else
    {
        cmd_left = s_motor_state.target_rpm_a;
        cmd_right = s_motor_state.target_rpm_b;
    }

    cmd_left = clamp_drive_cmd(cmd_left);
    cmd_right = clamp_drive_cmd(cmd_right);
    if (s_brake_active != 0U)
    {
        cmd_left = 0;
        cmd_right = 0;
        pid_reset(&s_pid_a);
        pid_reset(&s_pid_b);
    }

    s_motor_state.actual_rpm_a = cmd_left;
    s_motor_state.actual_rpm_b = cmd_right;

    apply_motor_pwm(cmd_left, cmd_right);
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