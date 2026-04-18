/**
 * @file motor_control_task.c
 * @brief 电机控制任务实现
 * @description 双电机目标计算与 CAN 下发控制
 */

#include "motor_control_task.h"
#include "main.h"
#include "can_task.h"
#include "can_protocol.h"
#include <string.h>

/* ============================================
 * 配置参数
 * ============================================ */
#define DRIVE_CMD_LIMIT       1000
#define DRIVE_TX_PERIOD_MS    2U   /* 500Hz */

/* ============================================
 * 私有变量
 * ============================================ */
static motor_control_state_t s_motor_state = {0};
static uint8_t s_brake_active = 0;
static pid_mode_t s_pid_mode = PID_MODE_SPEED;
static uint8_t s_control_flags = 0U;
static uint32_t s_last_tx_tick = 0U;

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
    memset(&s_motor_state, 0, sizeof(s_motor_state));
    s_brake_active = 0U;
    s_control_flags = 0U;
    s_last_tx_tick = 0U;
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
    (void)motor;
    (void)param;
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
    drive_cmd_t tx_cmd;
    uint32_t now;

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
    }

    (void)s_pid_mode;

    now = HAL_GetTick();
    if ((now - s_last_tx_tick) < DRIVE_TX_PERIOD_MS)
    {
        return;
    }

    s_last_tx_tick = now;

    tx_cmd.left_cmd = clamp_drive_cmd(s_motor_state.target_rpm_a);
    tx_cmd.right_cmd = clamp_drive_cmd(s_motor_state.target_rpm_b);
    tx_cmd.flags = s_control_flags;
    if (s_brake_active != 0U)
    {
        tx_cmd.flags |= CAN_DRIVE_FLAG_BRAKE;
        tx_cmd.left_cmd = 0;
        tx_cmd.right_cmd = 0;
    }
    tx_cmd.seq = 0U;
    tx_cmd.reserved = 0U;

    can_send_drive_cmd(&tx_cmd);
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