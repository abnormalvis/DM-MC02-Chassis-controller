/**
 * @file safety_task.c
 * @brief 安全/故障管理任务实现
 */

#include "safety_task.h"

#include "main.h"
#include "crsf_task.h"
#include "imu_task.h"
#include "motor_control_task.h"

/* 需要与 app_tasks 状态同步 */
#include "app_tasks.h"

/* ============================================
 * 私有状态
 * ============================================ */
static uint8_t s_estop_active = 0U;
static uint32_t s_fault_bits = 0U;
static control_mode_t s_mode = MODE_IDLE;
static uint32_t s_fault_clear_tick = 0U;

#define FAULT_BRAKE_MASK (FAULT_RC_TIMEOUT | FAULT_IMU_FAULT)

static void safety_sync_fault_to_app(uint32_t fault, uint8_t set)
{
    if (set != 0U)
    {
        AppTasks_SetFault(fault);
    }
    else
    {
        AppTasks_ClearFault(fault);
    }
}

static void safety_set_fault_internal(uint32_t fault)
{
    if ((s_fault_bits & fault) == 0U)
    {
        s_fault_bits |= fault;
        safety_sync_fault_to_app(fault, 1U);
    }
}

static void safety_clear_fault_internal(uint32_t fault)
{
    if ((s_fault_bits & fault) != 0U)
    {
        s_fault_bits &= ~fault;
        safety_sync_fault_to_app(fault, 0U);
    }
}

void SafetyTask_Init(void)
{
    s_estop_active = 0U;
    s_fault_bits = 0U;
    s_mode = MODE_IDLE;
    s_fault_clear_tick = 0U;
}

void SafetyTask_TriggerEStop(void)
{
    s_estop_active = 1U;
    s_mode = MODE_BRAKE_PROTECT;
    MotorControlTask_Brake();
}

void SafetyTask_ClearEStop(void)
{
    s_estop_active = 0U;
}

bool SafetyTask_IsEStop(void)
{
    return (s_estop_active != 0U);
}

void SafetyTask_SetFault(uint32_t fault)
{
    safety_set_fault_internal(fault);
}

void SafetyTask_ClearFault(uint32_t fault)
{
    safety_clear_fault_internal(fault);
}

uint32_t SafetyTask_GetFault(void)
{
    return s_fault_bits;
}

void SafetyTask_SetMode(control_mode_t mode)
{
    s_mode = mode;
}

control_mode_t SafetyTask_GetMode(void)
{
    return s_mode;
}

void SafetyTask_Process(void *argument)
{
    (void)argument;

    /* 连接性检查 */
    if (!CRSFTask_IsConnected())
    {
        safety_set_fault_internal(FAULT_RC_TIMEOUT);
    }
    else
    {
        safety_clear_fault_internal(FAULT_RC_TIMEOUT);
    }

    if (!IMUTask_IsReady())
    {
        safety_set_fault_internal(FAULT_IMU_FAULT);
    }
    else
    {
        safety_clear_fault_internal(FAULT_IMU_FAULT);
    }

    /* CRSF 急停联动 */
    crsf_input_t rc_input;
    CRSFTask_GetInput(&rc_input);
    if (rc_input.valid && (rc_input.estop != 0U))
    {
        s_estop_active = 1U;
    }

    /* 故障/急停优先级最高 */
    if ((s_estop_active != 0U) || ((s_fault_bits & FAULT_BRAKE_MASK) != 0U))
    {
        s_mode = MODE_BRAKE_PROTECT;
        MotorControlTask_Brake();
        AppTasks_SetControlMode(MODE_BRAKE_PROTECT);
        s_fault_clear_tick = 0U;
        return;
    }

    /* 恢复逻辑：连续稳定一段时间后解除刹车 */
    if (s_fault_clear_tick == 0U)
    {
        s_fault_clear_tick = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - s_fault_clear_tick) >= TIMEOUT_RECOVERY)
    {
        if (rc_input.valid)
        {
            if (rc_input.mode_request == 1U)
            {
                s_mode = MODE_RC;
            }
            else if (rc_input.mode_request == 2U)
            {
                s_mode = MODE_OFFBOARD;
            }
            else
            {
                s_mode = MODE_IDLE;
            }
        }
        else
        {
            s_mode = MODE_IDLE;
        }

        MotorControlTask_ReleaseBrake();
        AppTasks_SetControlMode(s_mode);
    }
}
