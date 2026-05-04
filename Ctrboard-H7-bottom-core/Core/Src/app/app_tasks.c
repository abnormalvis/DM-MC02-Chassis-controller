/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   Application task creation and management.
  ******************************************************************************
  */

#include "app_tasks.h"
#include "cmsis_os2.h"
#include <string.h>
#include "main.h"
#include "adc.h"
#include "comm_dispatcher.h"
#include "motor_control_task.h"
#include "led_task.h"
#include "buzzer_task.h"
#include "crsf_task.h"
#include "usb_task.h"
#include "rs485_task.h"
#include "imu_task.h"
#include "safety_task.h"
#include "protocol_adapter.h"
#include "vofa_debug.h"
#include <math.h>

/* 任务句柄定义 */
static osThreadId_t s_motor_task_handle;
static osThreadId_t s_crsf_task_handle;
static osThreadId_t s_usb_task_handle;
static osThreadId_t s_rs485_task_handle;
static osThreadId_t s_imu_task_handle;
static osThreadId_t s_safety_task_handle;
static osThreadId_t s_led_task_handle;
static osThreadId_t s_buzzer_task_handle;
static osThreadId_t s_comm_dispatcher_task_handle;

/* 系统状态 */
static system_state_t s_system_state;
static chassis_cmd_t s_rc_cmd;
static chassis_cmd_t s_offboard_cmd;

/* 任务函数原型 */
__NO_RETURN static void MotorControlTask(void *argument);
__NO_RETURN static void CRSFTask(void *argument);
__NO_RETURN static void USBTaskWrapper(void *argument);
__NO_RETURN static void RS485TaskWrapper(void *argument);
__NO_RETURN static void IMUTaskWrapper(void *argument);
__NO_RETURN static void SafetyTaskWrapper(void *argument);
__NO_RETURN static void LEDTaskWrapper(void *argument);
__NO_RETURN static void BuzzerTaskWrapper(void *argument);

#define APP_VX_GAIN_RPM_PER_MPS   400.0f
#define APP_WZ_GAIN_RPM_PER_RPS   100.0f
#define APP_RPM_LIMIT             900
#define APP_ESTOP_CLEAR_VX_TH     0.10f
#define APP_THROTTLE_LOCK_VX_MPS  0.55f

#define APP_CTRL_FLAG_ESTOP          (1U << 0)
#define APP_CTRL_FLAG_BRAKE          (1U << 1)
#define APP_CTRL_FLAG_THROTTLE_LOCK  (1U << 2)
#define APP_CTRL_FLAG_MODE_OFFBOARD  (1U << 3)
#define APP_CTRL_FLAG_SOURCE_RC      (1U << 4)

static uint8_t s_estop_latched;
static uint8_t s_brake_latched;
static uint8_t s_throttle_lock_latched;
static uint8_t s_estop_need_center;

static int16_t app_clamp_rpm(float rpm)
{
    if (rpm > (float)APP_RPM_LIMIT)
    {
        rpm = (float)APP_RPM_LIMIT;
    }
    else if (rpm < (float)(-APP_RPM_LIMIT))
    {
        rpm = (float)(-APP_RPM_LIMIT);
    }

    return (int16_t)rpm;
}

static void app_chassis_cmd_to_rpm(const chassis_ctrl_cmd_t *cmd, int16_t *rpm_left, int16_t *rpm_right)
{
    float left;
    float right;

    left = cmd->vx_mps * APP_VX_GAIN_RPM_PER_MPS - cmd->wz_rps * APP_WZ_GAIN_RPM_PER_RPS;
    right = cmd->vx_mps * APP_VX_GAIN_RPM_PER_MPS + cmd->wz_rps * APP_WZ_GAIN_RPM_PER_RPS;

    *rpm_left = app_clamp_rpm(left);
    *rpm_right = app_clamp_rpm(right);
}

void comm_dispatcher_on_ctrl_cmd(const void *cmd)
{
    const chassis_ctrl_cmd_t *ctrl_cmd = (const chassis_ctrl_cmd_t *)cmd;
    int16_t rpm_left;
    int16_t rpm_right;
    uint8_t drive_flags = 0U;
    control_mode_t mode;
    chassis_ctrl_cmd_t effective_cmd;

    if ((ctrl_cmd == NULL) || (ctrl_cmd->valid == 0U))
    {
        return;
    }

    effective_cmd = *ctrl_cmd;

    if ((effective_cmd.estop != 0U) || (effective_cmd.estop_cmd == (uint8_t)CTRL_SW_CMD_ENABLE))
    {
        s_estop_latched = 1U;
        s_estop_need_center = 1U;
    }
    else if (effective_cmd.estop_cmd == (uint8_t)CTRL_SW_CMD_DISABLE)
    {
        if ((s_estop_need_center == 0U) || (fabsf(effective_cmd.vx_mps) <= APP_ESTOP_CLEAR_VX_TH))
        {
            s_estop_latched = 0U;
            s_estop_need_center = 0U;
            SafetyTask_ClearEStop();
        }
    }

    if ((s_estop_need_center != 0U) && (fabsf(effective_cmd.vx_mps) <= APP_ESTOP_CLEAR_VX_TH))
    {
        s_estop_need_center = 0U;
    }

    if (effective_cmd.brake_cmd == (uint8_t)CTRL_SW_CMD_ENABLE)
    {
        s_brake_latched = 1U;
    }
    else if (effective_cmd.brake_cmd == (uint8_t)CTRL_SW_CMD_DISABLE)
    {
        s_brake_latched = 0U;
    }

    if (effective_cmd.throttle_lock_cmd == (uint8_t)CTRL_SW_CMD_ENABLE)
    {
        s_throttle_lock_latched = 1U;
    }
    else if (effective_cmd.throttle_lock_cmd == (uint8_t)CTRL_SW_CMD_DISABLE)
    {
        s_throttle_lock_latched = 0U;
    }

    if (s_estop_latched != 0U)
    {
        drive_flags |= APP_CTRL_FLAG_ESTOP;
        drive_flags |= APP_CTRL_FLAG_BRAKE;
        MotorControlTask_SetControlFlags(drive_flags);
        AppTasks_TriggerEStop();
        MotorControlTask_Brake();
        return;
    }

    if (s_brake_latched != 0U)
    {
        drive_flags |= APP_CTRL_FLAG_BRAKE;
        if ((effective_cmd.source == (uint8_t)OFFBOARD_SRC_NONE))
        {
            drive_flags |= APP_CTRL_FLAG_SOURCE_RC;
        }
        MotorControlTask_SetControlFlags(drive_flags);
        s_system_state.mode = MODE_BRAKE_PROTECT;
        s_system_state.last_update_tick = HAL_GetTick();
        MotorControlTask_Brake();
        return;
    }

    MotorControlTask_ReleaseBrake();

    if ((s_throttle_lock_latched != 0U) && (effective_cmd.source == (uint8_t)OFFBOARD_SRC_NONE))
    {
        drive_flags |= APP_CTRL_FLAG_THROTTLE_LOCK;
        effective_cmd.vx_mps = APP_THROTTLE_LOCK_VX_MPS;
    }

    mode = MODE_IDLE;
    if (effective_cmd.mode == (uint8_t)CTRL_MODE_RC)
    {
        mode = MODE_RC;
    }
    else if (effective_cmd.mode == (uint8_t)CTRL_MODE_OFFBOARD)
    {
        mode = MODE_OFFBOARD;
        drive_flags |= APP_CTRL_FLAG_MODE_OFFBOARD;
    }

    s_system_state.mode = mode;
    s_system_state.rc_connected = 0U;
    s_system_state.usb_connected = 0U;
    s_system_state.rs485_connected = 0U;

    if (effective_cmd.source == (uint8_t)OFFBOARD_SRC_USB)
    {
        s_system_state.usb_connected = 1U;
    }
    else if (effective_cmd.source == (uint8_t)OFFBOARD_SRC_RS485)
    {
        s_system_state.rs485_connected = 1U;
    }
    else
    {
        s_system_state.rc_connected = 1U;
        drive_flags |= APP_CTRL_FLAG_SOURCE_RC;
    }

    MotorControlTask_SetControlFlags(drive_flags);

    if (mode == MODE_RC)
    {
        s_rc_cmd.vx_mps = effective_cmd.vx_mps;
        s_rc_cmd.wz_rps = effective_cmd.wz_rps;
        s_rc_cmd.estop = effective_cmd.estop;
        s_rc_cmd.valid = effective_cmd.valid;
        s_rc_cmd.timestamp = effective_cmd.ts_ms;
    }
    else
    {
        s_offboard_cmd.vx_mps = effective_cmd.vx_mps;
        s_offboard_cmd.wz_rps = effective_cmd.wz_rps;
        s_offboard_cmd.estop = effective_cmd.estop;
        s_offboard_cmd.valid = effective_cmd.valid;
        s_offboard_cmd.timestamp = effective_cmd.ts_ms;
    }

    app_chassis_cmd_to_rpm(&effective_cmd, &rpm_left, &rpm_right);
    MotorControlTask_SetTarget(rpm_left, rpm_right);
    s_system_state.last_update_tick = HAL_GetTick();
}

/**
 * @brief 创建所有应用层任务
 */
void AppTasks_Create(void)
{
    /* 初始化系统状态 */
    memset(&s_system_state, 0, sizeof(s_system_state));
    s_system_state.mode = MODE_IDLE;

    /* 初始化各模块 */
    comm_dispatcher_init_queues();

    /* 创建通信分发任务 */
    static const osThreadAttr_t comm_attr = {
        .name = "CommDispatch",
        .stack_size = 512U * 4U,
        .priority = (osPriority_t)osPriorityAboveNormal
    };
    s_comm_dispatcher_task_handle = osThreadNew(CommDispatcherTask, NULL, &comm_attr);

    /* 创建电机控制任务 (1kHz) */
    static const osThreadAttr_t motor_attr = {
        .name = "MotorControl",
        .stack_size = STACK_SIZE_MOTOR_CONTROL * 4,
        .priority = (osPriority_t)osPriorityAboveNormal
    };
    s_motor_task_handle = osThreadNew(MotorControlTask, NULL, &motor_attr);

    /* 创建 CRSF 接收任务 (1kHz) */
    static const osThreadAttr_t crsf_attr = {
        .name = "CRSFTask",
        .stack_size = STACK_SIZE_CRSF * 4,
        .priority = (osPriority_t)PRIORITY_CRSF + 1
    };
    s_crsf_task_handle = osThreadNew(CRSFTask, NULL, &crsf_attr);

    /* 创建 USB 通信任务 (100Hz) */
    // static const osThreadAttr_t usb_attr = {
    //     .name = "USBTask",
    //     .stack_size = STACK_SIZE_USB * 4,
    //     .priority = (osPriority_t)PRIORITY_USB + 1
    // };
    // s_usb_task_handle = osThreadNew(USBTaskWrapper, NULL, &usb_attr);

    /* 创建 RS485 通信任务 (100Hz) */
    static const osThreadAttr_t rs485_attr = {
        .name = "RS485Task",
        .stack_size = STACK_SIZE_RS485 * 4,
        .priority = (osPriority_t)PRIORITY_RS485 + 1
    };
    s_rs485_task_handle = osThreadNew(RS485TaskWrapper, NULL, &rs485_attr);

    /* 创建 IMU 采集任务 (1kHz) */
    static const osThreadAttr_t imu_attr = {
        .name = "IMUTask",
        .stack_size = STACK_SIZE_IMU * 4,
        .priority = (osPriority_t)PRIORITY_IMU + 1
    };
    s_imu_task_handle = osThreadNew(IMUTaskWrapper, NULL, &imu_attr);

    /* 创建安全/故障管理任务 (200Hz) */
    static const osThreadAttr_t safety_attr = {
        .name = "SafetyTask",
        .stack_size = STACK_SIZE_SAFETY * 4,
        .priority = (osPriority_t)PRIORITY_SAFETY + 1
    };
    s_safety_task_handle = osThreadNew(SafetyTaskWrapper, NULL, &safety_attr);

    /* 创建 LED 指示任务 (50Hz) */
    static const osThreadAttr_t led_attr = {
        .name = "LEDTask",
        .stack_size = STACK_SIZE_LED * 4,
        .priority = (osPriority_t)PRIORITY_LED + 1
    };
    s_led_task_handle = osThreadNew(LEDTaskWrapper, NULL, &led_attr);

    /* 创建蜂鸣器任务 (100Hz) */
    static const osThreadAttr_t buzzer_attr = {
        .name = "BuzzerTask",
        .stack_size = STACK_SIZE_BUZZER * 4,
        .priority = (osPriority_t)PRIORITY_BUZZER + 1
    };
    s_buzzer_task_handle = osThreadNew(BuzzerTaskWrapper, NULL, &buzzer_attr);
}

/**
 * @brief 电机控制任务线程
 */
__NO_RETURN static void MotorControlTask(void *argument)
{
    (void)argument;
    MotorControlTask_Init();

    for (;;)
    {
        MotorControlTask_Process(NULL);
        VofaDebug_Process();
        osDelay(PERIOD_MOTOR_CONTROL);
    }
}

/**
 * @brief CRSF 接收任务线程
 */
__NO_RETURN static void CRSFTask(void *argument)
{
    (void)argument;
    CRSFTask_Init();

    for (;;)
    {
        CRSFTask_Process(NULL);
        osDelay(PERIOD_CRSF);
    }
}

/**
 * @brief USB 任务包装器
 */
__NO_RETURN static void USBTaskWrapper(void *argument)
{
    (void)argument;
    USBTask_Init();

    for (;;)
    {
        USBTask_Process(NULL);
        osDelay(PERIOD_USB);
    }
}

/**
 * @brief RS485 任务包装器
 */
__NO_RETURN static void RS485TaskWrapper(void *argument)
{
    (void)argument;
    RS485Task_Init();

    for (;;)
    {
        RS485Task_Process(NULL);
        osDelay(PERIOD_RS485);
    }
}

/**
 * @brief IMU 任务包装器
 */
__NO_RETURN static void IMUTaskWrapper(void *argument)
{
    (void)argument;
    IMUTask_Init();

    for (;;)
    {
        IMUTask_Process(NULL);
        osDelay(PERIOD_IMU);
    }
}

/**
 * @brief 安全任务包装器
 */
__NO_RETURN static void SafetyTaskWrapper(void *argument)
{
    (void)argument;
    SafetyTask_Init();

    for (;;)
    {
        SafetyTask_Process(NULL);
        osDelay(PERIOD_SAFETY);
    }
}

/**
 * @brief LED 任务包装器
 */
__NO_RETURN static void LEDTaskWrapper(void *argument)
{
    (void)argument;
    LEDTask_Init();

    for (;;)
    {
        LEDTask_Process(NULL);
        osDelay(PERIOD_LED);
    }
}

/**
 * @brief 蜂鸣器任务包装器
 */
__NO_RETURN static void BuzzerTaskWrapper(void *argument)
{
    (void)argument;
    BuzzerTask_Init();

    for (;;)
    {
        BuzzerTask_Process(NULL);
        osDelay(PERIOD_BUZZER);
    }
}

/**
 * @brief 获取系统状态
 */
void AppTasks_GetSystemState(system_state_t *state)
{
    if (state != NULL)
    {
        *state = s_system_state;
    }
}

/**
 * @brief 设置控制模式
 */
void AppTasks_SetControlMode(control_mode_t mode)
{
    s_system_state.mode = mode;
    s_system_state.last_update_tick = HAL_GetTick();
}

/**
 * @brief 获取 RC 控制命令
 */
chassis_cmd_t *AppTasks_GetRCCommand(void)
{
    return &s_rc_cmd;
}

/**
 * @brief 获取上位机控制命令
 */
chassis_cmd_t *AppTasks_GetOffboardCommand(void)
{
    return &s_offboard_cmd;
}

/**
 * @brief 获取电机状态
 */
void AppTasks_GetMotorState(motor_state_t *state)
{
    const uint16_t *adc_samples;

    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
        adc_samples = ADC1_GetControlDmaBuffer();
        state->current_a = (int16_t)(adc_samples[0] / 100U);
        state->current_b = (int16_t)(adc_samples[1] / 100U);
        state->brake_active = (s_system_state.mode == MODE_BRAKE_PROTECT) ? 1U : 0U;
    }
}

/**
 * @brief 触发急停
 */
void AppTasks_TriggerEStop(void)
{
    s_system_state.mode = MODE_BRAKE_PROTECT;
    s_system_state.last_update_tick = HAL_GetTick();
    SafetyTask_TriggerEStop();
}

/**
 * @brief 设置故障位
 */
void AppTasks_SetFault(uint32_t fault_bit)
{
    s_system_state.fault_bits |= fault_bit;
    s_system_state.last_update_tick = HAL_GetTick();
}

/**
 * @brief 清除故障位
 */
void AppTasks_ClearFault(uint32_t fault_bit)
{
    s_system_state.fault_bits &= ~fault_bit;
    s_system_state.last_update_tick = HAL_GetTick();
}