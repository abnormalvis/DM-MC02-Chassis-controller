/**
 * @file can_task.c
 * @brief CAN 通信任务实现
 * @description 处理 C8T6 电流数据接收，与控制环解耦
 */

#include "can_task.h"
#include "can_protocol.h"
#include "cmsis_os2.h"
#include "main.h"
#include <string.h>

/* ============================================
 * 全局变量 (供其他任务读取)
 * ============================================ */
current_data_t g_motor_current = {0};
static uint8_t g_can_connected = 0;
static uint32_t g_last_recv_tick = 0;

/* ============================================
 * 任务句柄
 * ============================================ */
static osThreadId_t s_can_task_handle = NULL;

/* 超时时间定义 (ms) */
#define CAN_TIMEOUT_MS        100     /* CAN 通信超时判定 */

/* ============================================
 * CAN 回调 - 数据接收
 * ============================================ */
static void can_data_callback(const current_data_t *data)
{
    if (data != NULL)
    {
        /* 复制数据到全局变量 (短临界区) */
        memcpy(&g_motor_current, data, sizeof(current_data_t));
        g_can_connected = 1;
        g_last_recv_tick = HAL_GetTick();
    }
}

/* ============================================
 * CAN 任务初始化
 * ============================================ */
void CANTask_Init(void)
{
    /* 初始化 CAN 协议层 */
    can_protocol_init();

    /* 注册回调函数 */
    can_protocol_register_callback(can_data_callback);

    /* 创建 CAN 任务 */
    osThreadAttr_t attr = {
        .name = "CANTask",
        .stack_size = CAN_TASK_STACK_SIZE,
        .priority = CAN_TASK_PRIORITY
    };
    s_can_task_handle = osThreadNew(CANTask_Process, NULL, &attr);
}

/* ============================================
 * 获取电机电流 (供控制环调用)
 * ============================================ */
void CANTask_GetCurrent(int16_t *current_a, int16_t *current_b)
{
    if (current_a != NULL)
    {
        *current_a = g_motor_current.current_a;
    }
    if (current_b != NULL)
    {
        *current_b = g_motor_current.current_b;
    }
}

/* ============================================
 * 检查 CAN 通信状态
 * ============================================ */
uint8_t CANTask_IsConnected(void)
{
    uint32_t tick = HAL_GetTick();

    /* 检查超时 */
    if ((tick - g_last_recv_tick) > CAN_TIMEOUT_MS)
    {
        g_can_connected = 0;
    }

    return g_can_connected;
}

/* ============================================
 * CAN 任务函数 (后台处理)
 * ============================================ */
__NO_RETURN void CANTask_Process(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* 任务周期: 10ms */
        osDelay(10);

        /* 检查连接状态 */
        if (!CANTask_IsConnected())
        {
            /* CAN 失联处理 - 可以在这里触发告警或进入安全态 */
            /* TODO: 可以添加告警逻辑，如 LED 闪烁等 */
        }
    }
}
