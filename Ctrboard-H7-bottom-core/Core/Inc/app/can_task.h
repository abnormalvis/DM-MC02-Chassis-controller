/**
 * @file can_task.h
 * @brief CAN 通信任务头文件
 * @description 处理 C8T6 电流数据接收
 */

#ifndef CAN_TASK_H
#define CAN_TASK_H

#include "can_protocol.h"
#include <stdint.h>

/* 任务优先级定义 */
#define CAN_TASK_PRIORITY    (osPriority_t)osPriorityHigh

/* 任务堆栈大小 */
#define CAN_TASK_STACK_SIZE  (256U * 4U)

/* 电流数据全局变量 (供控制环使用) */
extern current_data_t g_motor_current;

/* 初始化 CAN 任务 */
void CANTask_Init(void);

/* 获取当前电机电流 */
void CANTask_GetCurrent(int16_t *current_a, int16_t *current_b);

/* 检查 CAN 通信状态 */
uint8_t CANTask_IsConnected(void);

/* CAN 任务处理函数 */
void CANTask_Process(void);

#endif /* CAN_TASK_H */
