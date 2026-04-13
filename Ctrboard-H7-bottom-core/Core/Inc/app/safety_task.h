/**
 * @file safety_task.h
 * @brief 安全/故障管理任务
 * @description 系统安全监控、故障检测、模式管理
 */

#ifndef SAFETY_TASK_H
#define SAFETY_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "app_tasks.h"

/* 故障位定义 */
#define FAULT_RC_TIMEOUT         (1 << 0)    /* RC 超时 */
#define FAULT_OFFBOARD_TIMEOUT  (1 << 1)    /* 上位机超时 */
#define FAULT_CAN_TIMEOUT      (1 << 2)    /* CAN 超时 */
#define FAULT_MODE_CONFLICT    (1 << 3)    /* 模式冲突 */
#define FAULT_OVERCURRENT_A    (1 << 4)    /* 电机A过流 */
#define FAULT_OVERCURRENT_B    (1 << 5)    /* 电机B过流 */
#define FAULT_IMU_FAULT        (1 << 6)    /* IMU故障 */
#define FAULT_PROTOCOL_ERR    (1 << 7)    /* 协议错误 */

/* 超时时间定义 (ms) */
#define TIMEOUT_RC              100
#define TIMEOUT_OFFBOARD        100
#define TIMEOUT_CAN             100
#define TIMEOUT_RECOVERY        500     /* 故障恢复稳定时间 */

/**
 * @brief 安全任务初始化
 */
void SafetyTask_Init(void);

/**
 * @brief 触发急停
 */
void SafetyTask_TriggerEStop(void);

/**
 * @brief 清除急停
 */
void SafetyTask_ClearEStop(void);

/**
 * @brief 检查是否处于急停状态
 */
bool SafetyTask_IsEStop(void);

/**
 * @brief 设置故障位
 */
void SafetyTask_SetFault(uint32_t fault);

/**
 * @brief 清除故障位
 */
void SafetyTask_ClearFault(uint32_t fault);

/**
 * @brief 获取当前故障位
 */
uint32_t SafetyTask_GetFault(void);

/**
 * @brief 设置控制模式
 */
void SafetyTask_SetMode(control_mode_t mode);

/**
 * @brief 获取当前控制模式
 */
control_mode_t SafetyTask_GetMode(void);

/**
 * @brief 安全任务处理 (在任务中调用)
 */
void SafetyTask_Process(void *argument);

#endif /* SAFETY_TASK_H */