/**
 * @file rs485_task.h
 * @brief RS485 通信任务
 * @description USART2/USART3 RS485 串口通信
 */

#ifndef RS485_TASK_H
#define RS485_TASK_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief RS485 任务初始化
 */
void RS485Task_Init(void);

/**
 * @brief 发送数据到 RS485
 * @param port 端口 (2 或 3)
 * @param data 数据指针
 * @param len 数据长度
 * @return 实际发送长度
 */
uint32_t RS485Task_Send(uint8_t port, const uint8_t *data, uint32_t len);

/**
 * @brief 检查 RS485 连接状态
 * @param port 端口 (2 或 3)
 */
bool RS485Task_IsConnected(uint8_t port);

/**
 * @brief RS485 任务处理 (在任务中调用)
 */
void RS485Task_Process(void *argument);

#endif /* RS485_TASK_H */