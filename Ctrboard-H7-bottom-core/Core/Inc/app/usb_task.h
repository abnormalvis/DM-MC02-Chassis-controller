/**
 * @file usb_task.h
 * @brief USB 通信任务
 * @description USB CDC 通信 (与上位机 RK3588)
 */

#ifndef USB_TASK_H
#define USB_TASK_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief USB 任务初始化
 */
void USBTask_Init(void);

/**
 * @brief 发送数据到 USB
 * @param data 数据指针
 * @param len 数据长度
 * @return 实际发送长度
 */
uint32_t USBTask_Send(const uint8_t *data, uint32_t len);

/**
 * @brief 检查 USB 连接状态
 */
bool USBTask_IsConnected(void);

/**
 * @brief USB 任务处理 (在任务中调用)
 */
void USBTask_Process(void *argument);

#endif /* USB_TASK_H */