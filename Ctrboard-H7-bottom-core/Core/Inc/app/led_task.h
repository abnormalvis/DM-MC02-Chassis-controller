/**
 * @file led_task.h
 * @brief LED 指示任务
 * @description 系统状态指示 LED 控制
 */

#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* LED 状态定义 */
typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK_SLOW,      /* 慢闪 1Hz */
    LED_BLINK_FAST,      /* 快闪 5Hz */
    LED_BLINK_VERY_FAST  /* 极速闪 10Hz */
} led_mode_t;

/* LED 指示定义 */
typedef enum {
    LED_ID_SYSTEM = 0,   /* 系统状态 LED */
    LED_ID_COMM,        /* 通信状态 LED */
    LED_ID_FAULT        /* 故障指示 LED */
} led_id_t;

/**
 * @brief LED 任务初始化
 */
void LEDTask_Init(void);

/**
 * @brief 设置 LED 模式
 * @param id LED 编号
 * @param mode LED 模式
 */
void LEDTask_SetMode(led_id_t id, led_mode_t mode);

/**
 * @brief LED 任务处理 (在任务中调用)
 */
void LEDTask_Process(void);

#endif /* LED_TASK_H */
