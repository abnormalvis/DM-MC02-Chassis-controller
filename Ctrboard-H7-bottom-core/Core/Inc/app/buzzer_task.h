/**
 * @file buzzer_task.h
 * @brief 蜂鸣器任务
 * @description 系统状态声音提示
 */

#ifndef BUZZER_TASK_H
#define BUZZER_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* 蜂鸣器提示模式 */
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_BEEP_ONCE,        /* 单次 beep */
    BUZZER_BEEP_TWICE,       /* 两次 beep */
    BUZZER_BEEP_LONG,        /* 长 beep */
    BUZZER_ALARM,            /* 报警音 */
    BUZZER_PATTERN           /* 自定义节奏 */
} buzzer_mode_t;

/**
 * @brief 蜂鸣器任务初始化
 */
void BuzzerTask_Init(void);

/**
 * @brief 设置蜂鸣器模式
 * @param mode 蜂鸣器模式
 */
void BuzzerTask_SetMode(buzzer_mode_t mode);

/**
 * @brief 蜂鸣器任务处理 (在任务中调用)
 */
void BuzzerTask_Process(void);

#endif /* BUZZER_TASK_H */
