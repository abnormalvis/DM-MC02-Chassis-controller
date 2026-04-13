/**
 * @file buzzer_task.c
 * @brief 蜂鸣器任务实现
 * @description 系统状态声音提示
 */

#include "buzzer_task.h"
#include "main.h"
#include "tim.h"
#include <string.h>

/* ============================================
 * 配置参数
 * ============================================ */
#define BUZZER_TASK_PERIOD    1       /* 1ms 周期 */

/* 蜂鸣器引脚定义 (TIM12_CH2 PB15) */
#define BUZZER_TIM             htim12
#define BUZZER_CHANNEL        TIM_CHANNEL_2

/* ============================================
 * 私有变量
 * ============================================ */
static buzzer_mode_t s_mode = BUZZER_OFF;
static uint16_t s_counter = 0;
static uint16_t s_beep_count = 0;
static uint8_t s_pattern_index = 0;

/* 节奏模式定义: {on_time, off_time, repeat_count} */
typedef struct {
    uint16_t on_time;
    uint16_t off_time;
    uint8_t repeat;
} buzzer_pattern_t;

static const buzzer_pattern_t s_patterns[] = {
    {50, 50, 1},    /* BEEP_ONCE: 50ms on, 50ms off, 1次 */
    {50, 50, 2},   /* BEEP_TWICE: 50ms on, 50ms off, 2次 */
    {200, 100, 1},  /* BEEP_LONG: 200ms on, 100ms off, 1次 */
    {100, 100, 10}, /* ALARM: 10次快速 */
};

/* ============================================
 * 初始化
 * ============================================ */
void BuzzerTask_Init(void)
{
    s_mode = BUZZER_OFF;
    s_counter = 0;
    s_beep_count = 0;
    s_pattern_index = 0;

    /* 确保蜂鸣器关闭 */
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 0);
}

/* ============================================
 * 设置蜂鸣器模式
 * ============================================ */
void BuzzerTask_SetMode(buzzer_mode_t mode)
{
    if (mode >= BUZZER_PATTERN) return;

    s_mode = mode;
    s_counter = 0;
    s_pattern_index = 0;

    if (mode == BUZZER_OFF)
    {
        __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 0);
    }
}

/* ============================================
 * 任务处理 (1ms 周期)
 * ============================================ */
void BuzzerTask_Process(void *argument)
{
    (void)argument;

    /* 非激活模式不处理 */
    if (s_mode == BUZZER_OFF)
    {
        return;
    }

    s_counter += BUZZER_TASK_PERIOD;

    /* 获取当前模式的时间参数 */
    const buzzer_pattern_t *pat = &s_patterns[s_mode - 1];
    uint16_t on_time = pat->on_time;
    uint16_t off_time = pat->off_time;
    uint8_t repeat = pat->repeat;

    /* 检查是否需要播放下一声 */
    if (s_counter >= (on_time + off_time))
    {
        s_counter = 0;
        s_beep_count++;

        /* 检查是否完成所有重复 */
        if (s_beep_count >= repeat)
        {
            s_mode = BUZZER_OFF;
            __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 0);
            return;
        }
    }

    /* 控制蜂鸣器开关 */
    if (s_counter < on_time)
    {
        /* 设置音量 (50% 占空比) */
        __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 50);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 0);
    }
}