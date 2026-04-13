/**
 * @file led_task.c
 * @brief LED 指示任务实现
 * @description 系统状态 LED 控制
 */

#include "led_task.h"
#include "main.h"
#include "gpio.h"
#include <string.h>

/* ============================================
 * 配置参数
 * ============================================ */
#define LED_TASK_PERIOD     20      /* 20ms 周期 */

/* 闪烁周期 (ms) */
#define BLINK_PERIOD_SLOW         500     /* 1Hz */
#define BLINK_PERIOD_FAST         100     /* 5Hz */
#define BLINK_PERIOD_VERY_FAST   50      /* 10Hz */

/* ============================================
 * 私有变量
 * ============================================ */
static led_mode_t s_led_modes[3] = {
    LED_OFF,
    LED_OFF,
    LED_OFF
};

static uint32_t s_led_timers[3] = {0};
static uint8_t s_led_state[3] = {0};  /* 0=off, 1=on */

/* LED GPIO 定义 */
#define LED_GPIO_PORT   LED_GPIO_Port
#define LED_GPIO_PIN    LED_Pin

/* ============================================
 * 初始化
 * ============================================ */
void LEDTask_Init(void)
{
    /* 初始化所有 LED 为关闭状态 */
    memset(s_led_modes, 0, sizeof(s_led_modes));
    memset(s_led_timers, 0, sizeof(s_led_timers));
    memset(s_led_state, 0, sizeof(s_led_state));

    /* 关闭 LED */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

/* ============================================
 * 设置 LED 模式
 * ============================================ */
void LEDTask_SetMode(led_id_t id, led_mode_t mode)
{
    if (id >= 3) return;

    s_led_modes[id] = mode;
    s_led_timers[id] = 0;
    s_led_state[id] = 0;

    /* 如果是常亮或常灭，立即更新 */
    if (mode == LED_ON)
    {
        s_led_state[id] = 1;
        if (id == 0)  /* System LED */
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
        }
    }
    else if (mode == LED_OFF)
    {
        s_led_state[id] = 0;
        if (id == 0)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
        }
    }
}

/* ============================================
 * 获取闪烁周期
 * ============================================ */
static uint32_t get_blink_period(led_mode_t mode)
{
    switch (mode)
    {
        case LED_BLINK_SLOW:        return BLINK_PERIOD_SLOW;
        case LED_BLINK_FAST:       return BLINK_PERIOD_FAST;
        case LED_BLINK_VERY_FAST:   return BLINK_PERIOD_VERY_FAST;
        default:                    return 0;
    }
}

/* ============================================
 * 任务处理 (20ms 周期)
 * ============================================ */
void LEDTask_Process(void *argument)
{
    (void)argument;

    /* 仅处理 System LED (PA7) */
    led_id_t id = LED_ID_SYSTEM;
    led_mode_t mode = s_led_modes[id];

    /* 非闪烁模式不处理 */
    if (mode != LED_BLINK_SLOW &&
        mode != LED_BLINK_FAST &&
        mode != LED_BLINK_VERY_FAST)
    {
        return;
    }

    /* 更新计时器 */
    s_led_timers[id] += LED_TASK_PERIOD;

    /* 获取闪烁周期 */
    uint32_t period = get_blink_period(mode);
    uint32_t half_period = period / 2;

    /* 切换状态 */
    if (s_led_timers[id] >= half_period)
    {
        s_led_timers[id] = 0;
        s_led_state[id] = !s_led_state[id];

        /* 输出到 GPIO */
        if (id == LED_ID_SYSTEM)
        {
            if (s_led_state[id])
            {
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            }
        }
    }
}

/* ============================================
 * 便捷函数: 设置系统状态
 * ============================================ */
void LEDTask_SetSystemMode(led_mode_t mode)
{
    LEDTask_SetMode(LED_ID_SYSTEM, mode);
}

void LEDTask_SetCommMode(led_mode_t mode)
{
    LEDTask_SetMode(LED_ID_COMM, mode);
}

void LEDTask_SetFaultMode(led_mode_t mode)
{
    LEDTask_SetMode(LED_ID_FAULT, mode);
}