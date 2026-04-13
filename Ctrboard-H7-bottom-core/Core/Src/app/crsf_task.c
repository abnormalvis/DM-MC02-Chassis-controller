/**
 * @file crsf_task.c
 * @brief CRSF 遥控器接收任务实现
 * @description CRSF/ELRS 遥控器数据接收和解析 (DMA模式)
 */

#include "crsf_task.h"
#include "crsf_protocol.h"
#include "main.h"
#include "usart.h"
#include "dma.h"
#include <string.h>

/* ============================================
 * 配置参数
 * ============================================ */
/* CRSF 超时时间 (ms) */
#define CRSF_TIMEOUT_MS         100
/* CRSF 通道范围 */
#define CRSF_CH_VALUE_MIN       172
#define CRSF_CH_VALUE_MAX       1811
#define CRSF_TASK_CH_VALUE_MID  ((CRSF_CH_VALUE_MIN + CRSF_CH_VALUE_MAX) / 2)

/* 模式通道定义 */
#define CRSF_CH_THROTTLE        0   /* 油门通道 */
#define CRSF_CH_STEERING       1   /* 转向通道 */
#define CRSF_CH_MODE           4   /* 模式选择通道 */
#define CRSF_CH_ESTOP          5   /* 急停通道 */

/* 缓冲区大小 (需大于最大帧长) */
#define CRSF_RX_BUF_SIZE      256

/* DMA 环形缓冲区 */
static uint8_t s_crsf_rx_buf[CRSF_RX_BUF_SIZE] = {0};

/* ============================================
 * 私有变量
 * ============================================ */
static crsf_input_t s_crsf_input = {0};
static crsf_status_t s_crsf_status = {0};

/* DMA 接收状态 */
static volatile uint8_t s_dma_idle_detected = 0;

/* ============================================
 * CRSF 通道值归一化
 * ============================================ */
static float normalize_ch(int16_t ch_value)
{
    if (ch_value < CRSF_CH_VALUE_MIN) ch_value = CRSF_CH_VALUE_MIN;
    if (ch_value > CRSF_CH_VALUE_MAX) ch_value = CRSF_CH_VALUE_MAX;

    float normalized = (float)(ch_value - CRSF_TASK_CH_VALUE_MID) / (float)(CRSF_CH_VALUE_MAX - CRSF_CH_VALUE_MIN) * 2.0f;
    return normalized;
}

/* ============================================
 * 处理接收数据
 * ============================================ */
static void crsf_process_data(uint8_t *buf, uint16_t size)
{
    if (size < 5 || size > CRSF_RX_BUF_SIZE)
    {
        s_crsf_status.error_count++;
        return;
    }

    crsf_rc_channels_t rc_data = {0};

    /* 尝试解析 RC 通道帧 */
    if (crsf_try_decode_rc_channels(buf, size, &rc_data))
    {
        /* 成功解析 */
        memcpy(s_crsf_input.ch, rc_data.ch, sizeof(rc_data.ch));

        /* 归一化油门和转向 */
        s_crsf_input.throttle = normalize_ch(rc_data.ch[CRSF_CH_THROTTLE]);
        s_crsf_input.steering = normalize_ch(rc_data.ch[CRSF_CH_STEERING]);

        /* 模式请求 (CH5 三段开关) */
        int16_t mode_ch = rc_data.ch[CRSF_CH_MODE];
        if (mode_ch < 800)
            s_crsf_input.mode_request = 0;  /* IDLE */
        else if (mode_ch < 1200)
            s_crsf_input.mode_request = 1;  /* RC */
        else
            s_crsf_input.mode_request = 2;  /* OFFBOARD */

        /* 急停 (CH6 两段开关) */
        s_crsf_input.estop = (rc_data.ch[CRSF_CH_ESTOP] > 1500) ? 1 : 0;

        s_crsf_input.valid = 1;
        s_crsf_input.timestamp = HAL_GetTick();

        /* 更新状态 */
        s_crsf_status.connected = 1;
        s_crsf_status.last_frame_tick = HAL_GetTick();
        s_crsf_status.frame_count++;
    }
    else
    {
        s_crsf_status.error_count++;
    }
}

/* ============================================
 * CRSF UART IDLE 中断回调
 * ============================================ */
void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        /* 计算接收到的数据长度 */
        uint16_t pos = CRSF_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        if (pos > 0)
        {
            crsf_process_data(s_crsf_rx_buf, pos);
        }

        /* 重新启动 DMA 接收 (环形模式会自动重置) */
        s_dma_idle_detected = 1;
    }
}

/* ============================================
 * CRSF DMA 半满回调 (可选)
 * ============================================ */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* 可以在这里处理半满情况 */
}

/* ============================================
 * CRSF 任务初始化
 * ============================================ */
void CRSFTask_Init(void)
{
    /* 清零状态 */
    memset(&s_crsf_input, 0, sizeof(crsf_input_t));
    memset(&s_crsf_status, 0, sizeof(crsf_status_t));
    memset(s_crsf_rx_buf, 0, sizeof(s_crsf_rx_buf));

    /* 启动 DMA 接收 (环形模式) */
    if (HAL_UART_Receive_DMA(&huart7, s_crsf_rx_buf, CRSF_RX_BUF_SIZE) != HAL_OK)
    {
        s_crsf_status.error_count++;
    }

    /* 启用 IDLE 中断 */
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
}

/**
 * @brief 获取 CRSF 输入数据
 */
void CRSFTask_GetInput(crsf_input_t *input)
{
    if (input != NULL)
    {
        *input = s_crsf_input;
    }
}

/**
 * @brief 检查 CRSF 连接状态
 */
bool CRSFTask_IsConnected(void)
{
    uint32_t tick = HAL_GetTick();

    /* 检查超时 */
    if ((tick - s_crsf_status.last_frame_tick) > CRSF_TIMEOUT_MS)
    {
        s_crsf_status.connected = 0;
        s_crsf_input.valid = 0;
    }

    return s_crsf_status.connected ? true : false;
}

/**
 * @brief 获取 CRSF 状态
 */
void CRSFTask_GetStatus(crsf_status_t *status)
{
    if (status != NULL)
    {
        *status = s_crsf_status;
    }
}

/**
 * @brief CRSF 任务处理 (在任务中调用)
 */
void CRSFTask_Process(void *argument)
{
    (void)argument;

    /* 单步处理：由 AppTasks 的包装线程控制周期 */
    if (!CRSFTask_IsConnected())
    {
        /* CRSF 失联，可以在这里触发告警 */
    }
}