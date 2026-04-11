/**
 * @file c8t6_can_protocol.c
 * @brief C8T6 电流采样 CAN 通信协议实现
 */

#include "c8t6_can_protocol.h"
#include <string.h>

/* ============================================
 * 全局变量
 * ============================================ */
current_data_t g_current_data;
volatile uint8_t g_can_tx_complete = 1;
static control_cmd_t g_control_cmd;
static uint8_t g_cmd_ready = 0;
static uint16_t g_sample_rate = 1000;  /* 默认 1kHz */
static volatile uint8_t g_sampling_active = 0;

/* ============================================
 * CAN 协议初始化
 * ============================================ */
void can_protocol_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* 配置 FDCAN1 过滤器 - 接收控制命令帧 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_CONTROL_CMD;
    sFilterConfig.FilterID2 = CAN_ID_CONTROL_CMD;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 激活接收 FIFO 0 通知 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK)
    {
        Error_Handler();
    }

    /* 激活发送完成通知 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY) != HAL_OK)
    {
        Error_Handler();
    }

    /* 初始化电流数据 */
    memset(&g_current_data, 0, sizeof(current_data_t));
    g_current_data.status = 0xFF;  /* 初始状态无效 */
}

/* ============================================
 * 发送电流数据到 H723
 * ============================================ */
HAL_StatusTypeDef can_send_current_data(const current_data_t *data)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[CAN_DLC_CURRENT_DATA];

    /* 等待上一个发送完成 */
    while (!g_can_tx_complete);

    TxHeader.Identifier = CAN_ID_CURRENT_DATA;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* 打包数据 (小端序) */
    txData[0] = (uint8_t)(data->current_a & 0xFF);
    txData[1] = (uint8_t)((data->current_a >> 8) & 0xFF);
    txData[2] = (uint8_t)(data->current_b & 0xFF);
    txData[3] = (uint8_t)((data->current_b >> 8) & 0xFF);
    txData[4] = data->seq;
    txData[5] = data->status;
    txData[6] = (uint8_t)(data->timestamp & 0xFF);
    txData[7] = (uint8_t)((data->timestamp >> 8) & 0xFF);

    g_can_tx_complete = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);
}

/* ============================================
 * 接收控制命令
 * ============================================ */
int can_receive_control_cmd(control_cmd_t *cmd)
{
    if (g_cmd_ready && cmd != NULL)
    {
        memcpy(cmd, &g_control_cmd, sizeof(control_cmd_t));
        g_cmd_ready = 0;
        return 1;
    }
    return 0;
}

/* ============================================
 * CAN 接收中断回调
 * ============================================ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[CAN_DLC_CONTROL_CMD];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            /* 解析控制命令 */
            g_control_cmd.cmd = RxData[0];
            g_control_cmd.param = RxData[1];
            g_cmd_ready = 1;

            /* 处理命令 */
            switch (g_control_cmd.cmd)
            {
                case CMD_START:
                    g_sampling_active = 1;
                    break;
                case CMD_STOP:
                    g_sampling_active = 0;
                    break;
                case CMD_SET_PARAM:
                    if (g_control_cmd.param > 0 && g_control_cmd.param <= 10000)
                    {
                        g_sample_rate = g_control_cmd.param;
                    }
                    break;
                case CMD_CALIB_ZERO:
                    /* 执行零点校准 - 需要 ADC 驱动配合 */
                    // TODO: 调用 ADC 校准函数
                    break;
                default:
                    break;
            }
        }
    }
}

/* ============================================
 * CAN 发送完成回调
 * ============================================ */
void HAL_FDCAN_TxCompleteCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    g_can_tx_complete = 1;
}

/* ============================================
 * 配置采样频率
 * ============================================ */
void can_set_sample_rate(uint16_t sample_rate)
{
    if (sample_rate > 0 && sample_rate <= 10000)
    {
        g_sample_rate = sample_rate;
    }
}

/* ============================================
 * 获取当前电流数据
 * ============================================ */
void can_get_current_data(current_data_t *data)
{
    if (data != NULL)
    {
        memcpy(data, &g_current_data, sizeof(current_data_t));
    }
}

/* ============================================
 * 示例: 1ms 定时器中断中调用的采样函数
 * ============================================ */
/*
 * 在 SysTick 或定时器中断中调用此函数
 * 采样频率由 g_sample_rate 控制
 */
void can_sample_task(void)
{
    static uint32_t last_tick = 0;
    static uint16_t seq = 0;
    uint32_t tick = HAL_GetTick();
    uint32_t interval = 1000 / g_sample_rate;

    /* 简单的时间分频 */
    if ((tick - last_tick) >= interval && g_sampling_active)
    {
        last_tick = tick;

        /* TODO: 这里需要调用 ADC 采样函数获取实际电流值
         * 假设有 adc_get_current() 函数返回电流值
         *
         * int16_t ia = adc_get_current_a();
         * int16_t ib = adc_get_current_b();
         */

        /* 模拟数据 (实际需要替换为真实 ADC 采样) */
        g_current_data.current_a = 0;  /* TODO: 替换为真实值 */
        g_current_data.current_b = 0;  /* TODO: 替换为真实值 */

        /* 更新序号和时间戳 */
        g_current_data.seq = seq++;
        g_current_data.status = STATUS_SAMPLE_VALID;  /* 数据有效 */

        /* 获取 CAN 时间戳 (需要在 FDCAN 配置中启用) */
        uint32_t txTimestamp = 0;
        HAL_FDCAN_GetTxTimestamp(&hfdcan1, &txTimestamp);
        g_current_data.timestamp = (uint16_t)txTimestamp;

        /* 发送数据 */
        can_send_current_data(&g_current_data);
    }
}
