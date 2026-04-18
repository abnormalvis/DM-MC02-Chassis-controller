/**
 * @file can_protocol.c
 * @brief C8T6 与 H723 CAN 通信协议实现
 * @description 用于电流数据采集的时间戳同步
 */

#include "can_protocol.h"
#include "main.h"
#include <string.h>

/* CAN 句柄声明 (需要在 main 中初始化) */
extern FDCAN_HandleTypeDef hfdcan3;

/* 接收缓冲区 */
static current_data_t g_current_data;
static uint8_t g_data_ready = 0;
static uint8_t g_drive_seq = 0U;

/* 回调函数指针 */
static void (*g_callback)(const current_data_t *data) = NULL;

/**
 * @brief CAN 协议初始化
 */
void can_protocol_init(void)
{
    /* 配置 FDCAN3 过滤器 - 接收电流数据帧 */
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_CURRENT_DATA;
    sFilterConfig.FilterID2 = CAN_ID_CURRENT_DATA;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 激活接收 FIFO 0 通知 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 发送控制命令到 C8T6
 * @param cmd 命令码
 * @param param 参数值
 */
void can_send_control_cmd(uint8_t cmd, uint8_t param)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[CAN_DLC_CONTROL_CMD];

    TxHeader.Identifier = CAN_ID_CONTROL_CMD;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    txData[0] = cmd;
    txData[1] = param;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, txData) != HAL_OK)
    {
        Error_Handler();
    }
}

void can_send_drive_cmd(const drive_cmd_t *cmd)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[CAN_DLC_DRIVE_CMD] = {0};
    drive_cmd_t local;

    if (cmd == NULL)
    {
        return;
    }

    local = *cmd;
    local.seq = g_drive_seq++;

    TxHeader.Identifier = CAN_ID_DRIVE_CMD;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    txData[0] = (uint8_t)(local.left_cmd & 0xFF);
    txData[1] = (uint8_t)((local.left_cmd >> 8) & 0xFF);
    txData[2] = (uint8_t)(local.right_cmd & 0xFF);
    txData[3] = (uint8_t)((local.right_cmd >> 8) & 0xFF);
    txData[4] = local.flags;
    txData[5] = local.seq;
    txData[6] = (uint8_t)(local.reserved & 0xFF);
    txData[7] = (uint8_t)((local.reserved >> 8) & 0xFF);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, txData) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 接收电流数据（非阻塞）
 * @param data 接收数据结构体指针
 * @return 1: 有新数据, 0: 无新数据
 */
int can_receive_current_data(current_data_t *data)
{
    if (g_data_ready && data != NULL)
    {
        memcpy(data, &g_current_data, sizeof(current_data_t));
        g_data_ready = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief 注册电流数据回调函数
 * @param callback 回调函数指针
 */
void can_protocol_register_callback(void (*callback)(const current_data_t *data))
{
    g_callback = callback;
}

/**
 * @brief CAN 接收回调 (在 FDCAN3 中断中调用)
 */
void HAL_FDCAN_RxFIFO0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[CAN_DLC_CURRENT_DATA];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        /* 读取接收到的数据 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            /* 解析电流数据 */
            g_current_data.current_a = (int16_t)(RxData[0] | (RxData[1] << 8));
            g_current_data.current_b = (int16_t)(RxData[2] | (RxData[3] << 8));
            g_current_data.seq = RxData[4];
            g_current_data.status = RxData[5];
            g_current_data.timestamp = (uint16_t)(RxData[6] | (RxData[7] << 8));

            /* 记录 H723 侧的接收时刻 */
            g_current_data.rx_timestamp = HAL_GetTick() * 1000; /* 转换为微秒 */

            g_data_ready = 1;

            /* 调用回调函数 */
            if (g_callback != NULL)
            {
                g_callback(&g_current_data);
            }
        }
    }
}
