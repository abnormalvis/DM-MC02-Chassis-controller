/**
 * @file c8t6_can_protocol.h
 * @brief C8T6 电流采样 CAN 通信协议
 * @description 与 H723 CAN 通信协议对应
 *
 * 硬件连接:
 *   CAN TX -> PD13 (对应 C8T6 的 CAN TX 引脚)
 *   CAN RX -> PD12 (对应 C8T6 的 CAN RX 引脚)
 */

#ifndef C8T6_CAN_PROTOCOL_H
#define C8T6_CAN_PROTOCOL_H

#include "main.h"
#include "fdcan.h"

/* ============================================
 * CAN 帧 ID 定义 (需与 H723 端一致)
 * ============================================ */
#define CAN_ID_CURRENT_DATA     0x101  /* C8T6 -> H723: 电流数据 */
#define CAN_ID_CONTROL_CMD      0x102  /* H723 -> C8T6: 控制命令 */

/* ============================================
 * CAN DLC 长度定义
 * ============================================ */
#define CAN_DLC_CURRENT_DATA    8
#define CAN_DLC_CONTROL_CMD     2

/* ============================================
 * 命令码定义
 * ============================================ */
#define CMD_REQ_SAMPLE          0x00  /* 请求采样 */
#define CMD_SET_PARAM           0x01  /* 设置参数 (param: 采样频率等) */
#define CMD_START               0x02  /* 启动采样 */
#define CMD_STOP                0x03  /* 停止采样 */
#define CMD_CALIB_ZERO          0x04  /* 零点校准 */

/* ============================================
 * 状态位定义
 * ============================================ */
#define STATUS_OVERCURRENT_A    (1 << 0)  /* 电机A过流 */
#define STATUS_OVERCURRENT_B    (1 << 1)  /* 电机B过流 */
#define STATUS_FAULT            (1 << 2)  /* C8T6异常 */
#define STATUS_SAMPLE_VALID     (1 << 3)  /* 采样数据有效 */

/* ============================================
 * 电流数据结构
 * ============================================ */
typedef struct {
    int16_t current_a;        /* 电机A电流, 单位 0.01A (有符号) */
    int16_t current_b;        /* 电机B电流, 单位 0.01A (有符号) */
    uint8_t seq;              /* 序号, 0-255 循环 */
    uint8_t status;            /* 状态位 */
    uint16_t timestamp;        /* CAN 时间戳, 单位 0.1ms */
} __attribute__((packed)) current_data_t;

/* ============================================
 * 控制命令结构
 * ============================================ */
typedef struct {
    uint8_t cmd;              /* 命令码 */
    uint8_t param;            /* 参数值 */
} __attribute__((packed)) control_cmd_t;

/* ============================================
 * 全局变量声明
 * ============================================ */
extern FDCAN_HandleTypeDef hfdcan1;
extern current_data_t g_current_data;
extern volatile uint8_t g_can_tx_complete;

/* ============================================
 * 函数声明
 * ============================================ */

/**
 * @brief CAN 协议初始化
 */
void can_protocol_init(void);

/**
 * @brief 发送电流数据到 H723
 * @param data 电流数据结构体指针
 * @return HAL状态
 */
HAL_StatusTypeDef can_send_current_data(const current_data_t *data);

/**
 * @brief 接收控制命令 (非阻塞)
 * @param cmd 接收命令的结构体指针
 * @return 1: 有新命令, 0: 无新命令
 */
int can_receive_control_cmd(control_cmd_t *cmd);

/**
 * @brief CAN 接收回调 (在 FDCAN 中断中调用)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief CAN 发送完成回调
 */
void HAL_FDCAN_TxCompleteCallback(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief 配置采样参数
 * @param sample_rate 采样频率, 单位 Hz (默认 1000)
 */
void can_set_sample_rate(uint16_t sample_rate);

/**
 * @brief 获取当前电流数据
 */
void can_get_current_data(current_data_t *data);

#endif /* C8T6_CAN_PROTOCOL_H */
