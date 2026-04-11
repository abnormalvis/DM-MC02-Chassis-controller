/**
 * @file can_protocol.h
 * @brief C8T6 与 H723 CAN 通信协议定义
 * @description 用于电流数据采集的时间戳同步
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

/* CAN 帧 ID 定义 */
#define CAN_ID_CURRENT_DATA     0x101  /* C8T6 -> H723: 电流数据 */
#define CAN_ID_CONTROL_CMD      0x102  /* H723 -> C8T6: 控制命令 */

/* CAN 帧长度 */
#define CAN_DLC_CURRENT_DATA    8
#define CAN_DLC_CONTROL_CMD     2

/* 命令定义 */
#define CMD_REQ_SAMPLE          0x00  /* 请求采样 */
#define CMD_SET_PARAM           0x01  /* 设置参数 */

/* 状态位定义 */
#define STATUS_OVERCURRENT_A    (1 << 0)  /* 电机A过流 */
#define STATUS_OVERCURRENT_B    (1 << 1)  /* 电机B过流 */
#define STATUS_FAULT            (1 << 2)  /* C8T6异常 */

/**
 * @brief 电流数据结构体
 */
typedef struct {
    int16_t current_a;        /* 电机A电流，单位 0.01A */
    int16_t current_b;        /* 电机B电流，单位 0.01A */
    uint8_t seq;              /* 序号，0-255循环 */
    uint8_t status;           /* 状态位 */
    uint16_t timestamp;       /* CAN时间戳，单位0.1ms */
    uint32_t rx_timestamp;    /* H723接收时刻的系统时间戳，单位1us */
} current_data_t;

/**
 * @brief 控制命令结构体
 */
typedef struct {
    uint8_t cmd;              /* 命令码 */
    uint8_t param;            /* 参数值 */
} control_cmd_t;

/* Exported functions */
void can_protocol_init(void);
void can_send_control_cmd(uint8_t cmd, uint8_t param);
int can_receive_current_data(current_data_t *data);
void can_protocol_register_callback(void (*callback)(const current_data_t *data));

#endif /* CAN_PROTOCOL_H */
