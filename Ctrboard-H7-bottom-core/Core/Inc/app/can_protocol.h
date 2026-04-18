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
#define CAN_ID_DRIVE_CMD        0x103  /* H723 -> C8T6: 左右履带控制 */

/* CAN 帧长度 */
#define CAN_DLC_CURRENT_DATA    8
#define CAN_DLC_CONTROL_CMD     2
#define CAN_DLC_DRIVE_CMD       8

/* Drive flags 位定义 */
#define CAN_DRIVE_FLAG_ESTOP          (1U << 0)
#define CAN_DRIVE_FLAG_BRAKE          (1U << 1)
#define CAN_DRIVE_FLAG_THROTTLE_LOCK  (1U << 2)
#define CAN_DRIVE_FLAG_MODE_OFFBOARD  (1U << 3)
#define CAN_DRIVE_FLAG_SOURCE_RC      (1U << 4)

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

/**
 * @brief 左右履带驱动命令
 */
typedef struct {
    int16_t left_cmd;         /* 左履带目标 [-1000, 1000] */
    int16_t right_cmd;        /* 右履带目标 [-1000, 1000] */
    uint8_t flags;            /* CAN_DRIVE_FLAG_xxx */
    uint8_t seq;              /* 序号 */
    uint16_t reserved;        /* 预留 */
} drive_cmd_t;

/* Exported functions */
void can_protocol_init(void);
void can_send_control_cmd(uint8_t cmd, uint8_t param);
void can_send_drive_cmd(const drive_cmd_t *cmd);
int can_receive_current_data(current_data_t *data);
void can_protocol_register_callback(void (*callback)(const current_data_t *data));

#endif /* CAN_PROTOCOL_H */
