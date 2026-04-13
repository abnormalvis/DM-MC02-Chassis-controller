/**
 * @file crsf_task.h
 * @brief CRSF 遥控器接收任务
 * @description CRSF/ELRS 遥控器数据接收和解析
 */

#ifndef CRSF_TASK_H
#define CRSF_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* CRSF 通道数据 */
typedef struct {
    int16_t ch[16];           /* 16 通道原始值 */
    float throttle;            /* 油门 [-1, 1] */
    float steering;           /* 转向 [-1, 1] */
    uint8_t mode_request;     /* 模式请求: 0=IDLE, 1=RC, 2=OFFBOARD */
    uint8_t estop;           /* 急停标志 */
    bool valid;             /* 数据有效 */
    uint32_t timestamp;    /* 时间戳 */
} crsf_input_t;

/* CRSF 连接状态 */
typedef struct {
    uint8_t connected;     /* 连接标志 */
    uint32_t last_frame_tick; /* 最后收到帧的tick */
    uint16_t error_count;   /* 错误计数 */
    uint16_t frame_count;   /* 成功帧计数 */
} crsf_status_t;

/**
 * @brief CRSF 任务初始化
 */
void CRSFTask_Init(void);

/**
 * @brief 获取 CRSF 输入数据
 * @param input: 输出结构体
 */
void CRSFTask_GetInput(crsf_input_t *input);

/**
 * @brief 检查 CRSF 连接状态
 * @return true: 已连接
 */
bool CRSFTask_IsConnected(void);

/**
 * @brief 获取 CRSF 状态
 * @param status: 输出结构体
 */
void CRSFTask_GetStatus(crsf_status_t *status);

/**
 * @brief CRSF 任务处理 (在任务中调用)
 * @param argument: FreeRTOS 传入参数
 */
void CRSFTask_Process(void *argument);

#endif /* CRSF_TASK_H */