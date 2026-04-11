/**
 * @file app_tasks.h
 * @brief 应用层任务定义头文件
 * @description 定义所有应用层任务的接口和数据结构
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================
 * 任务优先级定义 (数值越小优先级越高)
 * ============================================ */
#define PRIORITY_MOTOR_CONTROL    1   /* 电机控制任务 - 最高 */
#define PRIORITY_CRSF             2   /* CRSF 接收任务 */
#define PRIORITY_CAN              3   /* CAN 通信任务 */
#define PRIORITY_USB              4   /* USB 通信任务 */
#define PRIORITY_RS485            5   /* RS485 通信任务 */
#define PRIORITY_IMU              6   /* IMU 采集任务 */
#define PRIORITY_SAFETY           7   /* 安全/故障管理任务 */
#define PRIORITY_LED              8   /* LED 指示任务 */
#define PRIORITY_BUZZER           9   /* 蜂鸣器任务 */

/* ============================================
 * 任务堆栈大小定义
 * ============================================ */
#define STACK_SIZE_MOTOR_CONTROL  512
#define STACK_SIZE_CRSF            256
#define STACK_SIZE_CAN             256
#define STACK_SIZE_USB             256
#define STACK_SIZE_RS485          256
#define STACK_SIZE_IMU             256
#define STACK_SIZE_SAFETY          256
#define STACK_SIZE_LED             128
#define STACK_SIZE_BUZZER          128

/* ============================================
 * 任务周期定义 (ms)
 * ============================================ */
#define PERIOD_MOTOR_CONTROL   1       /* 1kHz */
#define PERIOD_CRSF            1       /* 1kHz 轮询 */
#define PERIOD_CAN             10       /* 100Hz 状态检查 */
#define PERIOD_USB             10       /* 100Hz 通信 */
#define PERIOD_RS485           10       /* 100Hz 通信 */
#define PERIOD_IMU             1       /* 1kHz */
#define PERIOD_SAFETY          5       /* 200Hz 安全检查 */
#define PERIOD_LED             20      /* 50Hz LED 更新 */
#define PERIOD_BUZZER          10      /* 100Hz 蜂鸣器 */

/* ============================================
 * 控制模式定义
 * ============================================ */
typedef enum {
    MODE_IDLE = 0,
    MODE_RC,           /* RC 遥控模式 */
    MODE_OFFBOARD,     /* 上位机控制模式 */
    MODE_BRAKE_PROTECT /* 刹车保护模式 */
} control_mode_t;

/* ============================================
 * 电机状态定义
 * ============================================ */
typedef struct {
    int16_t current_a;     /* 电机A电流, 单位 0.01A */
    int16_t current_b;     /* 电机B电流, 单位 0.01A */
    int16_t target_speed_a; /* 电机A目标速度 */
    int16_t target_speed_b; /* 电机B目标速度 */
    uint8_t brake_active;  /* 刹车激活标志 */
} motor_state_t;

/* ============================================
 * 系统状态定义
 * ============================================ */
typedef struct {
    control_mode_t mode;           /* 当前控制模式 */
    uint8_t rc_connected;          /* RC 连接状态 */
    uint8_t can_connected;         /* CAN 连接状态 */
    uint8_t usb_connected;         /* USB 连接状态 */
    uint8_t rs485_connected;       /* RS485 连接状态 */
    uint32_t fault_bits;           /* 故障位 */
    uint32_t last_update_tick;    /* 最后更新时间 */
} system_state_t;

/* ============================================
 * 控制命令定义
 * ============================================ */
typedef struct {
    float vx_mps;          /* 线速度, m/s */
    float wz_rps;          /* 角速度, rad/s */
    uint8_t estop;         /* 急停标志 */
    uint8_t valid;         /* 数据有效标志 */
    uint32_t timestamp;    /* 时间戳 */
} chassis_cmd_t;

/* ============================================
 * 任务初始化函数声明
 * ============================================ */

/**
 * @brief 创建所有应用层任务
 */
void AppTasks_Create(void);

/**
 * @brief 电机控制任务
 */
void MotorControlTask_Init(void);
void MotorControlTask_Process(void);

/**
 * @brief CRSF 接收任务
 */
void CRSFTask_Init(void);
void CRSFTask_Process(void);

/**
 * @brief CAN 通信任务
 */
void CANTask_Init(void);
void CANTask_Process(void);

/**
 * @brief USB 通信任务
 */
void USBTask_Init(void);
void USBTask_Process(void);

/**
 * @brief RS485 通信任务
 */
void RS485Task_Init(void);
void RS485Task_Process(void);

/**
 * @brief IMU 采集任务
 */
void IMUTask_Init(void);
void IMUTask_Process(void);

/**
 * @brief 安全/故障管理任务
 */
void SafetyTask_Init(void);
void SafetyTask_Process(void);

/**
 * @brief LED 指示任务
 */
void LEDTask_Init(void);
void LEDTask_Process(void);

/**
 * @brief 蜂鸣器任务
 */
void BuzzerTask_Init(void);
void BuzzerTask_Process(void);

/* ============================================
 * 公共接口 (供其他模块调用)
 * ============================================ */

/* 获取系统状态 */
void AppTasks_GetSystemState(system_state_t *state);

/* 设置控制模式 */
void AppTasks_SetControlMode(control_mode_t mode);

/* 获取控制命令 */
chassis_cmd_t* AppTasks_GetRCCommand(void);
chassis_cmd_t* AppTasks_GetOffboardCommand(void);

/* 获取电机状态 */
void AppTasks_GetMotorState(motor_state_t *state);

/* 触发急停 */
void AppTasks_TriggerEStop(void);

/* 设置故障位 */
void AppTasks_SetFault(uint32_t fault_bit);

/* 清除故障位 */
void AppTasks_ClearFault(uint32_t fault_bit);

#endif /* APP_TASKS_H */
