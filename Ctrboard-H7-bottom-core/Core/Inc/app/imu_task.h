/**
 * @file imu_task.h
 * @brief IMU 采集任务
 * @description BMI088 IMU 数据采集和姿态解算
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* IMU 数据结构 */
typedef struct {
    float gx, gy, gz;        /* 角速度 rad/s */
    float ax, ay, az;        /* 加速度 m/s^2 */
    float roll, pitch, yaw;    /* 姿态角 rad */
    uint32_t timestamp;     /* 时间戳 */
    bool valid;           /* 数据有效 */
} imu_data_t;

/**
 * @brief IMU 任务初始化
 */
void IMUTask_Init(void);

/**
 * @brief 获取 IMU 数据
 */
void IMUTask_GetData(imu_data_t *data);

/**
 * @brief 检查 IMU 状态
 */
bool IMUTask_IsReady(void);

/**
 * @brief IMU 任务处理 (在任务中调用)
 */
void IMUTask_Process(void *argument);

#endif /* IMU_TASK_H */