/**
 * @file imu_task.c
 * @brief IMU 采集任务实现
 * @description BMI088 IMU 数据采集和姿态解算
 */

#include "imu_task.h"
#include "main.h"
#include "spi.h"
#include <string.h>
#include <math.h>

/* ============================================
 * 配置参数
 * ============================================ */
/* 互补滤波参数 */
#define COMPLEMENT_ALPHA       0.98f   /* 陀螺仪权重 */

/* ============================================
 * 私有变量
 * ============================================ */
static imu_data_t s_imu_data = {0};
static uint8_t s_imu_ready = 0;

/* 姿态积分 */
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;
static uint32_t s_last_tick = 0;

/* ============================================
 * BMI088 驱动接口 (需要从例程复制或引用)
 * ============================================ */
/* 外部声明 - 需要在项目中添加 BMI088 驱动 */
extern uint8_t BMI088_init(void);
extern void BMI088_read(float gyro[3], float accel[3], float *temperature);

/* ============================================
 * 姿态解算 (简单互补滤波)
 * ============================================ */
static void imu_update_attitude(float gx, float gy, float gz, float ax, float ay, float az, uint32_t tick)
{
    float dt = 0.001f;  /* 固定 1ms */

    if (s_last_tick > 0)
    {
        dt = (float)(tick - s_last_tick) / 1000.0f;
        if (dt < 0.0001f) dt = 0.001f;  /* 防止除零 */
        if (dt > 0.1f) dt = 0.001f;    /* 防止长时间未更新 */
    }
    s_last_tick = tick;

    /* 陀螺仪积分 (高速) */
    s_roll += gx * dt;
    s_pitch += gy * dt;
    s_yaw += gz * dt;

    /* 加速度计修正 (低速) */
    float roll_acc = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

    /* 互补滤波 */
    s_roll = COMPLEMENT_ALPHA * s_roll + (1.0f - COMPLEMENT_ALPHA) * roll_acc;
    s_pitch = COMPLEMENT_ALPHA * s_pitch + (1.0f - COMPLEMENT_ALPHA) * pitch_acc;

    /* 归一化角度到 [-PI, PI] */
    while (s_roll > M_PI) s_roll -= 2.0f * M_PI;
    while (s_roll < -M_PI) s_roll += 2.0f * M_PI;
    while (s_pitch > M_PI) s_pitch -= 2.0f * M_PI;
    while (s_pitch < -M_PI) s_pitch += 2.0f * M_PI;
    while (s_yaw > M_PI) s_yaw -= 2.0f * M_PI;
    while (s_yaw < -M_PI) s_yaw += 2.0f * M_PI;
}

/* ============================================
 * IMU 任务初始化
 * ============================================ */
void IMUTask_Init(void)
{
    /* 初始化 BMI088 */
    uint8_t ret = BMI088_init();
    s_imu_ready = (ret == 0) ? 1 : 0;

    /* 清零数据 */
    memset(&s_imu_data, 0, sizeof(imu_data_t));
}

/**
 * @brief 获取 IMU 数据
 */
void IMUTask_GetData(imu_data_t *data)
{
    if (data != NULL)
    {
        *data = s_imu_data;
    }
}

/**
 * @brief 检查 IMU 状态
 */
bool IMUTask_IsReady(void)
{
    return s_imu_ready ? true : false;
}

/**
 * @brief IMU 任务处理 (在任务中调用)
 */
void IMUTask_Process(void *argument)
{
    (void)argument;

    /* 单步处理：由 AppTasks 的包装线程控制周期 */
    if (!s_imu_ready)
    {
        return;
    }

    /* 读取 BMI088 */
    float gyro[3] = {0};
    float accel[3] = {0};
    float temperature = 0;

    BMI088_read(gyro, accel, &temperature);

    /* 更新姿态 */
    uint32_t tick = HAL_GetTick();
    imu_update_attitude(gyro[0], gyro[1], gyro[2],
                        accel[0], accel[1], accel[2], tick);

    /* 填充数据 */
    s_imu_data.gx = gyro[0];
    s_imu_data.gy = gyro[1];
    s_imu_data.gz = gyro[2];
    s_imu_data.ax = accel[0];
    s_imu_data.ay = accel[1];
    s_imu_data.az = accel[2];
    s_imu_data.roll = s_roll;
    s_imu_data.pitch = s_pitch;
    s_imu_data.yaw = s_yaw;
    s_imu_data.timestamp = tick;
    s_imu_data.valid = 1;
}