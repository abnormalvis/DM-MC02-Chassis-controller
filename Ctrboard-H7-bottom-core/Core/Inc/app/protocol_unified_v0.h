/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_unified_v0.h
  * @brief   Unified protocol (USB/RS485) interfaces and payload structures.
  ******************************************************************************
  * @attention
  *
  * This is the primary protocol for current project phase.
  * It is designed for H723 as speed executor + DC motor current-loop chassis.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PROTOCOL_UNIFIED_V0_H__
#define __PROTOCOL_UNIFIED_V0_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "protocol_ids.h"

typedef struct
{
  uint16_t sof;      /* PROTO_V0_SOF */
  uint8_t ver;       /* PROTO_V0_VER */
  uint16_t msg_id;
  uint16_t seq;
  uint32_t ts_ms;
  uint16_t len;
} proto_v0_header_t;

typedef struct
{
  proto_v0_header_t hdr;
  const uint8_t *payload;
  uint16_t crc16;
} proto_v0_frame_view_t;

/* 0x0102 */
typedef struct
{
  uint8_t mode;      /* ctrl_mode_t */
  uint8_t source;    /* offboard_src_t */
} msg_cmd_set_mode_t;

/* 0x0103 */
typedef struct
{
  float vx_mps;
  float wz_rps;
  uint16_t reserved;
} msg_cmd_set_chassis_vel_t;

/* 0x0107: legacy wheel-rpm semantic bridge */
typedef struct
{
  int16_t rpm_a;
  int16_t rpm_b;
  int16_t rpm_c;
  int16_t rpm_d;
} msg_cmd_set_wheel_rpm_t;

/* 0x0108 */
typedef struct
{
  uint8_t target;    /* 0: speed-loop, 1: current-loop */
  float p;
  float i;
  float d;
} msg_cmd_set_pid_t;

/* 0x0109 */
typedef struct
{
  uint8_t servo1_deg;
  uint8_t servo2_deg;
} msg_cmd_set_servo_t;

/* 0x0202 */
typedef struct
{
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float roll;
  float pitch;
  float yaw;
} msg_sta_imu_t;

/* 0x0203 */
typedef struct
{
  float i_a;
  float i_b;
  float duty_a_fwd;
  float duty_a_rev;
  float duty_b_fwd;
  float duty_b_rev;
  uint8_t brake_active;
  uint16_t battery_mv;
} msg_sta_motor_t;

/* 0x0207 */
typedef struct
{
  int32_t enc_a;
  int32_t enc_b;
  int32_t enc_c;
  int32_t enc_d;
} msg_sta_encoder_t;

/* CRC and framing */
uint16_t proto_v0_crc16_ccitt_false(const uint8_t *data, uint16_t len);

bool proto_v0_try_decode(const uint8_t *buf,
                         uint16_t buf_len,
                         proto_v0_frame_view_t *out_view,
                         uint16_t *out_total_len);

bool proto_v0_encode(uint16_t msg_id,
                     uint16_t seq,
                     uint32_t ts_ms,
                     const uint8_t *payload,
                     uint16_t payload_len,
                     uint8_t *out_buf,
                     uint16_t out_cap,
                     uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_UNIFIED_V0_H__ */
