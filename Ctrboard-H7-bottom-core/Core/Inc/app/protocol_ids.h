/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_ids.h
  * @brief   Protocol message IDs and common constants.
  ******************************************************************************
  * @attention
  *
  * This header defines the message ID plan used by USB/RS485 unified protocol.
  * Legacy 12-byte protocol IDs are defined in protocol_legacy_v1.h.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PROTOCOL_IDS_H__
#define __PROTOCOL_IDS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Protocol version */
#define PROTO_V0_VER                      ((uint8_t)0x01)

/* Unified frame constants */
#define PROTO_V0_SOF                      ((uint16_t)0x55AA)
#define PROTO_V0_MAX_PAYLOAD              ((uint16_t)128U)

/* Downlink: host -> H723 */
#define MSG_CMD_HEARTBEAT                 ((uint16_t)0x0101)
#define MSG_CMD_SET_MODE                  ((uint16_t)0x0102)
#define MSG_CMD_SET_CHASSIS_VEL           ((uint16_t)0x0103)
#define MSG_CMD_ESTOP                     ((uint16_t)0x0104)
#define MSG_CMD_PARAM_SET                 ((uint16_t)0x0105)
#define MSG_CMD_PARAM_GET                 ((uint16_t)0x0106)
#define MSG_CMD_SET_WHEEL_RPM             ((uint16_t)0x0107)
#define MSG_CMD_SET_PID                   ((uint16_t)0x0108)
#define MSG_CMD_SET_SERVO                 ((uint16_t)0x0109)

/* Uplink: H723 -> host */
#define MSG_STA_HEARTBEAT                 ((uint16_t)0x0201)
#define MSG_STA_IMU                       ((uint16_t)0x0202)
#define MSG_STA_MOTOR                     ((uint16_t)0x0203)
#define MSG_STA_RC                        ((uint16_t)0x0204)
#define MSG_STA_FAULT                     ((uint16_t)0x0205)
#define MSG_STA_PARAM                     ((uint16_t)0x0206)
#define MSG_STA_ENCODER                   ((uint16_t)0x0207)

/* Mode definition */
typedef enum
{
  CTRL_MODE_IDLE = 0,
  CTRL_MODE_RC = 1,
  CTRL_MODE_OFFBOARD = 2
} ctrl_mode_t;

/* Offboard source */
typedef enum
{
  OFFBOARD_SRC_NONE = 0,
  OFFBOARD_SRC_USB = 1,
  OFFBOARD_SRC_RS485 = 2
} offboard_src_t;

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_IDS_H__ */
