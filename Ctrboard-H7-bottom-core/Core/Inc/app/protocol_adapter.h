/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_adapter.h
  * @brief   Adapter interfaces from protocol layer to chassis control layer.
  ******************************************************************************
  * @attention
  *
  * This file isolates protocol semantics from control semantics.
  * Legacy encoder-motor commands are translated here and SHALL NOT directly
  * drive DC current-loop output without conversion/safety checks.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PROTOCOL_ADAPTER_H__
#define __PROTOCOL_ADAPTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "protocol_legacy_v1.h"
#include "protocol_unified_v0.h"

typedef enum
{
  CTRL_SW_CMD_HOLD = 0,
  CTRL_SW_CMD_ENABLE = 1,
  CTRL_SW_CMD_DISABLE = 2
} ctrl_sw_cmd_t;

/* Internal control command used by H723 execution layer */
typedef struct
{
  float vx_mps;
  float wz_rps;
  uint8_t mode;       /* ctrl_mode_t */
  uint8_t estop;
  uint8_t estop_cmd;        /* ctrl_sw_cmd_t */
  uint8_t brake_cmd;        /* ctrl_sw_cmd_t */
  uint8_t throttle_lock_cmd;/* ctrl_sw_cmd_t */
  uint8_t source;     /* offboard_src_t */
  uint32_t ts_ms;
  uint8_t valid;
} chassis_ctrl_cmd_t;

/* Motor telemetry for protocol uplink */
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
} motor_telemetry_t;

/* Adapter configuration for compatibility behavior */
typedef struct
{
  bool enable_legacy_v1;
  bool legacy_rpm_use_ab_only;   /* true: use A/B only, ignore C/D */
  float rpm_to_vx_scale;         /* optional k for rpm->vx conversion */
  float rpm_to_wz_scale;         /* optional k for differential mapping */
} proto_adapter_cfg_t;

void proto_adapter_init(const proto_adapter_cfg_t *cfg);

/* Unified protocol entry */
bool proto_adapter_on_unified_frame(const proto_v0_frame_view_t *frame,
                                    chassis_ctrl_cmd_t *out_cmd);

/* Legacy protocol entry (compatibility only) */
bool proto_adapter_on_legacy_frame(const legacy_v1_frame_t *frame,
                                   chassis_ctrl_cmd_t *out_cmd);

/* Build uplink payloads from internal telemetry */
void proto_adapter_fill_sta_motor(const motor_telemetry_t *mt, msg_sta_motor_t *out);

/* Optional helper: legacy wheel-rpm command -> chassis command */
bool proto_adapter_legacy_rpm_to_chassis(const msg_cmd_set_wheel_rpm_t *rpm_cmd,
                                         uint32_t ts_ms,
                                         chassis_ctrl_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_ADAPTER_H__ */
