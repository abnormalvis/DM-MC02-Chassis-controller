/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_adapter.c
  * @brief   Protocol adapter skeleton implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protocol_adapter.h"
#include <stddef.h>

static proto_adapter_cfg_t g_cfg = {
  true,
  true,
  0.001f,
  0.001f
};

static int16_t be_i16(uint8_t hi, uint8_t lo)
{
  uint16_t u = ((uint16_t)hi << 8) | (uint16_t)lo;
  return (int16_t)u;
}

void proto_adapter_init(const proto_adapter_cfg_t *cfg)
{
  if (cfg != NULL)
  {
    g_cfg = *cfg;
  }
}

bool proto_adapter_legacy_rpm_to_chassis(const msg_cmd_set_wheel_rpm_t *rpm_cmd,
                                         uint32_t ts_ms,
                                         chassis_ctrl_cmd_t *out_cmd)
{
  float left_rpm;
  float right_rpm;

  if ((rpm_cmd == NULL) || (out_cmd == NULL))
  {
    return false;
  }

  left_rpm = (float)rpm_cmd->rpm_a;
  right_rpm = (float)rpm_cmd->rpm_b;

  if (!g_cfg.legacy_rpm_use_ab_only)
  {
    left_rpm = 0.5f * ((float)rpm_cmd->rpm_a + (float)rpm_cmd->rpm_c);
    right_rpm = 0.5f * ((float)rpm_cmd->rpm_b + (float)rpm_cmd->rpm_d);
  }

  out_cmd->vx_mps = 0.5f * (left_rpm + right_rpm) * g_cfg.rpm_to_vx_scale;
  out_cmd->wz_rps = (right_rpm - left_rpm) * g_cfg.rpm_to_wz_scale;
  out_cmd->mode = (uint8_t)CTRL_MODE_OFFBOARD;
  out_cmd->estop = 0U;
  out_cmd->estop_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
  out_cmd->brake_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
  out_cmd->throttle_lock_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
  out_cmd->source = (uint8_t)OFFBOARD_SRC_RS485;
  out_cmd->ts_ms = ts_ms;
  out_cmd->valid = 1U;
  return true;
}

bool proto_adapter_on_unified_frame(const proto_v0_frame_view_t *frame,
                                    chassis_ctrl_cmd_t *out_cmd)
{
  const msg_cmd_set_mode_t *m;
  const msg_cmd_set_chassis_vel_t *v;
  const msg_cmd_set_wheel_rpm_t *r;

  if ((frame == NULL) || (out_cmd == NULL))
  {
    return false;
  }

  out_cmd->valid = 0U;
  out_cmd->estop = 0U;
  out_cmd->estop_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
  out_cmd->brake_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
  out_cmd->throttle_lock_cmd = (uint8_t)CTRL_SW_CMD_HOLD;

  switch (frame->hdr.msg_id)
  {
    case MSG_CMD_SET_MODE:
      if (frame->hdr.len < sizeof(msg_cmd_set_mode_t))
      {
        return false;
      }
      m = (const msg_cmd_set_mode_t *)frame->payload;
      out_cmd->mode = m->mode;
      out_cmd->source = m->source;
      out_cmd->ts_ms = frame->hdr.ts_ms;
      out_cmd->valid = 1U;
      return true;

    case MSG_CMD_SET_CHASSIS_VEL:
      if (frame->hdr.len < sizeof(msg_cmd_set_chassis_vel_t))
      {
        return false;
      }
      v = (const msg_cmd_set_chassis_vel_t *)frame->payload;
      out_cmd->vx_mps = v->vx_mps;
      out_cmd->wz_rps = v->wz_rps;
      out_cmd->mode = (uint8_t)CTRL_MODE_OFFBOARD;
      out_cmd->source = (uint8_t)OFFBOARD_SRC_USB;
      out_cmd->estop = 0U;
      out_cmd->estop_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
      out_cmd->brake_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
      out_cmd->throttle_lock_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
      out_cmd->ts_ms = frame->hdr.ts_ms;
      out_cmd->valid = 1U;
      return true;

    case MSG_CMD_ESTOP:
      out_cmd->estop = 1U;
      out_cmd->estop_cmd = (uint8_t)CTRL_SW_CMD_ENABLE;
      out_cmd->brake_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
      out_cmd->throttle_lock_cmd = (uint8_t)CTRL_SW_CMD_HOLD;
      out_cmd->mode = (uint8_t)CTRL_MODE_OFFBOARD;
      out_cmd->source = (uint8_t)OFFBOARD_SRC_USB;
      out_cmd->ts_ms = frame->hdr.ts_ms;
      out_cmd->valid = 1U;
      return true;

    case MSG_CMD_SET_WHEEL_RPM:
      if (frame->hdr.len < sizeof(msg_cmd_set_wheel_rpm_t))
      {
        return false;
      }
      r = (const msg_cmd_set_wheel_rpm_t *)frame->payload;
      return proto_adapter_legacy_rpm_to_chassis(r, frame->hdr.ts_ms, out_cmd);

    default:
      return false;
  }
}

bool proto_adapter_on_legacy_frame(const legacy_v1_frame_t *frame,
                                   chassis_ctrl_cmd_t *out_cmd)
{
  msg_cmd_set_wheel_rpm_t rpm_cmd;

  if ((frame == NULL) || (out_cmd == NULL) || (!g_cfg.enable_legacy_v1))
  {
    return false;
  }

  out_cmd->valid = 0U;

  if (frame->func == LEGACY_FUNC_WHEEL_TARGET)
  {
    rpm_cmd.rpm_a = be_i16(frame->data[0], frame->data[1]);
    rpm_cmd.rpm_b = be_i16(frame->data[2], frame->data[3]);
    rpm_cmd.rpm_c = be_i16(frame->data[4], frame->data[5]);
    rpm_cmd.rpm_d = be_i16(frame->data[6], frame->data[7]);
    return proto_adapter_legacy_rpm_to_chassis(&rpm_cmd, 0U, out_cmd);
  }

  return false;
}

void proto_adapter_fill_sta_motor(const motor_telemetry_t *mt, msg_sta_motor_t *out)
{
  if ((mt == NULL) || (out == NULL))
  {
    return;
  }

  out->i_a = mt->i_a;
  out->i_b = mt->i_b;
  out->duty_a_fwd = mt->duty_a_fwd;
  out->duty_a_rev = mt->duty_a_rev;
  out->duty_b_fwd = mt->duty_b_fwd;
  out->duty_b_rev = mt->duty_b_rev;
  out->brake_active = mt->brake_active;
  out->battery_mv = mt->battery_mv;
}
