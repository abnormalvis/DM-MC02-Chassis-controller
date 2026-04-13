#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "protocol_adapter.h"
#include "crsf_protocol.h"
#include "protocol_ids.h"
#include "protocol_unified_v0.h"

static void test_unified_decode_and_adapt(void)
{
  msg_cmd_set_chassis_vel_t cmd = {
    .vx_mps = 1.25f,
    .wz_rps = -0.4f,
    .reserved = 0U
  };

  uint8_t frame[128];
  uint16_t frame_len = 0U;
  proto_v0_frame_view_t view;
  uint16_t consumed = 0U;
  chassis_ctrl_cmd_t out;
  bool ok;

  ok = proto_v0_encode(MSG_CMD_SET_CHASSIS_VEL,
                       1U,
                       1234U,
                       (const uint8_t *)&cmd,
                       (uint16_t)sizeof(cmd),
                       frame,
                       (uint16_t)sizeof(frame),
                       &frame_len);
  assert(ok);

  ok = proto_v0_try_decode(frame, frame_len, &view, &consumed);
  assert(ok);
  assert(consumed == frame_len);
  assert(view.hdr.msg_id == MSG_CMD_SET_CHASSIS_VEL);

  memset(&out, 0, sizeof(out));
  ok = proto_adapter_on_unified_frame(&view, &out);
  assert(ok);
  assert(out.valid == 1U);
  assert(out.mode == CTRL_MODE_OFFBOARD);
  assert(out.source == OFFBOARD_SRC_USB);
  assert(out.vx_mps > 1.24f && out.vx_mps < 1.26f);
  assert(out.wz_rps < -0.39f && out.wz_rps > -0.41f);
}

static void test_unified_reject_bad_crc_and_header(void)
{
  msg_cmd_set_chassis_vel_t cmd = {
    .vx_mps = 0.5f,
    .wz_rps = 0.2f,
    .reserved = 0U
  };
  uint8_t frame[128];
  uint16_t frame_len = 0U;
  proto_v0_frame_view_t view;
  uint16_t consumed = 0U;
  bool ok;

  ok = proto_v0_encode(MSG_CMD_SET_CHASSIS_VEL,
                       7U,
                       888U,
                       (const uint8_t *)&cmd,
                       (uint16_t)sizeof(cmd),
                       frame,
                       (uint16_t)sizeof(frame),
                       &frame_len);
  assert(ok);

  /* CRC 错误应被拒绝 */
  frame[frame_len - 1U] ^= 0x5AU;
  ok = proto_v0_try_decode(frame, frame_len, &view, &consumed);
  assert(!ok);

  /* 修正 CRC 后，错误帧头仍应被拒绝 */
  ok = proto_v0_encode(MSG_CMD_SET_CHASSIS_VEL,
                       7U,
                       888U,
                       (const uint8_t *)&cmd,
                       (uint16_t)sizeof(cmd),
                       frame,
                       (uint16_t)sizeof(frame),
                       &frame_len);
  assert(ok);
  frame[0] = 0x00U;
  frame[1] = 0x00U;
  ok = proto_v0_try_decode(frame, frame_len, &view, &consumed);
  assert(!ok);
}

static void test_legacy_decode_and_adapt(void)
{
  uint8_t legacy_raw[12];
  uint8_t data8[8] = {
    0x00, 0x64, /* A = 100 rpm */
    0x00, 0x64, /* B = 100 rpm */
    0x00, 0x00, /* C = 0 */
    0x00, 0x00  /* D = 0 */
  };
  legacy_v1_frame_t parsed;
  chassis_ctrl_cmd_t out;
  proto_adapter_cfg_t cfg = {
    .enable_legacy_v1 = true,
    .legacy_rpm_use_ab_only = true,
    .rpm_to_vx_scale = 0.001f,
    .rpm_to_wz_scale = 0.001f
  };
  bool ok;

  proto_adapter_init(&cfg);
  legacy_v1_make((uint8_t)LEGACY_FUNC_WHEEL_TARGET, data8, legacy_raw);
  ok = legacy_v1_parse(legacy_raw, &parsed);
  assert(ok);

  memset(&out, 0, sizeof(out));
  ok = proto_adapter_on_legacy_frame(&parsed, &out);
  assert(ok);
  assert(out.valid == 1U);
  assert(out.mode == CTRL_MODE_OFFBOARD);
  assert(out.vx_mps > 0.09f && out.vx_mps < 0.11f);
}

static void test_legacy_reject_when_disabled(void)
{
  uint8_t legacy_raw[12];
  uint8_t data8[8] = {
    0x00, 0x64,
    0x00, 0x64,
    0x00, 0x00,
    0x00, 0x00
  };
  legacy_v1_frame_t parsed;
  chassis_ctrl_cmd_t out;
  proto_adapter_cfg_t cfg = {
    .enable_legacy_v1 = false,
    .legacy_rpm_use_ab_only = true,
    .rpm_to_vx_scale = 0.001f,
    .rpm_to_wz_scale = 0.001f
  };
  bool ok;

  proto_adapter_init(&cfg);
  legacy_v1_make((uint8_t)LEGACY_FUNC_WHEEL_TARGET, data8, legacy_raw);
  ok = legacy_v1_parse(legacy_raw, &parsed);
  assert(ok);

  memset(&out, 0, sizeof(out));
  ok = proto_adapter_on_legacy_frame(&parsed, &out);
  assert(!ok);
  assert(out.valid == 0U);
}

static void pack_crsf_16ch(const uint16_t ch[16], uint8_t out_payload[22])
{
  uint32_t bitpos = 0U;
  uint8_t i;

  memset(out_payload, 0, 22U);
  for (i = 0U; i < 16U; i++)
  {
    uint32_t v = (uint32_t)(ch[i] & 0x07FFU);
    uint32_t byte_idx = bitpos >> 3;
    uint32_t shift = bitpos & 0x7U;
    uint32_t w = v << shift;

    out_payload[byte_idx] |= (uint8_t)(w & 0xFFU);
    out_payload[byte_idx + 1U] |= (uint8_t)((w >> 8) & 0xFFU);
    out_payload[byte_idx + 2U] |= (uint8_t)((w >> 16) & 0xFFU);
    bitpos += 11U;
  }
}

static void test_crsf_decode(void)
{
  uint16_t in_ch[16] = {
    1200, 800, 992, 992, 1811, 172, 992, 992,
    992, 992, 992, 992, 992, 992, 992, 992
  };
  uint8_t frame[26];
  crsf_rc_channels_t rc;
  bool ok;

  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = 24U;
  frame[2] = CRSF_FRAME_TYPE_RC_CHANNELS;
  pack_crsf_16ch(in_ch, &frame[3]);
  frame[25] = crsf_crc8_dvb_s2(&frame[2], 23U);

  ok = crsf_try_decode_rc_channels(frame, 26U, &rc);
  assert(ok);
  assert(rc.valid == 1U);
  assert(rc.ch[0] == in_ch[0]);
  assert(rc.ch[1] == in_ch[1]);
  assert(rc.ch[4] == in_ch[4]);
  assert(rc.ch[5] == in_ch[5]);
}

static void test_crsf_reject_bad_crc(void)
{
  uint16_t in_ch[16] = {
    1000, 1000, 992, 992, 992, 992, 992, 992,
    992, 992, 992, 992, 992, 992, 992, 992
  };
  uint8_t frame[26];
  crsf_rc_channels_t rc;
  bool ok;

  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = 24U;
  frame[2] = CRSF_FRAME_TYPE_RC_CHANNELS;
  pack_crsf_16ch(in_ch, &frame[3]);
  frame[25] = crsf_crc8_dvb_s2(&frame[2], 23U);

  frame[25] ^= 0xA5U;
  ok = crsf_try_decode_rc_channels(frame, 26U, &rc);
  assert(!ok);
}

int main(void)
{
  test_unified_decode_and_adapt();
  test_unified_reject_bad_crc_and_header();
  test_legacy_decode_and_adapt();
  test_legacy_reject_when_disabled();
  test_crsf_decode();
  test_crsf_reject_bad_crc();

  puts("protocol_smoke_test: PASS");
  return 0;
}
