/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_unified_v0.c
  * @brief   Unified protocol v0 skeleton implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protocol_unified_v0.h"

static uint16_t rd_le16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_le32(const uint8_t *p)
{
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static void wr_le16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void wr_le32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
  p[2] = (uint8_t)((v >> 16) & 0xFFU);
  p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

uint16_t proto_v0_crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t j;

  if (data == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (j = 0U; j < 8U; j++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

bool proto_v0_try_decode(const uint8_t *buf,
                         uint16_t buf_len,
                         proto_v0_frame_view_t *out_view,
                         uint16_t *out_total_len)
{
  uint16_t payload_len;
  uint16_t total_len;
  uint16_t crc_calc;
  uint16_t crc_recv;

  if ((buf == 0U) || (out_view == 0U) || (out_total_len == 0U))
  {
    return false;
  }

  if (buf_len < 15U)
  {
    return false;
  }

  if (rd_le16(&buf[0]) != PROTO_V0_SOF)
  {
    return false;
  }

  if (buf[2] != PROTO_V0_VER)
  {
    return false;
  }

  payload_len = rd_le16(&buf[11]);
  if (payload_len > PROTO_V0_MAX_PAYLOAD)
  {
    return false;
  }

  total_len = (uint16_t)(13U + payload_len + 2U);
  if (buf_len < total_len)
  {
    return false;
  }

  crc_recv = rd_le16(&buf[13U + payload_len]);
  crc_calc = proto_v0_crc16_ccitt_false(buf, (uint16_t)(13U + payload_len));
  if (crc_calc != crc_recv)
  {
    return false;
  }

  out_view->hdr.sof = PROTO_V0_SOF;
  out_view->hdr.ver = buf[2];
  out_view->hdr.msg_id = rd_le16(&buf[3]);
  out_view->hdr.seq = rd_le16(&buf[5]);
  out_view->hdr.ts_ms = rd_le32(&buf[7]);
  out_view->hdr.len = payload_len;
  out_view->payload = (payload_len > 0U) ? &buf[13] : 0U;
  out_view->crc16 = crc_recv;
  *out_total_len = total_len;

  return true;
}

bool proto_v0_encode(uint16_t msg_id,
                     uint16_t seq,
                     uint32_t ts_ms,
                     const uint8_t *payload,
                     uint16_t payload_len,
                     uint8_t *out_buf,
                     uint16_t out_cap,
                     uint16_t *out_len)
{
  uint16_t total_len;
  uint16_t i;
  uint16_t crc;

  if ((out_buf == 0U) || (out_len == 0U))
  {
    return false;
  }

  if (payload_len > PROTO_V0_MAX_PAYLOAD)
  {
    return false;
  }

  if ((payload_len > 0U) && (payload == 0U))
  {
    return false;
  }

  total_len = (uint16_t)(13U + payload_len + 2U);
  if (out_cap < total_len)
  {
    return false;
  }

  wr_le16(&out_buf[0], PROTO_V0_SOF);
  out_buf[2] = PROTO_V0_VER;
  wr_le16(&out_buf[3], msg_id);
  wr_le16(&out_buf[5], seq);
  wr_le32(&out_buf[7], ts_ms);
  wr_le16(&out_buf[11], payload_len);

  for (i = 0U; i < payload_len; i++)
  {
    out_buf[13U + i] = payload[i];
  }

  crc = proto_v0_crc16_ccitt_false(out_buf, (uint16_t)(13U + payload_len));
  wr_le16(&out_buf[13U + payload_len], crc);

  *out_len = total_len;
  return true;
}
