/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crsf_protocol.c
  * @brief   CRSF RC parser skeleton implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "crsf_protocol.h"

#include <stddef.h>

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
  if (v < lo)
  {
    return lo;
  }
  if (v > hi)
  {
    return hi;
  }
  return v;
}

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0U;
  uint16_t i;
  uint8_t b;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (b = 0U; b < 8U; b++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ CRSF_CRC_POLY);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void crsf_unpack_16ch_11bit(const uint8_t payload[CRSF_RC_PAYLOAD_LEN],
                                   uint16_t out_ch[CRSF_CHANNEL_COUNT])
{
  uint8_t ch;
  uint16_t bit_ofs = 0U;

  for (ch = 0U; ch < CRSF_CHANNEL_COUNT; ch++)
  {
    uint16_t byte_idx = (uint16_t)(bit_ofs >> 3);
    uint8_t shift = (uint8_t)(bit_ofs & 0x07U);
    uint32_t w = (uint32_t)payload[byte_idx]
               | ((uint32_t)payload[byte_idx + 1U] << 8)
               | ((uint32_t)payload[byte_idx + 2U] << 16);
    out_ch[ch] = (uint16_t)((w >> shift) & 0x07FFU);
    bit_ofs = (uint16_t)(bit_ofs + 11U);
  }
}

bool crsf_try_decode_rc_channels(const uint8_t *frame,
                                 uint16_t frame_len,
                                 crsf_rc_channels_t *out)
{
  uint8_t len_field;
  uint16_t expected_frame_len;
  uint8_t type;
  uint8_t crc_recv;
  uint8_t crc_calc;

  if ((frame == NULL) || (out == NULL))
  {
    return false;
  }

  out->valid = 0U;

  if (frame_len < 5U)
  {
    return false;
  }

  if (frame[0] != CRSF_SYNC_BYTE)
  {
    return false;
  }

  len_field = frame[1];
  expected_frame_len = (uint16_t)(len_field + 2U);
  if (frame_len != expected_frame_len)
  {
    return false;
  }

  type = frame[2];
  if (type != CRSF_FRAME_TYPE_RC_CHANNELS)
  {
    return false;
  }

  if (len_field != (uint8_t)(1U + CRSF_RC_PAYLOAD_LEN + 1U))
  {
    return false;
  }

  crc_recv = frame[frame_len - 1U];
  crc_calc = crsf_crc8_dvb_s2(&frame[2], (uint16_t)(1U + CRSF_RC_PAYLOAD_LEN));
  if (crc_recv != crc_calc)
  {
    return false;
  }

  crsf_unpack_16ch_11bit(&frame[3], out->ch);
  out->valid = 1U;
  return true;
}

float crsf_channel_to_norm(uint16_t ch_value)
{
  float num;
  const float den = (float)(CRSF_CH_VALUE_STD_MAX - CRSF_CH_VALUE_STD_MIN) * 0.5f;
  uint16_t v = clamp_u16(ch_value, CRSF_CH_VALUE_STD_MIN, CRSF_CH_VALUE_STD_MAX);

  num = (float)v - (float)CRSF_CH_VALUE_MID;
  return num / den;
}
