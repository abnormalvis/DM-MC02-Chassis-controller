/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_legacy_v1.c
  * @brief   Legacy 12-byte protocol skeleton implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protocol_legacy_v1.h"
#include <stddef.h>

uint8_t legacy_v1_xor10(const uint8_t *frame_10_bytes)
{
  uint8_t i;
  uint8_t x = 0U;

  if (frame_10_bytes == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < 10U; i++)
  {
    x ^= frame_10_bytes[i];
  }
  return x;
}

bool legacy_v1_is_valid(const uint8_t frame[LEGACY_V1_FRAME_LEN])
{
  if (frame == NULL)
  {
    return false;
  }

  if ((frame[0] != LEGACY_V1_SOF) || (frame[11] != LEGACY_V1_EOF))
  {
    return false;
  }

  return (legacy_v1_xor10(frame) == frame[10]);
}

void legacy_v1_make(uint8_t func, const uint8_t data8[8], uint8_t out_frame[LEGACY_V1_FRAME_LEN])
{
  uint8_t i;

  if (out_frame == NULL)
  {
    return;
  }

  out_frame[0] = LEGACY_V1_SOF;
  out_frame[1] = func;

  for (i = 0U; i < 8U; i++)
  {
    out_frame[2U + i] = (data8 != NULL) ? data8[i] : 0U;
  }

  out_frame[10] = legacy_v1_xor10(out_frame);
  out_frame[11] = LEGACY_V1_EOF;
}

bool legacy_v1_parse(const uint8_t raw[LEGACY_V1_FRAME_LEN], legacy_v1_frame_t *out)
{
  uint8_t i;

  if ((raw == NULL) || (out == NULL))
  {
    return false;
  }

  if (!legacy_v1_is_valid(raw))
  {
    return false;
  }

  out->sof = raw[0];
  out->func = raw[1];
  for (i = 0U; i < 8U; i++)
  {
    out->data[i] = raw[2U + i];
  }
  out->xor_checksum = raw[10];
  out->eof = raw[11];

  return true;
}
