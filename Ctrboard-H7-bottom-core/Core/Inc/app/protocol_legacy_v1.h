/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol_legacy_v1.h
  * @brief   Legacy 12-byte protocol compatibility layer (FC ... DF).
  ******************************************************************************
  * @attention
  *
  * IMPORTANT:
  * - This protocol originated from encoder-motor baseboard communication.
  * - Current project uses DC motor current-loop control.
  * - Therefore legacy_v1 is COMPATIBILITY ONLY, not the primary control protocol.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PROTOCOL_LEGACY_V1_H__
#define __PROTOCOL_LEGACY_V1_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Fixed 12-byte frame */
#define LEGACY_V1_FRAME_LEN               ((uint8_t)12U)
#define LEGACY_V1_SOF                     ((uint8_t)0xFC)
#define LEGACY_V1_EOF                     ((uint8_t)0xDF)

/* Function codes in PDF */
typedef enum
{
  LEGACY_FUNC_BATTERY = 0x01,
  LEGACY_FUNC_ENCODER = 0x02,
  LEGACY_FUNC_GYRO = 0x03,
  LEGACY_FUNC_ACCEL = 0x04,
  LEGACY_FUNC_EULER = 0x05,
  LEGACY_FUNC_WHEEL_TARGET = 0x06,
  LEGACY_FUNC_PID = 0x07,
  LEGACY_FUNC_SERVO = 0x08
} legacy_func_t;

typedef struct
{
  uint8_t sof;
  uint8_t func;
  uint8_t data[8];
  uint8_t xor_checksum;
  uint8_t eof;
} legacy_v1_frame_t;

/* Validation and checksum */
uint8_t legacy_v1_xor10(const uint8_t *frame_10_bytes);
bool legacy_v1_is_valid(const uint8_t frame[LEGACY_V1_FRAME_LEN]);

/* Encode/decode */
void legacy_v1_make(uint8_t func, const uint8_t data8[8], uint8_t out_frame[LEGACY_V1_FRAME_LEN]);
bool legacy_v1_parse(const uint8_t raw[LEGACY_V1_FRAME_LEN], legacy_v1_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_LEGACY_V1_H__ */
