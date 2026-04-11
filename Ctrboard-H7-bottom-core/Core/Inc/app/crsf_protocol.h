/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crsf_protocol.h
  * @brief   CRSF RC packet parser interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __CRSF_PROTOCOL_H__
#define __CRSF_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define CRSF_CRC_POLY                     ((uint8_t)0xD5U)

#define CRSF_SYNC_BYTE                    ((uint8_t)0xC8U)
#define CRSF_FRAME_TYPE_RC_CHANNELS       ((uint8_t)0x16U)

#define CRSF_CHANNEL_COUNT                ((uint8_t)16U)
#define CRSF_RC_PAYLOAD_LEN               ((uint8_t)22U)

#define CRSF_CH_VALUE_STD_MIN             ((uint16_t)172U)
#define CRSF_CH_VALUE_MID                 ((uint16_t)992U)
#define CRSF_CH_VALUE_STD_MAX             ((uint16_t)1811U)

typedef struct
{
  uint16_t ch[CRSF_CHANNEL_COUNT];
  uint8_t valid;
} crsf_rc_channels_t;

/* CRC8/DVB-S2 with polynomial 0xD5 */
uint8_t crsf_crc8_dvb_s2(const uint8_t *data, uint16_t len);

/*
 * Try parsing a full CRSF frame.
 * Frame format: [sync][len][type][payload...][crc]
 * len counts bytes after len field, i.e. type + payload + crc.
 */
bool crsf_try_decode_rc_channels(const uint8_t *frame,
                                 uint16_t frame_len,
                                 crsf_rc_channels_t *out);

/* Convert one CRSF channel value to normalized [-1, 1]. */
float crsf_channel_to_norm(uint16_t ch_value);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_PROTOCOL_H__ */
