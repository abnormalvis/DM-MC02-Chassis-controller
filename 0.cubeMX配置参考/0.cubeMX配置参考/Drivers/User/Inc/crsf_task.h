#ifndef __CRSF_TASK_H__
#define __CRSF_TASK_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t ch[16];       /* 16 raw CRSF channel values (172-1811) */
    float    throttle;     /* CH3 normalized [-1, 1], center deadband applied */
    float    steering;     /* CH1 normalized [-1, 1] */
    uint8_t  estop;        /* CH6 > mid → 1 (emergency stop) */
    uint8_t  handbrake;    /* CH5 < mid → 1 (handbrake / self-lock) */
    uint8_t  valid;        /* 1 = fresh data within timeout */
    uint32_t timestamp;    /* HAL_GetTick() of last good frame */
} crsf_input_t;

typedef struct {
    uint32_t sync_loss_events;   /* Contiguous non-sync runs */
    uint32_t sync_skip_bytes;    /* Bytes skipped while searching for sync */
    uint32_t len_invalid;        /* len_field + 2 != frame_len */
    uint32_t crc_error;          /* CRC mismatch */
    uint32_t unknown_type;       /* Frame type not RC/LinkStats/Heartbeat */
    uint32_t rc_decode_fail;     /* RC channels frame CRC OK but decode failed */
    uint32_t frame_too_short;    /* frame_len < 5 or incomplete in buffer */
} crsf_parse_detail_t;

typedef struct {
    uint8_t  connected;
    uint32_t last_frame_tick;
    uint32_t error_count;
    uint32_t parse_error_count;
    uint32_t frame_count;
    uint32_t uart_error_total;
    uint32_t uart_err_pe;
    uint32_t uart_err_fe;
    uint32_t uart_err_ne;
    uint32_t uart_err_ore;
    uint32_t last_uart_error;
    crsf_parse_detail_t parse_detail;
} crsf_status_t;

typedef struct {
    uint16_t channels[16];
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;
    float S1;
    float S2;
    uint8_t A;
    uint8_t B;
    uint8_t C;
    uint8_t D;
    uint8_t E;
    uint8_t F;

    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t  uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t  downlink_SNR;

    uint16_t heartbeat_counter;
    uint8_t valid;
    uint32_t timestamp;
} ELRS_Data;

extern ELRS_Data elrs_data;

void    CRSF_Init(void);
void    CRSF_Process(void);
void    CRSF_GetInput(crsf_input_t *out);
void    CRSF_GetStatus(crsf_status_t *out);
void    CRSF_GetElrsData(ELRS_Data *out);
uint8_t CRSF_IsConnected(void);

/* Called from HAL_UARTEx_RxEventCallback — do not call directly */
void CRSF_OnRxEvent(uint16_t size);

/* Called from HAL_UART_ErrorCallback for USART3 */
void CRSF_OnUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_TASK_H__ */
