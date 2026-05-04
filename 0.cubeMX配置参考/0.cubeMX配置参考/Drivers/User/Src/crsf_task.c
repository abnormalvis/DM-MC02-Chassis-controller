#include "crsf_task.h"
#include "crsf_protocol.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define CRSF_RX_BUF_SIZE         64U
#define CRSF_TIMEOUT_MS          100U
#define CRSF_THROTTLE_DEADBAND   0.05f   /* ±5 % around centre → output 0 */
#define CRSF_FRAME_TYPE_LINK_STATS      ((uint8_t)0x14U)
#define CRSF_FRAME_TYPE_HEARTBEAT       ((uint8_t)0x0BU)

/* DMA RX buffer must be in AXI SRAM (0x24000000), not in DTCM (0x20000000).
 * Keil IRAM1 defaults to DTCM — DMA1 cannot access DTCM on STM32H7.
 * .ARM.__at_0x... section name tells the ARM linker to place this at a fixed
 * address in IRAM2 (AXI SRAM). Place at the start (0x24000000) to avoid STACK
 * collision at the end; STACK grows downward from 0x2407FFFF. */
uint8_t s_rx_buf[CRSF_RX_BUF_SIZE] __attribute__((section(".ARM.__at_0x24000000")));

static crsf_input_t  s_input  = {0};
static crsf_status_t s_status = {0};
ELRS_Data elrs_data = {0};

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  hdma_usart3_rx;

static float mapf(float input_value,
                  float input_min,
                  float input_max,
                  float output_min,
                  float output_max)
{
    if (input_value < input_min)
    {
        return output_min;
    }
    if (input_value > input_max)
    {
        return output_max;
    }
    return output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
}

static float mapf_with_median(float input_value,
                              float input_min,
                              float input_max,
                              float median,
                              float output_min,
                              float output_max)
{
    float output_median = (output_max - output_min) * 0.5f + output_min;

    if ((input_min >= input_max) || (output_min >= output_max) || (median <= input_min) || (median >= input_max))
    {
        return output_min;
    }

    if (input_value < median)
    {
        return mapf(input_value, input_min, median, output_min, output_median);
    }

    return mapf(input_value, median, input_max, output_median, output_max);
}

static void crsf_update_input_from_channels(const uint16_t ch[16])
{
    float thr_raw = crsf_channel_to_norm(ch[2]);

    if ((thr_raw > -CRSF_THROTTLE_DEADBAND) && (thr_raw < CRSF_THROTTLE_DEADBAND))
    {
        thr_raw = 0.0f;
    }

    s_input.throttle = thr_raw;
    s_input.steering = crsf_channel_to_norm(ch[0]);
    s_input.handbrake = (crsf_channel_to_norm(ch[4]) < 0.0f) ? 1U : 0U;
    s_input.estop = (ch[5] > CRSF_CH_VALUE_MID) ? 1U : 0U;
    s_input.valid = 1U;
    s_input.timestamp = HAL_GetTick();
    memcpy(s_input.ch, ch, sizeof(s_input.ch));
}

static void crsf_update_elrs_from_channels(const uint16_t ch[16])
{
    memcpy(elrs_data.channels, ch, sizeof(elrs_data.channels));

    elrs_data.Left_X = mapf_with_median((float)ch[3], 174.0f, 1808.0f, 992.0f, -100.0f, 100.0f);
    elrs_data.Left_Y = mapf_with_median((float)ch[2], 174.0f, 1811.0f, 992.0f, 0.0f, 100.0f);
    elrs_data.Right_X = mapf_with_median((float)ch[0], 174.0f, 1811.0f, 992.0f, -100.0f, 100.0f);
    elrs_data.Right_Y = mapf_with_median((float)ch[1], 174.0f, 1808.0f, 992.0f, -100.0f, 100.0f);
    elrs_data.S1 = mapf_with_median((float)ch[8], 191.0f, 1792.0f, 992.0f, 0.0f, 100.0f);
    elrs_data.S2 = mapf_with_median((float)ch[9], 191.0f, 1792.0f, 992.0f, 0.0f, 100.0f);

    elrs_data.A = (ch[10] > 1000U) ? 1U : 0U;
    elrs_data.B = (ch[5] == 992U) ? 1U : ((ch[5] == 1792U) ? 2U : 0U);
    elrs_data.C = (ch[6] == 992U) ? 1U : ((ch[6] == 1792U) ? 2U : 0U);
    elrs_data.D = (ch[11] > 1000U) ? 1U : 0U;
    elrs_data.E = (ch[4] == 992U) ? 1U : ((ch[4] == 1792U) ? 2U : 0U);
    elrs_data.F = (ch[7] == 992U) ? 1U : ((ch[7] == 1792U) ? 2U : 0U);

    elrs_data.valid = 1U;
    elrs_data.timestamp = HAL_GetTick();
}

static uint8_t crsf_try_decode_non_rc_frames(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t len_field;
    uint8_t type;
    uint8_t crc_recv;
    uint8_t crc_calc;

    if ((frame == NULL) || (frame_len < 5U))
    {
        return 0U;
    }

    if (frame[0] != CRSF_SYNC_BYTE)
    {
        return 0U;
    }

    len_field = frame[1];
    if ((uint16_t)(len_field + 2U) != frame_len)
    {
        return 0U;
    }

    crc_recv = frame[frame_len - 1U];
    crc_calc = crsf_crc8_dvb_s2(&frame[2], (uint16_t)(len_field - 1U));
    if (crc_recv != crc_calc)
    {
        return 0U;
    }

    type = frame[2];
    /* Link statistics is: sync + len + type + 10-byte payload + crc = 14 bytes total. */
    if ((type == CRSF_FRAME_TYPE_LINK_STATS) && (frame_len >= 14U))
    {
        elrs_data.uplink_RSSI_1 = frame[3];
        elrs_data.uplink_RSSI_2 = frame[4];
        elrs_data.uplink_Link_quality = frame[5];
        elrs_data.uplink_SNR = (int8_t)frame[6];
        elrs_data.active_antenna = frame[7];
        elrs_data.rf_Mode = frame[8];
        elrs_data.uplink_TX_Power = frame[9];
        elrs_data.downlink_RSSI = frame[10];
        elrs_data.downlink_Link_quality = frame[11];
        elrs_data.downlink_SNR = (int8_t)frame[12];
        return 1U;
    }
    else if ((type == CRSF_FRAME_TYPE_HEARTBEAT) && (frame_len >= 5U))
    {
        elrs_data.heartbeat_counter = frame[3];
        return 1U;
    }

    return 0U;
}

void CRSF_Init(void)
{
    memset(&s_input,  0, sizeof(s_input));
    memset(&s_status, 0, sizeof(s_status));
    memset(&elrs_data, 0, sizeof(elrs_data));
    memset(s_rx_buf,  0, sizeof(s_rx_buf));
    printf("CRSF: buf@0x%08lX (must be 0x24xxxxxx)\r\n", (unsigned long)s_rx_buf);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_buf, CRSF_RX_BUF_SIZE) != HAL_OK)
    {
        s_status.error_count++;
    }
    /* Suppress DMA half-transfer interrupt — we only need IDLE and TC events */
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void CRSF_OnRxEvent(uint16_t size)
{
    crsf_rc_channels_t rc = {0};
    uint8_t frame_buf[CRSF_RX_BUF_SIZE];
    uint16_t offset = 0U;
    uint8_t decoded_any = 0U;
    uint8_t seeking_sync = 0U;

    if (size > CRSF_RX_BUF_SIZE)
    {
        size = CRSF_RX_BUF_SIZE;
    }

    /* Invalidate D-Cache before CPU reads DMA destination buffer. */
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_buf,
                                 (int32_t)((CRSF_RX_BUF_SIZE + 31U) & ~31U));
    memcpy(frame_buf, s_rx_buf, size);

    /* Restart reception immediately to minimize the RX gap and reduce ORE risk. */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_buf, CRSF_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    while ((offset + 2U) <= size)
    {
        uint16_t frame_len;
        uint8_t len_field;
        uint8_t type;
        uint8_t crc_recv;
        uint8_t crc_calc;

        if (frame_buf[offset] != CRSF_SYNC_BYTE)
        {
            s_status.parse_detail.sync_skip_bytes++;
            if (seeking_sync == 0U)
            {
                s_status.parse_detail.sync_loss_events++;
                seeking_sync = 1U;
            }
            offset++;
            continue;
        }

        seeking_sync = 0U;

        len_field = frame_buf[offset + 1U];
        frame_len = (uint16_t)len_field + 2U;

            if (frame_len < 5U)
        {
                s_status.parse_detail.frame_too_short++;
                offset++;
                continue;
        }

            if ((offset + frame_len) > size)
            {
                break;  /* Incomplete frame, wait for next DMA callback. */
            }

        /* Check CRC and frame type for detailed error classification. */
        crc_recv = frame_buf[offset + frame_len - 1U];
        crc_calc = crsf_crc8_dvb_s2(&frame_buf[offset + 2U], (uint16_t)(len_field - 1U));

        if (crc_recv != crc_calc)
        {
            s_status.parse_detail.crc_error++;
            s_status.parse_error_count++;
            s_status.error_count++;
            offset = (uint16_t)(offset + frame_len);
            continue;
        }

        type = frame_buf[offset + 2U];

        if (crsf_try_decode_rc_channels(&frame_buf[offset], frame_len, &rc))
        {
            crsf_update_input_from_channels(rc.ch);
            crsf_update_elrs_from_channels(rc.ch);

            s_status.connected = 1U;
            s_status.last_frame_tick = HAL_GetTick();
            s_status.frame_count++;
            decoded_any = 1U;
        }
        else if (type == CRSF_FRAME_TYPE_RC_CHANNELS)
        {
            /* RC frame with good CRC but decode failed. */
            s_status.parse_detail.rc_decode_fail++;
            s_status.parse_error_count++;
            s_status.error_count++;
        }
        else if (crsf_try_decode_non_rc_frames(&frame_buf[offset], frame_len) != 0U)
        {
            s_status.connected = 1U;
            s_status.last_frame_tick = HAL_GetTick();
            s_status.frame_count++;
            decoded_any = 1U;
        }
        else
        {
            /* Link-stats, Heartbeat, or other known type but decode failed (rare). */
            if ((type == CRSF_FRAME_TYPE_LINK_STATS) || (type == CRSF_FRAME_TYPE_HEARTBEAT))
            {
                s_status.parse_detail.len_invalid++;
            }
            else
            {
                s_status.parse_detail.unknown_type++;
            }
            s_status.parse_error_count++;
            s_status.error_count++;
        }

        offset = (uint16_t)(offset + frame_len);
    }

}

void CRSF_Process(void)
{
    if ((HAL_GetTick() - s_status.last_frame_tick) > CRSF_TIMEOUT_MS)
    {
        s_status.connected = 0U;
        s_input.valid      = 0U;
        s_input.throttle   = 0.0f;
        s_input.steering   = 0.0f;
        s_input.handbrake  = 0U;
    }
}

void CRSF_GetInput(crsf_input_t *out)
{
    if (out != NULL)
    {
        *out = s_input;
    }
}

void CRSF_GetStatus(crsf_status_t *out)
{
    if (out != NULL)
    {
        *out = s_status;
    }
}

void CRSF_GetElrsData(ELRS_Data *out)
{
    if (out != NULL)
    {
        *out = elrs_data;
    }
}

uint8_t CRSF_IsConnected(void)
{
    return s_status.connected;
}

void CRSF_OnUartError(UART_HandleTypeDef *huart)
{
    uint32_t err;

    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    err = huart->ErrorCode;
    s_status.uart_error_total++;
    s_status.error_count++;
    s_status.last_uart_error = err;

    if ((err & HAL_UART_ERROR_PE) != 0U)
    {
        s_status.uart_err_pe++;
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    if ((err & HAL_UART_ERROR_FE) != 0U)
    {
        s_status.uart_err_fe++;
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if ((err & HAL_UART_ERROR_NE) != 0U)
    {
        s_status.uart_err_ne++;
        __HAL_UART_CLEAR_NEFLAG(huart);
    }
    if ((err & HAL_UART_ERROR_ORE) != 0U)
    {
        s_status.uart_err_ore++;
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    /* Re-arm DMA reception after UART RX errors so CRSF link can self-recover. */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_buf, CRSF_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

/* Override HAL weak callback — called on IDLE line or DMA buffer full */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART3)
    {
        CRSF_OnRxEvent(size);
    }
}
