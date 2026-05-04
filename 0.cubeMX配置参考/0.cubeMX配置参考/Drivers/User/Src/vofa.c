#include "vofa.h"

#include "usart.h"

static const uint8_t s_vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

typedef enum
{
	VOFA_RX_IDLE = 0,
	VOFA_RX_HASH,
	VOFA_RX_P,
	VOFA_RX_ID,
	VOFA_RX_VALUE
} vofa_rx_state_t;

static vofa_rx_state_t s_rx_state = VOFA_RX_IDLE;
static uint16_t s_current_id = 0U;
static uint8_t s_value_len = 0U;
static uint8_t s_rx_byte = 0U;
static char s_value_buf[32];
static VOFA_ParamCallback s_param_callback = 0;

static volatile uint8_t s_rx_frame_ready    = 0U;
static uint16_t         s_pending_id        = 0U;
static uint8_t          s_pending_value_len = 0U;
static char             s_pending_value_buf[32];
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_uart_last_error  = 0U;
static volatile uint32_t s_uart_last_isr    = 0U;

static void vofa_reset_parser(void)
{
	s_rx_state = VOFA_RX_IDLE;
	s_current_id = 0U;
	s_value_len = 0U;
	s_value_buf[0] = '\0';
}

static float vofa_parse_float(const char *text, uint8_t len, uint8_t *ok)
{
	uint8_t index = 0U;
	int sign = 1;
	float int_part = 0.0f;
	float frac_part = 0.0f;
	float scale = 1.0f;
	uint8_t has_digit = 0U;

	if (ok != NULL)
	{
		*ok = 0U;
	}

	if ((text == NULL) || (len == 0U))
	{
		return 0.0f;
	}

	if (text[index] == '-')
	{
		sign = -1;
		index++;
	}

	while ((index < len) && (text[index] >= '0') && (text[index] <= '9'))
	{
		has_digit = 1U;
		int_part = (int_part * 10.0f) + (float)(text[index] - '0');
		index++;
	}

	if ((index < len) && (text[index] == '.'))
	{
		index++;
		while ((index < len) && (text[index] >= '0') && (text[index] <= '9'))
		{
			has_digit = 1U;
			scale *= 10.0f;
			frac_part += (float)(text[index] - '0') / scale;
			index++;
		}
	}

	if ((has_digit == 0U) || (index != len))
	{
		return 0.0f;
	}

	if (ok != NULL)
	{
		*ok = 1U;
	}

	return (float)sign * (int_part + frac_part);
}

static void vofa_handle_frame(void)
{
	uint8_t i;
	s_pending_id        = s_current_id;
	s_pending_value_len = s_value_len;
	for (i = 0U; i <= s_value_len; i++)
	{
		s_pending_value_buf[i] = s_value_buf[i];
	}
	s_rx_frame_ready = 1U;
	vofa_reset_parser();
}

static void vofa_process_byte(uint8_t byte)
{
	switch (s_rx_state)
	{
		case VOFA_RX_IDLE:
			if (byte == '#')
			{
				s_rx_state = VOFA_RX_HASH;
			}
			break;

		case VOFA_RX_HASH:
			if (byte == 'P')
			{
				s_rx_state = VOFA_RX_P;
				s_current_id = 0U;
			}
			else
			{
				vofa_reset_parser();
			}
			break;

		case VOFA_RX_P:
		case VOFA_RX_ID:
			if ((byte >= '0') && (byte <= '9'))
			{
				uint32_t next = ((uint32_t)s_current_id * 10U) + (uint32_t)(byte - '0');
				s_current_id = (next > 0xFFFFU) ? 0xFFFFU : (uint16_t)next;
				s_rx_state = VOFA_RX_ID;
			}
			else if (byte == '=')
			{
				s_rx_state = VOFA_RX_VALUE;
				s_value_len = 0U;
				s_value_buf[0] = '\0';
			}
			else
			{
				vofa_reset_parser();
			}
			break;

		case VOFA_RX_VALUE:
			if (byte == '!')
			{
				vofa_handle_frame();
			}
			else if (s_value_len < (uint8_t)(sizeof(s_value_buf) - 1U))
			{
				s_value_buf[s_value_len++] = (char)byte;
				s_value_buf[s_value_len] = '\0';
			}
			else
			{
				vofa_reset_parser();
			}
			break;

		default:
			vofa_reset_parser();
			break;
	}
}

void VOFA_Init(void)
{
	vofa_reset_parser();
}

void VOFA_ProcessRx(void)
{
	if (s_rx_frame_ready == 0U)
	{
		return;
	}
	s_rx_frame_ready = 0U;

	while ((s_pending_value_len > 0U) &&
	       ((s_pending_value_buf[s_pending_value_len - 1U] == '\r') ||
	        (s_pending_value_buf[s_pending_value_len - 1U] == '\n') ||
	        (s_pending_value_buf[s_pending_value_len - 1U] == ' ')))
	{
		s_pending_value_len--;
		s_pending_value_buf[s_pending_value_len] = '\0';
	}

	uint8_t ok = 0U;
	float value = vofa_parse_float(s_pending_value_buf, s_pending_value_len, &ok);

	if ((ok != 0U) && (s_param_callback != 0))
	{
		s_param_callback(s_pending_id, value);
	}
}

void VOFA_SetParamCallback(VOFA_ParamCallback callback)
{
	s_param_callback = callback;
}

HAL_StatusTypeDef VOFA_Begin(void)
{
	vofa_reset_parser();
	return HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

void VOFA_OnUartRxCplt(UART_HandleTypeDef *huart)
{
	if (huart != &huart2)
	{
		return;
	}

	vofa_process_byte(s_rx_byte);
	(void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

void VOFA_OnUartError(UART_HandleTypeDef *huart)
{
	if (huart != &huart2)
	{
		return;
	}

	s_uart_error_count++;
	s_uart_last_error = huart->ErrorCode;
	s_uart_last_isr = huart2.Instance->ISR;

	vofa_reset_parser();
	(void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

uint32_t VOFA_GetUartErrorCount(void)
{
	return s_uart_error_count;
}

uint32_t VOFA_GetUartLastError(void)
{
	return s_uart_last_error;
}

uint32_t VOFA_GetUartLastIsr(void)
{
	return s_uart_last_isr;
}

static HAL_StatusTypeDef vofa_uart_tx(const uint8_t *buf, uint16_t len)
{
	for (uint16_t i = 0U; i < len; i++)
	{
		uint32_t start = HAL_GetTick();
		while ((huart2.Instance->ISR & USART_ISR_TXE_TXFNF) == 0U)
		{
			if ((HAL_GetTick() - start) > 10U)
			{
				return HAL_TIMEOUT;
			}
		}
		huart2.Instance->TDR = (uint32_t)buf[i];
	}
	return HAL_OK;
}

HAL_StatusTypeDef VOFA_SendFloats(const float *data, uint8_t count)
{
	HAL_StatusTypeDef status;

	if ((data == NULL) || (count == 0U))
	{
		return HAL_ERROR;
	}

	status = vofa_uart_tx((const uint8_t *)data, (uint16_t)(count * (uint8_t)sizeof(float)));
	if (status != HAL_OK)
	{
		return status;
	}

	return vofa_uart_tx(s_vofa_tail, (uint16_t)sizeof(s_vofa_tail));
}

HAL_StatusTypeDef VOFA_SendOneFloat(float value)
{
	return VOFA_SendFloats(&value, 1U);
}
