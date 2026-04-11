#include "comm_uart_rx.h"

#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "usart.h"

static uint8_t s_uart7_rx_buf[COMM_DISPATCHER_MAX_FRAME_LEN];
static uint8_t s_usart2_rx_buf[COMM_DISPATCHER_MAX_FRAME_LEN];
static uint8_t s_usart3_rx_buf[COMM_DISPATCHER_MAX_FRAME_LEN];

static uint32_t s_last_tick_uart7;
static uint32_t s_last_tick_usart2;
static uint32_t s_last_tick_usart3;

static uint32_t s_drop_uart7;
static uint32_t s_drop_usart2;
static uint32_t s_drop_usart3;

static void restart_rx(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart7, s_uart7_rx_buf, sizeof(s_uart7_rx_buf));
  }
  else if (huart == &huart2)
  {
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_usart2_rx_buf, sizeof(s_usart2_rx_buf));
  }
  else if (huart == &huart3)
  {
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_usart3_rx_buf, sizeof(s_usart3_rx_buf));
  }
}

void comm_uart_rx_init(void)
{
  restart_rx(&huart7);
  restart_rx(&huart2);
  restart_rx(&huart3);
}

uint32_t comm_uart_rx_get_last_tick(comm_link_t link)
{
  if (link == COMM_LINK_CRSF_UART)
  {
    return s_last_tick_uart7;
  }
  if (link == COMM_LINK_RS485_USART2)
  {
    return s_last_tick_usart2;
  }
  if (link == COMM_LINK_RS485_USART3)
  {
    return s_last_tick_usart3;
  }
  return 0U;
}

uint32_t comm_uart_rx_get_drop_count(comm_link_t link)
{
  if (link == COMM_LINK_CRSF_UART)
  {
    return s_drop_uart7;
  }
  if (link == COMM_LINK_RS485_USART2)
  {
    return s_drop_usart2;
  }
  if (link == COMM_LINK_RS485_USART3)
  {
    return s_drop_usart3;
  }
  return 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint32_t tick = HAL_GetTick();
  bool queued = false;

  if ((huart == NULL) || (Size == 0U) || (Size > COMM_DISPATCHER_MAX_FRAME_LEN))
  {
    restart_rx(huart);
    return;
  }

  if (huart == &huart7)
  {
    queued = comm_dispatcher_submit_rx(COMM_LINK_CRSF_UART, s_uart7_rx_buf, Size);
    s_last_tick_uart7 = tick;
    if (!queued)
    {
      ++s_drop_uart7;
    }
  }
  else if (huart == &huart2)
  {
    queued = comm_dispatcher_submit_rx(COMM_LINK_RS485_USART2, s_usart2_rx_buf, Size);
    s_last_tick_usart2 = tick;
    if (!queued)
    {
      ++s_drop_usart2;
    }
  }
  else if (huart == &huart3)
  {
    queued = comm_dispatcher_submit_rx(COMM_LINK_RS485_USART3, s_usart3_rx_buf, Size);
    s_last_tick_usart3 = tick;
    if (!queued)
    {
      ++s_drop_usart3;
    }
  }

  restart_rx(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  restart_rx(huart);
}