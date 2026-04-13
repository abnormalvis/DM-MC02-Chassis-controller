/**
 * @file rs485_task.c
 * @brief RS485 通信任务实现
 */

#include "rs485_task.h"

#include "comm_uart_rx.h"
#include "main.h"
#include "usart.h"

#define RS485_TIMEOUT_MS 100U

static uint8_t s_rs485_connected_2 = 0U;
static uint8_t s_rs485_connected_3 = 0U;

void RS485Task_Init(void)
{
    s_rs485_connected_2 = 0U;
    s_rs485_connected_3 = 0U;
    comm_uart_rx_init();
}

uint32_t RS485Task_Send(uint8_t port, const uint8_t *data, uint32_t len)
{
    UART_HandleTypeDef *huart = NULL;

    if ((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (port == 2U)
    {
        huart = &huart2;
    }
    else if (port == 3U)
    {
        huart = &huart3;
    }
    else
    {
        return 0U;
    }

    if (HAL_UART_Transmit(huart, (uint8_t *)data, (uint16_t)len, 10U) == HAL_OK)
    {
        return len;
    }

    return 0U;
}

bool RS485Task_IsConnected(uint8_t port)
{
    uint32_t now = HAL_GetTick();
    uint32_t last_tick;

    if (port == 2U)
    {
        last_tick = comm_uart_rx_get_last_tick(COMM_LINK_RS485_USART2);
        if ((last_tick != 0U) && ((now - last_tick) <= RS485_TIMEOUT_MS))
        {
            s_rs485_connected_2 = 1U;
        }
        else if ((now - last_tick) > RS485_TIMEOUT_MS)
        {
            s_rs485_connected_2 = 0U;
        }
        return (s_rs485_connected_2 != 0U);
    }

    if (port == 3U)
    {
        last_tick = comm_uart_rx_get_last_tick(COMM_LINK_RS485_USART3);
        if ((last_tick != 0U) && ((now - last_tick) <= RS485_TIMEOUT_MS))
        {
            s_rs485_connected_3 = 1U;
        }
        else if ((now - last_tick) > RS485_TIMEOUT_MS)
        {
            s_rs485_connected_3 = 0U;
        }
        return (s_rs485_connected_3 != 0U);
    }

    return false;
}

void RS485Task_Process(void *argument)
{
    (void)argument;
    (void)RS485Task_IsConnected(2U);
    (void)RS485Task_IsConnected(3U);
}
