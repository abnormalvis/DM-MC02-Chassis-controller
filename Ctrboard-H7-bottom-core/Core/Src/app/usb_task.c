/**
 * @file usb_task.c
 * @brief USB 通信任务实现
 */

#include "usb_task.h"

#include "comm_usb_rx.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define USB_TIMEOUT_MS 100U

static uint8_t s_usb_connected = 0U;

void USBTask_Init(void)
{
    s_usb_connected = 0U;
}

uint32_t USBTask_Send(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) || (len == 0U) || (len > 0xFFFFU))
    {
        return 0U;
    }

    if (CDC_Transmit_HS((uint8_t *)data, (uint16_t)len) == USBD_OK)
    {
        s_usb_connected = 1U;
        return len;
    }

    return 0U;
}

bool USBTask_IsConnected(void)
{
    uint32_t last_tick = comm_usb_rx_get_last_tick();
    if ((last_tick != 0U) && ((HAL_GetTick() - last_tick) <= USB_TIMEOUT_MS))
    {
        s_usb_connected = 1U;
    }
    else if ((HAL_GetTick() - last_tick) > USB_TIMEOUT_MS)
    {
        s_usb_connected = 0U;
    }

    return (s_usb_connected != 0U);
}

void USBTask_Process(void *argument)
{
    (void)argument;
    (void)USBTask_IsConnected();
}
