#include "comm_usb_rx.h"

#include "comm_dispatcher.h"
#include "main.h"

static uint32_t s_last_usb_tick;
static uint32_t s_drop_usb;

void comm_usb_rx_mark_connected(void)
{
  s_last_usb_tick = HAL_GetTick();
}

bool comm_usb_rx_submit(const uint8_t *data, uint16_t len)
{
  bool queued;

  if ((data == NULL) || (len == 0U))
  {
    return false;
  }

  queued = comm_dispatcher_submit_rx(COMM_LINK_USB, data, len);
  s_last_usb_tick = HAL_GetTick();
  if (!queued)
  {
    ++s_drop_usb;
  }
  return queued;
}

uint32_t comm_usb_rx_get_last_tick(void)
{
  return s_last_usb_tick;
}

uint32_t comm_usb_rx_get_drop_count(void)
{
  return s_drop_usb;
}