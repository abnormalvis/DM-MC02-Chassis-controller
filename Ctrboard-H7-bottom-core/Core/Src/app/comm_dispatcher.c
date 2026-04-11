/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_dispatcher.c
  * @brief   Unified dispatcher skeleton for USB/RS485 inbound frames.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "comm_dispatcher.h"

#include <string.h>

#include "protocol_adapter.h"
#include "crsf_protocol.h"
#include "protocol_legacy_v1.h"
#include "protocol_unified_v0.h"

static osMessageQueueId_t s_comm_rx_queue;

__attribute__((weak)) void comm_dispatcher_on_ctrl_cmd(const void *cmd)
{
  (void)cmd;
}

static void dispatch_unified(const comm_rx_item_t *item)
{
  proto_v0_frame_view_t view;
  uint16_t total_len = 0U;
  chassis_ctrl_cmd_t cmd;

  if (proto_v0_try_decode(item->data, item->len, &view, &total_len))
  {
    if (proto_adapter_on_unified_frame(&view, &cmd) && (cmd.valid != 0U))
    {
      if ((item->link == (uint8_t)COMM_LINK_RS485_USART2) ||
          (item->link == (uint8_t)COMM_LINK_RS485_USART3))
      {
        cmd.source = (uint8_t)OFFBOARD_SRC_RS485;
      }
      else if (item->link == (uint8_t)COMM_LINK_USB)
      {
        cmd.source = (uint8_t)OFFBOARD_SRC_USB;
      }
      comm_dispatcher_on_ctrl_cmd(&cmd);
    }
  }
}

static void dispatch_crsf(const comm_rx_item_t *item)
{
  crsf_rc_channels_t rc;
  chassis_ctrl_cmd_t cmd;
  float throttle;
  float steering;

  if (!crsf_try_decode_rc_channels(item->data, item->len, &rc))
  {
    return;
  }

  throttle = crsf_channel_to_norm(rc.ch[0]);
  steering = crsf_channel_to_norm(rc.ch[1]);

  memset(&cmd, 0, sizeof(cmd));
  cmd.vx_mps = throttle;
  cmd.wz_rps = steering;
  cmd.mode = (uint8_t)CTRL_MODE_RC;
  cmd.estop = (rc.ch[5] > CRSF_CH_VALUE_MID) ? 1U : 0U;
  cmd.source = (uint8_t)OFFBOARD_SRC_NONE;
  cmd.valid = 1U;

  if (rc.ch[4] > CRSF_CH_VALUE_MID)
  {
    /* CH5 high can be interpreted as offboard request in upper state machine. */
    cmd.mode = (uint8_t)CTRL_MODE_OFFBOARD;
  }

  comm_dispatcher_on_ctrl_cmd(&cmd);
}

static void dispatch_legacy(const comm_rx_item_t *item)
{
  legacy_v1_frame_t legacy;
  chassis_ctrl_cmd_t cmd;

  if (item->len != LEGACY_V1_FRAME_LEN)
  {
    return;
  }

  if (legacy_v1_parse(item->data, &legacy))
  {
    if (proto_adapter_on_legacy_frame(&legacy, &cmd) && (cmd.valid != 0U))
    {
      cmd.source = (item->link == COMM_LINK_USB) ? (uint8_t)OFFBOARD_SRC_USB : (uint8_t)OFFBOARD_SRC_RS485;
      comm_dispatcher_on_ctrl_cmd(&cmd);
    }
  }
}

void comm_dispatcher_init_queues(void)
{
  if (s_comm_rx_queue == 0U)
  {
    s_comm_rx_queue = osMessageQueueNew(16U, sizeof(comm_rx_item_t), 0U);
  }
}

bool comm_dispatcher_submit_rx(comm_link_t link, const uint8_t *data, uint16_t len)
{
  comm_rx_item_t item;

  if ((data == 0U) || (len == 0U) || (len > COMM_DISPATCHER_MAX_FRAME_LEN))
  {
    return false;
  }

  if (s_comm_rx_queue == 0U)
  {
    return false;
  }

  item.link = (uint8_t)link;
  item.len = len;
  memcpy(item.data, data, len);

  return (osMessageQueuePut(s_comm_rx_queue, &item, 0U, 0U) == osOK);
}

void CommDispatcherTask(void *argument)
{
  comm_rx_item_t item;

  (void)argument;

  for (;;)
  {
    if (osMessageQueueGet(s_comm_rx_queue, &item, 0U, osWaitForever) == osOK)
    {
      if (item.link == (uint8_t)COMM_LINK_CRSF_UART)
      {
        dispatch_crsf(&item);
      }
      else
      {
        /* unified protocol is primary; legacy is compatibility fallback */
        dispatch_unified(&item);
        dispatch_legacy(&item);
      }
    }
  }
}
