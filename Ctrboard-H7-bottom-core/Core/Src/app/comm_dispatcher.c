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

#define CRSF_CH_THROTTLE        0U
#define CRSF_CH_STEERING        1U
#define CRSF_CH_SPEED_SCALE     2U
#define CRSF_CH_TURN_SCALE      3U
#define CRSF_CH_MODE            4U
#define CRSF_CH_ESTOP           5U
#define CRSF_CH_BRAKE           6U
#define CRSF_CH_THROTTLE_LOCK   7U

#define CRSF_SWITCH_LOW_TH      700U
#define CRSF_SWITCH_HIGH_TH     1300U

static uint8_t crsf_decode_sw_cmd(uint16_t ch)
{
  if (ch <= CRSF_SWITCH_LOW_TH)
  {
    return (uint8_t)CTRL_SW_CMD_DISABLE;
  }
  if (ch >= CRSF_SWITCH_HIGH_TH)
  {
    return (uint8_t)CTRL_SW_CMD_ENABLE;
  }
  return (uint8_t)CTRL_SW_CMD_HOLD;
}

static float crsf_decode_scale_3pos(uint16_t ch)
{
  if (ch <= CRSF_SWITCH_LOW_TH)
  {
    return 0.40f;
  }
  if (ch >= CRSF_SWITCH_HIGH_TH)
  {
    return 1.00f;
  }
  return 0.70f;
}

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
  float speed_scale;
  float turn_scale;

  if (!crsf_try_decode_rc_channels(item->data, item->len, &rc))
  {
    return;
  }

  throttle = crsf_channel_to_norm(rc.ch[CRSF_CH_THROTTLE]);
  steering = crsf_channel_to_norm(rc.ch[CRSF_CH_STEERING]);
  speed_scale = crsf_decode_scale_3pos(rc.ch[CRSF_CH_SPEED_SCALE]);
  turn_scale = crsf_decode_scale_3pos(rc.ch[CRSF_CH_TURN_SCALE]);

  memset(&cmd, 0, sizeof(cmd));
  cmd.vx_mps = throttle * speed_scale;
  cmd.wz_rps = steering * turn_scale;
  cmd.mode = (uint8_t)CTRL_MODE_RC;
  cmd.estop = 0U;
  cmd.estop_cmd = crsf_decode_sw_cmd(rc.ch[CRSF_CH_ESTOP]);
  cmd.brake_cmd = crsf_decode_sw_cmd(rc.ch[CRSF_CH_BRAKE]);
  cmd.throttle_lock_cmd = crsf_decode_sw_cmd(rc.ch[CRSF_CH_THROTTLE_LOCK]);
  cmd.source = (uint8_t)OFFBOARD_SRC_NONE;
  cmd.valid = 1U;
  cmd.ts_ms = osKernelGetTickCount();

  if (rc.ch[CRSF_CH_MODE] <= CRSF_SWITCH_LOW_TH)
  {
    cmd.mode = (uint8_t)CTRL_MODE_IDLE;
  }
  else if (rc.ch[CRSF_CH_MODE] >= CRSF_SWITCH_HIGH_TH)
  {
    /* CH5 high requests OFFBOARD mode. */
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
