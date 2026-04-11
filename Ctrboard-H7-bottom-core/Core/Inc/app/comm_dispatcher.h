/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_dispatcher.h
  * @brief   Unified communication dispatcher interfaces (USB/RS485).
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __COMM_DISPATCHER_H__
#define __COMM_DISPATCHER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

#define COMM_DISPATCHER_MAX_FRAME_LEN      ((uint16_t)160U)

typedef enum
{
  COMM_LINK_USB = 0,
  COMM_LINK_RS485_USART2 = 1,
  COMM_LINK_RS485_USART3 = 2,
  COMM_LINK_CRSF_UART = 3
} comm_link_t;

typedef struct
{
  uint8_t link;
  uint16_t len;
  uint8_t data[COMM_DISPATCHER_MAX_FRAME_LEN];
} comm_rx_item_t;

void comm_dispatcher_init_queues(void);
bool comm_dispatcher_submit_rx(comm_link_t link, const uint8_t *data, uint16_t len);

void CommDispatcherTask(void *argument);

/* weak hook: control layer can override this to consume decoded command */
void comm_dispatcher_on_ctrl_cmd(const void *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_DISPATCHER_H__ */
