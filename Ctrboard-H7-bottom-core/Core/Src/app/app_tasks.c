/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   Application task creation template implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_tasks.h"

#include "cmsis_os2.h"

#include <string.h>

#include "main.h"

#include "comm_dispatcher.h"
#include "can_task.h"
#include "comm_uart_rx.h"
#include "comm_usb_rx.h"
#include "key_task.h"
#include "protocol_adapter.h"

#define COMM_LINK_TIMEOUT_MS ((uint32_t)100U)

static osThreadId_t s_uart_task_handle;
static const osThreadAttr_t s_uart_task_attr = {
  .name = "UartComm",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityNormal
};

static osThreadId_t s_usb_task_handle;
static const osThreadAttr_t s_usb_task_attr = {
  .name = "UsbComm",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityBelowNormal
};

static osThreadId_t s_key_task_handle;
static const osThreadAttr_t s_key_task_attr = {
  .name = "KeyTask",
  .stack_size = 128U * 4U,
  .priority = (osPriority_t)osPriorityLow
};

static system_state_t s_system_state;
static chassis_cmd_t s_rc_cmd;
static chassis_cmd_t s_offboard_cmd;

static chassis_cmd_t s_usb_cmd;
static chassis_cmd_t s_rs485_cmd;

static uint32_t s_last_rs485_tick;

static void update_uart_link_state(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t crsf_tick = comm_uart_rx_get_last_tick(COMM_LINK_CRSF_UART);
  uint32_t rs485_2_tick = comm_uart_rx_get_last_tick(COMM_LINK_RS485_USART2);
  uint32_t rs485_3_tick = comm_uart_rx_get_last_tick(COMM_LINK_RS485_USART3);

  s_system_state.rc_connected = ((crsf_tick != 0U) && ((now - crsf_tick) <= COMM_LINK_TIMEOUT_MS)) ? 1U : 0U;

  s_system_state.rs485_connected = (((rs485_2_tick != 0U) && ((now - rs485_2_tick) <= COMM_LINK_TIMEOUT_MS)) ||
                                    ((rs485_3_tick != 0U) && ((now - rs485_3_tick) <= COMM_LINK_TIMEOUT_MS)))
                                     ? 1U
                                     : 0U;
}

static void update_usb_link_state(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t usb_tick = comm_usb_rx_get_last_tick();
  s_system_state.usb_connected = ((usb_tick != 0U) && ((now - usb_tick) <= COMM_LINK_TIMEOUT_MS)) ? 1U : 0U;
}

static void UartCommTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    update_uart_link_state();
    s_system_state.can_connected = CANTask_IsConnected();
    osDelay(PERIOD_RS485);
  }
}

static void UsbCommTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    update_usb_link_state();
    osDelay(PERIOD_USB);
  }
}

static void KeyMonitorTask(void *argument)
{
  (void)argument;

  KeyTask_Init();

  for (;;)
  {
    KeyTask_Process();
    osDelay(PERIOD_KEY);
  }
}

static osThreadId_t s_comm_dispatcher_task_handle;
static const osThreadAttr_t s_comm_dispatcher_task_attr = {
  .name = "CommDispatch",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityAboveNormal
};

void AppTasks_Create(void)
{
  memset(&s_system_state, 0, sizeof(s_system_state));
  s_system_state.mode = MODE_IDLE;

  comm_dispatcher_init_queues();
  comm_uart_rx_init();

  if (s_comm_dispatcher_task_handle == 0U)
  {
    s_comm_dispatcher_task_handle = osThreadNew(CommDispatcherTask, 0U, &s_comm_dispatcher_task_attr);
  }

  if (s_uart_task_handle == 0U)
  {
    s_uart_task_handle = osThreadNew(UartCommTask, 0U, &s_uart_task_attr);
  }

  if (s_usb_task_handle == 0U)
  {
    s_usb_task_handle = osThreadNew(UsbCommTask, 0U, &s_usb_task_attr);
  }

  if (s_key_task_handle == 0U)
  {
    s_key_task_handle = osThreadNew(KeyMonitorTask, 0U, &s_key_task_attr);
  }

  /* 初始化 CAN 任务 */
  CANTask_Init();
}

void AppTasks_GetSystemState(system_state_t *state)
{
  if (state != NULL)
  {
    *state = s_system_state;
  }
}

void AppTasks_SetControlMode(control_mode_t mode)
{
  s_system_state.mode = mode;
  s_system_state.last_update_tick = HAL_GetTick();
}

chassis_cmd_t *AppTasks_GetRCCommand(void)
{
  return &s_rc_cmd;
}

chassis_cmd_t *AppTasks_GetOffboardCommand(void)
{
  return &s_offboard_cmd;
}

void AppTasks_GetMotorState(motor_state_t *state)
{
  int16_t ia = 0;
  int16_t ib = 0;

  if (state == NULL)
  {
    return;
  }

  CANTask_GetCurrent(&ia, &ib);

  memset(state, 0, sizeof(*state));
  state->current_a = ia;
  state->current_b = ib;
  state->brake_active = (s_system_state.mode == MODE_BRAKE_PROTECT) ? 1U : 0U;
}

void AppTasks_TriggerEStop(void)
{
  s_system_state.mode = MODE_BRAKE_PROTECT;
  s_system_state.last_update_tick = HAL_GetTick();
}

void AppTasks_SetFault(uint32_t fault_bit)
{
  s_system_state.fault_bits |= fault_bit;
  s_system_state.last_update_tick = HAL_GetTick();
}

void AppTasks_ClearFault(uint32_t fault_bit)
{
  s_system_state.fault_bits &= ~fault_bit;
  s_system_state.last_update_tick = HAL_GetTick();
}

void comm_dispatcher_on_ctrl_cmd(const void *cmd)
{
  const chassis_ctrl_cmd_t *ctrl = (const chassis_ctrl_cmd_t *)cmd;
  uint32_t now = HAL_GetTick();

  if ((ctrl == NULL) || (ctrl->valid == 0U))
  {
    return;
  }

  if (ctrl->estop != 0U)
  {
    AppTasks_TriggerEStop();
    return;
  }

  if (ctrl->mode == (uint8_t)CTRL_MODE_RC)
  {
    s_rc_cmd.vx_mps = ctrl->vx_mps;
    s_rc_cmd.wz_rps = ctrl->wz_rps;
    s_rc_cmd.estop = ctrl->estop;
    s_rc_cmd.valid = ctrl->valid;
    s_rc_cmd.timestamp = now;

    s_system_state.mode = MODE_RC;
    s_system_state.last_update_tick = now;
    s_system_state.rc_connected = 1U;
    return;
  }

  if (ctrl->mode == (uint8_t)CTRL_MODE_OFFBOARD)
  {
    if (ctrl->source == (uint8_t)OFFBOARD_SRC_USB)
    {
      s_usb_cmd.vx_mps = ctrl->vx_mps;
      s_usb_cmd.wz_rps = ctrl->wz_rps;
      s_usb_cmd.estop = ctrl->estop;
      s_usb_cmd.valid = ctrl->valid;
      s_usb_cmd.timestamp = now;
      comm_usb_rx_mark_connected();
    }
    else if (ctrl->source == (uint8_t)OFFBOARD_SRC_RS485)
    {
      s_rs485_cmd.vx_mps = ctrl->vx_mps;
      s_rs485_cmd.wz_rps = ctrl->wz_rps;
      s_rs485_cmd.estop = ctrl->estop;
      s_rs485_cmd.valid = ctrl->valid;
      s_rs485_cmd.timestamp = now;
      s_last_rs485_tick = now;
    }

    if (comm_usb_rx_get_last_tick() >= s_last_rs485_tick)
    {
      s_offboard_cmd = s_usb_cmd;
    }
    else
    {
      s_offboard_cmd = s_rs485_cmd;
    }

    s_system_state.mode = MODE_OFFBOARD;
    s_system_state.last_update_tick = now;
    update_usb_link_state();
    return;
  }

  if (ctrl->mode == (uint8_t)CTRL_MODE_IDLE)
  {
    s_system_state.mode = MODE_IDLE;
    s_system_state.last_update_tick = now;
  }
}
