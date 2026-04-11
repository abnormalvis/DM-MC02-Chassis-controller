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

#include "comm_dispatcher.h"
#include "can_task.h"

static osThreadId_t s_comm_dispatcher_task_handle;
static const osThreadAttr_t s_comm_dispatcher_task_attr = {
  .name = "CommDispatch",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityAboveNormal
};

void AppTasks_Create(void)
{
  comm_dispatcher_init_queues();

  if (s_comm_dispatcher_task_handle == 0U)
  {
    s_comm_dispatcher_task_handle = osThreadNew(CommDispatcherTask, 0U, &s_comm_dispatcher_task_attr);
  }

  /* 初始化 CAN 任务 */
  CANTask_Init();
}
