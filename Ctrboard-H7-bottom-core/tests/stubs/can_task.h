#ifndef TEST_STUB_CAN_TASK_H
#define TEST_STUB_CAN_TASK_H

#include <stdint.h>

void CANTask_Init(void);
void CANTask_Process(void *argument);
void CANTask_GetCurrent(int16_t *current_a, int16_t *current_b);

#endif
