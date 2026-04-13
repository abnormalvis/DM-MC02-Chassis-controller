#ifndef TEST_STUB_SAFETY_TASK_H
#define TEST_STUB_SAFETY_TASK_H

void SafetyTask_Init(void);
void SafetyTask_Process(void *argument);
void SafetyTask_TriggerEStop(void);

#endif
