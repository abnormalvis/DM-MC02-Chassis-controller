#ifndef KEY_TASK_H
#define KEY_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  KEY_EVENT_NONE = 0,
  KEY_EVENT_PRESS,
  KEY_EVENT_RELEASE,
  KEY_EVENT_CLICK,
  KEY_EVENT_LONG_PRESS
} key_event_t;

void KeyTask_Init(void);
void KeyTask_Process(void *argument);

uint8_t KeyTask_IsPressed(void);
key_event_t KeyTask_GetAndClearEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_TASK_H */
