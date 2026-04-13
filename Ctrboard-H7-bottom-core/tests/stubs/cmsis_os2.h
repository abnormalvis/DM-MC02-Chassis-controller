#ifndef TEST_STUB_CMSIS_OS2_H
#define TEST_STUB_CMSIS_OS2_H

#include <stdint.h>

#ifndef __NO_RETURN
#define __NO_RETURN
#endif

typedef void *osThreadId_t;
typedef void (*osThreadFunc_t)(void *argument);
typedef int32_t osPriority_t;

enum
{
    osPriorityHigh = 40,
    osPriorityAboveNormal = 32
};

typedef struct
{
    const char *name;
    uint32_t stack_size;
    osPriority_t priority;
} osThreadAttr_t;

osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr);
void osDelay(uint32_t ms);

#endif
