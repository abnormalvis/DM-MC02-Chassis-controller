#ifndef TEST_STUB_TIM_H
#define TEST_STUB_TIM_H

#include <stdint.h>

typedef struct
{
    uint32_t autoreload;
    uint32_t ccr1;
    uint32_t ccr3;
} TIM_HandleTypeDef;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_3 3U

#define __HAL_TIM_GET_AUTORELOAD(htim) ((htim)->autoreload)

#define __HAL_TIM_SET_COMPARE(htim, channel, value) \
    do { \
        if ((channel) == TIM_CHANNEL_1) { \
            (htim)->ccr1 = (value); \
        } else if ((channel) == TIM_CHANNEL_3) { \
            (htim)->ccr3 = (value); \
        } \
    } while (0)

#endif
