/*
 * @file vofa_debug.h
 * @brief Vofa upper-computer telemetry output on USART10.
 */

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void VofaDebug_Init(void);
void VofaDebug_Process(void);
uint8_t VofaDebug_SendFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_DEBUG_H */