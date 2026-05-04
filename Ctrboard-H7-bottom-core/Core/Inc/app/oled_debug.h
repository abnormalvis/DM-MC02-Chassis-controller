/*
 * @file oled_debug.h
 * @brief Minimal SSD1306 OLED debug view over I2C1.
 */

#ifndef OLED_DEBUG_H
#define OLED_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

void OledDebug_Init(void);
void OledDebug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_DEBUG_H */
