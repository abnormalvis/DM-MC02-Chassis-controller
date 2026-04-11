#include "key_task.h"

#include "main.h"

#define KEY_DEBOUNCE_MS    ((uint32_t)20U)
#define KEY_LONG_PRESS_MS  ((uint32_t)1000U)

typedef struct {
  uint8_t stable_level;
  uint8_t debounced_pressed;
  uint8_t long_press_latched;
  uint32_t debounce_tick;
  uint32_t press_tick;
  key_event_t pending_event;
} key_ctx_t;

static key_ctx_t s_key_ctx;

static uint8_t key_read_raw(void)
{
  GPIO_PinState pin = HAL_GPIO_ReadPin(User_Key_GPIO_Port, User_Key_Pin);
  return (pin == GPIO_PIN_SET) ? 1U : 0U;
}

void KeyTask_Init(void)
{
  s_key_ctx.stable_level = key_read_raw();
  s_key_ctx.debounced_pressed = s_key_ctx.stable_level;
  s_key_ctx.long_press_latched = 0U;
  s_key_ctx.debounce_tick = HAL_GetTick();
  s_key_ctx.press_tick = HAL_GetTick();
  s_key_ctx.pending_event = KEY_EVENT_NONE;
}

void KeyTask_Process(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t raw = key_read_raw();

  if (raw != s_key_ctx.stable_level)
  {
    if ((now - s_key_ctx.debounce_tick) >= KEY_DEBOUNCE_MS)
    {
      s_key_ctx.stable_level = raw;
      s_key_ctx.debounce_tick = now;

      if (raw != 0U)
      {
        s_key_ctx.debounced_pressed = 1U;
        s_key_ctx.press_tick = now;
        s_key_ctx.long_press_latched = 0U;
        s_key_ctx.pending_event = KEY_EVENT_PRESS;
      }
      else
      {
        s_key_ctx.debounced_pressed = 0U;
        if (s_key_ctx.long_press_latched == 0U)
        {
          s_key_ctx.pending_event = KEY_EVENT_CLICK;
        }
        else
        {
          s_key_ctx.pending_event = KEY_EVENT_RELEASE;
        }
      }
    }
  }
  else
  {
    s_key_ctx.debounce_tick = now;
  }

  if ((s_key_ctx.debounced_pressed != 0U) &&
      (s_key_ctx.long_press_latched == 0U) &&
      ((now - s_key_ctx.press_tick) >= KEY_LONG_PRESS_MS))
  {
    s_key_ctx.long_press_latched = 1U;
    s_key_ctx.pending_event = KEY_EVENT_LONG_PRESS;
  }
}

uint8_t KeyTask_IsPressed(void)
{
  return s_key_ctx.debounced_pressed;
}

key_event_t KeyTask_GetAndClearEvent(void)
{
  key_event_t event = s_key_ctx.pending_event;
  s_key_ctx.pending_event = KEY_EVENT_NONE;
  return event;
}
