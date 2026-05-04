/*
 * @file oled_debug.c
 * @brief Minimal OLED renderer for current-loop bring-up.
 */

#include "oled_debug.h"

#include "i2c.h"
#include "main.h"
#include "motor_control_task.h"

#define OLED_I2C_ADDR            (0x3CU << 1)
#define OLED_WIDTH               128U
#define OLED_HEIGHT              64U
#define OLED_PAGE_COUNT          (OLED_HEIGHT / 8U)
#define OLED_PROCESS_PERIOD_MS   100U

#define BAR_CENTER_X             63U
#define BAR_MAX_PIXELS           60U

static uint8_t s_framebuffer[OLED_PAGE_COUNT][OLED_WIDTH];
static uint32_t s_last_tick = 0U;
static uint8_t s_heartbeat = 0U;
static uint8_t s_oled_ready = 0U;

static void oled_write_cmd(uint8_t cmd)
{
    uint8_t packet[2] = {0x00U, cmd};
    (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, packet, sizeof(packet), 10U);
}

static void oled_write_data(const uint8_t *data, uint16_t len)
{
    uint8_t packet[17];
    uint16_t offset = 0U;

    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    packet[0] = 0x40U;
    while (offset < len)
    {
        uint16_t chunk = len - offset;
        if (chunk > 16U)
        {
            chunk = 16U;
        }

        for (uint16_t i = 0U; i < chunk; ++i)
        {
            packet[1U + i] = data[offset + i];
        }

        (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, packet, (uint16_t)(1U + chunk), 20U);
        offset = (uint16_t)(offset + chunk);
    }
}

static void oled_set_pixel(uint8_t x, uint8_t y)
{
    uint8_t page;
    uint8_t bit;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    page = (uint8_t)(y >> 3);
    bit = (uint8_t)(y & 0x07U);
    s_framebuffer[page][x] |= (uint8_t)(1U << bit);
}

static void oled_clear(void)
{
    for (uint8_t p = 0U; p < OLED_PAGE_COUNT; ++p)
    {
        for (uint8_t x = 0U; x < OLED_WIDTH; ++x)
        {
            s_framebuffer[p][x] = 0U;
        }
    }
}

static void oled_draw_hline(uint8_t x0, uint8_t x1, uint8_t y)
{
    uint8_t left = x0;
    uint8_t right = x1;

    if (y >= OLED_HEIGHT)
    {
        return;
    }

    if (left > right)
    {
        uint8_t tmp = left;
        left = right;
        right = tmp;
    }

    if (right >= OLED_WIDTH)
    {
        right = (uint8_t)(OLED_WIDTH - 1U);
    }

    for (uint8_t x = left; x <= right; ++x)
    {
        oled_set_pixel(x, y);
    }
}

static void oled_draw_vline(uint8_t x, uint8_t y0, uint8_t y1)
{
    uint8_t top = y0;
    uint8_t bottom = y1;

    if (x >= OLED_WIDTH)
    {
        return;
    }

    if (top > bottom)
    {
        uint8_t tmp = top;
        top = bottom;
        bottom = tmp;
    }

    if (bottom >= OLED_HEIGHT)
    {
        bottom = (uint8_t)(OLED_HEIGHT - 1U);
    }

    for (uint8_t y = top; y <= bottom; ++y)
    {
        oled_set_pixel(x, y);
    }
}

static uint8_t map_abs_to_pixels(int16_t value, int16_t full_scale)
{
    int32_t abs_v;

    if (full_scale <= 0)
    {
        return 0U;
    }

    abs_v = (value >= 0) ? (int32_t)value : -(int32_t)value;
    if (abs_v > full_scale)
    {
        abs_v = full_scale;
    }

    return (uint8_t)((abs_v * BAR_MAX_PIXELS) / full_scale);
}

static void draw_signed_bar(uint8_t row, int16_t value, int16_t full_scale)
{
    uint8_t y_base;
    uint8_t y_top;
    uint8_t y_bottom;
    uint8_t len;

    if (row >= 4U)
    {
        return;
    }

    y_base = (uint8_t)(row * 16U + 8U);
    y_top = (uint8_t)(y_base - 3U);
    y_bottom = (uint8_t)(y_base + 3U);

    oled_draw_hline(2U, 125U, y_base);
    len = map_abs_to_pixels(value, full_scale);

    if (value >= 0)
    {
        for (uint8_t x = 0U; x < len; ++x)
        {
            oled_draw_vline((uint8_t)(BAR_CENTER_X + 1U + x), y_top, y_bottom);
        }
    }
    else
    {
        for (uint8_t x = 0U; x < len; ++x)
        {
            oled_draw_vline((uint8_t)(BAR_CENTER_X - 1U - x), y_top, y_bottom);
        }
    }
}

static void oled_flush(void)
{
    for (uint8_t page = 0U; page < OLED_PAGE_COUNT; ++page)
    {
        oled_write_cmd((uint8_t)(0xB0U + page));
        oled_write_cmd(0x00U);
        oled_write_cmd(0x10U);
        oled_write_data(&s_framebuffer[page][0], OLED_WIDTH);
    }
}

static void oled_init_panel(void)
{
    static const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };

    for (uint32_t i = 0U; i < (sizeof(init_cmds) / sizeof(init_cmds[0])); ++i)
    {
        oled_write_cmd(init_cmds[i]);
    }
}

void OledDebug_Init(void)
{
    s_last_tick = HAL_GetTick();
    s_heartbeat = 0U;
    s_oled_ready = 1U;

    oled_init_panel();
    oled_clear();
    oled_flush();
}

void OledDebug_Process(void)
{
    motor_control_state_t st;
    uint32_t now;

    if (s_oled_ready == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_last_tick) < OLED_PROCESS_PERIOD_MS)
    {
        return;
    }
    s_last_tick = now;

    MotorControlTask_GetState(&st);

    oled_clear();
    oled_draw_vline(BAR_CENTER_X, 0U, (uint8_t)(OLED_HEIGHT - 1U));

    draw_signed_bar(0U, st.current_a, 400);
    draw_signed_bar(1U, st.current_b, 400);
    draw_signed_bar(2U, st.target_rpm_a, 300);
    draw_signed_bar(3U, st.actual_rpm_a, 1000);

    if (st.brake_active)
    {
        oled_draw_hline(112U, 126U, 2U);
        oled_draw_hline(112U, 126U, 3U);
    }

    s_heartbeat ^= 1U;
    if (s_heartbeat != 0U)
    {
        oled_set_pixel(0U, 0U);
    }

    oled_flush();
}
