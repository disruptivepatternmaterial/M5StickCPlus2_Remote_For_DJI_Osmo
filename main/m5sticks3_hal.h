/*
 * M5 StickS3 (ESP32-S3) — same API as m5stickc_plus2_hal.h for app/ui/motion.
 * Pin map: https://docs.m5stack.com/en/core/StickS3
 */

#ifndef M5STICKS3_HAL_H
#define M5STICKS3_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

/* ST7789P3 — logical 240x135 after swap_xy + mirror (panel is 135x240) */
#define M5_LCD_MOSI_PIN     39
#define M5_LCD_SCLK_PIN     40
#define M5_LCD_DC_PIN       45
#define M5_LCD_CS_PIN       41
#define M5_LCD_RST_PIN      21
#define M5_LCD_BL_PIN       38

#define M5_LCD_H_RES        240
#define M5_LCD_V_RES        135

#define M5_BTN_A_PIN        11
#define M5_BTN_B_PIN        12
/* No dedicated PWR GPIO — side button is device reset/power per M5; software power-off via M5PM1 */
#define M5_BTN_PWR_PIN      255

#define M5_I2C_SDA_PIN      47
#define M5_I2C_SCL_PIN      48

#define M5PM1_I2C_ADDR      0x6E
#define BMI270_I2C_ADDR     0x69

int m5stickc_plus2_init(void);
int m5stickc_plus2_power_init(void);
void m5stickc_plus2_power_off(void);
int m5stickc_plus2_i2c_init(void);
int m5stickc_plus2_display_init(void);
int m5stickc_plus2_buttons_init(void);
bool m5stickc_plus2_button_a_pressed(void);
bool m5stickc_plus2_button_b_pressed(void);
bool m5stickc_plus2_button_pwr_pressed(void);
void m5stickc_plus2_display_set_brightness(uint8_t brightness);
esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void);
void m5stickc_plus2_display_clear(uint16_t color);
void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color);
void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale);
void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height, const uint8_t *bitmap, uint16_t color, uint16_t bg_color);
void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color);
void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color);

/* StickS3 has no onboard buzzer — these are no-op stubs for API parity. */
int m5stickc_plus2_buzzer_init(void);
void m5stickc_plus2_buzzer_beep(uint16_t freq_hz, uint16_t duration_ms);
void m5stickc_plus2_buzzer_beep_double(uint16_t freq_hz, uint16_t duration_ms, uint16_t gap_ms);

int m5stickc_plus2_imu_init(void);
int m5stickc_plus2_imu_read_accel(float *ax, float *ay, float *az);

#define M5_COLOR_BLACK      0x0000
#define M5_COLOR_WHITE      0xFFFF
#define M5_COLOR_RED        0xF800
#define M5_COLOR_GREEN      0x001F
#define M5_COLOR_BLUE       0x07E0
#define M5_COLOR_YELLOW     0xF81F
#define M5_COLOR_CYAN       0x07FF
#define M5_COLOR_MAGENTA    0xFFE0
#define M5_COLOR_ORANGE     0xF81F
#define M5_COLOR_PURPLE     0x8010
#define M5_COLOR_DARKGREY   0x39C6
#define M5_COLOR_GREY       0x7BEF

/* ── Intent-named colors that actually render correctly on this panel ───────────
 *
 * The StickS3 LCD is configured with LCD_RGB_ELEMENT_ORDER_BGR, so any RGB565
 * value we send has its R and B 5-bit fields swapped on screen. Many of the
 * legacy M5_COLOR_* constants above were named for plain RGB565 (e.g.
 * M5_COLOR_RED = 0xF800), which displays as BLUE here, and M5_COLOR_YELLOW
 * = 0xF81F displays as MAGENTA. Earlier code worked around this by picking
 * counter-named constants (e.g. using M5_COLOR_CYAN to get yellow GPS text).
 *
 * Use the M5_TRUE_* names below in new UI code so the symbol matches what the
 * user actually sees, and the workaround stays out of feature code.
 */
#define M5_TRUE_RED         0x001F   /* B=31 source → displayed as R=31 (red) */
#define M5_TRUE_GREEN       0x07E0   /* G=63, R=B=0 — symmetric, true green */
#define M5_TRUE_BLUE        0xF800   /* R=31 source → displayed as B=31 (blue) */
#define M5_TRUE_YELLOW      0x07FF   /* R=0,G=63,B=31 source → R=31,G=63,B=0 displayed */
#define M5_TRUE_ORANGE      0x067F   /* approx R=200,G=140,B=0 displayed */

#endif /* M5STICKS3_HAL_H */
