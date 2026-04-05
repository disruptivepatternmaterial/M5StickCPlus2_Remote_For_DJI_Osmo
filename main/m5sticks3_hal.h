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

#endif /* M5STICKS3_HAL_H */
