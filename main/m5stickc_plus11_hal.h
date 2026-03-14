/*
 * M5StickC Plus 1.1 Hardware Abstraction Layer Header
 * Same API as m5stickc_plus2_hal.h; different LCD pins (DC=23, RST=18).
 */

#ifndef M5STICKC_PLUS11_HAL_H
#define M5STICKC_PLUS11_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

/* M5StickC Plus 1.1 GPIO (ESP32-PICO-D4) – LCD differs from Plus2 */
#define M5_LCD_MOSI_PIN     15
#define M5_LCD_SCLK_PIN     13
#define M5_LCD_CS_PIN       5
#define M5_LCD_DC_PIN       23  /* Plus 1.1: DC=23 (Plus2 uses 14) */
#define M5_LCD_RST_PIN      18  /* Plus 1.1: RST=18 (Plus2 uses 12) */
#define M5_LCD_BL_PIN       27

#define M5_LCD_H_RES        240
#define M5_LCD_V_RES        135

#define M5_BTN_A_PIN        37
#define M5_BTN_B_PIN        39
#define M5_BTN_PWR_PIN      35

#define M5_I2C_SDA_PIN      21
#define M5_I2C_SCL_PIN      22
#define M5_PWR_EN_PIN       4
#define M5_LED_PIN          10
#define M5_BUZZER_PIN       2
#define M5_IR_PIN           9
#define MPU6886_I2C_ADDR    0x68
#define BM8563_I2C_ADDR     0x51

/* Same API as Plus2 HAL so app_main/ui can use one include and same calls */
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

/**
 * @brief Initialize MPU6886 6-axis IMU
 *
 * Wakes the chip from sleep, configures accelerometer to ±8g range,
 * and verifies WHO_AM_I register (expected 0x19).
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_imu_init(void);

/**
 * @brief Read accelerometer data from MPU6886
 *
 * Returns calibrated acceleration in g units for each axis.
 * Call after m5stickc_plus2_imu_init().
 *
 * @param ax Pointer to X-axis output (g)
 * @param ay Pointer to Y-axis output (g)
 * @param az Pointer to Z-axis output (g)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
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

#endif /* M5STICKC_PLUS11_HAL_H */
