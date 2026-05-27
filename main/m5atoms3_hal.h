/*
 * M5 AtomS3 (ESP32-S3) — same API surface as m5stickc_plus2_hal.h so app/ui/motion code
 * can call the shared `m5stickc_plus2_*` symbols without #ifdefs at every call site.
 *
 * Pin map source: M5Stack AtomS3 product docs (ref R1) and esp-bsp m5_atom_s3 (ref R5).
 * Display: 0.85" IPS, 128x128. Datasheet says GC9107; we drive it via the ST7789
 * panel driver for first bring-up (decision D-002), with `gc9a01` as the documented
 * fallback (refs R5, R6).
 *
 * AtomS3 has no PMIC, no battery, no buzzer, no second user button. The screen IS
 * the button (G41, active-low, internal pull-up).
 */

#ifndef M5ATOMS3_HAL_H
#define M5ATOMS3_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

/* ── LCD pins (ref R1: M5 AtomS3 docs "LCD" table; the row label "MPU6886" in
 * those docs is a documentation typo — values map to LCD MOSI/SCK/CS/RS/RST/BL) */
#define M5_LCD_MOSI_PIN     21
#define M5_LCD_SCLK_PIN     17
#define M5_LCD_CS_PIN       15
#define M5_LCD_DC_PIN       33
#define M5_LCD_RST_PIN      34
#define M5_LCD_BL_PIN       16

/* ── Display resolution (128x128, RGB565, BGR element order — ref R4 M5GFX) */
#define M5_LCD_H_RES        128
#define M5_LCD_V_RES        128

/* ── Single user button: the screen IS a button at G41 (ref R1) */
#define M5_BTN_A_PIN        41
/* AtomS3 has no second button or dedicated power button; sentinels keep the
 * API parity but the implementations always return false. */
#define M5_BTN_B_PIN        255
#define M5_BTN_PWR_PIN      255

/* ── I2C for MPU6886 (ref R1, R10) */
#define M5_I2C_SDA_PIN      38
#define M5_I2C_SCL_PIN      39

#define MPU6886_I2C_ADDR    0x68

/* ── HAL function prototypes — same names as the StickS3/Plus2 HAL so shared
 * call sites in main/ui.c and main/app_main.c compile unchanged. */
int  m5stickc_plus2_init(void);
int  m5stickc_plus2_power_init(void);
void m5stickc_plus2_power_off(void);
int  m5stickc_plus2_i2c_init(void);
int  m5stickc_plus2_display_init(void);
int  m5stickc_plus2_buttons_init(void);
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

/* AtomS3 has no onboard buzzer — these are no-op stubs for API parity. */
int  m5stickc_plus2_buzzer_init(void);
void m5stickc_plus2_buzzer_beep(uint16_t freq_hz, uint16_t duration_ms);
void m5stickc_plus2_buzzer_beep_double(uint16_t freq_hz, uint16_t duration_ms, uint16_t gap_ms);

int  m5stickc_plus2_imu_init(void);
int  m5stickc_plus2_imu_read_accel(float *ax, float *ay, float *az);

/* ── Color constants — standard RGB565 (M5GFX path).
 *
 * The previous esp_lcd path on AtomS3 was configured with BGR element order,
 * which forced an awkward swap workaround inside the constants here (RED was
 * actually 0x001F because the panel reordered to BGR on the way out, etc.).
 * Now that M5GFX owns the panel init, it emits standard RGB565 over the wire
 * and the constants below are the plain, textbook values. M5_TRUE_* are kept
 * as aliases of M5_COLOR_* so existing call sites keep compiling.
 */
#define M5_COLOR_BLACK      0x0000
#define M5_COLOR_WHITE      0xFFFF
#define M5_COLOR_RED        0xF800   /* R=31                       */
#define M5_COLOR_GREEN      0x07E0   /* G=63                       */
#define M5_COLOR_BLUE       0x001F   /* B=31                       */
#define M5_COLOR_YELLOW     0xFFE0   /* R=31, G=63                 */
#define M5_COLOR_CYAN       0x07FF   /* G=63, B=31                 */
#define M5_COLOR_MAGENTA    0xF81F   /* R=31, B=31                 */
#define M5_COLOR_ORANGE     0xFD20   /* R=31, G=41                 */
#define M5_COLOR_PURPLE     0x8010   /* R=16, B=16                 */
#define M5_COLOR_DARKGREY   0x39C6   /* ≈ #404040                  */
#define M5_COLOR_GREY       0x7BEF   /* ≈ #808080                  */

#define M5_TRUE_RED         M5_COLOR_RED
#define M5_TRUE_GREEN       M5_COLOR_GREEN
#define M5_TRUE_BLUE        M5_COLOR_BLUE
#define M5_TRUE_YELLOW      M5_COLOR_YELLOW
#define M5_TRUE_ORANGE      M5_COLOR_ORANGE

#endif /* M5ATOMS3_HAL_H */
