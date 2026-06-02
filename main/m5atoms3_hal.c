/*
 * M5 AtomS3 HAL — ESP32-S3, 128x128 IPS LCD, MPU6886, single screen button.
 *
 * Display path: M5GFX (LovyanGFX-based) via the C shim in m5atoms3_gfx.cpp /
 * .h. This file no longer owns the SPI bus or the panel — M5GFX init() does
 * panel auto-detection on AtomS3 and configures the bus, color order, and
 * gamma correctly. See decisions D-014..D-016 (replace hand-rolled draw
 * path with M5GFX) in docs/ATOMS3_DECISIONS.md.
 *
 * What this file still owns:
 *   - Top-level init order
 *   - Power / backlight bring-up
 *   - I2C bus for MPU6886
 *   - Single screen button (G41)
 *   - MPU6886 IMU init + accel read
 *   - Buzzer no-op stubs (AtomS3 has no piezo)
 *   - Boot splash (now drawn via the M5GFX shim)
 *
 * The previous implementation also held the font + bitmap drawing helpers;
 * those are removed because every drawing call site for AtomS3 now flows
 * through atoms3_gfx_*.
 */

#ifdef M5ATOMS3

#include "m5atoms3_hal.h"
#include "m5atoms3_gfx.h"

#include "logo_bitmap.h"
#include "driver/gpio.h"
/* Use the NEW ESP-IDF I2C driver (driver/i2c_master.h, "driver_ng"). The old
 * driver (driver/i2c.h) cannot coexist with new-driver consumers in the same
 * firmware — the ESP-IDF aborts at boot with "driver_ng is not allowed to be
 * used with this old driver". M5GFX/LovyanGFX on AtomS3 pulls in the new
 * driver, so the HAL has to follow. */
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "M5ATOMS3_HAL";

/* ──────────────────────────────────────────────────────────────────────────
 * Forward decls
 * ──────────────────────────────────────────────────────────────────────── */

static void display_boot_self_test(void);

/* ──────────────────────────────────────────────────────────────────────────
 * I2C — driver_ng (driver/i2c_master.h). MPU6886 lives on SDA=38 / SCL=39.
 * Refs R1, R10. We hold a bus handle and a per-device handle for the IMU.
 * ──────────────────────────────────────────────────────────────────────── */

static i2c_master_bus_config_t i2c_bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = M5_I2C_SDA_PIN,
    .scl_io_num = M5_I2C_SCL_PIN,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = { .enable_internal_pullup = true },
};

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mpu_dev = NULL;

/* ──────────────────────────────────────────────────────────────────────────
 * Top-level init — order:
 *   1. (no separate power init — M5GFX will manage backlight via its own
 *       init path; we keep the shared API entry point as a no-op so the
 *       cross-target init flow in app_main.c still calls it.)
 *   2. I2C
 *   3. Display (M5GFX)
 *   4. Buttons
 *   5. IMU (non-fatal)
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_init(void) {
    ESP_LOGI(TAG, "Initializing M5 AtomS3 hardware (M5GFX path)");

    int ret = m5stickc_plus2_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "power_init failed");
        return ret;
    }

    ret = m5stickc_plus2_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_init failed");
        return ret;
    }

    ret = m5stickc_plus2_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "display_init failed");
        return ret;
    }

    ret = m5stickc_plus2_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "buttons_init failed");
        return ret;
    }

    /* IMU is non-fatal: continue if absent. */
    ret = m5stickc_plus2_imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (%s) — motion detection disabled",
                 esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "M5 AtomS3 hardware initialized successfully");
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Power / I2C
 *
 * The backlight pin (M5_LCD_BL_PIN) is driven by M5GFX once display_init
 * runs. We don't pre-configure it as a plain GPIO output anymore; doing so
 * conflicts with M5GFX's internal LEDC setup. power_init is now a logging
 * stub kept around so the cross-target init flow doesn't change shape.
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_power_init(void) {
    ESP_LOGI(TAG, "power_init: AtomS3 has no PMIC; backlight is M5GFX-managed");
    return ESP_OK;
}

void m5stickc_plus2_power_off(void) {
    /* AtomS3 has no PMIC; the only "off" is unplugging USB-C. Park the CPU
     * so the cross-target shutdown call site doesn't fall through. */
    ESP_LOGI(TAG, "Power off requested (AtomS3 has no PMIC; entering idle loop)");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int m5stickc_plus2_i2c_init(void) {
    /* driver_ng: create the bus handle. The MPU6886 device handle is added
     * later in m5stickc_plus2_imu_init() once we've confirmed the device
     * actually responds. */
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C bus initialized on SDA=%d, SCL=%d (driver_ng)",
             M5_I2C_SDA_PIN, M5_I2C_SCL_PIN);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Display — delegated to the M5GFX shim. The shim instantiates the panel,
 * runs auto-detection, and exposes a small C-callable surface
 * (atoms3_gfx_*). All draw helpers below forward to that.
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_display_init(void) {
    int rc = atoms3_gfx_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "M5GFX init failed (rc=%d)", rc);
        return ESP_FAIL;
    }
    display_boot_self_test();
    return ESP_OK;
}

/* No esp_lcd panel handle in the M5GFX path — return NULL. The cross-target
 * API keeps this function so feature code that compiles against the header
 * still links; on AtomS3 nothing actually consumes the handle. */
esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void) {
    return NULL;
}

void m5stickc_plus2_display_set_brightness(uint8_t brightness) {
    atoms3_gfx_set_brightness(brightness);
}

void m5stickc_plus2_display_clear(uint16_t color) {
    atoms3_gfx_clear(color);
}

void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color) {
    atoms3_gfx_fill_rect(x, y, width, height, color);
}

void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color) {
    atoms3_gfx_fill_circle(x, y, radius, color);
}

/* The legacy 1-bit bitmap path (used by the boot logo) maps onto M5GFX's
 * drawXBitmap. The bg_color parameter is preserved for API compatibility but
 * unused — drawXBitmap leaves zero-bits transparent. Call sites that wanted
 * an opaque background draw a fill_rect first. */
void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height,
                                        const uint8_t *bitmap, uint16_t color, uint16_t bg_color) {
    (void)bg_color;
    atoms3_gfx_draw_xbitmap(x, y, width, height, bitmap, color);
}

/* The HAL's "scaled bitmap font" text APIs are kept for cross-target source
 * compatibility, but on AtomS3 they map onto M5GFX's tiered text path:
 *   scale 1 → tier 1 (LABEL)
 *   scale 2 → tier 2 (VALUE)
 *   scale 3+→ tier 3 (HERO)
 * Feature code that wants pixel-exact placement should call the
 * atoms3_gfx_print* API directly. */
void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color) {
    atoms3_gfx_print(x, y, text, color, 1);
}

void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale) {
    int tier = (scale >= 3) ? 3 : (scale >= 2 ? 2 : 1);
    atoms3_gfx_print(x, y, text, color, tier);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Buttons — single screen button on G41, active-low, internal pull-up.
 * No second button or dedicated power button on AtomS3.
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_buttons_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << M5_BTN_A_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Buttons initialized — single screen button on G%d (no B / no PWR)", M5_BTN_A_PIN);
    return ESP_OK;
}

bool m5stickc_plus2_button_a_pressed(void) {
    return gpio_get_level(M5_BTN_A_PIN) == 0;
}

bool m5stickc_plus2_button_b_pressed(void) {
    return false;
}

bool m5stickc_plus2_button_pwr_pressed(void) {
    return false;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Boot splash — M5GFX-rendered. We draw the existing 1-bit logo via the
 * shim (drawXBitmap clips for us), then a small label below.
 * ──────────────────────────────────────────────────────────────────────── */

static void display_boot_self_test(void) {
    /* Shared boot screen with the Ditch LEDs controller: bäärgsiitsch logo
     * (zoomed 128x65, MSB-first → drawBitmap) centered up top, product name
     * below. Same layout as the trigger4p boot for a matching pair. */
    int draw_x = (atoms3_gfx_width() - LOGO_W) / 2;
    if (draw_x < 0) draw_x = 0;
    int draw_y = 6;
    atoms3_gfx_clear(M5_COLOR_BLACK);
    atoms3_gfx_draw_bitmap(draw_x, draw_y, LOGO_W, LOGO_H,
                           logo_bitmap, M5_COLOR_WHITE);
    atoms3_gfx_print_centered(draw_y + LOGO_H + 10, "Dash Cam", M5_COLOR_GREEN, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    atoms3_gfx_clear(M5_COLOR_BLACK);
}

/* ──────────────────────────────────────────────────────────────────────────
 * MPU6886 IMU (I2C 0x68 on SDA=38 / SCL=39)
 *
 * Datasheet refs:
 *   WHO_AM_I (0x75) -> 0x19
 *   PWR_MGMT_1 (0x6B): bit7=DEVICE_RESET; clear all to wake
 *   ACCEL_CONFIG (0x1C) bits[4:3]: 00=±2g, 01=±4g, 10=±8g, 11=±16g
 *   ACCEL_XOUT_H starts at 0x3B (6 bytes big-endian)
 *
 * ±8g (4096 LSB/g) — same scale as the StickS3 BMI270 path so motion_logic
 * thresholds are comparable across boards.
 * ──────────────────────────────────────────────────────────────────────── */

#define MPU6886_REG_WHO_AM_I       0x75
#define MPU6886_REG_PWR_MGMT_1     0x6B
#define MPU6886_REG_ACCEL_CONFIG   0x1C
#define MPU6886_REG_ACCEL_XOUT_H   0x3B
#define MPU6886_WHO_AM_I_VAL       0x19
#define MPU6886_ACCEL_SCALE        4096.0f /* ±8g */

/* MPU6886 driver_ng helpers — use the device handle returned by
 * i2c_master_bus_add_device. All transactions block up to 80 ms. */
static esp_err_t mpu_write8(uint8_t reg, uint8_t val) {
    if (s_mpu_dev == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_mpu_dev, buf, 2, 80);
}

static esp_err_t mpu_read8(uint8_t reg, uint8_t *out) {
    if (s_mpu_dev == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, out, 1, 80);
}

static esp_err_t mpu_read_block(uint8_t reg, uint8_t *out, size_t len) {
    if (s_mpu_dev == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, out, len, 80);
}

int m5stickc_plus2_imu_init(void) {
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "IMU init called before I2C bus init");
        return ESP_ERR_INVALID_STATE;
    }
    /* Probe before registering as a device so a missing IMU doesn't leave a
     * dangling device handle on the bus. The new driver supports a probe
     * helper that does a 0-byte transaction and reports ACK/NACK. */
    esp_err_t probe = i2c_master_probe(s_i2c_bus, MPU6886_I2C_ADDR, 50);
    if (probe != ESP_OK) {
        ESP_LOGE(TAG, "MPU6886 probe at 0x%02X failed: %s",
                 MPU6886_I2C_ADDR, esp_err_to_name(probe));
        return probe;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6886_I2C_ADDR,
        .scl_speed_hz = 400000, /* dedicated back I2C — 400 kHz is fine */
    };
    esp_err_t add = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu_dev);
    if (add != ESP_OK) {
        ESP_LOGE(TAG, "MPU6886 add_device failed: %s", esp_err_to_name(add));
        return add;
    }

    uint8_t who = 0;
    esp_err_t ret = mpu_read8(MPU6886_REG_WHO_AM_I, &who);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6886 WHO_AM_I read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (who != MPU6886_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU6886 WHO_AM_I=0x%02X (expect 0x%02X)", who, MPU6886_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    /* Reset, wait, then wake (clear sleep + clock auto-select). */
    (void)mpu_write8(MPU6886_REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    (void)mpu_write8(MPU6886_REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* ±8g: AFS_SEL=2 → bits[4:3]=10 → 0x10 */
    (void)mpu_write8(MPU6886_REG_ACCEL_CONFIG, 0x10);
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG, "MPU6886 initialized (WHO_AM_I=0x%02X)", who);
    return ESP_OK;
}

int m5stickc_plus2_imu_read_accel(float *ax, float *ay, float *az) {
    if (ax == NULL || ay == NULL || az == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t raw[6];
    esp_err_t ret = mpu_read_block(MPU6886_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (ret != ESP_OK) return ret;
    int16_t rx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ry = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t rz = (int16_t)((raw[4] << 8) | raw[5]);
    *ax = (float)rx / MPU6886_ACCEL_SCALE;
    *ay = (float)ry / MPU6886_ACCEL_SCALE;
    *az = (float)rz / MPU6886_ACCEL_SCALE;
    return ESP_OK;
}

/* AtomS3 has no onboard buzzer — no-op stubs match Plus2/StickS3 API. */
int m5stickc_plus2_buzzer_init(void) { return ESP_OK; }
void m5stickc_plus2_buzzer_beep(uint16_t freq_hz, uint16_t duration_ms) {
    (void)freq_hz; (void)duration_ms;
}
void m5stickc_plus2_buzzer_beep_double(uint16_t freq_hz, uint16_t duration_ms, uint16_t gap_ms) {
    (void)freq_hz; (void)duration_ms; (void)gap_ms;
}

#endif /* M5ATOMS3 */
