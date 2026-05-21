/*
 * M5 StickS3 HAL — ESP32-S3, ST7789, BMI270, M5PM1.
 * Same API as Plus HAL; compiled only when M5STICKS3 is defined.
 *
 * BMI270 config blob from M5Unified (MIT): bmi270_config_data.inc
 */
#ifdef M5STICKS3

#include "m5sticks3_hal.h"

#include "logo_bitmap.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_heap_caps.h"

/* LCD support for M5StickC Plus2 */
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"

/* 8x8 Pixel Font Definition
 * 
 * Compact monospace font for text rendering on the display.
 * Each character is 8x8 pixels, stored as 8 bytes (1 byte per row).
 * Bit pattern: 1 = foreground pixel, 0 = transparent/background
 * 
 * Character set: ASCII 32-122 (space through lowercase z)
 * Optimized for readability at small sizes on TFT displays
 */
static const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // " (empty for now)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // # (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // $ (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // % (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // & (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' (empty)
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // (
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // )
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // * (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // + (empty)
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // / (empty)
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, // 0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 1
    {0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0x00}, // 2
    {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, // 3
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, // 4
    {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, // 5
    {0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00}, // 6
    {0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00}, // 7
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, // 8
    {0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00}, // 9
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // : - fine dots
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ; (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // < (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // = (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // > (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ? (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // @ (empty)
    {0x18,0x24,0x42,0x7E,0x42,0x42,0x42,0x00}, // A - finer
    {0x7C,0x42,0x42,0x7C,0x42,0x42,0x7C,0x00}, // B - finer
    {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, // C
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // D
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x7E,0x00}, // E - finer
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x40,0x00}, // F - finer
    {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00}, // G
    {0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x00}, // H - finer
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, // I
    {0x06,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // J
    {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // K
    {0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x00}, // L - finer
    {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // M
    {0x42,0x62,0x72,0x5A,0x4E,0x46,0x42,0x00}, // N - finer
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // O
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // P
    {0x3C,0x66,0x66,0x66,0x66,0x3C,0x0E,0x00}, // Q
    {0x7C,0x42,0x42,0x7C,0x48,0x44,0x42,0x00}, // R - finer
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, // S
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // T - finer
    {0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00}, // U - finer
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // X
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // Y
    {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, // Z
    {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // [
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // \ (empty)
    {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // ]
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ^ (empty)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00}, // _
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ` (empty)
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, // a
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, // c
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // d
    {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // e
    {0x1C,0x36,0x30,0x78,0x30,0x30,0x30,0x00}, // f
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C}, // g
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x0E,0x06,0x06,0x66,0x66,0x3C}, // j
    {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0x66,0x7F,0x7F,0x6B,0x63,0x00}, // m
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // o
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // p
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, // q
    {0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00}, // r
    {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, // s
    {0x10,0x30,0x7C,0x30,0x30,0x36,0x1C,0x00}, // t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, // y
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}, // z
};

/* Logging tag for ESP_LOG functions */
static const char *TAG = "M5STICKS3_HAL";

#include "bmi270_config_data.inc"

/* ESP-LCD driver handles for display communication */
static esp_lcd_panel_handle_t panel_handle = NULL;  /* ST7789 panel handle */
static esp_lcd_panel_io_handle_t io_handle = NULL;  /* SPI I/O handle */
/* ESP32-S3 SPI LCD uses DMA; draw buffers must be in DMA-capable internal RAM (not stack). */
static uint16_t s_lcd_one_pixel_dma;

/* Forward declarations for internal helper functions */
static void draw_char_scaled(int x, int y, char c, uint16_t color, uint16_t bg_color, int scale);
static void display_boot_self_test(void);

/* I2C Bus Configuration for M5StickC Plus2
 * Used for IMU (MPU6886) and RTC (BM8563) communication
 * 400kHz fast mode for optimal performance
 */
static i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = M5_I2C_SDA_PIN,
    .scl_io_num = M5_I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    /* 100 kHz: shared M5PM1 + BMI270 bus is more reliable after PMIC traffic */
    .master.clk_speed = 100000,
};

/**
 * @brief One-time wipe of panel RAM beyond the normal 240×135 viewport.
 *
 * On this StickS3 panel we observe stray pixels at the very right column
 * and a band of uninitialized RAM at the very bottom row. Those addresses
 * are *outside* the (40, 52)..(279, 186) window M5GFX nominates as visible,
 * but the physical glass actually shows them. esp_lcd_panel_draw_bitmap()
 * happily accepts coordinates past M5_LCD_H_RES/V_RES — there is no
 * driver-side clipping — so we just push black bytes to a slightly larger
 * region once, at boot, to wipe whatever junk powered up there. Subsequent
 * UI math stays at 240×135 unchanged.
 */
static void display_wipe_edge_padding(void) {
    if (panel_handle == NULL) return;
    const int pad_w = M5_LCD_H_RES + 4;   /* +4 cols past the right edge */
    const int pad_h = M5_LCD_V_RES + 4;   /* +4 rows past the bottom edge */
    /* One row at a time keeps the DMA buffer tiny. */
    uint16_t *row = (uint16_t *)heap_caps_malloc(pad_w * sizeof(uint16_t),
                                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!row) return;
    for (int i = 0; i < pad_w; i++) row[i] = M5_COLOR_BLACK;
    for (int y = 0; y < pad_h; y++) {
        (void)esp_lcd_panel_draw_bitmap(panel_handle, 0, y, pad_w, y + 1, row);
    }
    heap_caps_free(row);
}

static void display_boot_self_test(void) {
    if (panel_handle == NULL) {
        return;
    }

    /* Wipe the panel-RAM "no-man's-land" just outside the spec'd 240×135 window
     * so leftover power-on RAM (rainbow pixels at bottom edge, dots at right edge)
     * isn't visible during normal operation. */
    display_wipe_edge_padding();

    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    m5stickc_plus2_display_draw_bitmap(LOGO_X, LOGO_Y, LOGO_W, LOGO_H,
                                       logo_bitmap, M5_COLOR_WHITE, M5_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(2000));
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
}

/* ── M5PM1 (StickS3) — Grove 5V / EXT_5V for Unit GPS on PORT.A ───────────── */
#define M5PM1_REG_PWR_CFG   0x06
#define M5PM1_REG_HOLD_CFG  0x07
#define M5PM1_REG_SYS_CMD   0x0C
#define M5PM1_REG_I2C_CFG   0x09
/* Registers for PMIC GPIO2 (L3B / LCD power) — same sequence as M5GFX StickS3 autodetect. */
#define M5PM1_REG_GPIO2_FN   0x16
#define M5PM1_REG_GPIO2_MODE 0x10
#define M5PM1_REG_GPIO2_PP   0x13
#define M5PM1_REG_GPIO2_OUT  0x11

/**
 * Send a dummy I2C START+addr+STOP to wake M5PM1 from I2C idle-sleep mode.
 * The M5PM1 library always does this before any register access (M5PM1.cpp line 644).
 * NACK is expected and ignored — the purpose is only to clock out a START condition.
 */
static void m5pm1_send_wake(void) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (M5PM1_I2C_ADDR << 1) | I2C_MASTER_WRITE, false /* NACK OK */);
    i2c_master_stop(cmd);
    /* Return value intentionally ignored — NACK is normal while device is asleep */
    (void)i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static esp_err_t m5pm1_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_NUM_0, M5PM1_I2C_ADDR, buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

static esp_err_t m5pm1_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(I2C_NUM_0, M5PM1_I2C_ADDR,
                                        &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

static esp_err_t m5pm1_rmw_bits(uint8_t reg, uint8_t set_mask, uint8_t clr_mask) {
    uint8_t v = 0;
    esp_err_t e = m5pm1_read_reg(reg, &v);
    if (e != ESP_OK) {
        return e;
    }
    v = (uint8_t)((v & (uint8_t)~clr_mask) | set_mask);
    return m5pm1_write_reg(reg, v);
}

/**
 * Turn on internal LCD rail via M5PM1 GPIO2 (L3B). Required before ST7789 init — see
 * M5GFX board_M5StickS3 autodetect (M5GFX.cpp): without this the panel stays off even
 * if backlight GPIO is high.
 */
static void sticks3_m5pm1_enable_lcd_power(void) {
    const uint8_t b2 = (uint8_t)(1u << 2);

    /* Send wake signal to pull M5PM1 out of I2C idle sleep, then verify it's alive.
     * Pattern mirrors M5PM1.cpp begin() (legacy path): wake → 10ms → probe → if fail, wait 800ms + retry. */
    m5pm1_send_wake();
    uint8_t id_val = 0xFF;
    esp_err_t id_ret = m5pm1_read_reg(0x00, &id_val);

    if (id_ret != ESP_OK) {
        ESP_LOGW(TAG, "M5PM1: first probe failed, retrying after 800ms");
        vTaskDelay(pdMS_TO_TICKS(800));
        m5pm1_send_wake();
        id_ret = m5pm1_read_reg(0x00, &id_val);
        if (id_ret != ESP_OK) {
            ESP_LOGE(TAG, "M5PM1: not responding after retry — LCD power (L3B) will NOT be enabled");
            return;
        }
    }

    /* M5PM1 is alive. Set GPIO2 as output-high to enable L3B (LCD power rail). */
    if (m5pm1_rmw_bits(M5PM1_REG_GPIO2_FN, 0, b2) != ESP_OK) { return; }
    if (m5pm1_rmw_bits(M5PM1_REG_GPIO2_MODE, b2, 0) != ESP_OK) { return; }
    if (m5pm1_rmw_bits(M5PM1_REG_GPIO2_PP, 0, b2) != ESP_OK) { return; }
    if (m5pm1_rmw_bits(M5PM1_REG_GPIO2_OUT, b2, 0) != ESP_OK) { return; }

    /* Disable I2C idle sleep so M5PM1 stays awake for subsequent operations */
    (void)m5pm1_write_reg(M5PM1_REG_I2C_CFG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "M5PM1: LCD power (L3B) enabled");
}

/** Enable BOOST / 5VINOUT so Grove red wire powers the GPS module (M5 docs: setExtOutput). */
static void sticks3_m5pm1_enable_grove_5v(void) {
    uint8_t pwr = 0;
    uint8_t hold = 0;
    /* I2C_CFG was set to 0 (sleep disabled) in lcd_power; no re-wake needed, but send anyway
     * in case lcd_power returned early due to earlier failure path. */
    m5pm1_send_wake();
    if (m5pm1_read_reg(M5PM1_REG_PWR_CFG, &pwr) != ESP_OK) {
        ESP_LOGW(TAG, "M5PM1: read PWR_CFG failed — Grove 5V may be off");
        return;
    }
    pwr |= (1u << 3); /* BOOST_EN */
    if (m5pm1_write_reg(M5PM1_REG_PWR_CFG, pwr) != ESP_OK) {
        ESP_LOGW(TAG, "M5PM1: write PWR_CFG failed");
        return;
    }
    if (m5pm1_read_reg(M5PM1_REG_HOLD_CFG, &hold) == ESP_OK) {
        hold |= (1u << 6); /* BOOST / Grove power hold */
        (void)m5pm1_write_reg(M5PM1_REG_HOLD_CFG, hold);
    }
    ESP_LOGI(TAG, "M5PM1: Grove 5V (BOOST) requested");
}

/**
 * @brief Initialize all M5StickC Plus2 hardware components
 * 
 * Performs complete hardware initialization in the correct dependency order:
 * 1. Power management (enables device hold circuit)
 * 2. I2C bus (for IMU and RTC communication)
 * 3. Display subsystem (ST7789 TFT LCD)
 * 4. Button inputs (A, B, Power buttons)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_init(void) {
    ESP_LOGI(TAG, "Initializing M5 StickS3 hardware");
    
    /* Initialize power management first - enables device hold circuit and backlight */
    int ret = m5stickc_plus2_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management");
        return ret;
    }
    
    /* Initialize I2C bus for sensor communication (IMU, RTC) */
    ret = m5stickc_plus2_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return ret;
    }

    /* PMIC: LCD rail (L3B) must be on before ST7789 — same as M5GFX StickS3 path. */
    sticks3_m5pm1_enable_lcd_power();

    /* Unit GPS v1.1 on Grove: 5 V from BOOST (see M5 StickS3 EXT_5V notes). */
    sticks3_m5pm1_enable_grove_5v();

    /* Initialize display subsystem (SPI, ST7789 controller) */
    ret = m5stickc_plus2_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ret;
    }
    
    /* Initialize button input handling */
    ret = m5stickc_plus2_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
        return ret;
    }

    /* BMI270 shares I2C with M5PM1 — short settle after PMIC/LCD sequence */
    vTaskDelay(pdMS_TO_TICKS(50));
    /* Initialize IMU (non-fatal: log warning if absent but continue) */
    ret = m5stickc_plus2_imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed (%s) - motion detection disabled", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "M5 StickS3 hardware initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize M5 StickS3 power management (backlight GPIO; hold via M5PM1).
 */
int m5stickc_plus2_power_init(void) {
    ESP_LOGI(TAG, "Initializing power management for M5 StickS3");
    
    /* Configure display backlight control pin
     * Used for both on/off control and PWM brightness adjustment
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << M5_LCD_BL_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Enable backlight at full brightness by default */
    gpio_set_level(M5_LCD_BL_PIN, 1);
    
    ESP_LOGI(TAG, "Power management initialized - BL=%d", M5_LCD_BL_PIN);
    return ESP_OK;
}

/**
 * @brief Power off the device immediately (M5PM1 system shutdown).
 */
void m5stickc_plus2_power_off(void) {
    ESP_LOGI(TAG, "Power off initiated");
    /* M5PM1_REG_SYS_CMD: high nibble key 0xA, low bits CMD=01 shutdown */
    (void)m5pm1_write_reg(M5PM1_REG_SYS_CMD, 0xA1);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initialize I2C bus for M5StickC Plus2
 * 
 * Configures I2C_NUM_0 for communication with onboard sensors:
 * - MPU6886 IMU (6-axis accelerometer/gyroscope)
 * - BM8563 RTC (real-time clock)
 * 
 * Uses 400kHz fast mode for optimal sensor communication performance.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_i2c_init(void) {
    /* Configure I2C parameters (pins, speed, pull-ups) */
    int ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return ret;
    }
    
    /* Install I2C driver with master mode, no slave buffers */
    ret = i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", M5_I2C_SDA_PIN, M5_I2C_SCL_PIN);
    return ESP_OK;
}

int m5stickc_plus2_display_init(void) {
    ESP_LOGI(TAG, "Display init pins: MOSI=%d SCLK=%d CS=%d DC=%d RST=%d BL=%d",
             M5_LCD_MOSI_PIN, M5_LCD_SCLK_PIN, M5_LCD_CS_PIN,
             M5_LCD_DC_PIN, M5_LCD_RST_PIN, M5_LCD_BL_PIN);

    /* Do not gpio_config CS or drive RST here — SPI master owns CS; ST7789 driver owns RST.
     * Manual CS as GPIO + hardware CS caused a dead bus on some ESP32-S3 panels. */

    spi_bus_config_t buscfg = {
        .mosi_io_num = M5_LCD_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = M5_LCD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        /* One full RGB565 frame; IDF LCD SPI chunks internally but needs a sane bus ceiling */
        .max_transfer_sz = (int)(M5_LCD_H_RES * M5_LCD_V_RES * sizeof(uint16_t) + 64),
    };
    
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized");

    
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = M5_LCD_DC_PIN,
        .cs_gpio_num = M5_LCD_CS_PIN,
        .pclk_hz = 40 * 1000 * 1000,  /* Match M5GFX StickS3 (40 MHz SPI write) */
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }
    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = M5_LCD_RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        /* Buffers are native uint16_t in RAM — must match ST7789 RAMCTRL little-endian */
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL,
    };
    
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_lcd_panel_invert_color(panel_handle, true);

    /* M5GFX StickS3 rotation=1: MX=1, MV=1, MY=0 → MADCTL 0x68
     * swap_xy=true → MV=1; mirror(true,false) → MX=1,MY=0.
     * Gap from M5GFX: offset_x=52 (physical cols), offset_y=40 (physical rows).
     * With MV=1 (row/col exchange): CASET addresses rows → x_gap=40; RASET addresses cols → y_gap=52. */
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_set_gap(panel_handle, 40, 52);

    esp_lcd_panel_disp_on_off(panel_handle, true);
    gpio_set_level(M5_LCD_BL_PIN, 1);
    ESP_LOGI(TAG, "Display initialization complete!");
    
    // Clear display to black and run a deterministic boot draw test
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    display_boot_self_test();
    
    return ESP_OK;
}

/**
 * @brief Initialize button input handling
 * 
 * Configures GPIO pins for the three physical buttons on M5StickC Plus2:
 * - Button A (GPIO35): Primary action button
 * - Button B (GPIO37): Secondary/navigation button  
 * - Power Button (GPIO39): Power control and system functions
 * 
 * Note: GPIOs 35, 37, 39 are input-only pins on ESP32 and cannot use
 * internal pull-up resistors. M5StickC Plus2 has external pull-ups.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
int m5stickc_plus2_buttons_init(void) {
    /* Configure button pins as inputs
     * GPIOs 35, 37, 39 are input-only on ESP32 - cannot use internal pull-ups
     * M5StickC Plus2 hardware provides external pull-up resistors
     */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << M5_BTN_A_PIN) | (1ULL << M5_BTN_B_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,  /* Cannot use internal pull-up on input-only pins */
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Buttons initialized - A=%d, B=%d (no dedicated PWR GPIO)", M5_BTN_A_PIN, M5_BTN_B_PIN);
    return ESP_OK;
}

/**
 * @brief Check if Button A is currently pressed
 * 
 * Button A is the primary action button used for executing current screen functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_a_pressed(void) {
    return gpio_get_level(M5_BTN_A_PIN) == 0;
}

/**
 * @brief Check if Button B is currently pressed
 * 
 * Button B is used for navigation between different screens and functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_b_pressed(void) {
    return gpio_get_level(M5_BTN_B_PIN) == 0;
}

/**
 * @brief Check if Power Button is currently pressed
 * 
 * Power button is used for device shutdown (3-second hold) and system functions.
 * Returns true when button is physically pressed (GPIO reads LOW).
 * 
 * @return true if button is pressed, false otherwise
 */
bool m5stickc_plus2_button_pwr_pressed(void) {
    return false;
}

/**
 * @brief Set display backlight brightness
 * 
 * Controls display brightness using PWM on the backlight pin.
 * Initializes LEDC PWM controller on first call for smooth brightness control.
 * 
 * @param brightness Brightness level (0-255, 0=off, 255=maximum)
 */
void m5stickc_plus2_display_set_brightness(uint8_t brightness) {
    /* Initialize PWM controller once for backlight control */
    static bool ledc_initialized = false;
    
    if (!ledc_initialized) {
        /* Configure LEDC timer for 8-bit PWM at 5kHz (flicker-free) */
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,  /* 8-bit resolution (0-255) */
            .freq_hz = 5000,                      /* 5kHz frequency to avoid flicker */
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);
        
        /* Configure LEDC channel for backlight control */
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = M5_LCD_BL_PIN,            /* Backlight control pin */
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
        ledc_initialized = true;
    }
    
    /* Set PWM duty cycle for brightness control (0-255) */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    ESP_LOGI(TAG, "Display brightness set to %d/255", brightness);
}

/**
 * @brief Get the ESP-LCD panel handle for direct access
 * 
 * Returns the internal LCD panel handle for advanced operations
 * that require direct ESP-LCD API access.
 * 
 * @return ESP-LCD panel handle, or NULL if not initialized
 */
esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void) {
    return panel_handle;
}

/**
 * @brief Clear entire display with solid color
 * 
 * Fills the complete display area (240x135) with the specified color.
 * Uses chunked rendering to optimize memory usage for large clears.
 * 
 * @param color RGB565 color value to fill the display with
 */
void m5stickc_plus2_display_clear(uint16_t color) {
    if (panel_handle) {
        /* Clear display using chunked approach to manage memory efficiently */
        const int chunk_height = 30;  /* Process 30 rows at a time */
        const int total_pixels = M5_LCD_H_RES * chunk_height;
        uint16_t *buffer = (uint16_t *)heap_caps_malloc(total_pixels * sizeof(uint16_t),
                                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        
        if (buffer) {
            /* Fill buffer with solid color (RGB565 format, no byte swapping needed) */
            for (int i = 0; i < total_pixels; i++) {
                buffer[i] = color;
            }
            
            /* Render display in horizontal chunks for memory efficiency */
            for (int y = 0; y < M5_LCD_V_RES; y += chunk_height) {
                int height = (y + chunk_height > M5_LCD_V_RES) ? (M5_LCD_V_RES - y) : chunk_height;
                esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, M5_LCD_H_RES, y + height, buffer);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to draw bitmap at y=%d: %s", y, esp_err_to_name(ret));
                }
            }
            heap_caps_free(buffer);
        } else {
            ESP_LOGE(TAG, "Failed to allocate display buffer");
        }
    }
}

/* Stack-buffer-based small filled-rect helper. Sibling implementation of
 * the one in m5stickc_plus2_hal.c — kept in lockstep across all three
 * HAL targets. Eliminates the per-pixel SPI transactions that the
 * previous implementation needed (and that the StickS3 author originally
 * worked around by introducing the static `s_lcd_one_pixel_dma` storage
 * — that workaround is no longer needed for text/bitmap paths). */
static void hal_draw_solid_run(int x, int y, int width, int height, uint16_t color) {
    if (panel_handle == NULL || width <= 0 || height <= 0) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;
    if (x < 0) { width += x; x = 0; }
    if (y < 0) { height += y; y = 0; }
    if (x + width  > M5_LCD_H_RES) width  = M5_LCD_H_RES - x;
    if (y + height > M5_LCD_V_RES) height = M5_LCD_V_RES - y;
    if (width <= 0 || height <= 0) return;

    uint16_t buf[64];
    const int max_pixels = (int)(sizeof(buf) / sizeof(buf[0]));
    int npx = width * height;
    if (npx > max_pixels) {
        for (int row = 0; row < height; row++) {
            hal_draw_solid_run(x, y + row, width, 1, color);
        }
        return;
    }
    for (int i = 0; i < npx; i++) buf[i] = color;
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + width, y + height, buf);
}

/**
 * @brief Draw single character at normal scale
 */
static void draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color) {
    draw_char_scaled(x, y, c, color, bg_color, 1);
}

/**
 * @brief Draw character with scaling support — run-batched implementation.
 */
static void draw_char_scaled(int x, int y, char c, uint16_t color, uint16_t bg_color, int scale) {
    (void)bg_color;
    if (panel_handle == NULL) return;
    if (scale < 1) scale = 1;

    int scaled_width  = 8 * scale;
    int scaled_height = 8 * scale;
    if (x < 0 || y < 0 || x + scaled_width > M5_LCD_H_RES || y + scaled_height > M5_LCD_V_RES) {
        return;
    }

    int idx = 0;
    if (c >= ' ' && c <= 'z') {
        idx = c - ' ';
    }

    const int char_width  = 8;
    const int char_height = 8;

    for (int row = 0; row < char_height; row++) {
        uint8_t line = font8x8[idx][row];
        int run_start = -1;
        for (int col = 0; col <= char_width; col++) {
            bool bit = (col < char_width) && ((line & (0x80 >> col)) != 0);
            if (bit && run_start == -1) {
                run_start = col;
            } else if (!bit && run_start != -1) {
                int run_cols = col - run_start;
                int px = x + run_start * scale;
                int py = y + row * scale;
                int rw = run_cols * scale;
                int rh = scale;
                hal_draw_solid_run(px, py, rw, rh, color);
                run_start = -1;
            }
        }
    }
}

/**
 * @brief Print text string at normal scale
 * 
 * Convenience wrapper for scaled text printing with scale factor of 1.
 * Renders text using the built-in 8x8 font.
 * 
 * @param x Starting horizontal position
 * @param y Starting vertical position
 * @param text Null-terminated string to print
 * @param color Text color (RGB565)
 */
void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color) {
    m5stickc_plus2_display_print_scaled(x, y, text, color, 1);
}

/**
 * @brief Print text string with scaling support
 * 
 * Renders text with specified scaling factor, supporting:
 * - Newline characters for explicit line breaks
 * - Automatic word wrapping at screen edge
 * - Transparent rendering (background shows through)
 * 
 * @param x Starting horizontal position
 * @param y Starting vertical position
 * @param text Null-terminated string to print (supports \n)
 * @param color Text color (RGB565)
 * @param scale Scaling factor (1=8x8, 2=16x16, etc.)
 */
void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale) {
    if (panel_handle == NULL || text == NULL) return;
    
    int cursor_x = x;
    const char *p = text;
    int char_width = 8 * scale;
    int char_height = 8 * scale;
    int line_spacing = char_height + 2;  /* Small spacing between lines */
    
    while (*p) {
        /* Handle explicit newline characters */
        if (*p == '\n') {
            cursor_x = x;
            y += line_spacing;  /* Move to next line */
        } else {
            /* Render character with transparency */
            draw_char_scaled(cursor_x, y, *p, color, M5_COLOR_BLACK, scale);
            cursor_x += char_width;  /* Advance cursor by scaled character width */
            
            /* Automatic word wrapping at screen edge */
            if (cursor_x > M5_LCD_H_RES - char_width) {
                cursor_x = x;
                y += line_spacing;
            }
        }
        p++;
    }
}

/**
 * @brief Draw monochrome bitmap with transparency
 * 
 * Renders a 1-bit bitmap (1=foreground, 0=transparent) at the specified position.
 * Only foreground pixels are drawn, allowing background to show through.
 * Automatically clips to screen boundaries.
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param width Bitmap width in pixels
 * @param height Bitmap height in pixels
 * @param bitmap Pointer to bitmap data (1 bit per pixel, row-major order)
 * @param color Foreground color (RGB565)
 * @param bg_color Background color (unused - transparent rendering)
 */
void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height, const uint8_t *bitmap, uint16_t color, uint16_t bg_color) {
    (void)bg_color;
    if (panel_handle == NULL || bitmap == NULL) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;

    int draw_width  = (x + width  > M5_LCD_H_RES) ? M5_LCD_H_RES - x : width;
    int draw_height = (y + height > M5_LCD_V_RES) ? M5_LCD_V_RES - y : height;
    if (draw_width <= 0 || draw_height <= 0) return;

    int byte_width = (width + 7) / 8;
    for (int row = 0; row < draw_height; row++) {
        const uint8_t *row_bytes = &bitmap[row * byte_width];
        int run_start = -1;
        for (int col = 0; col <= draw_width; col++) {
            bool pixel_set = false;
            if (col < draw_width) {
                int byte_idx = col >> 3;
                int bit_idx  = 7 - (col & 7);
                pixel_set = ((row_bytes[byte_idx] >> bit_idx) & 1) != 0;
            }
            if (pixel_set && run_start == -1) {
                run_start = col;
            } else if (!pixel_set && run_start != -1) {
                int run_cols = col - run_start;
                hal_draw_solid_run(x + run_start, y + row, run_cols, 1, color);
                run_start = -1;
            }
        }
    }
}

/**
 * @brief Draw filled circle (simplified as square)
 * 
 * Currently implemented as a filled square for simplicity and performance.
 * Most UI elements in the camera remote use rectangular shapes anyway.
 * 
 * @param x Center horizontal position
 * @param y Center vertical position
 * @param radius Circle radius in pixels
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color) {
    if (panel_handle == NULL || radius <= 0) return;
    
    /* Simple implementation: draw filled square instead of circle
     * This avoids complex circle drawing algorithms and potential artifacts
     * Most UI elements in this application use rectangular shapes
     */
    int size = radius * 2;
    m5stickc_plus2_display_fill_rect(x - radius, y - radius, size, size, color);
}

/**
 * @brief Draw filled rectangle
 * 
 * Renders a solid-colored rectangle using DMA-capable memory for optimal
 * performance. Automatically clips to screen boundaries.
 * 
 * @param x Horizontal position
 * @param y Vertical position
 * @param width Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color Fill color (RGB565)
 */
void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color) {
    if (panel_handle == NULL) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;
    
    /* Clip rectangle to screen boundaries */
    int draw_width = (x + width > M5_LCD_H_RES) ? M5_LCD_H_RES - x : width;
    int draw_height = (y + height > M5_LCD_V_RES) ? M5_LCD_V_RES - y : height;
    if (draw_width <= 0 || draw_height <= 0) return;
    
    /* Allocate DMA-capable buffer for hardware-accelerated transfer */
    size_t buffer_size = draw_width * draw_height * sizeof(uint16_t);
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(buffer_size,
                                                     MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate rectangle buffer");
        return;
    }
    
    /* Fill buffer with solid color (RGB565 format) */
    for (int i = 0; i < draw_width * draw_height; i++) {
        buffer[i] = color;
    }
    
    /* Transfer buffer to display using hardware acceleration */
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + draw_width, y + draw_height, buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw rectangle: %s", esp_err_to_name(ret));
    }
    
    heap_caps_free(buffer);
}

/* ──────────────────────────────────────────────────────────────────────────
 * BMI270 (M5Unified init pattern + config blob from bmi270_config_data.inc)
 * ──────────────────────────────────────────────────────────────────────── */

#define BMI270_REG_CHIP_ID        0x00
#define BMI270_REG_INTERNAL_STAT  0x21
#define BMI270_REG_ACC_CONF         0x40
#define BMI270_REG_INIT_CTRL        0x59
#define BMI270_REG_INIT_ADDR_0      0x5B
#define BMI270_REG_INIT_DATA        0x5E
#define BMI270_REG_INT_MAP_DATA     0x58
#define BMI270_REG_PWR_CTRL         0x7D
#define BMI270_REG_PWR_CONF         0x7C
#define BMI270_REG_CMD              0x7E
#define BMI270_REG_ACC_X_LSB        0x0C

#define BMI270_CHIP_ID_VAL          0x24
#define BMI270_SOFT_RESET           0xB6

/* ±8g range: same 4096 LSB/g as MPU6886 ±8g for comparable motion_logic thresholds */
#define BMI270_ACCEL_SCALE          4096.0f

/* Bosch BMI270: SDO pin selects 0x68 vs 0x69 — probe both */
static uint8_t s_bmi270_i2c_addr = 0x69;

static esp_err_t bmi270_write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_NUM_0, s_bmi270_i2c_addr, buf, 2, pdMS_TO_TICKS(80));
}

static esp_err_t bmi270_read8(uint8_t reg, uint8_t *out) {
    return i2c_master_write_read_device(I2C_NUM_0, s_bmi270_i2c_addr, &reg, 1, out, 1, pdMS_TO_TICKS(80));
}

static esp_err_t bmi270_read_block(uint8_t reg, uint8_t *out, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, s_bmi270_i2c_addr, &reg, 1, out, len, pdMS_TO_TICKS(80));
}

static esp_err_t bmi270_upload_config(void) {
    const size_t cfg_sz = sizeof(bmi270_config_file);
    for (size_t off = 0; off < cfg_sz; off += 32) {
        size_t chunk = cfg_sz - off;
        if (chunk > 32) {
            chunk = 32;
        }
        uint8_t ia[2] = { (uint8_t)((off >> 1) & 0x0F), (uint8_t)(off >> 5) };
        uint8_t hdr[3] = { BMI270_REG_INIT_ADDR_0, ia[0], ia[1] };
        esp_err_t e = i2c_master_write_to_device(I2C_NUM_0, s_bmi270_i2c_addr, hdr, 3, pdMS_TO_TICKS(80));
        if (e != ESP_OK) {
            return e;
        }
        uint8_t buf[33];
        buf[0] = BMI270_REG_INIT_DATA;
        memcpy(&buf[1], &bmi270_config_file[off], chunk);
        e = i2c_master_write_to_device(I2C_NUM_0, s_bmi270_i2c_addr, buf, 1 + chunk, pdMS_TO_TICKS(80));
        if (e != ESP_OK) {
            return e;
        }
    }
    return ESP_OK;
}

int m5stickc_plus2_imu_init(void) {
    uint8_t who = 0;
    esp_err_t ret = ESP_FAIL;
    for (unsigned try = 0; try < 2u; try++) {
        s_bmi270_i2c_addr = (try == 0u) ? 0x69u : 0x68u;
        ret = bmi270_read8(BMI270_REG_CHIP_ID, &who);
        if (ret == ESP_OK && who == BMI270_CHIP_ID_VAL) {
            ESP_LOGI(TAG, "BMI270 at 7-bit I2C addr 0x%02X", (unsigned)s_bmi270_i2c_addr);
            break;
        }
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 CHIP_ID read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (who != BMI270_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "BMI270 unexpected CHIP_ID 0x%02X (expect 0x%02X)", who, BMI270_CHIP_ID_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    (void)bmi270_write8(BMI270_REG_CMD, BMI270_SOFT_RESET);
    {
        int retry = 20;
        uint8_t pwr = 1;
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
            (void)bmi270_read8(BMI270_REG_PWR_CONF, &pwr);
        } while (pwr != 0 && --retry > 0);
    }

    (void)bmi270_write8(BMI270_REG_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    ret = bmi270_upload_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 config upload failed: %s", esp_err_to_name(ret));
        return ret;
    }
    (void)bmi270_write8(BMI270_REG_INIT_CTRL, 0x01);
    (void)bmi270_write8(BMI270_REG_INT_MAP_DATA, 0xFF);

    {
        int retry = 20;
        uint8_t ist = 0;
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
            (void)bmi270_read8(BMI270_REG_INTERNAL_STAT, &ist);
        } while (ist == 0 && --retry > 0);
        if (retry <= 0) {
            ESP_LOGW(TAG, "BMI270 INTERNAL_STATUS timeout (continuing)");
        }
    }

    /* PWR_CTRL bits per BMI270 datasheet §5.3.20: [0]=aux_en [1]=gyr_en [2]=acc_en [3]=temp_en.
     * StickS3 has no BMM150 on the BMI270 aux bus → AUX OFF.
     * Accel + gyro + temp = 0b00001110 = 0x0E.
     * (Previous value 0x0B = aux+gyr+temp had ACC OFF, so reads always returned 0,0,0
     *  and motion_logic saw constant deviation=1.0g → permanent false "moving" state.) */
    (void)bmi270_write8(BMI270_REG_PWR_CTRL, 0x0E);
    /* ±8g, normal performance (see Bosch BMI270 AN) */
    (void)bmi270_write8(BMI270_REG_ACC_CONF, 0xA8);
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG, "BMI270 initialized (CHIP_ID=0x%02X)", who);
    return ESP_OK;
}

int m5stickc_plus2_imu_read_accel(float *ax, float *ay, float *az) {
    if (ax == NULL || ay == NULL || az == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw[6];
    esp_err_t ret = bmi270_read_block(BMI270_REG_ACC_X_LSB, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }
    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);
    *ax = (float)rx / BMI270_ACCEL_SCALE;
    *ay = (float)ry / BMI270_ACCEL_SCALE;
    *az = (float)rz / BMI270_ACCEL_SCALE;
    return ESP_OK;
}

/* StickS3 has no onboard piezo buzzer; provide no-op stubs so the shared
 * call sites in app_main / ui don't need #ifdef guards. */
int m5stickc_plus2_buzzer_init(void) {
    return ESP_OK;
}
void m5stickc_plus2_buzzer_beep(uint16_t freq_hz, uint16_t duration_ms) {
    (void)freq_hz; (void)duration_ms;
}
void m5stickc_plus2_buzzer_beep_double(uint16_t freq_hz, uint16_t duration_ms, uint16_t gap_ms) {
    (void)freq_hz; (void)duration_ms; (void)gap_ms;
}

#endif /* M5STICKS3 */