/*
 * M5 AtomS3 HAL — ESP32-S3, 128x128 IPS LCD, MPU6886, single screen button.
 * Same API as the Plus2/StickS3 HAL; compiled only when M5ATOMS3 is defined.
 *
 * References (see docs/ATOMS3_REFERENCE_GROUNDING.md):
 *   R1 — m5stack/M5AtomS3 (vendor pin map)
 *   R4 — m5stack/M5GFX (panel init / color order)
 *   R5 — espressif/esp-bsp m5_atom_s3 (gc9a01 path; documented fallback)
 *   R6 — espressif/esp_lcd_gc9a01 (managed component)
 *   R10 — m5stack/M5Atomic-Motion (confirms MPU6886 SDA=38 / SCL=39)
 *
 * Decisions applied (see docs/ATOMS3_DECISIONS.md):
 *   D-002 — first bring-up uses ST7789 driver; switch to gc9a01 if needed.
 *   D-003 — SPIRAM disabled in sdkconfig defaults (AtomS3 has none).
 *   D-006 — no PMIC / no power-off path.
 */

#ifdef M5ATOMS3

#include "m5atoms3_hal.h"

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

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"

/* 8x8 font — kept identical to the StickS3 HAL so text rendering looks the same
 * across all targets. ASCII 32..122. */
static const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // (
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // )
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
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
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // :
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x24,0x42,0x7E,0x42,0x42,0x42,0x00}, // A
    {0x7C,0x42,0x42,0x7C,0x42,0x42,0x7C,0x00}, // B
    {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, // C
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // D
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x7E,0x00}, // E
    {0x7E,0x40,0x40,0x78,0x40,0x40,0x40,0x00}, // F
    {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00}, // G
    {0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x00}, // H
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, // I
    {0x06,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // J
    {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // K
    {0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x00}, // L
    {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // M
    {0x42,0x62,0x72,0x5A,0x4E,0x46,0x42,0x00}, // N
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // O
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // P
    {0x3C,0x66,0x66,0x66,0x66,0x3C,0x0E,0x00}, // Q
    {0x7C,0x42,0x42,0x7C,0x48,0x44,0x42,0x00}, // R
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, // S
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // T
    {0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00}, // U
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // X
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // Y
    {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, // Z
    {0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00}, // [
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00}, // ]
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00}, // _
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
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

static const char *TAG = "M5ATOMS3_HAL";

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

static void draw_char_scaled(int x, int y, char c, uint16_t color, uint16_t bg_color, int scale);
static void display_boot_self_test(void);

static i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = M5_I2C_SDA_PIN,
    .scl_io_num = M5_I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000, /* MPU6886 dedicated bus, 400 kHz is fine */
};

/* ──────────────────────────────────────────────────────────────────────────
 * Top-level init
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_init(void) {
    ESP_LOGI(TAG, "Initializing M5 AtomS3 hardware");

    int ret = m5stickc_plus2_power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management");
        return ret;
    }

    ret = m5stickc_plus2_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return ret;
    }

    ret = m5stickc_plus2_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display");
        return ret;
    }

    ret = m5stickc_plus2_buttons_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
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
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_power_init(void) {
    ESP_LOGI(TAG, "Initializing power management for M5 AtomS3");
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
    gpio_set_level(M5_LCD_BL_PIN, 1);
    ESP_LOGI(TAG, "Power management initialized — BL=%d", M5_LCD_BL_PIN);
    return ESP_OK;
}

void m5stickc_plus2_power_off(void) {
    /* AtomS3 has no PMIC; the only "off" is unplugging USB-C.
     * Park the CPU in a low-power loop so the call site doesn't crash if
     * something invokes this from the shared shutdown path. */
    ESP_LOGI(TAG, "Power off requested (AtomS3 has no PMIC; entering idle loop)");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int m5stickc_plus2_i2c_init(void) {
    int ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return ret;
    }
    ret = i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }
    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", M5_I2C_SDA_PIN, M5_I2C_SCL_PIN);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Display — first bring-up uses the ST7789 driver path (D-002).
 * The AtomS3 GC9107 controller speaks a similar SPI command subset; if first
 * bring-up shows scrambled colors / wrong window, switch to the gc9a01
 * managed component (R5 / R6).
 * ──────────────────────────────────────────────────────────────────────── */

int m5stickc_plus2_display_init(void) {
    ESP_LOGI(TAG, "Display init pins: MOSI=%d SCLK=%d CS=%d DC=%d RST=%d BL=%d",
             M5_LCD_MOSI_PIN, M5_LCD_SCLK_PIN, M5_LCD_CS_PIN,
             M5_LCD_DC_PIN, M5_LCD_RST_PIN, M5_LCD_BL_PIN);

    spi_bus_config_t buscfg = {
        .mosi_io_num = M5_LCD_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = M5_LCD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
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
        /* AtomS3 panel: M5GFX runs at 40 MHz; keep parity with StickS3 path. */
        .pclk_hz = 40 * 1000 * 1000,
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
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL,
    };

    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) return ret;
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) return ret;

    /* AtomS3 panel needs color inversion to render correctly (matches M5GFX
     * Panel_GC9107 configuration in R4). */
    esp_lcd_panel_invert_color(panel_handle, true);

    /* AtomS3 panel needs 180° rotation versus the ST7789 driver's default
     * orientation (confirmed by user during first bring-up on 2026-05-22).
     * 180° = mirror both axes; the panel's RAM offset stays at (2, 1) since
     * the controller still addresses the same 128-row × 128-column window. */
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, true, true);
    esp_lcd_panel_set_gap(panel_handle, 2, 1);

    esp_lcd_panel_disp_on_off(panel_handle, true);
    gpio_set_level(M5_LCD_BL_PIN, 1);

    ESP_LOGI(TAG, "Display initialization complete!");
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    display_boot_self_test();
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Buttons — single screen button on G41, active-low, internal pull-up.
 * The AtomS3 has no second button or dedicated power button, so the
 * button_b / button_pwr functions return false.
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
 * Backlight (LEDC PWM on M5_LCD_BL_PIN). M5 docs note "PWM signal frequency
 * for the LCD backlight driver is recommended to be 500Hz" — we follow that.
 * ──────────────────────────────────────────────────────────────────────── */

void m5stickc_plus2_display_set_brightness(uint8_t brightness) {
    static bool ledc_initialized = false;
    if (!ledc_initialized) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 500,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = M5_LCD_BL_PIN,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
        ledc_initialized = true;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "Display brightness set to %d/255", brightness);
}

esp_lcd_panel_handle_t m5stickc_plus2_get_display_handle(void) {
    return panel_handle;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Drawing helpers (clear / fill_rect / draw_bitmap / text)
 * — same shape as the StickS3 path; sized for 128x128 instead of 240x135.
 * ──────────────────────────────────────────────────────────────────────── */

void m5stickc_plus2_display_clear(uint16_t color) {
    if (panel_handle == NULL) return;
    /* On 128x128 a full-screen buffer is 32 KB — too big for a static, but
     * fine via heap_caps_malloc in DMA-capable internal RAM. We do it in
     * 32-row chunks anyway so behavior matches the StickS3 path. */
    const int chunk_height = 32;
    const int total_pixels = M5_LCD_H_RES * chunk_height;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(total_pixels * sizeof(uint16_t),
                                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "display_clear: malloc failed");
        return;
    }
    for (int i = 0; i < total_pixels; i++) buffer[i] = color;
    for (int y = 0; y < M5_LCD_V_RES; y += chunk_height) {
        int height = (y + chunk_height > M5_LCD_V_RES) ? (M5_LCD_V_RES - y) : chunk_height;
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, M5_LCD_H_RES, y + height, buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw bitmap at y=%d: %s", y, esp_err_to_name(ret));
        }
    }
    heap_caps_free(buffer);
}

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
    if (c >= ' ' && c <= 'z') idx = c - ' ';

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
                hal_draw_solid_run(x + run_start * scale, y + row * scale,
                                   run_cols * scale, scale, color);
                run_start = -1;
            }
        }
    }
}

void m5stickc_plus2_display_print(int x, int y, const char *text, uint16_t color) {
    m5stickc_plus2_display_print_scaled(x, y, text, color, 1);
}

void m5stickc_plus2_display_print_scaled(int x, int y, const char *text, uint16_t color, int scale) {
    if (panel_handle == NULL || text == NULL) return;
    int cursor_x = x;
    const char *p = text;
    int char_width = 8 * scale;
    int char_height = 8 * scale;
    int line_spacing = char_height + 2;

    while (*p) {
        if (*p == '\n') {
            cursor_x = x;
            y += line_spacing;
        } else {
            draw_char_scaled(cursor_x, y, *p, color, M5_COLOR_BLACK, scale);
            cursor_x += char_width;
            if (cursor_x > M5_LCD_H_RES - char_width) {
                cursor_x = x;
                y += line_spacing;
            }
        }
        p++;
    }
}

void m5stickc_plus2_display_draw_bitmap(int x, int y, int width, int height,
                                        const uint8_t *bitmap, uint16_t color, uint16_t bg_color) {
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

void m5stickc_plus2_display_fill_circle(int x, int y, int radius, uint16_t color) {
    if (panel_handle == NULL || radius <= 0) return;
    int size = radius * 2;
    m5stickc_plus2_display_fill_rect(x - radius, y - radius, size, size, color);
}

void m5stickc_plus2_display_fill_rect(int x, int y, int width, int height, uint16_t color) {
    if (panel_handle == NULL) return;
    if (x >= M5_LCD_H_RES || y >= M5_LCD_V_RES) return;

    int draw_width  = (x + width  > M5_LCD_H_RES) ? M5_LCD_H_RES - x : width;
    int draw_height = (y + height > M5_LCD_V_RES) ? M5_LCD_V_RES - y : height;
    if (draw_width <= 0 || draw_height <= 0) return;

    size_t buffer_size = draw_width * draw_height * sizeof(uint16_t);
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(buffer_size,
                                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "fill_rect: malloc failed");
        return;
    }
    for (int i = 0; i < draw_width * draw_height; i++) buffer[i] = color;
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + draw_width, y + draw_height, buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw rectangle: %s", esp_err_to_name(ret));
    }
    heap_caps_free(buffer);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Boot logo splash. Draws the existing logo_bitmap centered on 128x128 then
 * waits a short moment so the operator can see it before normal UI takes over.
 * ──────────────────────────────────────────────────────────────────────── */

/* Draw the bärgsiitsch logo (logo_bitmap.h: 231x87) at 2:1 downsample so it
 * fits the 128x128 panel. We read every other pixel and every other row from
 * the source bitmap and OR-merge each 2x2 source block into a single output
 * pixel (any '1' in the block lights the output). This preserves the logo's
 * silhouette at ~116x44 — the user explicitly asked to keep this graphic on
 * boot, and a downsampled-but-recognizable logo beats a clipped-and-broken one. */
static void atoms3_draw_logo_downsampled(int dest_x, int dest_y, uint16_t color) {
    if (panel_handle == NULL) return;
    int dst_w = LOGO_W / 2;
    int dst_h = LOGO_H / 2;
    int byte_w = (LOGO_W + 7) / 8;

    for (int dy = 0; dy < dst_h; dy++) {
        int sy = dy * 2;
        const uint8_t *row0 = &logo_bitmap[sy * byte_w];
        const uint8_t *row1 = &logo_bitmap[(sy + 1) * byte_w];
        int run_start = -1;
        for (int dx = 0; dx <= dst_w; dx++) {
            bool any_set = false;
            if (dx < dst_w) {
                int sx = dx * 2;
                /* Sample 2x2 source block; mark output set if any source bit is. */
                for (int xo = 0; xo < 2; xo++) {
                    int sxx = sx + xo;
                    if (sxx >= LOGO_W) break;
                    int byte_idx = sxx >> 3;
                    int bit_idx  = 7 - (sxx & 7);
                    if (((row0[byte_idx] >> bit_idx) & 1) ||
                        ((row1[byte_idx] >> bit_idx) & 1)) {
                        any_set = true;
                        break;
                    }
                }
            }
            if (any_set && run_start == -1) {
                run_start = dx;
            } else if (!any_set && run_start != -1) {
                int run_cols = dx - run_start;
                m5stickc_plus2_display_fill_rect(dest_x + run_start, dest_y + dy,
                                                 run_cols, 1, color);
                run_start = -1;
            }
        }
    }
}

static void display_boot_self_test(void) {
    if (panel_handle == NULL) return;
    /* User-required boot graphic: render the existing bärgsiitsch logo at 2:1
     * downsample so it fits 128x128. Centered horizontally; vertically biased
     * up a few rows so a small "REMOTE" label can sit underneath. The original
     * 231x87 logo becomes ~116x44 here. */
    int dst_w = LOGO_W / 2;
    int dst_h = LOGO_H / 2;
    int draw_x = (M5_LCD_H_RES - dst_w) / 2;
    int draw_y = 18;
    if (draw_x < 0) draw_x = 0;
    if (draw_y < 0) draw_y = 0;
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
    atoms3_draw_logo_downsampled(draw_x, draw_y, M5_COLOR_WHITE);
    /* Sub-label below the logo, dim grey. */
    int label_y = draw_y + dst_h + 8;
    if (label_y + 8 > M5_LCD_V_RES) label_y = M5_LCD_V_RES - 10;
    m5stickc_plus2_display_print(28, label_y, "DJI REMOTE", M5_COLOR_GREY);
    vTaskDelay(pdMS_TO_TICKS(1500));
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);
}

/* ──────────────────────────────────────────────────────────────────────────
 * MPU6886 IMU (well-known InvenSense register set)
 *
 * Datasheet refs:
 *   WHO_AM_I (0x75) -> 0x19
 *   PWR_MGMT_1 (0x6B): bit7=DEVICE_RESET; clear all to wake
 *   SMPLRT_DIV (0x19), CONFIG (0x1A), GYRO_CONFIG (0x1B), ACCEL_CONFIG (0x1C)
 *   ACCEL_XOUT_H starts at 0x3B (6 bytes big-endian)
 *
 * We use ±8g full-scale (ACCEL_CONFIG = 0x10) so motion_logic thresholds match
 * the StickS3 BMI270 path (4096 LSB/g).
 * ──────────────────────────────────────────────────────────────────────── */

#define MPU6886_REG_WHO_AM_I       0x75
#define MPU6886_REG_PWR_MGMT_1     0x6B
#define MPU6886_REG_ACCEL_CONFIG   0x1C
#define MPU6886_REG_ACCEL_XOUT_H   0x3B
#define MPU6886_WHO_AM_I_VAL       0x19
#define MPU6886_ACCEL_SCALE        4096.0f /* ±8g */

static esp_err_t mpu_write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_NUM_0, MPU6886_I2C_ADDR, buf, 2, pdMS_TO_TICKS(80));
}

static esp_err_t mpu_read8(uint8_t reg, uint8_t *out) {
    return i2c_master_write_read_device(I2C_NUM_0, MPU6886_I2C_ADDR, &reg, 1, out, 1, pdMS_TO_TICKS(80));
}

static esp_err_t mpu_read_block(uint8_t reg, uint8_t *out, size_t len) {
    return i2c_master_write_read_device(I2C_NUM_0, MPU6886_I2C_ADDR, &reg, 1, out, len, pdMS_TO_TICKS(80));
}

int m5stickc_plus2_imu_init(void) {
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
