/*
 * GPS — M5 StickS3: UART NMEA from Unit GPS v1.1 on Grove PORT.A.
 * Other targets: Zurich stub for BLE GPS push testing.
 */

#include "gps.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef M5STICKS3
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

#define TAG "GPS"

#ifndef M5STICKS3
/* Stub fix: Zurich, Switzerland (M5StickC Plus builds) */
#define GPS_STUB_LATITUDE    47.3769f
#define GPS_STUB_LONGITUDE    8.5417f
#define GPS_STUB_ALTITUDE   408.0f
#endif

static gps_data_t s_data = {0};

#ifdef M5STICKS3

#define GPS_UART_NUM       UART_NUM_1
#define GPS_UART_BAUD      115200
#define GPS_RX_BUF         2048
#define GPS_LINE_MAX       128

static SemaphoreHandle_t s_gps_mutex = NULL;

static int nmea_field_copy(const char *line, int field_index, char *out, size_t out_sz) {
    if (line == NULL || out == NULL || out_sz == 0) {
        return -1;
    }
    int idx = 0;
    size_t i = 0;
    while (line[i] && line[i] != '*' && line[i] != '\r' && line[i] != '\n') {
        if (line[i] == ',') {
            if (idx == field_index) {
                out[0] = '\0';
                return 0;
            }
            idx++;
            i++;
            continue;
        }
        if (idx == field_index) {
            size_t j = 0;
            while (line[i] && line[i] != ',' && line[i] != '*' && line[i] != '\r' && line[i] != '\n' && j + 1 < out_sz) {
                out[j++] = line[i++];
            }
            out[j] = '\0';
            return (int)j;
        }
        i++;
    }
    return -1;
}

static float nmea_dm_to_deg(const char *dm) {
    if (dm == NULL || dm[0] == '\0') {
        return NAN;
    }
    float v = strtof(dm, NULL);
    int deg = (int)(v / 100.0f);
    float minutes = v - (float)deg * 100.0f;
    return (float)deg + minutes / 60.0f;
}

static void gps_apply_gga(const char *line) {
    char buf[24];
    /* $GxGGA: f2=lat dm, f3=N/S, f4=lon dm, f5=E/W, f6=fix, f7=sats, f9=alt MSL */
    if (nmea_field_copy(line, 6, buf, sizeof(buf)) < 0) {
        return;
    }
    int fixq = (int)strtol(buf, NULL, 10);
    if (nmea_field_copy(line, 2, buf, sizeof(buf)) <= 0 || buf[0] == '\0') {
        return;
    }
    float lat = nmea_dm_to_deg(buf);
    char hemi[4];
    if (nmea_field_copy(line, 3, hemi, sizeof(hemi)) <= 0) {
        return;
    }
    if (hemi[0] == 'S') {
        lat = -fabsf(lat);
    }
    if (nmea_field_copy(line, 4, buf, sizeof(buf)) <= 0 || buf[0] == '\0') {
        return;
    }
    float lon = nmea_dm_to_deg(buf);
    if (nmea_field_copy(line, 5, hemi, sizeof(hemi)) <= 0) {
        return;
    }
    if (hemi[0] == 'W') {
        lon = -fabsf(lon);
    }
    float alt = 0.0f;
    if (nmea_field_copy(line, 9, buf, sizeof(buf)) > 0 && buf[0] != '\0') {
        alt = strtof(buf, NULL);
    }
    uint8_t nsat = 0;
    if (nmea_field_copy(line, 7, buf, sizeof(buf)) > 0 && buf[0] != '\0') {
        nsat = (uint8_t)strtoul(buf, NULL, 10);
    }
    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_data.latitude = lat;
        s_data.longitude = lon;
        s_data.altitude = alt;
        s_data.satellite_count = nsat;
        s_data.has_fix = (fixq > 0) && !isnan(lat) && !isnan(lon);
        xSemaphoreGive(s_gps_mutex);
    }
}

static void gps_apply_rmc(const char *line) {
    char buf[16];
    /* $GxRMC: f2=status (A/V), f7=speed knots, f8=course */
    if (nmea_field_copy(line, 2, buf, sizeof(buf)) <= 0 || buf[0] != 'A') {
        return;
    }
    float speed_knots = 0.0f;
    if (nmea_field_copy(line, 7, buf, sizeof(buf)) > 0 && buf[0] != '\0') {
        speed_knots = strtof(buf, NULL);
    }
    float course = 0.0f;
    if (nmea_field_copy(line, 8, buf, sizeof(buf)) > 0 && buf[0] != '\0') {
        course = strtof(buf, NULL);
    }
    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_data.speed = speed_knots * 0.514444f;
        s_data.course = course;
        xSemaphoreGive(s_gps_mutex);
    }
}

static void gps_parse_line(const char *line) {
    if (line == NULL || line[0] != '$') {
        return;
    }
    if (strstr(line, "GGA") != NULL) {
        gps_apply_gga(line);
    } else if (strstr(line, "RMC") != NULL) {
        gps_apply_rmc(line);
    }
}

static void gps_uart_task(void *arg) {
    uint8_t *buf = (uint8_t *)malloc(GPS_RX_BUF);
    if (buf == NULL) {
        ESP_LOGE(TAG, "GPS task malloc failed");
        vTaskDelete(NULL);
        return;
    }
    char line[GPS_LINE_MAX];
    size_t line_len = 0;

    for (;;) {
        int n = uart_read_bytes(GPS_UART_NUM, buf, GPS_RX_BUF, pdMS_TO_TICKS(200));
        if (n <= 0) {
            continue;
        }
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\r') {
                continue;
            }
            if (c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0) {
                    gps_parse_line(line);
                }
                line_len = 0;
                continue;
            }
            if (line_len + 1 < GPS_LINE_MAX) {
                line[line_len++] = c;
            } else {
                line_len = 0;
            }
        }
    }
}

#endif /* M5STICKS3 */

void gps_init(void) {
#ifdef M5STICKS3
    memset(&s_data, 0, sizeof(s_data));
    s_gps_mutex = xSemaphoreCreateMutex();
    if (s_gps_mutex == NULL) {
        ESP_LOGE(TAG, "GPS mutex create failed");
        return;
    }

    uart_config_t u = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(GPS_UART_NUM, GPS_RX_BUF * 2, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return;
    }
    err = uart_param_config(GPS_UART_NUM, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return;
    }
    err = uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        return;
    }

    if (xTaskCreate(gps_uart_task, "gps_uart", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "GPS task create failed");
        return;
    }
    ESP_LOGI(TAG, "GPS UART on TX=%d RX=%d @ %d baud (Unit GPS v1.1)", GPS_UART_TX_PIN, GPS_UART_RX_PIN,
             GPS_UART_BAUD);
#else
    memset(&s_data, 0, sizeof(s_data));
    s_data.latitude = GPS_STUB_LATITUDE;
    s_data.longitude = GPS_STUB_LONGITUDE;
    s_data.altitude = GPS_STUB_ALTITUDE;
    s_data.speed = 0.0f;
    s_data.course = 0.0f;
    s_data.satellite_count = 8;
    s_data.has_fix = true;
    ESP_LOGI(TAG, "GPS placeholder — stub fix Zurich CH (TX=%d RX=%d)", GPS_UART_TX_PIN, GPS_UART_RX_PIN);
#endif
}

bool gps_has_fix(void) {
#ifdef M5STICKS3
    bool f = false;
    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        f = s_data.has_fix;
        xSemaphoreGive(s_gps_mutex);
    }
    return f;
#else
    return s_data.has_fix;
#endif
}

void gps_get_data(gps_data_t *out) {
    if (out == NULL) {
        return;
    }
#ifdef M5STICKS3
    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = s_data;
        xSemaphoreGive(s_gps_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
#else
    *out = s_data;
#endif
}
