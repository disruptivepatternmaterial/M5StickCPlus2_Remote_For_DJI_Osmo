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
/* Unit GPS v1.1 (AT6668/ATGM336H) ships at 115200 8N1; older Unit GPS (AT6558) is 9600.
 * Try 115200 first, fall back to 9600 if no NMEA '$' seen within GPS_BAUD_PROBE_MS. */
#define GPS_UART_BAUD_PRIMARY    115200
#define GPS_UART_BAUD_FALLBACK   9600
#define GPS_BAUD_PROBE_MS        4000
#define GPS_RX_BUF         2048
#define GPS_LINE_MAX       128

static SemaphoreHandle_t s_gps_mutex = NULL;
static int s_gps_current_baud = GPS_UART_BAUD_PRIMARY;

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

/* NMEA $GxGGA / $GxRMC field1: hhmmss[.sss] -> int32_t HHMMSS (matches DJI Osmo-GPS-Controller-Demo) */
static bool nmea_parse_hms_utc(const char *s, int32_t *out_hms) {
    if (s == NULL || out_hms == NULL) {
        return false;
    }
    while (s[0] == ' ' || s[0] == '\t') {
        s++;
    }
    if (strlen(s) < 6) {
        return false;
    }
    int hh = (s[0] - '0') * 10 + (s[1] - '0');
    int mm = (s[2] - '0') * 10 + (s[3] - '0');
    int ss = (s[4] - '0') * 10 + (s[5] - '0');
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) {
        return false;
    }
    *out_hms = (int32_t)(hh * 10000 + mm * 100 + ss);
    return true;
}

/* RMC field9: ddmmyy (UTC date) -> YYYYMMDD; GNSS NMEA year is 2000+yy */
static bool nmea_parse_rmc_ddmmyy(const char *s, int32_t *out_ymd) {
    if (s == NULL || out_ymd == NULL) {
        return false;
    }
    if (strlen(s) < 6U) {
        return false;
    }
    int dd = (s[0] - '0') * 10 + (s[1] - '0');
    int mo = (s[2] - '0') * 10 + (s[3] - '0');
    int yy = (s[4] - '0') * 10 + (s[5] - '0');
    if (dd < 1 || dd > 31 || mo < 1 || mo > 12) {
        return false;
    }
    int year = 2000 + yy;
    *out_ymd = (int32_t)(year * 10000 + mo * 100 + dd);
    return true;
}

static void gps_apply_gga(const char *line) {
    char buf[24];
    int32_t hms_utc = 0;
    /* f1=UTC, f2=lat dm, f3=N/S, f4=lon, f5=E/W, f6=fix, f7=sats, f9=alt MSL */
    if (nmea_field_copy(line, 1, buf, sizeof(buf)) > 0) {
        (void)nmea_parse_hms_utc(buf, &hms_utc);
    }
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
        if (hms_utc > 0) {
            s_data.hour_minute_second = hms_utc;
        }
        xSemaphoreGive(s_gps_mutex);
    }
}

static void gps_apply_rmc(const char *line) {
    char buf[16];
    /* $GxRMC: f1=UTC, f2=status (A/V), f7=speed knots, f8=course, f9=ddmmyy */
    if (nmea_field_copy(line, 2, buf, sizeof(buf)) <= 0 || buf[0] != 'A') {
        return;
    }
    int32_t hms_utc = 0;
    if (nmea_field_copy(line, 1, buf, sizeof(buf)) > 0) {
        (void)nmea_parse_hms_utc(buf, &hms_utc);
    }
    int32_t ymd = 0;
    if (nmea_field_copy(line, 9, buf, sizeof(buf)) > 0) {
        (void)nmea_parse_rmc_ddmmyy(buf, &ymd);
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
        if (hms_utc > 0) {
            s_data.hour_minute_second = hms_utc;
        }
        if (ymd > 0) {
            s_data.year_month_day = ymd;
        }
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

    /* Diagnostics: confirm UART activity, NMEA framing, and fix transitions in serial logs. */
    uint32_t total_bytes = 0;
    uint32_t total_lines = 0;
    uint32_t lines_since_log = 0;
    bool first_byte_logged = false;
    bool first_dollar_logged = false;
    bool last_has_fix = false;
    TickType_t start_tick = xTaskGetTickCount();
    bool baud_fallback_done = false;

    for (;;) {
        int n = uart_read_bytes(GPS_UART_NUM, buf, GPS_RX_BUF, pdMS_TO_TICKS(200));
        if (n > 0) {
            total_bytes += (uint32_t)n;
            if (!first_byte_logged) {
                first_byte_logged = true;
                ESP_LOGI(TAG, "First UART bytes received (n=%d, baud=%d)", n, s_gps_current_baud);
            }
        }

        /* Baud auto-fallback: if no '$' (NMEA start) seen after probe window, try 9600.
         * Some Unit GPS modules ship at 9600 even though docs say 115200 — see M5 community reports. */
        if (!baud_fallback_done && !first_dollar_logged) {
            uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS);
            if (elapsed_ms > GPS_BAUD_PROBE_MS) {
                baud_fallback_done = true;
                ESP_LOGW(TAG, "No NMEA '$' seen in %lums at %d baud (rx_bytes=%lu) — trying %d baud",
                         (unsigned long)elapsed_ms, s_gps_current_baud,
                         (unsigned long)total_bytes, GPS_UART_BAUD_FALLBACK);
                s_gps_current_baud = GPS_UART_BAUD_FALLBACK;
                (void)uart_set_baudrate(GPS_UART_NUM, GPS_UART_BAUD_FALLBACK);
                line_len = 0;
                total_bytes = 0;
            }
        }

        if (n <= 0) {
            continue;
        }
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '$' && !first_dollar_logged) {
                first_dollar_logged = true;
                ESP_LOGI(TAG, "First NMEA '$' detected — UART framing OK at %d baud", s_gps_current_baud);
            }
            if (c == '\r') {
                continue;
            }
            if (c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0) {
                    gps_parse_line(line);
                    total_lines++;
                    lines_since_log++;
                    /* One sample sentence every ~50 lines (~5 s at 10 Hz) so we can see content
                     * without flooding the log. */
                    if (lines_since_log >= 50U) {
                        lines_since_log = 0U;
                        bool has_fix_now = false;
                        uint8_t sats = 0;
                        if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                            has_fix_now = s_data.has_fix;
                            sats = s_data.satellite_count;
                            xSemaphoreGive(s_gps_mutex);
                        }
                        ESP_LOGI(TAG, "rx_lines=%lu rx_bytes=%lu fix=%d sats=%u sample=\"%s\"",
                                 (unsigned long)total_lines, (unsigned long)total_bytes,
                                 has_fix_now ? 1 : 0, (unsigned)sats, line);
                    }
                    /* Log fix transitions once each direction. */
                    bool has_fix_now = false;
                    uint8_t sats = 0;
                    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        has_fix_now = s_data.has_fix;
                        sats = s_data.satellite_count;
                        xSemaphoreGive(s_gps_mutex);
                    }
                    if (has_fix_now != last_has_fix) {
                        last_has_fix = has_fix_now;
                        ESP_LOGI(TAG, "fix=%d sats=%u", has_fix_now ? 1 : 0, (unsigned)sats);
                    }
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

    s_gps_current_baud = GPS_UART_BAUD_PRIMARY;
    uart_config_t u = {
        .baud_rate = s_gps_current_baud,
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
    ESP_LOGI(TAG, "GPS UART on TX=%d RX=%d @ %d baud (Unit GPS v1.1; auto-fallback to %d after %dms)",
             GPS_UART_TX_PIN, GPS_UART_RX_PIN, s_gps_current_baud,
             GPS_UART_BAUD_FALLBACK, GPS_BAUD_PROBE_MS);
#else
    memset(&s_data, 0, sizeof(s_data));
    s_data.latitude = GPS_STUB_LATITUDE;
    s_data.longitude = GPS_STUB_LONGITUDE;
    s_data.altitude = GPS_STUB_ALTITUDE;
    s_data.speed = 0.0f;
    s_data.course = 0.0f;
    s_data.satellite_count = 8;
    s_data.has_fix = true;
    s_data.year_month_day = 20260101;
    s_data.hour_minute_second = 120000;
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
