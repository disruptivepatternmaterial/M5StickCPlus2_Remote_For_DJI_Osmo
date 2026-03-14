/*
 * GPS Module — Placeholder Implementation
 *
 * Replace this file with a UART/NMEA driver when the M5Stack GPS Module v2.0
 * hardware arrives.  The interface in gps.h stays the same.
 *
 * To enable real GPS:
 *  1. Wire the module (TX → GPS_UART_RX_PIN, RX → GPS_UART_TX_PIN).
 *  2. Confirm GPS_UART_TX_PIN / GPS_UART_RX_PIN in gps.h.
 *  3. Implement gps_init() with uart_driver_install() on UART_NUM_2.
 *  4. Add an NMEA parse task (GPGGA, GPRMC) that updates s_data under a mutex.
 *  5. Remove the "placeholder" log lines from gps_init().
 */

#include "gps.h"
#include "esp_log.h"
#include <string.h>

#define TAG "GPS"

/* Stub fix: Zurich, Switzerland (for testing GPS push until real module is wired) */
#define GPS_STUB_LATITUDE    47.3769f
#define GPS_STUB_LONGITUDE    8.5417f
#define GPS_STUB_ALTITUDE   408.0f

static gps_data_t s_data = {0};

void gps_init(void) {
    memset(&s_data, 0, sizeof(s_data));
    s_data.latitude       = GPS_STUB_LATITUDE;
    s_data.longitude      = GPS_STUB_LONGITUDE;
    s_data.altitude       = GPS_STUB_ALTITUDE;
    s_data.speed          = 0.0f;
    s_data.course         = 0.0f;
    s_data.satellite_count = 8;
    s_data.has_fix        = true;
    ESP_LOGI(TAG, "GPS placeholder — stub fix Zurich CH (TX=%d RX=%d)",
             GPS_UART_TX_PIN, GPS_UART_RX_PIN);
    ESP_LOGI(TAG, "GPS will use real module when M5Stack GPS Module v2.0 is wired");
}

bool gps_has_fix(void) {
    return s_data.has_fix;
}

void gps_get_data(gps_data_t *out) {
    if (out == NULL) { return; }
    *out = s_data;
}
