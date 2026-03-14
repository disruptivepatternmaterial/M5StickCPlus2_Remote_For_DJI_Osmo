/*
 * GPS Module Interface
 *
 * This is a placeholder implementation.  When the M5Stack GPS Module v2.0
 * hardware arrives, replace gps.c with a real UART/NMEA driver.
 *
 * The API is intentionally identical to the one used in
 * rhoenschrat/DJI-Remote so swapping in the full driver later only
 * requires replacing gps.c without touching any call sites.
 *
 * UART pins are defined below — update them to match your physical wiring
 * before enabling the real driver.
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * TODO: Update these to match how you wire the GPS module to the Plus 1.1.
 * Common options on M5StickC Plus:
 *   Grove port bottom: G32 (data) / G33 (clock) — check your specific unit
 *   HAT connector:     G0  / G26
 */
#define GPS_UART_TX_PIN   0   /* TODO: confirm pin */
#define GPS_UART_RX_PIN   26  /* TODO: confirm pin */

typedef struct {
    float    latitude;        /* Decimal degrees, positive = North */
    float    longitude;       /* Decimal degrees, positive = East  */
    float    altitude;        /* Metres above sea level            */
    float    speed;           /* Ground speed in m/s               */
    float    course;          /* Course over ground, 0-360 degrees */
    uint8_t  satellite_count; /* Satellites in view                */
    bool     has_fix;         /* Valid GPS fix available           */
} gps_data_t;

/**
 * @brief Initialize GPS subsystem.
 *
 * Placeholder: logs a notice and returns immediately.
 * Replace with UART/NMEA init when hardware is available.
 */
void gps_init(void);

/**
 * @brief Returns true if a valid GPS fix is available.
 *
 * Always false in the placeholder implementation.
 */
bool gps_has_fix(void);

/**
 * @brief Copy current GPS data into *out.
 *
 * Placeholder sets all fields to zero and has_fix = false.
 *
 * @param out Destination struct; must not be NULL.
 */
void gps_get_data(gps_data_t *out);

#endif /* GPS_H */
