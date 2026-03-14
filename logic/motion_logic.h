/*
 * Motion Detection Logic
 *
 * Reads the MPU6886 accelerometer at a fixed interval and maintains a simple
 * moving / not-moving state.  The single-tick "edge" functions let app_main
 * react to transitions without polling raw state every loop.
 *
 * Tuning constants live at the top of motion_logic.c.
 */

#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize motion detection.
 *
 * Must be called after the HAL (and therefore the IMU) is initialized.
 * Safe to call even if IMU init failed — all functions become no-ops.
 */
void motion_logic_init(void);

/**
 * @brief Advance the motion state machine by one tick.
 *
 * Call from the main loop every MOTION_SAMPLE_INTERVAL_MS milliseconds
 * (currently 100 ms).  Reads one accelerometer sample and updates internal
 * state.  The just_started / just_stopped edge flags are cleared after each
 * call, so check them immediately after calling update.
 */
void motion_logic_update(void);

/**
 * @brief Returns true while the device is considered moving.
 */
bool motion_logic_is_moving(void);

/**
 * @brief Returns true for exactly one call after motion begins.
 *
 * The flag is cleared by motion_logic_update() on the following tick.
 */
bool motion_logic_just_started(void);

/**
 * @brief Returns true for exactly one call after motion stops.
 *
 * "Stopped" means no significant movement for MOTION_STOP_TIMEOUT_MS.
 * The flag is cleared by motion_logic_update() on the following tick.
 */
bool motion_logic_just_stopped(void);

/**
 * @brief Seconds remaining until "motion stopped" (sleep/stop recording).
 *
 * Only valid while in countdown state (motion ceased, waiting for timeout).
 * Returns 0 when not in countdown (idle or actively moving).
 */
uint32_t motion_logic_get_stop_countdown_sec_remaining(void);

#endif /* MOTION_LOGIC_H */
