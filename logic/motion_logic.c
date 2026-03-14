/*
 * Motion Detection Logic
 *
 * Uses the MPU6886 accelerometer to determine whether the device (and the
 * vehicle it is mounted in) is moving.  A simple magnitude-deviation
 * approach is used:
 *
 *   magnitude  = sqrt(ax² + ay² + az²)
 *   deviation  = |magnitude - 1.0g|
 *
 * At rest the magnitude is ~1g (gravity).  Road vibration and acceleration
 * push it away from 1g.  Three consecutive samples above the motion
 * threshold triggers the MOVING state; MOTION_STOP_TIMEOUT_MS of
 * consecutive quiet samples triggers the STOPPED edge.
 *
 * ── Tuning ───────────────────────────────────────────────────────────────
 * Adjust the constants below after testing in the actual vehicle.
 * ─────────────────────────────────────────────────────────────────────── */

#include "motion_logic.h"

#ifdef M5STICKC_PLUS_11
#include "m5stickc_plus11_hal.h"
#else
#include "m5stickc_plus2_hal.h"
#endif

#include "esp_log.h"
#include <math.h>
#include <stdint.h>

#define TAG "MOTION"

/* ── Tunable constants ──────────────────────────────────────────────────── */

/* Minimum deviation from 1g (in g units) to count as "motion"
 * 0.10f = sensitive (hand/tap); 0.15f = light movement; 0.25f = vehicle-only. */
#define MOTION_START_THRESHOLD_G    0.10f

/* Maximum deviation from 1g that still counts as "at rest" (must be < START for hysteresis) */
#define MOTION_STOP_THRESHOLD_G     0.06f

/* Number of consecutive above-threshold samples before declaring motion */
#define MOTION_CONFIRM_SAMPLES      3U

/* Duration of continuous quiet before declaring vehicle stopped (ms).
 * Set to 1 for production (5 min); 0 for testing (15 s). See SPEC.md. */
#ifndef MOTION_USE_PRODUCTION_TIMEOUT
#define MOTION_USE_PRODUCTION_TIMEOUT  0
#endif
#define MOTION_STOP_TIMEOUT_MS  ((MOTION_USE_PRODUCTION_TIMEOUT) ? 300000UL : 15000UL)

/* Interval between accelerometer reads, in ms (call motion_logic_update
 * at this rate from the main loop) */
#define MOTION_SAMPLE_INTERVAL_MS   100UL

/* ── Internal state ─────────────────────────────────────────────────────── */

typedef enum {
    MOTION_STATE_IDLE = 0,  /* At rest, no pending record */
    MOTION_STATE_MOVING,    /* Vehicle in motion          */
    MOTION_STATE_COUNTDOWN, /* Motion stopped; counting down before stop */
} motion_state_t;

static motion_state_t s_state          = MOTION_STATE_IDLE;
static bool           s_imu_ok         = false;
static bool           s_just_started   = false;
static bool           s_just_stopped   = false;

/* Consecutive above-threshold sample counter for start debounce */
static uint32_t s_start_counter = 0U;

/* Milliseconds of quiet accumulated (countdown to stop) */
static uint32_t s_quiet_ms = 0U;

/* Debug: log every N updates to capture deviation/state without spam */
static uint32_t s_update_count = 0U;

/* ── Public API ─────────────────────────────────────────────────────────── */

void motion_logic_init(void) {
    s_state        = MOTION_STATE_IDLE;
    s_imu_ok       = false;
    s_just_started = false;
    s_just_stopped = false;
    s_start_counter = 0U;
    s_quiet_ms      = 0U;

    /* IMU should already be initialized by the HAL; just note readiness */
    float ax, ay, az;
    esp_err_t ret = m5stickc_plus2_imu_read_accel(&ax, &ay, &az);
    if (ret == ESP_OK) {
        s_imu_ok = true;
        ESP_LOGI(TAG, "Motion detection ready (first sample: %.2fg %.2fg %.2fg)", ax, ay, az);
        ESP_LOGI("FLOW", "motion imu_ok=1");
    } else {
        ESP_LOGW(TAG, "IMU not responding — motion detection disabled");
        ESP_LOGI("FLOW", "motion imu_ok=0");
    }
}

void motion_logic_update(void) {
    /* Clear single-tick edge flags from the previous call */
    s_just_started = false;
    s_just_stopped = false;

    if (!s_imu_ok) {
        return;
    }

    float ax, ay, az;
    if (m5stickc_plus2_imu_read_accel(&ax, &ay, &az) != ESP_OK) {
        return;
    }

    float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    float deviation = fabsf(magnitude - 1.0f);

    s_update_count++;
    if (s_update_count % 20U == 0U) {
        ESP_LOGI("FLOW", "motion dev=%.3f state=%d", (double)deviation, (int)s_state);
    }

    switch (s_state) {
        case MOTION_STATE_IDLE:
            if (deviation > MOTION_START_THRESHOLD_G) {
                s_start_counter++;
                if (s_start_counter >= MOTION_CONFIRM_SAMPLES) {
                    s_state        = MOTION_STATE_MOVING;
                    s_just_started = true;
                    s_start_counter = 0U;
                    ESP_LOGI(TAG, "Motion started (dev=%.3fg)", deviation);
                    ESP_LOGI("FLOW", "motion_started");
                }
            } else {
                s_start_counter = 0U;
            }
            break;

        case MOTION_STATE_MOVING:
            if (deviation > MOTION_START_THRESHOLD_G) {
                s_start_counter = 0U; /* Reset any quiet counter */
            }
            if (deviation < MOTION_STOP_THRESHOLD_G) {
                /* Transition to countdown */
                s_state    = MOTION_STATE_COUNTDOWN;
                s_quiet_ms = MOTION_SAMPLE_INTERVAL_MS;
                ESP_LOGD(TAG, "Quiet detected, starting stop countdown");
            }
            break;

        case MOTION_STATE_COUNTDOWN:
            if (deviation > MOTION_START_THRESHOLD_G) {
                /* Motion resumed — go back to MOVING */
                s_state    = MOTION_STATE_MOVING;
                s_quiet_ms = 0U;
                ESP_LOGD(TAG, "Motion resumed during countdown");
            } else {
                s_quiet_ms += MOTION_SAMPLE_INTERVAL_MS;
                if (s_quiet_ms >= MOTION_STOP_TIMEOUT_MS) {
                    s_state        = MOTION_STATE_IDLE;
                    s_just_stopped = true;
                    s_quiet_ms     = 0U;
                    ESP_LOGI(TAG, "Motion stopped after %lu ms quiet", (unsigned long)MOTION_STOP_TIMEOUT_MS);
                    ESP_LOGI("FLOW", "motion_stopped");
                }
            }
            break;

        default:
            s_state = MOTION_STATE_IDLE;
            break;
    }
}

bool motion_logic_is_moving(void) {
    return (s_state == MOTION_STATE_MOVING || s_state == MOTION_STATE_COUNTDOWN);
}

bool motion_logic_just_started(void) {
    return s_just_started;
}

bool motion_logic_just_stopped(void) {
    return s_just_stopped;
}

uint32_t motion_logic_get_stop_countdown_sec_remaining(void) {
    if (s_state != MOTION_STATE_COUNTDOWN) {
        return 0U;
    }
    uint32_t remaining_ms = (s_quiet_ms >= MOTION_STOP_TIMEOUT_MS)
        ? 0U
        : (MOTION_STOP_TIMEOUT_MS - s_quiet_ms);
    return remaining_ms / 1000U;
}
