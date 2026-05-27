/*
 * DJI Camera Remote Control - Main Application Entry Point
 * 
 * This file contains the main application entry point and core system initialization
 * for the DJI camera remote control system running on M5StickC Plus2 hardware.
 * 
 * The system provides a Bluetooth Low Energy (BLE) interface to control DJI cameras
 * with features including:
 * - Camera connection management
 * - Shutter control (photo/video recording)
 * - Camera mode switching
 * - Sleep/wake functionality
 * - GPIO trigger support for external hardware
 * 
 * Hardware: M5StickC Plus2 (ESP32-based)
 * Display: 240x135 TFT LCD
 * Connectivity: Bluetooth Low Energy
 * 
 * Based on original DJI SDK implementation
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "light_logic.h"
#include "motion_logic.h"
#include "status_logic.h"
#include "gps.h"
#if defined(M5ATOMS3)
#include "m5atoms3_hal.h"
#elif defined(M5STICKS3)
#include "m5sticks3_hal.h"
#elif defined(M5STICKC_PLUS_11)
#include "m5stickc_plus11_hal.h"
#else
#include "m5stickc_plus2_hal.h"
#endif
#include "ui.h"
#include "dji_protocol_data_structures.h"
#include <math.h>

/**
 * @brief Main application entry point
 * 
 * This function serves as the ESP-IDF application entry point and implements
 * the complete system initialization sequence followed by the main event loop.
 * 
 * Initialization sequence:
 * 1. M5StickC Plus2 hardware (display, buttons, power management)
 * 2. RGB LED light system
 * 3. Bluetooth Low Energy subsystem
 * 4. User interface system (including GPIO triggers)
 * 
 * Main loop handles:
 * - Power button monitoring (3-second hold for shutdown)
 * - User input from physical buttons
 * - GPIO trigger processing from external hardware
 * - Display updates
 * 
 * @note This function never returns under normal operation
 */
/* ── Pending-stop retry state (visible to ui.c via app_main.h) ────────────────
 * Camera occasionally ignores a one-shot stop_record (field-observed
 * 2026-05-26). Any path that sends stop_record should also call
 * app_request_pending_stop(); the main loop ticks down the retries until the
 * status push reports !is_camera_recording() or the retry budget expires.
 * Asymmetric to start_record by design — see comment in the main loop.
 */
static volatile bool     s_pending_stop          = false;
static volatile uint32_t s_pending_stop_last_ms  = 0U;
static volatile uint8_t  s_pending_stop_retries  = 0U;
static const    uint32_t PENDING_STOP_RETRY_MS   = 1500U;
static const    uint8_t  PENDING_STOP_MAX_RETRIES = 5U;

void app_request_pending_stop(void) {
    /* Idempotent — calling this while a stop is already pending is fine; the
     * tick logic only retries while is_camera_recording() == true. */
    s_pending_stop         = true;
    s_pending_stop_last_ms = 0U;
    s_pending_stop_retries = 0U;
}

void app_main(void) {
    static const char *TAG = "MAIN";
    int res = 0;

    /* 
     * HARDWARE INITIALIZATION PHASE
     * Initialize all hardware components in dependency order
     */
    
    /* Initialize M5StickC Plus2 hardware platform
     * This includes: display controller, button GPIO, power management,
     * I2C bus, SPI bus, and other core hardware peripherals
     */
    res = m5stickc_plus2_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize M5 hardware");
        return;
    }
#if defined(M5ATOMS3)
    ESP_LOGI(TAG, "M5 AtomS3 hardware initialized");
#elif defined(M5STICKS3)
    ESP_LOGI(TAG, "M5 StickS3 hardware initialized");
#else
    ESP_LOGI(TAG, "M5StickC Plus2 hardware initialized");
#endif

    /* Initialize RGB LED light system
     * Sets up the WS2812 LED strip driver for status indication
     * Colors indicate: connection state, operation status, errors
     */
    res = init_light_logic();
    if (res != 0) {
        ESP_LOGW(TAG, "Light logic init failed — continuing (no WS2812/status LED timers)");
    } else {
        ESP_LOGI(TAG, "Light logic initialized");
    }

    /* PHY calibration corruption is plausible after brownout (under-voltage during BLE TX).
     * Erasing phy_init + restart on *every* panic/WDT was causing a connect-crash → reboot loop:
     * unrelated firmware faults looked like “PHY” issues and triggered endless restarts.
     * Only recover PHY after brownout here. */
    esp_reset_reason_t reset_reason = esp_reset_reason();
    if (reset_reason == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "Brownout on previous boot — erasing PHY calibration partition");
        const esp_partition_t *phy_part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_PHY, NULL);
        if (phy_part != NULL) {
            esp_err_t erase_err = esp_partition_erase_range(phy_part, 0, phy_part->size);
            if (erase_err == ESP_OK) {
                ESP_LOGW(TAG, "PHY partition erased — rebooting for fresh BLE calibration");
                esp_restart();
            } else {
                ESP_LOGE(TAG, "PHY partition erase failed: %s", esp_err_to_name(erase_err));
            }
        }
    } else if (reset_reason == ESP_RST_PANIC    ||
               reset_reason == ESP_RST_INT_WDT  ||
               reset_reason == ESP_RST_TASK_WDT ||
               reset_reason == ESP_RST_WDT) {
        ESP_LOGW(TAG, "Previous boot ended abnormally (reason=%d) — continuing without PHY erase",
                 (int)reset_reason);
    }

    /* Initialize NVS before BLE — the ESP32 BLE stack requires NVS for PHY
     * calibration data storage.  Erase and reinit if the partition is corrupted. */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupted, erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    if (nvs_ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(nvs_ret));
    }

    /* Initialize Bluetooth Low Energy subsystem
     * Configures ESP32 BLE stack for DJI camera communication
     * Sets up GATT client, advertising scanner, and connection management
     */
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    res = connect_logic_ble_init();
    if (res != 0) {
        ESP_LOGW(TAG, "BLE init failed — UI still starts; fix BT/NVS/coexistence to use camera");
    } else {
        ESP_LOGI(TAG, "Bluetooth initialized successfully");
    }

    /* Initialize user interface system
     * Sets up: display rendering, screen management, button handlers,
     * GPIO triggers, camera state management, NVS storage
     */
    ui_init();
    ESP_LOGI(TAG, "UI system initialized");

    /* Initialize motion detection (depends on IMU being ready via HAL init) */
    motion_logic_init();
    ESP_LOGI(TAG, "Motion detection initialized");

    /* Initialize GPS subsystem (placeholder until hardware arrives) */
    gps_init();
    ESP_LOGI(TAG, "GPS initialized");

    /* System ready - log operational information for user */
    ESP_LOGI(TAG, "System ready - Icon-based UI active!");
    ESP_LOGI(TAG, "Button A: Select/execute current option");
    ESP_LOGI(TAG, "Button B: Cycle through options");

    /*
     * MAIN APPLICATION EVENT LOOP
     * Handles all user input, system events, and display updates
     */
    
    /* Power button state tracking for shutdown detection */
    static uint32_t power_button_hold_time = 0;
    static bool power_button_was_pressed = false;
    /* Require power button to be released at least once after boot before we accept a 3s hold.
     * Prevents false shutdown on M5Stick C Plus 1.1 if GPIO 35 floats low. */
    static bool power_button_ever_released = false;

    /* Background reconnection timer - attempt every 15 seconds when disconnected */
    static uint32_t reconnection_timer = 0;
    static const uint32_t RECONNECTION_INTERVAL_MS = 15000;

    /* Track connection state to refresh status square on change */
    static connect_state_t s_last_conn_state = (connect_state_t)-1;

    /* Motion-triggered recording state */
    static bool     s_want_record    = false;  /* pending: set mode + start recording once connected */
    static bool     s_is_recording   = false;  /* true while camera is recording via motion trigger */
    /* Motion update runs every 100ms (2 × 50ms main-loop ticks) */
    static uint32_t s_motion_tick    = 0U;
    static const uint32_t MOTION_UPDATE_INTERVAL_MS = 100U;

    /* GPS push — send to camera once per second when connected and fix available */
    static uint32_t s_gps_tick = 0U;
    static const uint32_t GPS_PUSH_INTERVAL_MS = 1000U;

    static uint32_t s_flow_tick = 0U;
    static const uint32_t FLOW_LOG_INTERVAL_MS = 2000U;
    static uint32_t s_auto_tick = 0U;
    static const uint32_t AUTO_SAMPLE_INTERVAL_MS = 500U;

    /* See module-static `s_pending_stop_*` defined above this function for
     * the pending-stop retry state. The main loop ticks it every iteration
     * and the manual-stop path in ui.c arms it via app_request_pending_stop(). */
    while (1) {
        if (g_pending_set_video_mode_after_connect && connect_logic_get_state() == PROTOCOL_CONNECTED) {
            g_pending_set_video_mode_after_connect = false;
            ESP_LOGI("FLOW", "set_mode_after_connect → Video (0x01)");
            if (command_logic_switch_camera_mode(CAMERA_MODE_NORMAL) != NULL) {
                ESP_LOGI("FLOW", "set_mode_after_connect OK");
            } else {
                ESP_LOGW("FLOW", "set_mode_after_connect FAIL");
            }
        }

        /*
         * POWER MANAGEMENT
         * Monitor power button for 3-second hold to initiate shutdown
         */
        bool power_button_pressed = m5stickc_plus2_button_pwr_pressed();

        if (!power_button_pressed) {
            power_button_ever_released = true;
        }

        if (power_button_pressed && !power_button_was_pressed && power_button_ever_released) {
            /* Power button press detected (after at least one release since boot) - start hold timer */
            power_button_hold_time = 0;
            power_button_was_pressed = true;
            ESP_LOGI(TAG, "Power button pressed - hold for 3s to shutdown");
            
        } else if (power_button_pressed && power_button_was_pressed) {
            /* Power button held - increment timer and check for shutdown threshold */
            power_button_hold_time += 50; // Increment by main loop delay (50ms)
            
            if (power_button_hold_time >= 3000) { // 3 second threshold
                ESP_LOGI(TAG, "Power button held for 3s - shutting down");
                
                /* Display shutdown message to user */
                ui_show_message("Shutting down...", M5_COLOR_RED, 1000);
                
                /* HAL power-off: GPIO hold release (Plus2) or AXP192 shutdown (Plus 1.1) */
                m5stickc_plus2_power_off();
                
                /* Infinite loop in case shutdown fails */
                while(1) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        } else if (!power_button_pressed && power_button_was_pressed) {
            /* Power button released before shutdown threshold - reset timer */
            power_button_was_pressed = false;
            power_button_hold_time = 0;
        }

        /*
         * USER INPUT HANDLING
         * Process physical button presses with debouncing.
         *
         * Two distinct flows live here:
         *   - Plus2 / Plus 1.1 / StickS3: dedicated A and B buttons
         *   - AtomS3: a single screen button measured by press duration —
         *     short press (< 600 ms) = Next-screen, long press (>= 1500 ms)
         *     = Execute-current-screen. See docs/ATOMS3_MIGRATION_SPEC.md (D-004).
         */
#if defined(M5ATOMS3)
        {
            static bool     s_btn_was_down  = false;
            static uint32_t s_btn_hold_ms   = 0;
            static bool     s_long_fired    = false;
            const  uint32_t LONG_PRESS_MS   = 1500U;
            const  uint32_t MAIN_LOOP_MS    = 50U;

            bool down = m5stickc_plus2_button_a_pressed();
            if (down) {
                s_btn_hold_ms += MAIN_LOOP_MS;
                /* Latch the long-press exactly once so we don't loop-fire it
                 * while the user keeps holding. */
                if (!s_long_fired && s_btn_hold_ms >= LONG_PRESS_MS) {
                    ESP_LOGI(TAG, "AtomS3 long-press: executing current screen");
                    ui_execute_current_screen();
                    s_long_fired = true;
                }
                s_btn_was_down = true;
            } else if (s_btn_was_down) {
                /* Released — if no long-press was latched, this counts as a
                 * short press and cycles to the next screen. */
                if (!s_long_fired) {
                    ESP_LOGI(TAG, "AtomS3 short-press: cycling screen");
                    ui_next_screen();
                }
                s_btn_was_down = false;
                s_btn_hold_ms  = 0;
                s_long_fired   = false;
            }
        }
#else
        /* Button A: Execute current screen function
         * Actions: Connect, Shutter, Mode change, Sleep, Wake
         */
        if (m5stickc_plus2_button_a_pressed()) {
            ESP_LOGI(TAG, "Button A pressed - selecting current option");
            ui_execute_current_screen();

            /* Wait for button release to prevent multiple triggers */
            while (m5stickc_plus2_button_a_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // Additional debounce delay
        }

        /* Button B: Navigate to next screen
         * Cycles through: Connect → Shutter → Mode → Sleep → Wake → Connect...
         */
        if (m5stickc_plus2_button_b_pressed()) {
            ESP_LOGI(TAG, "Button B pressed - cycling to next option");
            ui_next_screen();

            /* Wait for button release to prevent multiple triggers */
            while (m5stickc_plus2_button_b_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // Additional debounce delay
        }
#endif /* M5ATOMS3 */

        /*
         * MOTION DETECTION + AUTO-RECORDING
         * Sample IMU every 100ms and drive the motion-triggered recording
         * state machine.
         *
         * Flow:
         *   motion starts → if disconnected: BLE wake + reconnect; switch to Video
         *                   mode → start recording
         *   motion stops (2.5 min quiet) → stop recording (camera stays awake so
         *                                   the next motion event can restart it
         *                                   without the unreliable BLE-wake step)
         */
        s_motion_tick += 50U;
        if (s_motion_tick >= MOTION_UPDATE_INTERVAL_MS) {
            s_motion_tick = 0U;
            motion_logic_update();

            if (motion_logic_just_started()) {
                if (motion_logic_is_armed()) {
                    ESP_LOGI(TAG, "Motion detected - initiating recording sequence");
                    ESP_LOGI("FLOW", "motion_just_started → want_record=1");
                    s_want_record = true;
                    /* Wake camera if it is sleeping / disconnected */
                    if (connect_logic_get_state() < PROTOCOL_CONNECTED) {
                        (void)connect_logic_ble_wakeup();
                        (void)ui_attempt_background_reconnection();
                    }
                } else {
                    /* Disarmed: respect the user's manual stop. Don't start recording,
                     * but the IMU state machine still tracks moving/stopped for telemetry. */
                    ESP_LOGI("FLOW", "motion_just_started ignored (auto disarmed)");
                }
            }

            /* Once connected and recording is pending, switch mode and start
             * Mode must be Video (0x01) for dashcam; see SPEC.md and plan. */
            if (s_want_record && connect_logic_get_state() == PROTOCOL_CONNECTED) {
                s_want_record = false;
                ESP_LOGI("FLOW", "motion → set_mode_then_start_record");
                (void)command_logic_switch_camera_mode(CAMERA_MODE_NORMAL);
                vTaskDelay(pdMS_TO_TICKS(200));
                ESP_LOGI("FLOW", "start_record");
                (void)command_logic_start_record();
                s_is_recording = true;
            }

            /* Auto-stop on quiet timeout: stop recording but keep BLE link + camera awake.
             *
             * Why we deliberately do NOT call power_mode_switch_sleep() here:
             *   The DJI Osmo Action does not reliably wake from BLE-only after a
             *   sleep command — the camera enters a deep state and only a physical
             *   button press resumes BLE responsiveness. For a motion-triggered
             *   dashcam this would brick auto-restart on the next bump. So we
             *   accept the small extra battery cost and keep the camera awake.
             *   See SPEC.md "Auto recording" section. */
            if (motion_logic_is_armed() && motion_logic_just_stopped()
                    && (s_is_recording || is_camera_recording())) {
                ESP_LOGI("FLOW", "motion_stopped → stop_record (camera stays awake)");
                (void)command_logic_stop_record();
                s_is_recording = false;
                /* Arm the periodic-retry watchdog: if the camera ignores
                 * this single stop, the main loop below will re-send. */
                s_pending_stop         = true;
                s_pending_stop_last_ms = 0U;  /* will trigger next tick */
                s_pending_stop_retries = 0U;
            }
        }

        /* Pending-stop retry loop. Camera occasionally swallows a one-shot
         * stop; we re-issue stop_record at PENDING_STOP_RETRY_MS until the
         * status push tells us recording stopped, or the retry budget
         * expires. */
        if (s_pending_stop) {
            if (!is_camera_recording()) {
                ESP_LOGI("FLOW", "pending_stop cleared after %u retries — camera reports stopped",
                         (unsigned)s_pending_stop_retries);
                s_pending_stop         = false;
                s_pending_stop_last_ms = 0U;
                s_pending_stop_retries = 0U;
            } else {
                s_pending_stop_last_ms += 50U;
                if (s_pending_stop_last_ms >= PENDING_STOP_RETRY_MS) {
                    s_pending_stop_last_ms = 0U;
                    if (s_pending_stop_retries >= PENDING_STOP_MAX_RETRIES) {
                        ESP_LOGE("FLOW", "pending_stop GAVE UP after %u retries — camera still recording",
                                 (unsigned)s_pending_stop_retries);
                        s_pending_stop         = false;
                        s_pending_stop_retries = 0U;
                    } else if (connect_logic_get_state() == PROTOCOL_CONNECTED) {
                        s_pending_stop_retries++;
                        ESP_LOGW("FLOW", "pending_stop retry %u/%u — re-sending stop_record",
                                 (unsigned)s_pending_stop_retries,
                                 (unsigned)PENDING_STOP_MAX_RETRIES);
                        (void)command_logic_stop_record();
                    }
                }
            }
        }

        /* FLOW state log every 2s */
        s_flow_tick += 50U;
        if (s_flow_tick >= FLOW_LOG_INTERVAL_MS) {
            s_flow_tick = 0U;
            ESP_LOGI("FLOW", "state conn=%d screen=%d armed=%d moving=%d want_rec=%d local_rec=%d cam_rec=%d pending_mode=%d",
                     (int)connect_logic_get_state(), (int)g_ui_state.current_screen,
                     motion_logic_is_armed() ? 1 : 0,
                     motion_logic_is_moving() ? 1 : 0, s_want_record ? 1 : 0, s_is_recording ? 1 : 0,
                     is_camera_recording() ? 1 : 0,
                     g_pending_set_video_mode_after_connect ? 1 : 0);
        }

        /*
         * GPS TELEMETRY PUSH
         * Send GPS data to camera once per second when connected and fix is valid.
         * Camera embeds the data into video metadata (satellite_number must be > 0).
         * year_month_day / hour_minute_second must be non-zero for Osmo to accept samples
         * (see dji-sdk/Osmo-GPS-Controller-Demo `gps_push_data`, CmdSet 0x00 / CmdID 0x17).
         */
        s_gps_tick += 50U;
        if (s_gps_tick >= GPS_PUSH_INTERVAL_MS) {
            s_gps_tick = 0U;
            if (gps_has_fix() && connect_logic_get_state() == PROTOCOL_CONNECTED) {
                gps_data_t gps;
                gps_get_data(&gps);
                /* DJI R SDK 0x00:0x17 expects (UTC_hour+8)*10000 + min*100 + sec.
                 * Source of truth: dji-sdk/Osmo-GPS-Controller-Demo logic/gps_logic.c
                 * line 518 — `(GPS_Data.Hour + 8) * 10000 + Minute * 100 + Second`.
                 * No date-rollover handling — match the demo exactly so the camera's
                 * sample-acceptance path sees the same shape it does from the reference
                 * controller. gps.hour_minute_second is raw UTC HHMMSS. */
                int32_t utc_hh = gps.hour_minute_second / 10000;
                int32_t utc_mm = (gps.hour_minute_second / 100) % 100;
                int32_t utc_ss = gps.hour_minute_second % 100;
                int32_t hms_dji = (utc_hh + 8) * 10000 + utc_mm * 100 + utc_ss;

                float course_rad = gps.course * (float)(3.14159265358979f / 180.0f);
                gps_data_push_command_frame_t frame = {
                    .year_month_day       = gps.year_month_day,
                    .hour_minute_second   = hms_dji,
                    .gps_longitude        = (int32_t)(gps.longitude * 1e7f),
                    .gps_latitude         = (int32_t)(gps.latitude  * 1e7f),
                    .height               = (int32_t)(gps.altitude  * 1000.0f),
                    .speed_to_north       = gps.speed * cosf(course_rad) * 100.0f,
                    .speed_to_east        = gps.speed * sinf(course_rad) * 100.0f,
                    .speed_to_wnward      = 0.0f,
                    /* Accuracies are uint32 in DJI's wire format, NOT float.
                     * Units: mm (vert/horiz), cm/s (speed). Matches reference values
                     * from Osmo-GPS-Controller-Demo logic/gps_logic.c lines 566–571. */
                    .vertical_accuracy    = 1000u,
                    .horizontal_accuracy  = 1000u,
                    .speed_accuracy       = 10u,
                    .satellite_number     = gps.satellite_count,
                };
                if (gps.year_month_day > 0 && gps.hour_minute_second > 0) {
                    command_logic_push_gps_data(&frame);
                }
            }
        }

        /*
         * GPIO TRIGGER PROCESSING
         * Handle external GPIO triggers with randomized delays
         * This allows external hardware to trigger camera functions
         */
        ui_process_pending_gpio_actions();

        /*
         * DISPLAY UPDATE
         * Full redraw only when display_needs_update. On Auto screen, sample motion at 500ms
         * and set dirty only when status content changes to avoid flicker.
         */
        if (g_ui_state.current_screen == SCREEN_AUTO) {
            static bool s_auto_last_moving = false;
            static bool s_auto_last_armed = true;
            static uint32_t s_auto_last_countdown_sec = 0U;
            static bool s_auto_last_recording = false;
            s_auto_tick += 50U;
            if (s_auto_tick >= AUTO_SAMPLE_INTERVAL_MS) {
                s_auto_tick = 0U;
                bool moving = motion_logic_is_moving();
                bool armed = motion_logic_is_armed();
                uint32_t countdown_sec = motion_logic_get_stop_countdown_sec_remaining();
                bool recording = is_camera_recording();
                /* Recording state is now part of the pill (REC vs WAITING comes
                 * from the camera, not the IMU), so a change there must trigger
                 * a refresh too. */
                if (moving != s_auto_last_moving
                        || armed != s_auto_last_armed
                        || countdown_sec != s_auto_last_countdown_sec
                        || recording != s_auto_last_recording) {
                    s_auto_last_moving = moving;
                    s_auto_last_armed = armed;
                    s_auto_last_countdown_sec = countdown_sec;
                    s_auto_last_recording = recording;
                    ui_update_auto_status_line_only();
                    ESP_LOGI("FLOW", "auto_status armed=%d moving=%d rec=%d countdown=%lu",
                             armed ? 1 : 0, moving ? 1 : 0, recording ? 1 : 0,
                             (unsigned long)countdown_sec);
                }
            }
        }
        /* Redraw whenever connection state changes so the status square updates.
         *
         * On the rising edge into PROTOCOL_CONNECTED (initial connect or reconnect)
         * also kick off the dashcam flow automatically: jump to the AUTO screen,
         * arm motion detection, switch the camera to Video mode, and start
         * recording. The user shouldn't have to remember to press Start every
         * time — the whole point of AUTO is hands-off capture. The 2.5-min
         * motion-quiet timeout still owns when recording stops.
         *
         * Audible cues (see SPEC.md "Audible Cues"):
         *   connect           → single high beep (2500 Hz / 60 ms)
         *   clean disconnect  → single low beep  (700 Hz / 80 ms)
         *   The unexpected-disconnect double beep fires below from the
         *   consume-flag path so the cue stays distinct from a manual drop. */
        {
            connect_state_t curr_conn = connect_logic_get_state();
            if (curr_conn != s_last_conn_state) {
                bool just_connected = (curr_conn == PROTOCOL_CONNECTED
                                       && s_last_conn_state != PROTOCOL_CONNECTED);
                bool just_disconnected = (s_last_conn_state == PROTOCOL_CONNECTED
                                          && curr_conn != PROTOCOL_CONNECTED);
                s_last_conn_state = curr_conn;
                g_ui_state.display_needs_update = true;

                if (just_connected) {
                    ESP_LOGI("FLOW", "PROTOCOL_CONNECTED → auto-jump to AUTO + start recording");
                    m5stickc_plus2_buzzer_beep(2500, 60);
                    g_ui_state.current_screen = SCREEN_AUTO;

                    /* Treat the moment of connect as "moving" so the stop countdown
                     * doesn't fire immediately on a still device — it'll only count
                     * down once the IMU has been quiet for the full timeout. */
                    motion_logic_force_active();
                    motion_logic_set_armed(true);

                    if (!is_camera_recording()) {
                        (void)command_logic_switch_camera_mode(CAMERA_MODE_NORMAL);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        ESP_LOGI("FLOW", "auto_on_connect → start_record");
                        (void)command_logic_start_record();
                        s_is_recording = true;
                    } else {
                        ESP_LOGI("FLOW", "auto_on_connect: camera already recording, skipping start");
                        s_is_recording = true;
                    }
                } else if (just_disconnected) {
                    ESP_LOGI("FLOW", "PROTOCOL_CONNECTED → cleared (clean disconnect)");
                    m5stickc_plus2_buzzer_beep(700, 80);
                }
            }
        }

        ui_update_display();

        /* Immediate reconnect trigger after unexpected BLE disconnect.
         * Reconnection work runs outside BLE callback context to avoid stack stalls. */
        if (connect_logic_consume_unexpected_disconnect()) {
            ESP_LOGW(TAG, "Unexpected disconnect detected - starting immediate background reconnection");
            /* Distinct double-beep so the user can tell unexpected drop apart
             * from a clean disconnect (single low beep). */
            m5stickc_plus2_buzzer_beep_double(700, 60, 80);
            (void)ui_attempt_background_reconnection();
        }

        /*
         * BACKGROUND RECONNECTION
         * Attempt automatic reconnection every 15 seconds when disconnected
         */
        reconnection_timer += 50; /* Increment by main loop delay (50ms) */
        
        if (reconnection_timer >= RECONNECTION_INTERVAL_MS) {
            reconnection_timer = 0; /* Reset timer */
            
            /* Attempt background reconnection through UI layer */
            ESP_LOGI(TAG, "Background reconnection timer triggered");
            int reconnect_result = ui_attempt_background_reconnection();
            if (reconnect_result == 0) {
                ESP_LOGI(TAG, "Background reconnection attempt completed successfully");
            } else {
                ESP_LOGI(TAG, "Background reconnection not needed or failed");
            }
        }

        /*
         * BACKGROUND TASKS
         * Light logic runs on its own FreeRTOS timer task
         * BLE operations run on ESP-IDF BLE stack tasks
         * No manual updates needed for these systems
         */

        /* Main loop timing - 50ms cycle time
         * Provides responsive UI while preventing excessive CPU usage
         */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}