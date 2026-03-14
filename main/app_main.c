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

#include "connect_logic.h"
#include "command_logic.h"
#include "light_logic.h"
#include "motion_logic.h"
#include "gps.h"
#ifdef M5STICKC_PLUS_11
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
        ESP_LOGE(TAG, "Failed to initialize M5StickC Plus2 hardware");
        return;
    }
    ESP_LOGI(TAG, "M5StickC Plus2 hardware initialized");

    /* Initialize RGB LED light system
     * Sets up the WS2812 LED strip driver for status indication
     * Colors indicate: connection state, operation status, errors
     */
    res = init_light_logic();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize light logic");
        return;
    }
    ESP_LOGI(TAG, "Light logic initialized");

    /* Initialize Bluetooth Low Energy subsystem
     * Configures ESP32 BLE stack for DJI camera communication
     * Sets up GATT client, advertising scanner, and connection management
     */
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    res = connect_logic_ble_init();
    if (res != 0) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth");
        return;
    }
    ESP_LOGI(TAG, "Bluetooth initialized successfully");

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
                
                /* Disable power hold circuit to turn off device
                 * M5StickC Plus2 uses a power hold circuit that must be
                 * actively maintained to keep the device powered on
                 */
                gpio_set_level(M5_PWR_EN_PIN, 0);
                
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
         * Process physical button presses with debouncing
         */
        
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

        /*
         * MOTION DETECTION + AUTO-RECORDING
         * Sample IMU every 100ms and drive the motion-triggered recording
         * state machine.
         *
         * Flow:
         *   motion starts → wake camera → BLE reconnect → switch to Video
         *                   mode → start recording
         *   motion stops (5 min quiet) → stop recording → sleep camera
         */
        s_motion_tick += 50U;
        if (s_motion_tick >= MOTION_UPDATE_INTERVAL_MS) {
            s_motion_tick = 0U;
            motion_logic_update();

            if (motion_logic_just_started()) {
                ESP_LOGI(TAG, "Motion detected - initiating recording sequence");
                ESP_LOGI("FLOW", "motion_just_started → want_record=1");
                s_want_record = true;
                /* Wake camera if it is sleeping / disconnected */
                if (connect_logic_get_state() < PROTOCOL_CONNECTED) {
                    (void)connect_logic_ble_wakeup();
                    (void)ui_attempt_background_reconnection();
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

            if (motion_logic_just_stopped() && s_is_recording) {
                ESP_LOGI("FLOW", "motion_stopped → stop_record");
                (void)command_logic_stop_record();
                vTaskDelay(pdMS_TO_TICKS(500));
                ESP_LOGI("FLOW", "sleep");
                (void)command_logic_power_mode_switch_sleep();
                s_is_recording = false;
            }
        }

        /* FLOW state log every 2s */
        s_flow_tick += 50U;
        if (s_flow_tick >= FLOW_LOG_INTERVAL_MS) {
            s_flow_tick = 0U;
            ESP_LOGI("FLOW", "state conn=%d screen=%d moving=%d want_rec=%d is_rec=%d pending_mode=%d",
                     (int)connect_logic_get_state(), (int)g_ui_state.current_screen,
                     motion_logic_is_moving() ? 1 : 0, s_want_record ? 1 : 0, s_is_recording ? 1 : 0,
                     g_pending_set_video_mode_after_connect ? 1 : 0);
        }

        /*
         * GPS TELEMETRY PUSH
         * Send GPS data to camera once per second when connected and fix is valid.
         * Camera embeds the data into video metadata (satellite_number must be > 0).
         * No-op while the GPS placeholder is active (gps_has_fix() returns false).
         */
        s_gps_tick += 50U;
        if (s_gps_tick >= GPS_PUSH_INTERVAL_MS) {
            s_gps_tick = 0U;
            if (gps_has_fix() && connect_logic_get_state() == PROTOCOL_CONNECTED) {
                gps_data_t gps;
                gps_get_data(&gps);

                float course_rad = gps.course * (float)(3.14159265358979f / 180.0f);
                gps_data_push_command_frame_t frame = {
                    .year_month_day       = 0,        /* TODO: fill from RTC or GPS when available */
                    .hour_minute_second   = 0,
                    .gps_longitude        = (int32_t)(gps.longitude * 1e7f),
                    .gps_latitude         = (int32_t)(gps.latitude  * 1e7f),
                    .height               = (int32_t)(gps.altitude  * 1000.0f),
                    .speed_to_north       = gps.speed * cosf(course_rad) * 100.0f,
                    .speed_to_east        = gps.speed * sinf(course_rad) * 100.0f,
                    .speed_to_wnward      = 0.0f,
                    .vertical_accuracy    = 3.0f,
                    .horizontal_accuracy  = 3.0f,
                    .speed_accuracy       = 0.1f,
                    .satellite_number     = gps.satellite_count,
                };
                command_logic_push_gps_data(&frame);
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
            static uint32_t s_auto_last_countdown_sec = 0U;
            s_auto_tick += 50U;
            if (s_auto_tick >= AUTO_SAMPLE_INTERVAL_MS) {
                s_auto_tick = 0U;
                bool moving = motion_logic_is_moving();
                uint32_t countdown_sec = motion_logic_get_stop_countdown_sec_remaining();
                if (moving != s_auto_last_moving || countdown_sec != s_auto_last_countdown_sec) {
                    s_auto_last_moving = moving;
                    s_auto_last_countdown_sec = countdown_sec;
                    /* Only redraw the status line — not the whole screen — to avoid flicker */
                    ui_update_auto_status_line_only();
                    ESP_LOGI("FLOW", "auto_status moving=%d countdown=%lu", moving ? 1 : 0, (unsigned long)countdown_sec);
                }
            }
        }
        ui_update_display();

        /* Immediate reconnect trigger after unexpected BLE disconnect.
         * Reconnection work runs outside BLE callback context to avoid stack stalls. */
        if (connect_logic_consume_unexpected_disconnect()) {
            ESP_LOGW(TAG, "Unexpected disconnect detected - starting immediate background reconnection");
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