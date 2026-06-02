/*
 * DJI Camera Remote Control - User Interface System
 * 
 * This file implements the complete user interface system for the DJI camera remote
 * control device, including:
 * 
 * - Display management and screen rendering
 * - Button-based navigation and function execution
 * - GPIO trigger system for external hardware integration
 * - Camera pairing and connection state management
 * - Persistent storage of camera information using NVS
 * - Device ID generation and management
 * - Status updates and real-time camera state tracking
 * 
 * The UI system supports multiple screens accessed via button navigation:
 * 1. Connect - Camera pairing and connection management
 * 2. Shutter - Photo capture and video recording control
 * 3. Mode - Camera mode switching (photo, video, timelapse, etc.)
 * 4. Sleep - Put camera into sleep mode
 * 5. Wake - Wake camera from sleep using BLE broadcast
 * 
 * GPIO Integration:
 * - G0 (pull LOW): Trigger shutter function
 * - G26 (pull HIGH): Trigger sleep function
 * - G25 (pull HIGH): Trigger wake function
 * 
 * All GPIO triggers include device ID-based randomized delays (1-100ms)
 * to prevent interference when multiple devices are used simultaneously.
 * 
 * Hardware: M5StickC Plus2 (ESP32-PICO-V3, 240x135 TFT LCD)
 * Framework: ESP-IDF v5.5 with FreeRTOS
 * 
 * Based on original DJI SDK implementation
 */

#include "ui.h"
#if defined(M5ATOMS3)
#include "m5atoms3_hal.h"
#elif defined(M5STICKS3)
#include "m5sticks3_hal.h"
#elif defined(M5STICKC_PLUS_11)
#include "m5stickc_plus11_hal.h"
#else
#include "m5stickc_plus2_hal.h"
#endif
#include "command_logic.h"
#include "app_main.h"   /* app_request_pending_stop() */
#include "status_logic.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "motion_logic.h"
#include "data.h"
#include "gps.h"
#include "ble.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "esp_random.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ── Non-blocking toast / worker-task plumbing ────────────────────────────────
 *
 * Old behavior: ui_show_message() blocked the calling task with a
 * vTaskDelay() for up to 2 seconds, which on the main task meant 2 s of
 * dropped button polls per status flash. Multi-second blocking BLE
 * operations on the same path made the device feel "frozen."
 *
 * New behavior:
 *   - ui_show_message() renders the message and sets a deadline. The main
 *     loop continues, polling buttons and IMU.
 *   - ui_update_display() and ui_update_auto_status_line_only() are gated:
 *     while a toast is active they leave the screen alone; once the toast
 *     expires the main loop forces a full repaint, which restores the
 *     normal screen.
 *   - Button presses (next-screen / execute) cancel any active toast so
 *     the user always sees an immediate response.
 *
 * Long-running BLE operations (manual connect, wake-broadcast, background
 * reconnect) are dispatched to a single worker task that pulls jobs from
 * a queue. Each job is allowed to use vTaskDelay()/blocking BLE calls
 * freely; the main loop is unaffected.
 *
 * See SPEC.md "Audible Cues" + "Auto Start/Stop Screen" for the
 * user-visible behavior this enables.
 * ────────────────────────────────────────────────────────────────────────── */

static volatile TickType_t s_toast_deadline   = 0;     /* 0 = no toast */
static volatile bool       s_toast_was_drawn  = false;

typedef enum {
    UI_WORK_BG_RECONNECT = 0,  /* quiet retry, no toasts */
    UI_WORK_UI_RECONNECT,      /* manual button — show toasts on each step */
    UI_WORK_PAIR,              /* fall-back to fresh pairing */
    UI_WORK_WAKE,              /* wake-broadcast + post-wake reconnect */
} ui_work_t;

static QueueHandle_t   s_work_queue   = NULL;
static TaskHandle_t    s_worker_task  = NULL;
static SemaphoreHandle_t s_disp_mutex = NULL;

static int  ui_perform_complete_reconnection(bool show_messages);
static void ui_try_manual_pairing(void);
static void ui_screen_connect_worker(void);
static void ui_screen_wake_worker(void);
static void ui_workers_init(void);
static void ui_dispatch_work(ui_work_t job);

static inline bool ui_toast_active(void) {
    if (s_toast_deadline == 0) return false;
    /* Signed-difference idiom handles tick wrap: positive = deadline in future. */
    return (int32_t)(s_toast_deadline - xTaskGetTickCount()) > 0;
}

static void ui_cancel_toast(void) {
    s_toast_deadline = 0;
    s_toast_was_drawn = false;
}

/* Forward declarations for partial-update cache invalidation helper. */
static void ui_auto_status_cache_invalidate(void);

#if defined(M5ATOMS3)
/* Forward declaration for the AtomS3 minimal renderer (defined further down
 * in this file under #ifdef M5ATOMS3). Both ui_update_display() and
 * ui_update_auto_status_line_only() short-circuit to atoms3_render() when
 * built for AtomS3. */
static void atoms3_render(bool force_full);
#endif

/* Logging tag for ESP_LOG functions */
#define TAG "UI"

/* NVS (Non-Volatile Storage) configuration for persistent data */
#define NVS_CAMERA_NAMESPACE "camera"      /* Namespace for camera pairing data */
#define NVS_CAMERA_KEY "paired_info"       /* Key for stored camera information */
#define NVS_DEVICE_NAMESPACE "device"      /* Namespace for device configuration */
#define NVS_DEVICE_ID_KEY "device_id"      /* Key for unique device identifier */

/* GPIO pin definitions for external hardware triggers
 * These pins allow external devices to trigger camera functions
 * with randomized delays based on the device's unique ID
 */
#define GPIO_SHUTTER_PIN    0   /* G0 - Pull LOW to trigger SHUTTER (photo/video) */
#define GPIO_SLEEP_PIN     26   /* G26 - Pull HIGH to trigger SLEEP mode */
#define GPIO_WAKE_PIN      25   /* G25 - Pull HIGH to trigger WAKE from sleep */

/* Enumeration of GPIO trigger types for external hardware integration
 * These correspond to the three main camera functions that can be
 * triggered via GPIO pins with randomized delays
 */
typedef enum {
    GPIO_TRIGGER_SHUTTER = 0,    /* Photo capture or video recording toggle */
    GPIO_TRIGGER_SLEEP,          /* Put camera into sleep mode */
    GPIO_TRIGGER_WAKE            /* Wake camera from sleep mode */
} gpio_trigger_type_t;

/* Camera information storage structure for persistent pairing data
 * This structure contains all necessary information to reconnect to
 * a previously paired camera without requiring manual pairing
 */
typedef struct {
    bool is_paired;                 /* True if camera has been successfully paired */
    char camera_name[64];           /* Camera BLE advertising name (e.g., "OsmoAction5Pro1C59") */
    uint8_t camera_mac[6];          /* Camera's BLE MAC address for targeted connection */
    uint32_t device_id;             /* Protocol-level device identifier from camera */
    uint8_t mac_addr_len;           /* Length of protocol MAC address (typically 6) */
    int8_t mac_addr[6];             /* Protocol-level MAC address from camera handshake */
    uint32_t fw_version;            /* Camera firmware version for compatibility checks */
    uint16_t verify_data;           /* Last successful verification code for reconnection */
    uint8_t camera_reserved;        /* Camera identifier number in multi-camera setups */
} stored_camera_t;

/* Global camera storage - holds information about the currently paired camera
 * This data is synchronized with NVS for persistence across power cycles
 */
static stored_camera_t g_stored_camera = {0};

/* Global DJI protocol connection parameters
 * These variables are used for camera communication and are either
 * generated randomly (device_id) or received from the camera during handshake
 */
uint32_t g_device_id = 0x12345678;                           /* Unique device identifier (randomized on first boot) */
uint8_t g_mac_addr_len = 6;                                  /* MAC address length (always 6 for BLE) */
int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC}; /* Protocol MAC address from remote device */
uint32_t g_fw_version = 0x00;                                /* Firmware version for compatibility */
uint8_t g_verify_mode = 0;                                   /* Authentication mode: 0=reconnect, 1=pair */
uint16_t g_verify_data = 0;                                  /* Random verification code for security */
uint8_t g_camera_reserved = 0;                               /* Camera number in multi-camera environments */

/* Global user interface state management
 * Tracks current screen, display update requirements, and device-specific
 * scaling parameters for proper rendering on different M5StickC variants
 */
ui_state_t g_ui_state = {
    .current_screen = SCREEN_CONNECT,    /* Start on connection screen */
    .display_needs_update = true,        /* Force initial display update */
    .is_plus2_device = false,           /* Detected device type (set during init) */
    .scale_factor = 1.0f,               /* Display scaling factor for text/graphics */
    .scaled_text_size = 1               /* Text size multiplier for readability */
};

bool g_pending_set_video_mode_after_connect = false;

/* Screen layout configuration - positions and sizes for UI elements
 * Automatically configured based on detected device type and screen resolution
 */
screen_layout_t g_layout = {0};

/**
 * @brief Save camera pairing information to non-volatile storage
 * 
 * Stores complete camera information including BLE details and protocol
 * parameters to NVS for automatic reconnection after device restart.
 * 
 * @param camera Pointer to camera information structure to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t save_camera_to_nvs(const stored_camera_t* camera) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, NVS_CAMERA_KEY, camera, sizeof(stored_camera_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving camera to NVS: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Camera info saved to NVS successfully");
        }
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Generate a unique random device identifier
 * 
 * Creates a 32-bit random device ID using ESP32's hardware random number
 * generator. This ID is used for DJI protocol communication and GPIO
 * trigger delay calculation to prevent interference between multiple devices.
 * 
 * @return 32-bit random device identifier (never zero)
 */
static uint32_t generate_random_device_id(void) {
    /* Use ESP32's hardware-based random number generator for cryptographic quality */
    uint32_t random_id = esp_random();
    
    /* Ensure ID is never zero to avoid protocol issues */
    if (random_id == 0) {
        random_id = 0x12345678;  /* Fallback pattern if hardware RNG fails */
    }
    
    ESP_LOGI(TAG, "Generated random device ID: 0x%08X", (unsigned int)random_id);
    return random_id;
}

/**
 * @brief Save device ID to non-volatile storage for persistence
 * 
 * Stores the device's unique identifier to NVS so it remains consistent
 * across power cycles and reboots. This ensures GPIO trigger delays and
 * protocol communication remain stable.
 * 
 * @param device_id The 32-bit device identifier to save
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t save_device_id_to_nvs(uint32_t device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving device ID to NVS: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error committing device ID to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Device ID 0x%08X saved to NVS successfully", (unsigned int)device_id);
        }
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Load device ID from non-volatile storage
 * 
 * Retrieves the previously stored device identifier from NVS.
 * Returns ESP_ERR_NVS_NOT_FOUND if this is the first boot.
 * 
 * @param device_id Pointer to store the loaded device ID
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_device_id_from_nvs(uint32_t* device_id) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_DEVICE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening device NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_u32(nvs_handle, NVS_DEVICE_ID_KEY, device_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Device ID not found in NVS, this is a first boot");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error loading device ID from NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device ID 0x%08X loaded from NVS successfully", (unsigned int)*device_id);
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Initialize device ID on system startup
 * 
 * Handles device ID initialization by either loading an existing ID from NVS
 * or generating a new random ID on first boot. The device ID is used for:
 * - DJI protocol communication
 * - GPIO trigger delay calculation
 * - Multi-device interference prevention
 */
static void initialize_device_id(void) {
    uint32_t stored_device_id;
    esp_err_t err = load_device_id_from_nvs(&stored_device_id);
    
    ESP_LOGI(TAG, "Device ID initialization starting, current g_device_id: 0x%08X", (unsigned int)g_device_id);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - generate new random device ID
        ESP_LOGI(TAG, "First boot detected, generating random device ID...");
        g_device_id = generate_random_device_id();
        
        // Save to NVS for future boots
        err = save_device_id_to_nvs(g_device_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save device ID to NVS, using session-only ID");
        }
    } else if (err == ESP_OK) {
        // Device ID found in NVS - use it
        g_device_id = stored_device_id;
        ESP_LOGI(TAG, "Using stored device ID: 0x%08X", (unsigned int)g_device_id);
    } else {
        // Error loading from NVS - use default but try to save random one
        ESP_LOGW(TAG, "Error loading device ID from NVS, generating new one");
        g_device_id = generate_random_device_id();
        save_device_id_to_nvs(g_device_id);  // Try to save for next time
    }
    
    ESP_LOGI(TAG, "Device ID initialization complete, final g_device_id: 0x%08X", (unsigned int)g_device_id);
}

/* GPIO control system components for external hardware integration
 * Implements thread-safe GPIO trigger processing with debouncing and delays
 */
static QueueHandle_t gpio_trigger_queue = NULL;                              /* FreeRTOS queue for GPIO events */
static TickType_t last_gpio_trigger[3] = {0, 0, 0};                         /* Per-pin debouncing timestamps */
static TickType_t last_global_gpio_trigger = 0;                             /* Global cooldown timestamp */
static volatile gpio_trigger_type_t pending_gpio_action = (gpio_trigger_type_t)-1; /* Pending action for main thread */

/**
 * @brief Calculate device-specific GPIO trigger delay
 * 
 * Generates a consistent but pseudo-random delay (1-100ms) based on the device's
 * unique ID. This prevents multiple devices from triggering simultaneously when
 * connected to the same external trigger source, reducing RF interference and
 * improving reliability in multi-device deployments.
 * 
 * @return Delay in milliseconds (1-100ms range)
 */
static uint32_t calculate_gpio_delay_ms(void) {
    /* Safety check: ensure device ID is initialized */
    if (g_device_id == 0) {
        ESP_LOGW(TAG, "Device ID not initialized, using default 50ms delay");
        return 50;
    }
    
    /* Use lower 16 bits of device ID as entropy source for consistent randomization */
    uint16_t id_hash = (uint16_t)(g_device_id & 0xFFFF);
    
    /* Map 16-bit hash (0-65535) to delay range (1-100ms) with minimum 1ms guarantee */
    uint32_t delay_ms = 1 + ((id_hash * 99) / 65535);
    
    ESP_LOGI(TAG, "GPIO trigger delay calculated: %lu ms (based on device ID 0x%08X)", 
             delay_ms, (unsigned int)g_device_id);
    
    return delay_ms;
}

/* GPIO debouncing and cooldown configuration
 * Aggressive timing to prevent multiple triggers from mechanical switch bounce
 * and to ensure stable operation with external hardware
 */
#define GPIO_DEBOUNCE_TIME_MS 1000      /* Per-pin debouncing period (1 second) */
#define GPIO_GLOBAL_COOLDOWN_MS 1000    /* Global cooldown between any GPIO triggers (1 second) */

/**
 * @brief GPIO interrupt service routine
 * 
 * Handles GPIO pin state changes by queuing trigger events for processing
 * in the main task context. The ISR must be kept minimal and fast, so all
 * actual processing is deferred to the GPIO monitor task.
 * 
 * @param arg Pointer to gpio_trigger_type_t indicating which trigger type
 */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_trigger_type_t trigger_type = (gpio_trigger_type_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send trigger type to queue for processing in main task
    xQueueSendFromISR(gpio_trigger_queue, &trigger_type, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Process GPIO trigger with debouncing and delay calculation
 * 
 * Implements comprehensive GPIO trigger processing including:
 * - Per-pin and global debouncing to prevent multiple triggers
 * - Device ID-based delay calculation for multi-device coordination
 * - Thread-safe pending action system for main thread execution
 * 
 * @param trigger_type Type of GPIO trigger to process
 */
static void process_gpio_trigger(gpio_trigger_type_t trigger_type) {
    TickType_t current_time = xTaskGetTickCount();
    TickType_t debounce_ticks = pdMS_TO_TICKS(GPIO_DEBOUNCE_TIME_MS);
    TickType_t global_cooldown_ticks = pdMS_TO_TICKS(GPIO_GLOBAL_COOLDOWN_MS);
    
    // Check for global cooldown (any GPIO trigger within 1 second)
    if ((current_time - last_global_gpio_trigger) < global_cooldown_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to global cooldown", trigger_type);
        return;
    }
    
    // Check for per-pin debouncing
    if (trigger_type < 3 && (current_time - last_gpio_trigger[trigger_type]) < debounce_ticks) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored due to pin debouncing", trigger_type);
        return;
    }
    
    // Check if there's already a pending action
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        ESP_LOGW(TAG, "GPIO trigger %d ignored, action already pending", trigger_type);
        return;
    }
    
    // Update timestamps
    last_global_gpio_trigger = current_time;
    if (trigger_type < 3) {
        last_gpio_trigger[trigger_type] = current_time;
    }
    
    uint32_t delay_ms = calculate_gpio_delay_ms();
    
    ESP_LOGI(TAG, "GPIO trigger %d processing, delaying %lu ms", trigger_type, delay_ms);
    
    // Wait for the calculated delay
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Set the pending action for main thread to execute
    pending_gpio_action = trigger_type;
    ESP_LOGI(TAG, "GPIO trigger %d ready for execution by main thread", trigger_type);
}

/**
 * @brief GPIO monitoring task - processes queued GPIO events
 * 
 * FreeRTOS task that receives GPIO trigger events from the ISR queue
 * and processes them with appropriate delays and debouncing. Runs
 * continuously waiting for events.
 * 
 * @param pvParameters Unused task parameter (required by FreeRTOS)
 */
static void gpio_monitor_task(void* pvParameters) {
    gpio_trigger_type_t trigger_type;
    
    while (1) {
        if (xQueueReceive(gpio_trigger_queue, &trigger_type, portMAX_DELAY)) {
            process_gpio_trigger(trigger_type);
        }
    }
}

/**
 * @brief Initialize the complete GPIO trigger system
 * 
 * Sets up GPIO pins, interrupt handlers, debouncing system, and monitoring task.
 * Configures:
 * - G0 (SHUTTER): Input with pull-up, trigger on falling edge (pull to LOW)
 * - G26 (SLEEP): Input with pull-down, trigger on rising edge (pull to HIGH)
 * - G25 (WAKE): Input with pull-down, trigger on rising edge (pull to HIGH)
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
static esp_err_t init_gpio_system(void) {
#if defined(M5STICKS3) || defined(M5ATOMS3)
    /* ESP32-S3 family (StickS3 / AtomS3): GPIO 25–32 are tied to SPI flash / OPI PSRAM
     * on StickS3 and the AtomS3 routes its 6 user GPIOs differently. The Plus2-style
     * G0/G25/G26 external-trigger wiring does not apply here; the single screen button
     * (AtomS3) or A/B buttons (StickS3) drive UI navigation instead. */
    ESP_LOGI(TAG, "ESP32-S3 build: external GPIO triggers skipped (no Plus2 G0/G25/G26 routing)");
    return ESP_OK;
#else
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing GPIO trigger system");
    ESP_LOGI(TAG, "Current device ID: 0x%08X", (unsigned int)g_device_id);
    
    /* Create FreeRTOS queue for GPIO trigger events (ISR to task communication) */
    gpio_trigger_queue = xQueueCreate(10, sizeof(gpio_trigger_type_t));
    if (gpio_trigger_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO trigger queue");
        return ESP_FAIL;
    }
    
    /* Initialize debouncing timestamps and pending action state */
    for (int i = 0; i < 3; i++) {
        last_gpio_trigger[i] = 0;
    }
    last_global_gpio_trigger = 0;
    pending_gpio_action = (gpio_trigger_type_t)-1;
    
    /* Configure GPIO pins with appropriate pull resistors and interrupt types */
    gpio_config_t io_conf = {};
    
    /* G0 (SHUTTER) - Input with internal pull-up resistor
     * External circuit should pull pin LOW to trigger shutter function
     */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;      /* Interrupt on falling edge (HIGH to LOW) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SHUTTER_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SHUTTER GPIO pin");
        return ret;
    }
    
    /* G26 (SLEEP) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger sleep function
     */
    io_conf.intr_type = GPIO_INTR_POSEDGE;      /* Interrupt on rising edge (LOW to HIGH) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_SLEEP_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SLEEP GPIO pin");
        return ret;
    }
    
    /* G25 (WAKE) - Input with internal pull-down resistor
     * External circuit should pull pin HIGH to trigger wake function
     */
    io_conf.intr_type = GPIO_INTR_POSEDGE;      /* Interrupt on rising edge (LOW to HIGH) */
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_WAKE_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure WAKE GPIO pin");
        return ret;
    }
    
    /* Install GPIO interrupt service if not already installed */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  /* ESP_ERR_INVALID_STATE = already installed */
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }
    
    /* Register interrupt handlers for each GPIO pin with trigger type identification */
    ret = gpio_isr_handler_add(GPIO_SHUTTER_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SHUTTER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHUTTER GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_SLEEP_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SLEEP GPIO ISR handler");
        return ret;
    }
    
    ret = gpio_isr_handler_add(GPIO_WAKE_PIN, gpio_isr_handler, (void*)GPIO_TRIGGER_WAKE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add WAKE GPIO ISR handler");
        return ret;
    }
    
    /* Create FreeRTOS task for GPIO event processing with sufficient stack size */
    BaseType_t task_ret = xTaskCreate(gpio_monitor_task, "gpio_monitor", 4096, NULL, 10, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "GPIO trigger system initialized successfully");
    ESP_LOGI(TAG, "  G0 (SHUTTER): Pull LOW to trigger");
    ESP_LOGI(TAG, "  G26 (SLEEP): Pull HIGH to trigger");
    ESP_LOGI(TAG, "  G25 (WAKE): Pull HIGH to trigger");
    
    return ESP_OK;
#endif /* !(M5STICKS3 || M5ATOMS3) */
}

/**
 * @brief Process pending GPIO actions from main thread
 * 
 * This function must be called regularly from the main application loop
 * to execute GPIO-triggered actions in the main thread context. This
 * ensures thread safety by avoiding UI function calls from GPIO tasks.
 */
void ui_process_pending_gpio_actions(void) {
    if (pending_gpio_action != (gpio_trigger_type_t)-1) {
        gpio_trigger_type_t action = pending_gpio_action;
        pending_gpio_action = (gpio_trigger_type_t)-1; // Clear the pending action first
        
        ESP_LOGI(TAG, "Executing pending GPIO action: %d", action);
        
        switch (action) {
            case GPIO_TRIGGER_SHUTTER:
                ui_screen_shutter();
                break;
                
            case GPIO_TRIGGER_SLEEP:
                ui_screen_sleep();
                break;
                
            case GPIO_TRIGGER_WAKE:
                ui_screen_wake();
                break;
        }
    }
}

/**
 * @brief Load camera pairing information from non-volatile storage
 * 
 * Retrieves previously stored camera information from NVS for automatic
 * reconnection capabilities. Returns ESP_ERR_NVS_NOT_FOUND if no camera
 * has been paired yet.
 * 
 * @param camera Pointer to structure to fill with loaded camera data
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found, ESP_ERR_* on other failures
 */
static esp_err_t load_camera_from_nvs(stored_camera_t* camera) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size = sizeof(stored_camera_t);
    
    err = nvs_open(NVS_CAMERA_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_get_blob(nvs_handle, NVS_CAMERA_KEY, camera, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera info loaded from NVS successfully");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No camera info found in NVS");
        // Initialize with default values
        memset(camera, 0, sizeof(stored_camera_t));
    } else {
        ESP_LOGE(TAG, "Error loading camera from NVS: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Store camera information after successful pairing
 * 
 * Captures and stores complete camera information including BLE details
 * and protocol parameters for future automatic reconnection. Called
 * automatically after successful camera pairing.
 */
static void store_camera_info(void) {
    g_stored_camera.is_paired = true;
    
    /* Capture BLE connection information from currently connected camera */
    if (ble_get_connected_device_info(g_stored_camera.camera_name, 
                                      sizeof(g_stored_camera.camera_name), 
                                      g_stored_camera.camera_mac)) {
        ESP_LOGI(TAG, "Captured camera BLE info: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
                 g_stored_camera.camera_name,
                 g_stored_camera.camera_mac[0], g_stored_camera.camera_mac[1], 
                 g_stored_camera.camera_mac[2], g_stored_camera.camera_mac[3],
                 g_stored_camera.camera_mac[4], g_stored_camera.camera_mac[5]);
    } else {
        ESP_LOGW(TAG, "Failed to get camera BLE info, using default values");
        strncpy(g_stored_camera.camera_name, "Unknown Camera", sizeof(g_stored_camera.camera_name) - 1);
        memset(g_stored_camera.camera_mac, 0, sizeof(g_stored_camera.camera_mac));
    }
    
    /* Store DJI protocol-level information obtained during connection handshake */
    g_stored_camera.device_id = g_device_id;
    g_stored_camera.mac_addr_len = g_mac_addr_len;
    memcpy(g_stored_camera.mac_addr, g_mac_addr, sizeof(g_mac_addr));
    g_stored_camera.fw_version = g_fw_version;
    g_stored_camera.verify_data = g_verify_data;
    g_stored_camera.camera_reserved = g_camera_reserved;
    
    /* Write all information to persistent NVS storage */
    esp_err_t err = save_camera_to_nvs(&g_stored_camera);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera information stored persistently for future auto-connect");
    } else {
        ESP_LOGE(TAG, "Failed to store camera info persistently");
    }
}

/**
 * @brief Load stored camera information for reconnection
 * 
 * Restores global protocol variables from stored camera information
 * to enable reconnection using previously established parameters.
 */
static void load_stored_camera_info(void) {
    if (g_stored_camera.is_paired) {
        g_device_id = g_stored_camera.device_id;
        g_mac_addr_len = g_stored_camera.mac_addr_len;
        memcpy(g_mac_addr, g_stored_camera.mac_addr, sizeof(g_stored_camera.mac_addr));
        g_fw_version = g_stored_camera.fw_version;
        g_verify_data = g_stored_camera.verify_data;
        g_camera_reserved = g_stored_camera.camera_reserved;
        ESP_LOGI(TAG, "Loaded stored camera info for reconnection");
    }
}

/**
 * @brief Attempt automatic connection to paired camera on startup
 * 
 * Called during system initialization to automatically connect to a
 * previously paired camera if one exists. Uses stored BLE and protocol
 * information to establish connection without user intervention.
 */
/**
 * @brief Perform complete camera reconnection (BLE + Protocol)
 * 
 * Internal function that handles both BLE and protocol reconnection.
 * Used by both startup auto-connect and background reconnection.
 * 
 * @param show_messages Whether to show UI status messages
 * @return int 0 on success, -1 on failure
 */
static int ui_perform_complete_reconnection(bool show_messages) {
    if (!g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "No paired camera found, cannot reconnect");
        return -1;
    }
    if (connect_logic_get_state() == BLE_NOT_INIT) {
        ESP_LOGW(TAG, "BLE stack not initialized — skip reconnect");
        return -1;
    }

    ESP_LOGI(TAG, "Performing complete reconnection");
    if (show_messages) {
        ui_show_message("Reconnecting...", M5_COLOR_CYAN, 1000);
    }
    
    /* Configure BLE layer to target the specific paired camera */
    ble_set_target_device(g_stored_camera.camera_name, g_stored_camera.camera_mac);
    
    load_stored_camera_info();
    g_verify_mode = 0;  /* Use reconnection mode (no pairing required) */
    
    /* Initiate BLE connection with reconnection flag */
    int res = connect_logic_ble_connect(true);  /* is_reconnecting = true */
    if (res == 0) {
        if (show_messages) {
            ui_show_message("BLE Connected\nConnecting protocol...", M5_COLOR_BLUE, 1000);
        }
        
        /* Establish DJI protocol connection using stored parameters */
        res = connect_logic_protocol_connect(
            g_device_id,
            g_mac_addr_len,
            g_mac_addr,
            g_fw_version,
            g_verify_mode,
            g_verify_data,
            g_camera_reserved
        );
        
        
        
        
        
        
        
        
        
        
        
        
        if (res == 0) {
            if (show_messages) {
                ui_show_message("Reconnected!", M5_COLOR_GREEN, 1000);
            }
            g_pending_set_video_mode_after_connect = true;
            subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            g_ui_state.current_screen = SCREEN_AUTO;
            g_ui_state.display_needs_update = true;
            ESP_LOGI(TAG, "Complete reconnection successful");
            return 0;
        } else {
            if (show_messages) {
                ui_show_message("Protocol connect failed", M5_COLOR_RED, 1500);
            }
            ESP_LOGW(TAG, "Protocol connection failed during reconnection");
        }
    } else {
        if (show_messages) {
            ui_show_message("BLE connect failed", M5_COLOR_RED, 1500);
        }
        ESP_LOGW(TAG, "BLE connection failed during reconnection");
    }
    return -1;
}

/** Stack for BLE + protocol handshake — too deep for app_main (see sdkconfig main stack). */
#define UI_AUTO_CONNECT_TASK_STACK  8192

static void ui_auto_connect_task(void *arg) {
    (void)arg;
    /* Let display init + BLE controller finish; avoids WDT / stack pressure on app_main. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    if (g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "Deferred auto-connect starting");
        (void)ui_perform_complete_reconnection(true);
    }
    vTaskDelete(NULL);
}

void ui_auto_connect_on_startup(void) {
    if (g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "Scheduling deferred auto-connect (paired camera)");
        BaseType_t ok = xTaskCreate(ui_auto_connect_task, "ui_auto_conn", UI_AUTO_CONNECT_TASK_STACK,
                                    NULL, 5, NULL);
        if (ok != pdPASS) {
            ESP_LOGW(TAG, "auto-connect task create failed — skipping startup reconnect");
        }
    } else {
        ESP_LOGI(TAG, "No paired camera found, manual pairing required");
    }
}

/* ── Single shared worker for long-running BLE / protocol operations ──────────
 *
 * Why this exists: connect_logic_ble_connect() blocks for up to ~20 s, and
 * connect_logic_protocol_connect() adds another ~30 s in worst case. Doing
 * either on the main task froze button input, IMU sampling, GPS push, and
 * UI redraws for the full duration. The worker drains a small job queue so
 * those operations always run off-main-task.
 *
 * Queue depth 4 + a single worker means duplicate job requests (e.g. the
 * 15 s background-reconnect timer firing while a worker is already busy)
 * are allowed to queue but not run in parallel. Per-job handlers re-check
 * connection state up front, so a queued reconnect that arrives after a
 * successful connect just bails harmlessly. */
static void ui_worker_task(void *arg) {
    (void)arg;
    ui_work_t job;
    while (xQueueReceive(s_work_queue, &job, portMAX_DELAY) == pdTRUE) {
        switch (job) {
        case UI_WORK_BG_RECONNECT:
            (void)ui_perform_complete_reconnection(false);
            break;
        case UI_WORK_UI_RECONNECT:
            ui_screen_connect_worker();
            break;
        case UI_WORK_PAIR:
            ui_try_manual_pairing();
            break;
        case UI_WORK_WAKE:
            ui_screen_wake_worker();
            break;
        default:
            break;
        }
    }
}

static void ui_workers_init(void) {
    if (s_work_queue != NULL) return;
    s_work_queue = xQueueCreate(4, sizeof(ui_work_t));
    if (s_work_queue == NULL) {
        ESP_LOGE(TAG, "ui_workers_init: failed to create work queue");
        return;
    }
    BaseType_t ok = xTaskCreate(ui_worker_task, "ui_worker", UI_AUTO_CONNECT_TASK_STACK,
                                NULL, 4, &s_worker_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "ui_workers_init: failed to create worker task");
        s_worker_task = NULL;
    } else {
        ESP_LOGI(TAG, "UI worker task created (depth 4, prio 4)");
    }
}

static void ui_dispatch_work(ui_work_t job) {
    if (s_work_queue == NULL) {
        ESP_LOGW(TAG, "ui_dispatch_work: queue not initialized — running inline (will block)");
        switch (job) {
        case UI_WORK_BG_RECONNECT:  (void)ui_perform_complete_reconnection(false); break;
        case UI_WORK_UI_RECONNECT:  ui_screen_connect_worker(); break;
        case UI_WORK_PAIR:          ui_try_manual_pairing(); break;
        case UI_WORK_WAKE:          ui_screen_wake_worker(); break;
        }
        return;
    }
    if (xQueueSend(s_work_queue, &job, 0) != pdTRUE) {
        ESP_LOGW(TAG, "ui_dispatch_work: queue full, dropping job %d", (int)job);
    }
}

/**
 * @brief Attempt background reconnection without UI messages
 *
 * Public function for background reconnection attempts. Dispatches the
 * actual BLE + protocol work to the UI worker task so the calling task
 * (typically app_main's main loop) returns immediately.
 *
 * @return int 0 on success or queued, -1 if no paired camera / BLE not ready
 */
int ui_attempt_background_reconnection(void) {
    /* Only attempt reconnection if we're disconnected but initialized */
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_SEARCHING) {
        /* Already connected or connecting */
        return 0;
    }

    if (current_state < BLE_INIT_COMPLETE) {
        /* BLE not initialized, cannot reconnect */
        return -1;
    }

    if (!g_stored_camera.is_paired) {
        /* No paired camera to reconnect to */
        return -1;
    }

    ESP_LOGI(TAG, "Background reconnection dispatched to worker");
    ui_dispatch_work(UI_WORK_BG_RECONNECT);
    return 0;
}

/**
 * @brief Initialize the complete user interface system
 * 
 * Performs comprehensive UI system initialization including:
 * - NVS (Non-Volatile Storage) initialization
 * - Device ID generation/loading
 * - Camera pairing data loading
 * - Data layer initialization
 * - Display configuration
 * - Automatic connection attempts
 * - GPIO trigger system setup
 */
void ui_init(void) {
    ESP_LOGI(TAG, "Initializing UI system");
    s_disp_mutex = xSemaphoreCreateMutex();
    
    /* Initialize NVS flash storage for persistent data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    /* Initialize unique device identifier (random generation on first boot) */
    initialize_device_id();
    
    /* Load any previously paired camera information from persistent storage */
    esp_err_t load_err = load_camera_from_nvs(&g_stored_camera);
    if (load_err == ESP_OK && g_stored_camera.is_paired) {
        ESP_LOGI(TAG, "Found paired camera in storage: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)", 
                 g_stored_camera.camera_name,
                 g_stored_camera.camera_mac[0], g_stored_camera.camera_mac[1],
                 g_stored_camera.camera_mac[2], g_stored_camera.camera_mac[3],
                 g_stored_camera.camera_mac[4], g_stored_camera.camera_mac[5]);
    } else {
        ESP_LOGI(TAG, "No paired camera found in storage");
    }
    
    /* Initialize DJI protocol data layer with status update callbacks */
    if (!is_data_layer_initialized()) {
        ESP_LOGI(TAG, "Initializing data layer...");
        data_init();
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
        if (!is_data_layer_initialized()) {
            /* Data layer failed — still initialise the display so the user sees
             * the UI (which will show "Not Connected") rather than a blank screen.
             * Camera commands will not work until the data layer is available. */
            ESP_LOGE(TAG, "UI init aborted: data layer failed to initialize — camera commands unavailable");
            ui_detect_device_and_set_scale();
            g_ui_state.current_screen = SCREEN_CONNECT;
            g_ui_state.display_needs_update = true;
            ui_update_display();
            return;
        }
        ESP_LOGI(TAG, "Data layer initialized successfully");
    }

    /* Configure display scaling and layout for detected hardware */
    ui_detect_device_and_set_scale();
    g_ui_state.current_screen = SCREEN_CONNECT;
    g_ui_state.display_needs_update = true;

    /* Bring up the worker task BEFORE auto-connect so any deferred work
     * dispatched from button presses during the auto-connect window has
     * a worker to run on. */
    ui_workers_init();

    /* Attempt automatic connection to previously paired camera */
    ui_auto_connect_on_startup();
    
    /* Perform initial display update */
    ui_update_display();
    
    /* Initialize GPIO trigger system after all other subsystems are ready */
    esp_err_t gpio_err = init_gpio_system();
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPIO system: %s", esp_err_to_name(gpio_err));
        ESP_LOGW(TAG, "Continuing without GPIO triggers");
    }
}

/**
 * @brief Detect M5StickC device type and configure display scaling
 * 
 * Determines the specific M5StickC variant and configures appropriate
 * scaling factors for optimal display rendering. Currently configured
 * for M5StickC Plus2 (240x135 resolution).
 */
void ui_detect_device_and_set_scale(void) {
#if defined(M5ATOMS3)
    /* AtomS3: 128x128 panel. The Plus2/StickS3 layout (240x135) does not fit;
     * we use a compact two-screen layout (BT, AUTO) with smaller coordinates.
     * is_plus2_device is left false so existing scale-1 code paths apply. */
    g_ui_state.is_plus2_device = false;
    g_ui_state.scale_factor = 1.0f;
    g_ui_state.scaled_text_size = 2;

    ESP_LOGI(TAG, "Detected: M5 AtomS3 (128x128)");

    /* Layout for 128x128. Icons (32px) center at x=48; navigation dots are
     * intentionally pushed off-screen (y>=128) so the dot loop in
     * ui_update_display() effectively becomes a no-op without further #ifdef. */
    g_layout.icon_x = (128 - 32) / 2;
    g_layout.icon_y = 22;
    g_layout.text_x = 128 / 2;
    g_layout.text_y = g_layout.icon_y + 32 + 6;
    g_layout.status_x = 110;
    g_layout.status_y = 4;
    g_layout.connection_radius = 5;
    g_layout.dots_y = 200;        /* off-screen — no nav dots on 128x128 */
    g_layout.dots_spacing = 20;
    g_layout.dots_start_x = 20;
    g_layout.instruct_x = 4;
    g_layout.instruct_y = 4;
#else
    /* M5StickC Plus2: 240x135 display, original M5StickC: 160x80 display
     * Currently targeting Plus2 hardware exclusively
     */
    g_ui_state.is_plus2_device = true;
    g_ui_state.scale_factor = 1.5f;
    g_ui_state.scaled_text_size = 2;

    ESP_LOGI(TAG, "Detected: M5StickC Plus2 (240x135)");
    ESP_LOGI(TAG, "Scale factor: %.1f, Text size: %d",
             g_ui_state.scale_factor, g_ui_state.scaled_text_size);

    /* Configure screen layout coordinates for M5StickC Plus2 (240x135 resolution)
     * All positions calculated for optimal visual balance and readability
     */
    g_layout.icon_x = (240 - 32) / 2;           /* Center 32px icons horizontally (104px from left) */
    g_layout.icon_y = 25 + 12;                  /* Icon vertical position with offset */
    g_layout.text_x = 240 / 2;                  /* Center text horizontally (120px from left) */
    g_layout.text_y = g_layout.icon_y + 32 + 12; /* Text below icon with spacing for double-size text */
    g_layout.status_x = 220;                    /* Connection status indicator position */
    g_layout.status_y = 12;
    g_layout.connection_radius = 7;             /* Size of connection status indicator */
    g_layout.dots_y = 128;                      /* Screen indicator dots vertical position */
    g_layout.dots_spacing = 25;                 /* Horizontal spacing between screen dots */
    g_layout.dots_start_x = 45;                 /* Starting x position for screen dots */
    g_layout.instruct_x = 8;                    /* Instruction text position */
    g_layout.instruct_y = 8;
#endif
}

/**
 * @brief Draw bitmap icon at specified position
 * 
 * Renders a bitmap icon with specified dimensions and color using the
 * hardware-specific display driver functions.
 * 
 * @param x Horizontal position
 * @param y Vertical position  
 * @param bitmap Pointer to bitmap data array
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Foreground color (RGB565 format)
 */
void ui_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
    /* Use hardware-optimized bitmap rendering with black background */
    m5stickc_plus2_display_draw_bitmap(x, y, w, h, bitmap, color, M5_COLOR_BLACK);
}

/**
 * @brief Draw the BLE connection indicator in the top-right corner.
 *
 * Layout (right-aligned, y=2):  "BT" [■]
 *   - Small label "BT" so the user can tell what the colored block represents
 *     (without it the bare square is just an unexplained dot).
 *   - 8×8 filled square next to it; same color as the label.
 *
 * Colors are chosen from the ICON_* set rather than M5_COLOR_*: this panel uses
 * BGR element ordering, and we have empirical evidence (the AUTO icon renders
 * green on screen) that ICON_GREEN actually shows as green here, while
 * M5_COLOR_GREEN renders as something else.  ICON_RED is the conventional R
 * channel that does render red on the visible side.
 *
 * The full bounding box (label + dot) is wiped to black first so a previous
 * wider/taller render can never leave a stray pixel ("white dot in the upper
 * square") behind.
 */
void ui_draw_connection_status(void) {
    connect_state_t state = connect_logic_get_state();
    bool connected = (state == PROTOCOL_CONNECTED);

    /* Bounding box for label + square. Right-aligned at x=237 (3 px from edge). */
    const int box_right = 237;
    const int box_y     = 2;
    const int box_h     = 12;
    const int label_w   = 16;          /* "BT" at scale 1 = 2 chars × 8 px */
    const int dot_w     = 8;
    const int gap       = 2;
    const int box_w     = label_w + gap + dot_w;       /* 26 px wide */
    const int box_left  = box_right - box_w;           /* x=211 */

    /* Clear the whole bounding box first to avoid any leftover pixels. */
    m5stickc_plus2_display_fill_rect(box_left, box_y, box_w, box_h, M5_COLOR_BLACK);

    uint16_t color = connected ? M5_TRUE_GREEN : M5_TRUE_RED;

    /* "BT" label */
    m5stickc_plus2_display_print(box_left, box_y + 2, "BT", color);

    /* Filled square next to label */
    m5stickc_plus2_display_fill_rect(box_left + label_w + gap, box_y + 2,
                                     dot_w, dot_w, color);
}

/**
 * @brief Calculate text width for centering calculations
 * 
 * Estimates the rendered width of text based on character count and scaling.
 * Used for centering text on the display.
 * 
 * @param text Text string to measure
 * @param text_size Text scaling factor
 * @return Estimated text width in pixels
 */
int ui_get_text_width(const char* text, int text_size) {
    /* Calculate width based on fixed-width font assumption (8 pixels base * scale) */
    int char_width = 8 * text_size;
    return strlen(text) * char_width;
}

/* ── AUTO screen layout constants ─────────────────────────────────────────────
 * Hand-tuned for 240×135. Vertical budget below the 32×32 icon (icon ends at
 * icon_y+32 = 69) is ~58 px before the nav-dots row at y=128. We use it for:
 *   - status pill   y=AUTO_PILL_Y  .. AUTO_PILL_Y+AUTO_PILL_H   (28 px)
 *   - GPS line      y=AUTO_GPS_Y   .. AUTO_GPS_Y+8              (10 px)
 * The "AUTO" word from the shared layout is intentionally NOT drawn on this
 * screen; the green/red/yellow icon + active nav dot already identify it,
 * and the freed row lets the status pill be big and unambiguous.
 */
#if defined(M5ATOMS3)
/* AtomS3: 128x128 panel, smaller AUTO pill / GPS row to fit. The pill stack
 * sits below the 32x32 icon (icon ends at icon_y+32 ≈ 54) and uses the lower
 * half of the screen for the recording state pill + GPS line. */
#define AUTO_PILL_X    4
#define AUTO_PILL_Y    62
#define AUTO_PILL_W    (128 - 2 * AUTO_PILL_X)   /* = 120 px wide */
#define AUTO_PILL_H    24
#define AUTO_GPS_Y     104                        /* scale-1 GPS row, leaves 8px below */
#else
#define AUTO_PILL_X    8
#define AUTO_PILL_Y    78
#define AUTO_PILL_W    (240 - 2 * AUTO_PILL_X)   /* = 224 px wide */
#define AUTO_PILL_H    28
#define AUTO_GPS_Y     112                        /* scale-1 GPS row, ends y≈120, 4 px above dots band */
#endif

/* Pre-computed state info for the AUTO status pill. */
typedef struct {
    const char *text;       /* Pill label (centered, scale 2) */
    uint16_t    icon_color; /* Color for the 32×32 record icon */
    uint16_t    pill_bg;    /* Pill fill color */
    uint16_t    pill_fg;    /* Pill text color */
} auto_pill_info_t;

/* Compose the pill state from arm + camera-recording + motion-countdown state.
 * `buf` is a scratch buffer used when the label needs formatting (countdown).
 *
 * Priority order:
 *   DISARMED  → user pressed Stop; auto-trigger off. Show clear OFF state.
 *   COUNTDOWN → IMU has been quiet long enough that the stop timer is now ticking.
 *   RECORDING → camera reports it is recording. (We trust the camera's status,
 *               not the IMU — IMU is just a trigger, recording can outlive
 *               momentary stillness.)
 *   IDLE      → armed but camera not recording — waiting for motion to kick it off.
 *
 * NOTE: use M5_TRUE_* (not M5_COLOR_*) for the colored states. On this BGR
 * panel M5_COLOR_GREEN renders as red and M5_COLOR_YELLOW renders as magenta.
 */
static void auto_compute_pill(auto_pill_info_t *out, char *buf, size_t buf_len) {
    bool     armed         = motion_logic_is_armed();
    bool     recording     = is_camera_recording();
    uint32_t countdown_sec = motion_logic_get_stop_countdown_sec_remaining();

    if (!armed) {
        out->text       = "OFF";
        out->icon_color = M5_COLOR_DARKGREY;
        out->pill_bg    = M5_COLOR_DARKGREY;
        out->pill_fg    = M5_COLOR_WHITE;
    } else if (recording && countdown_sec > 0U) {
        snprintf(buf, buf_len, "STOP %lu:%02lu",
                 (unsigned long)(countdown_sec / 60U),
                 (unsigned long)(countdown_sec % 60U));
        out->text       = buf;
        out->icon_color = M5_TRUE_YELLOW;
        out->pill_bg    = M5_TRUE_YELLOW;
        out->pill_fg    = M5_COLOR_BLACK;
    } else if (recording) {
        out->text       = "REC";
        out->icon_color = M5_TRUE_GREEN;
        out->pill_bg    = M5_TRUE_GREEN;
        out->pill_fg    = M5_COLOR_BLACK;
    } else {
        out->text       = "WAITING";
        out->icon_color = M5_COLOR_GREY;
        out->pill_bg    = M5_COLOR_DARKGREY;
        out->pill_fg    = M5_COLOR_WHITE;
    }
}

/* Draw the colored status pill + centered scale-2 label. Always erases the
 * pill rectangle first so there are no leftovers from a previous wider label. */
static void auto_draw_pill(const auto_pill_info_t *info) {
    m5stickc_plus2_display_fill_rect(AUTO_PILL_X, AUTO_PILL_Y,
                                     AUTO_PILL_W, AUTO_PILL_H, info->pill_bg);
    int text_w = ui_get_text_width(info->text, 2);
    int text_x = AUTO_PILL_X + (AUTO_PILL_W - text_w) / 2;
    int text_y = AUTO_PILL_Y + (AUTO_PILL_H - 16) / 2;   /* 16 = scale-2 char height */
    m5stickc_plus2_display_print_scaled(text_x, text_y, info->text, info->pill_fg, 2);
}

/* Draw the centered GPS line just below the pill, leaving a clear band
 * before the nav dots. Erases its row first to avoid digit-overlap artifacts. */
static void auto_draw_gps_line(void) {
    /* Clear a 10-px row so changing-length strings don't leave ghosts. */
    m5stickc_plus2_display_fill_rect(0, AUTO_GPS_Y, M5_LCD_H_RES, 10, M5_COLOR_BLACK);

    gps_data_t gps;
    gps_get_data(&gps);

    char buf[40];
    uint16_t color;
    if (gps.has_fix) {
        snprintf(buf, sizeof(buf), "%.4f, %.4f  %u sat",
                 (double)gps.latitude, (double)gps.longitude,
                 (unsigned)gps.satellite_count);
        color = M5_TRUE_YELLOW;
    } else {
        snprintf(buf, sizeof(buf), "GPS searching%s",
                 gps.satellite_count > 0 ? "..." : "");
        color = M5_COLOR_DARKGREY;
    }
    int w = ui_get_text_width(buf, 1);
    int x = (M5_LCD_H_RES - w) / 2;
    if (x < 2) x = 2;
    m5stickc_plus2_display_print(x, AUTO_GPS_Y + 1, buf, color);
}

/**
 * @brief Refresh only the Auto Start/Stop status pill + GPS line without a full redraw.
 *
 * Cached partial repaint: re-touches a UI region only when the value driving
 * that region actually changed. This keeps the icon from flashing black-then-color
 * every 500 ms and prevents the pill / GPS rows from rewriting (and tearing)
 * when there is nothing new to show.
 *
 *   icon  → only when icon_color changes (motion state transition)
 *   pill  → only when pill_text or pill_bg/fg changes (label or color change)
 *   GPS   → only when fix or coords or sat-count changes
 *
 * Caller is welcome to invoke this at any cadence; it will cheaply early-out
 * to an `if` per region when nothing to do.
 */
/* Module-scope cache for the AUTO partial-update path so a separate
 * helper can invalidate it when a toast is dismissed or a full repaint
 * fires. Previously this was a function-local `static` block, which
 * meant a full repaint elsewhere couldn't tell the partial-update path
 * "your cache is stale; redraw everything next time."
 */
static bool        s_auto_init           = false;
static uint16_t    s_auto_last_icon_color = 0;
static uint16_t    s_auto_last_pill_bg    = 0;
static uint16_t    s_auto_last_pill_fg    = 0;
static char        s_auto_last_pill_text[24] = {0};
static bool        s_auto_last_gps_fix    = false;
static float       s_auto_last_lat        = 0.0f;
static float       s_auto_last_lon        = 0.0f;
static uint32_t    s_auto_last_sats       = 0xFFFFFFFFu;

static void ui_auto_status_cache_invalidate(void) {
    s_auto_init = false;
    s_auto_last_pill_text[0] = '\0';
}

void ui_update_auto_status_line_only(void) {
    /* While a toast is on screen, leave it alone — the user is still
     * reading it, and overpainting a status line would be racey with
     * the toast's clear+text. The next full ui_update_display() after
     * toast expiry will repaint the whole AUTO screen via the cache
     * invalidation set in ui_show_message(). */
    if (ui_toast_active()) return;

#if defined(M5ATOMS3)
    /* AtomS3: forward to the minimal renderer; it diffs internally and
     * only repaints the regions whose value changed. */
    atoms3_render(false);
    return;
#endif

    char scratch[24];
    auto_pill_info_t info;
    auto_compute_pill(&info, scratch, sizeof(scratch));

    if (s_disp_mutex && xSemaphoreTake(s_disp_mutex, pdMS_TO_TICKS(150)) != pdTRUE) return;

    /* Icon: erase + redraw only when color actually changed, or first-time call. */
    if (!s_auto_init || info.icon_color != s_auto_last_icon_color) {
        m5stickc_plus2_display_fill_rect(g_layout.icon_x, g_layout.icon_y,
                                         32, 32, M5_COLOR_BLACK);
        ui_draw_bitmap(g_layout.icon_x, g_layout.icon_y,
                       screen_info[SCREEN_AUTO].icon, 32, 32, info.icon_color);
        s_auto_last_icon_color = info.icon_color;
    }

    /* Pill: redraw when label, bg, or fg color changed. */
    if (!s_auto_init
            || info.pill_bg != s_auto_last_pill_bg
            || info.pill_fg != s_auto_last_pill_fg
            || strncmp(info.text, s_auto_last_pill_text, sizeof(s_auto_last_pill_text)) != 0) {
        auto_draw_pill(&info);
        s_auto_last_pill_bg = info.pill_bg;
        s_auto_last_pill_fg = info.pill_fg;
        strncpy(s_auto_last_pill_text, info.text, sizeof(s_auto_last_pill_text) - 1);
        s_auto_last_pill_text[sizeof(s_auto_last_pill_text) - 1] = '\0';
    }

    /* GPS: redraw when fix transitions, coords change ≥0.0001°, or sat count changes. */
    gps_data_t gps;
    gps_get_data(&gps);
    bool gps_changed = !s_auto_init
        || gps.has_fix != s_auto_last_gps_fix
        || gps.satellite_count != s_auto_last_sats
        || fabsf(gps.latitude  - s_auto_last_lat) > 0.0001f
        || fabsf(gps.longitude - s_auto_last_lon) > 0.0001f;
    if (gps_changed) {
        auto_draw_gps_line();
        s_auto_last_gps_fix = gps.has_fix;
        s_auto_last_lat     = gps.latitude;
        s_auto_last_lon     = gps.longitude;
        s_auto_last_sats    = gps.satellite_count;
    }

    /* Belt-and-braces: keep the bottom-of-screen edge clean even on partial paths.
     * 4 rows wipes both the literal last scanline and any nav-dot edge artifacts. */
    m5stickc_plus2_display_fill_rect(0, M5_LCD_V_RES - 4,
                                     M5_LCD_H_RES, 4, M5_COLOR_BLACK);

    if (s_disp_mutex) xSemaphoreGive(s_disp_mutex);
    s_auto_init = true;
}

#if defined(M5ATOMS3)
/* ── AtomS3 minimal renderer (M5GFX path, D-008..D-016) ────────────────────────
 *
 * Decisions in scope:
 *   D-008 — no icons / nav dots / instruction strip
 *   D-009 — 3-tier type scale (LABEL / VALUE / HERO)
 *   D-010 — colors: M5GFX standard RGB565 (M5_COLOR_* directly; the BGR
 *           swap-aliases were retired together with the esp_lcd path)
 *   D-011 — one accent color per state
 *   D-012 — diff-based partial repaint (no full-screen flash mid-loop)
 *   D-013 — text overflow handled by M5GFX's drawString (clips at fontHeight
 *           boundary) — no need for hand-rolled clipping
 *   D-014..D-016 — switch to M5GFX (proportional fonts, anti-aliasing)
 *
 * The renderer calls into m5atoms3_gfx.cpp (`atoms3_gfx_*`) instead of the
 * legacy 8x8 bitmap path. Cross-target call sites elsewhere in this file
 * still go through m5stickc_plus2_display_*, but on AtomS3 those forward
 * to the same M5GFX shim.
 * ────────────────────────────────────────────────────────────────────────── */

#include "m5atoms3_gfx.h"
#include "icons_png.h"
#include "status_logic.h"   /* is_camera_recording(), current_record_time */

/* Shared visual frame with the trigger4p "Ditch LEDs" remote:
 *
 *   +----------------------+
 *   | DASH          [link] |  TOP BAND: title + 🔗 icon when PROTOCOL_CONNECTED
 *   |                      |
 *   |        ( o )         |  MAIN: emoji status — 📡 finding/linking (blink),
 *   |        ICON          |        🤝 pairing (blink), 📸 recording (+ M:SS),
 *   |                      |        📷 connected idle. AUTO screen keeps its
 *   +----------------------+        motion hero word + countdown value.
 *   |     HOLD = PAIR      |  BOTTOM BAND: button/push hint (GPS on AUTO).
 *   +----------------------+
 */
#define A_SCREEN_W      128
#define A_TOP_H         20
#define A_BOT_H         24
#define A_MAIN_TOP      (A_TOP_H + 1)
#define A_MAIN_BOT      (128 - A_BOT_H - 1)
#define A_MAIN_H        (A_MAIN_BOT - A_MAIN_TOP)
#define A_ICON_W        48
#define A_ICON_X        ((A_SCREEN_W - A_ICON_W) / 2)
#define A_ICON_Y        (A_MAIN_TOP + (A_MAIN_H - A_ICON_W) / 2)
#define A_REC_ICON_Y    (A_MAIN_TOP + 2)
#define A_REC_TIME_Y    (A_REC_ICON_Y + A_ICON_W + 2)
#define A_HERO_Y        (A_MAIN_TOP + 12)   /* AUTO text hero  */
#define A_VALUE_Y       (A_MAIN_TOP + 46)   /* AUTO countdown  */
#define A_LINK_W        16
#define A_BAND_BG       M5_COLOR_DARKGREY
#define A_BLINK_TICKS   8                   /* ~400 ms at 50 ms/loop */

typedef enum { MI_NONE = 0, MI_SAT, MI_PAIR, MI_REC, MI_IDLE } main_icon_t;

/* Cache of the last-rendered state so the steady-state path can early-out
 * when nothing has changed. */
typedef struct {
    bool                initialized;
    ui_screen_t         screen;
    connect_state_t     conn;
    main_icon_t         icon;        /* current CONNECT-screen emoji        */
    bool                blink;       /* does the current icon blink         */
    bool                blink_visible;
    char                hero[16];    /* AUTO text hero                      */
    char                value[16];   /* REC M:SS or AUTO countdown          */
    char                hint[24];
    uint16_t            hero_color;
} atoms3_cache_t;

static atoms3_cache_t s_atoms3 = { .initialized = false };

static const uint8_t *atoms3_main_icon_png(main_icon_t i, unsigned *len) {
    switch (i) {
        case MI_SAT:  *len = icon_sat_png_len;  return icon_sat_png;
        case MI_PAIR: *len = icon_pair_png_len; return icon_pair_png;
        case MI_REC:  *len = icon_rec_png_len;  return icon_rec_png;
        case MI_IDLE: *len = icon_idle_png_len; return icon_idle_png;
        default:      *len = 0; return NULL;
    }
}

static void atoms3_draw_icon(main_icon_t i, int y) {
    unsigned len = 0;
    const uint8_t *png = atoms3_main_icon_png(i, &len);
    if (png) atoms3_gfx_draw_png(A_ICON_X, y, png, len);
}

static void atoms3_clear_main(void) {
    atoms3_gfx_fill_rect(0, A_MAIN_TOP, A_SCREEN_W, A_MAIN_H + 1, M5_COLOR_BLACK);
}

/* Erase a horizontal band sized for the given M5GFX font tier. Used before
 * each text rewrite so a previous longer string can't ghost. The +2 pad
 * absorbs descenders / kerning artifacts. */
static void atoms3_erase_band_for_tier(int y, int tier) {
    int h = atoms3_gfx_tier_pixel_height(tier) + 2;
    atoms3_gfx_erase_rect(0, y, atoms3_gfx_width(), h);
}

/* Top-right connectivity indicator: 🔗 when fully connected to the camera,
 * otherwise clear (the main area carries the dish/handshake status). */
static void atoms3_draw_top_indicator(connect_state_t conn) {
    atoms3_gfx_fill_rect(A_SCREEN_W - 20, 0, 20, A_TOP_H, A_BAND_BG);
    if (conn == PROTOCOL_CONNECTED) {
        atoms3_gfx_draw_png(A_SCREEN_W - A_LINK_W - 2, (A_TOP_H - A_LINK_W) / 2,
                            icon_link_png, icon_link_png_len);
    }
}

static void atoms3_draw_top_band(connect_state_t conn) {
    atoms3_gfx_fill_rect(0, 0, A_SCREEN_W, A_TOP_H, A_BAND_BG);
    atoms3_gfx_print(4, 2, "DASH", M5_COLOR_WHITE, 1);
    atoms3_draw_top_indicator(conn);
}

/* Pick the CONNECT-screen emoji for the current link/camera state. Sets
 * *blink for the states that should flash, and fills value with the recording
 * elapsed time (M:SS) when recording. */
static main_icon_t atoms3_connect_icon(connect_state_t conn, bool *blink,
                                       char *value, size_t value_sz) {
    value[0] = '\0';
    *blink = false;
    switch (conn) {
        case PROTOCOL_CONNECTED:
            if (is_camera_recording()) {
                unsigned t = (unsigned)current_record_time;
                snprintf(value, value_sz, "%u:%02u", t / 60U, t % 60U);
                return MI_REC;
            }
            return MI_IDLE;
        case BLE_CONNECTED:
            *blink = true;
            return (g_verify_mode == 1) ? MI_PAIR : MI_SAT;
        case BLE_SEARCHING:
            *blink = true;
            return MI_SAT;
        default:
            return MI_SAT;   /* offline / init: dish, static */
    }
}

/* Compose the AUTO-screen hero word + accent for the current motion state. */
static void atoms3_compose_auto(char *out_hero, size_t hero_sz,
                                char *out_value, size_t value_sz,
                                uint16_t *out_color,
                                bool armed, bool recording, uint32_t countdown) {
    out_value[0] = '\0';
    if (!armed) {
        snprintf(out_hero, hero_sz, "OFF");
        *out_color = M5_COLOR_DARKGREY;
    } else if (recording && countdown > 0U) {
        snprintf(out_hero, hero_sz, "STOP");
        snprintf(out_value, value_sz, "%lu:%02lu",
                 (unsigned long)(countdown / 60U),
                 (unsigned long)(countdown % 60U));
        *out_color = M5_COLOR_YELLOW;
    } else if (recording) {
        snprintf(out_hero, hero_sz, "REC");
        *out_color = M5_COLOR_RED;
    } else {
        snprintf(out_hero, hero_sz, "WAIT");
        *out_color = M5_COLOR_GREY;
    }
}

/* Compose the bottom-row hint for either screen. */
static void atoms3_compose_hint(char *out, size_t out_sz, ui_screen_t screen) {
    if (screen == SCREEN_CONNECT) {
        snprintf(out, out_sz, "HOLD = PAIR");
    } else {
        gps_data_t gps;
        gps_get_data(&gps);
        if (gps.has_fix) {
            snprintf(out, out_sz, "GPS OK %u",
                     (unsigned)gps.satellite_count);
        } else if (gps.satellite_count > 0) {
            snprintf(out, out_sz, "GPS %u sat",
                     (unsigned)gps.satellite_count);
        } else {
            snprintf(out, out_sz, "NO GPS");
        }
    }
}

static void atoms3_render(bool force_full) {
    /* Toast still has the screen — leave it alone. */
    if (ui_toast_active()) return;

    if (s_disp_mutex && xSemaphoreTake(s_disp_mutex, pdMS_TO_TICKS(150)) != pdTRUE) {
        return;
    }

    static uint32_t tick = 0;
    tick++;
    bool blink_visible = ((tick / A_BLINK_TICKS) & 1u) == 0u;

    bool screen_changed = !s_atoms3.initialized
                       || s_atoms3.screen != g_ui_state.current_screen;
    bool full = force_full || screen_changed;

    connect_state_t conn = connect_logic_get_state();

    if (full) {
        atoms3_gfx_clear(M5_COLOR_BLACK);
        atoms3_draw_top_band(conn);
        atoms3_clear_main();
        atoms3_gfx_fill_rect(0, A_MAIN_BOT + 1, A_SCREEN_W, A_BOT_H, A_BAND_BG);
        s_atoms3.initialized  = true;
        s_atoms3.screen       = g_ui_state.current_screen;
        s_atoms3.conn         = conn;
        s_atoms3.icon         = MI_NONE;
        s_atoms3.blink        = false;
        s_atoms3.blink_visible= blink_visible;
        s_atoms3.hero[0]      = '\0';
        s_atoms3.value[0]     = '\0';
        s_atoms3.hint[0]      = '\0';
        s_atoms3.hero_color   = 0;
    } else if (conn != s_atoms3.conn) {
        atoms3_draw_top_indicator(conn);
        s_atoms3.conn = conn;
    }

    if (g_ui_state.current_screen == SCREEN_CONNECT) {
        /* ── Emoji status ────────────────────────────────────────────────── */
        char        value[16];
        bool        blink = false;
        main_icon_t want  = atoms3_connect_icon(conn, &blink, value, sizeof(value));
        bool        icon_changed = (s_atoms3.icon != want);

        if (full || icon_changed) {
            atoms3_clear_main();
            s_atoms3.icon  = want;
            s_atoms3.blink = blink;
            s_atoms3.value[0] = '\0';
            int iy = (want == MI_REC) ? A_REC_ICON_Y : A_ICON_Y;
            if (!blink || blink_visible) atoms3_draw_icon(want, iy);
            s_atoms3.blink_visible = blink_visible;
        } else if (blink && blink_visible != s_atoms3.blink_visible) {
            if (blink_visible) atoms3_draw_icon(want, A_ICON_Y);
            else atoms3_gfx_fill_rect(A_ICON_X, A_ICON_Y, A_ICON_W, A_ICON_W, M5_COLOR_BLACK);
            s_atoms3.blink_visible = blink_visible;
        }

        /* Recording elapsed time under the camera icon. */
        if (want == MI_REC
            && (full || icon_changed
                || strncmp(value, s_atoms3.value, sizeof(s_atoms3.value)) != 0)) {
            atoms3_erase_band_for_tier(A_REC_TIME_Y, 2);
            if (value[0] != '\0')
                atoms3_gfx_print_centered(A_REC_TIME_Y, value, M5_COLOR_WHITE, 2);
            strncpy(s_atoms3.value, value, sizeof(s_atoms3.value) - 1);
            s_atoms3.value[sizeof(s_atoms3.value) - 1] = '\0';
        }
    } else {
        /* AUTO is still the camera-status screen on AtomS3. Keep the shared
         * visual language: camera emoji in the main area, elapsed record time
         * only when recording. No giant WAIT/REC/STOP words. */
        char        value[16];
        bool        blink = false;
        main_icon_t want  = atoms3_connect_icon(conn, &blink, value, sizeof(value));
        bool        icon_changed = (s_atoms3.icon != want);
        int         iy = (want == MI_REC) ? A_REC_ICON_Y : A_ICON_Y;

        if (full || icon_changed) {
            atoms3_clear_main();
            s_atoms3.icon = want;
            s_atoms3.blink = blink;
            s_atoms3.hero[0] = '\0';
            s_atoms3.value[0] = '\0';
            if (!blink || blink_visible) atoms3_draw_icon(want, iy);
            s_atoms3.blink_visible = blink_visible;
        } else if (blink && blink_visible != s_atoms3.blink_visible) {
            if (blink_visible) atoms3_draw_icon(want, iy);
            else atoms3_gfx_fill_rect(A_ICON_X, iy, A_ICON_W, A_ICON_W, M5_COLOR_BLACK);
            s_atoms3.blink_visible = blink_visible;
        }

        if (want == MI_REC
            && (full || icon_changed
                || strncmp(value, s_atoms3.value, sizeof(s_atoms3.value)) != 0)) {
            atoms3_erase_band_for_tier(A_REC_TIME_Y, 2);
            if (value[0] != '\0')
                atoms3_gfx_print_centered(A_REC_TIME_Y, value, M5_COLOR_WHITE, 2);
            strncpy(s_atoms3.value, value, sizeof(s_atoms3.value) - 1);
            s_atoms3.value[sizeof(s_atoms3.value) - 1] = '\0';
        }
    }

    /* ── Bottom hint band ─────────────────────────────────────────────────── */
    char hint[24];
    atoms3_compose_hint(hint, sizeof(hint), g_ui_state.current_screen);
    if (full || strncmp(hint, s_atoms3.hint, sizeof(s_atoms3.hint)) != 0) {
        int h1 = atoms3_gfx_tier_pixel_height(1);
        atoms3_gfx_fill_rect(0, A_MAIN_BOT + 1, A_SCREEN_W, A_BOT_H, A_BAND_BG);
        atoms3_gfx_print_centered((A_MAIN_BOT + 1) + (A_BOT_H - h1) / 2,
                                  hint, M5_COLOR_WHITE, 1);
        strncpy(s_atoms3.hint, hint, sizeof(s_atoms3.hint) - 1);
        s_atoms3.hint[sizeof(s_atoms3.hint) - 1] = '\0';
    }

    g_ui_state.display_needs_update = false;
    if (s_disp_mutex) xSemaphoreGive(s_disp_mutex);
}
#endif /* M5ATOMS3 */

/**
 * @brief Update display with current UI state
 * 
 * Renders the complete user interface including:
 * - Connection status indicator
 * - Current screen icon and text
 * - Screen navigation dots
 * - Instruction text
 * 
 * Only updates when display_needs_update flag is set for efficiency.
 */
void ui_update_display(void) {
    /* While a toast is on screen, leave it alone. */
    if (ui_toast_active()) {
        return;
    }

    /* Toast just expired (was drawn, deadline passed) — force a fresh
     * full repaint of the underlying screen so the user transitions
     * smoothly back from the message. atoms3_render() treats
     * display_needs_update as the force_full signal. */
    if (s_toast_was_drawn) {
        s_toast_was_drawn = false;
        s_toast_deadline  = 0;
        ui_auto_status_cache_invalidate();
        g_ui_state.display_needs_update = true;
    }

#if defined(M5ATOMS3)
    /* AtomS3 path: minimal text-first renderer with diff-based repaint.
     * Always run — atoms3_render() early-outs cheaply when nothing changed,
     * and a screen change forces a one-time full repaint internally. */
    atoms3_render(g_ui_state.display_needs_update);
    return;
#endif

    if (!g_ui_state.display_needs_update) {
        return;
    }

    /* Serialize all SPI display writes — worker task and main loop both call
     * this path; concurrent access to esp_lcd_panel_draw_bitmap asserts. */
    if (s_disp_mutex && xSemaphoreTake(s_disp_mutex, pdMS_TO_TICKS(150)) != pdTRUE) {
        return;
    }

    /* Clear entire display to prevent visual artifacts. Use the HAL-defined
     * resolution (M5_LCD_H_RES x M5_LCD_V_RES) so this works on 240x135 Stick
     * panels and 128x128 AtomS3 alike. */
    m5stickc_plus2_display_fill_rect(0, 0, M5_LCD_H_RES, M5_LCD_V_RES, M5_COLOR_BLACK);

    /* Draw connection status indicator in top-right corner */
    ui_draw_connection_status();

#if !defined(M5ATOMS3)
    /* Draw screen navigation indicators at bottom of display
     * Shows current screen position and total available screens.
     * AtomS3 has only two screens and a 128x128 panel — there's no room
     * (and no value) in drawing nav dots there. */
    for (int i = 0; i < SCREEN_COUNT; i++) {
        int x = g_layout.dots_start_x + (i * g_layout.dots_spacing);
        int y = g_layout.dots_y;
        /* Smaller dots — the previous 8/6 px squares packed in a tight row read as
         * a continuous "white line at the bottom" from a distance. 5/3 px keeps
         * them visible as page indicators without the line illusion. */
        int dot_size = g_ui_state.is_plus2_device ? 5 : 4;
        int inactive_size = g_ui_state.is_plus2_device ? 3 : 3;

        if (i == g_ui_state.current_screen) {
            /* Current screen - larger white rectangle */
            m5stickc_plus2_display_fill_rect(x - dot_size/2, y - dot_size/2, dot_size, dot_size, M5_COLOR_WHITE);
        } else {
            /* Other screens - smaller gray rectangles */
            m5stickc_plus2_display_fill_rect(x - inactive_size/2, y - inactive_size/2, inactive_size, inactive_size, M5_COLOR_DARKGREY);
        }
    }
#endif /* !M5ATOMS3 */
    
    /* Get information for currently selected screen */
    const screen_info_t* screen = &screen_info[g_ui_state.current_screen];

    if (g_ui_state.current_screen == SCREEN_AUTO) {
        /* AUTO is a status screen, not an action screen — uses a custom layout
         * that drops the redundant "AUTO" text and uses the freed row for a
         * full-width state pill (REC / WAITING / STOP m:ss). The icon itself
         * is recolored to reflect motion state. */
        char scratch[24];
        auto_pill_info_t info;
        auto_compute_pill(&info, scratch, sizeof(scratch));

        ui_draw_bitmap(g_layout.icon_x, g_layout.icon_y,
                       screen->icon, 32, 32, info.icon_color);
        auto_draw_pill(&info);
        auto_draw_gps_line();
    } else {
        /* Action screens: icon + scale-2 name centered below it. */
        ui_draw_bitmap(g_layout.icon_x, g_layout.icon_y, screen->icon, 32, 32, screen->color);

        int text_scale = 2;
        int text_width = ui_get_text_width(screen->name, text_scale);
        int centered_text_x = g_layout.text_x - (text_width / 2);
        m5stickc_plus2_display_print_scaled(centered_text_x, g_layout.text_y,
                                            screen->name, M5_COLOR_WHITE, text_scale);
    }

    /* Display button instructions in top-left corner */
#if defined(M5ATOMS3)
    /* AtomS3 has a single screen-button — the instruction line tells the
     * operator that short cycles screens and long acts on the current one. */
    const char *instruct = (g_ui_state.current_screen == SCREEN_AUTO)
        ? "S:swap L:rec"
        : "S:swap L:bt";
#else
    const char *instruct = (g_ui_state.current_screen == SCREEN_AUTO)
        ? "A:Toggle  B:Next"
        : "A:Run     B:Next";
#endif
    m5stickc_plus2_display_print(g_layout.instruct_x, g_layout.instruct_y, instruct, M5_COLOR_GREY);

    /* Belt-and-braces: explicitly clear the bottom 2 rows so any prior render
     * (or panel-edge artifact from the ST7789 gap) can't leave a stray bright
     * line at the very bottom of the screen. Clear 4 rows so we cover the bottom
     * edge of any nav-dot leftovers as well as the literal last row. */
    m5stickc_plus2_display_fill_rect(0, M5_LCD_V_RES - 4, M5_LCD_H_RES, 4, M5_COLOR_BLACK);

    g_ui_state.display_needs_update = false;
    if (s_disp_mutex) xSemaphoreGive(s_disp_mutex);
    ESP_LOGI(TAG, "Display updated - Screen: %s", screen->name);
}

/**
 * @brief Navigate to next screen in sequence
 * 
 * Cycles through available screens: Connect → Shutter → Mode → Sleep → Wake → Connect...
 * Called when Button B is pressed.
 */
void ui_next_screen(void) {
    /* User pressed Button B — make the response immediate by cancelling
     * any in-flight toast so ui_update_display() repaints the new screen
     * on the very next iteration. */
    ui_cancel_toast();
    ui_auto_status_cache_invalidate();

#if defined(M5ATOMS3)
    /* AtomS3 ships a minimal two-screen UI: BT (CONNECT) and AUTO. Anything
     * else (SHUTTER/MODE/SLEEP/WAKE) is hidden — short press toggles only
     * between these two. See docs/ATOMS3_MIGRATION_SPEC.md (D-005). */
    g_ui_state.current_screen =
        (g_ui_state.current_screen == SCREEN_AUTO) ? SCREEN_CONNECT : SCREEN_AUTO;
#else
    g_ui_state.current_screen = (g_ui_state.current_screen + 1) % SCREEN_COUNT;
#endif
    g_ui_state.display_needs_update = true;
    ESP_LOGI(TAG, "Switched to screen: %d (%s)",
             g_ui_state.current_screen, screen_info[g_ui_state.current_screen].name);
}

/**
 * @brief Execute function for currently selected screen
 * 
 * Calls the screen-specific function (connect, shutter, mode switch, etc.)
 * when Button A is pressed or GPIO trigger occurs.
 */
void ui_execute_current_screen(void) {
    const screen_info_t* screen = &screen_info[g_ui_state.current_screen];
    ESP_LOGI(TAG, "Executing function for screen: %s", screen->name);

    /* Button A: cancel any in-flight toast so the user sees the result
     * of THIS press, not a stale message from the previous one. The
     * screen handler may immediately call ui_show_message() again to
     * replace it with new feedback. */
    ui_cancel_toast();
    ui_auto_status_cache_invalidate();

    if (screen->execute_func) {
        screen->execute_func();
    }

    /* Schedule display update to reflect any changes */
    g_ui_state.display_needs_update = true;
}

/**
 * @brief Display a temporary on-screen message (non-blocking).
 *
 * Renders @p message immediately, sets an expiry deadline, and returns
 * without delay. The main loop's ui_update_display() leaves the toast on
 * screen until the deadline elapses, then forces a full redraw so the
 * underlying screen comes back automatically.
 *
 * Calling this from a worker task that already does sequential
 * vTaskDelay() between steps still works as expected — each call resets
 * the deadline. Calling from the main task no longer steals 0.5–2 s of
 * button-poll time, which is the whole point of this refactor.
 *
 * @param message Text to display.
 * @param color Text color (RGB565 format).
 * @param duration_ms How long to keep the message on screen (≥ 1 ms).
 */
void ui_show_message(const char* message, uint16_t color, int duration_ms) {
    if (message == NULL) return;
    if (duration_ms < 1) duration_ms = 1;

#if defined(M5ATOMS3)
    /* AtomS3 uses the same persistent chrome as the trigger4p remote. Full-screen
     * progress/error toasts ("Reconnecting...", red failures, etc.) look like
     * random flashes on the 128x128 panel and break the shared UI pattern.
     * Keep the workflow logs and let atoms3_render() show state with icons. */
    (void)color;
    (void)duration_ms;
    g_ui_state.display_needs_update = true;
    return;
#endif

    if (s_disp_mutex) xSemaphoreTake(s_disp_mutex, pdMS_TO_TICKS(300));
    m5stickc_plus2_display_clear(M5_COLOR_BLACK);

    int msg_x = g_ui_state.is_plus2_device ? 40 : 30;
    int msg_y = g_ui_state.is_plus2_device ? 60 : 40;
    m5stickc_plus2_display_print(msg_x, msg_y, message, color);
    if (s_disp_mutex) xSemaphoreGive(s_disp_mutex);

    s_toast_deadline  = xTaskGetTickCount() + pdMS_TO_TICKS((uint32_t)duration_ms);
    s_toast_was_drawn = true;
    /* Partial-update cache must be invalidated so the post-toast full
     * repaint of the AUTO screen never skips a region whose value
     * happened to match the (stale) cached value at toast time. */
    ui_auto_status_cache_invalidate();
}

/**
 * @brief Display "not connected" error message
 * 
 * Convenience function to show connection error when user attempts
 * camera operations without an active connection.
 */
void ui_show_not_connected_message(void) {
    ui_show_message("Not Connected!", M5_COLOR_RED, 1500);
}

/**
 * @brief Attempt manual camera pairing as fallback
 * 
 * Called when automatic reconnection fails, initiating a fresh pairing
 * process that requires user interaction on the camera side.
 */
static void ui_try_manual_pairing(void) {
    ESP_LOGI(TAG, "Attempting manual pairing fallback");
    if (connect_logic_get_state() == BLE_NOT_INIT) {
        ui_show_message("BLE not ready", M5_COLOR_RED, 1500);
        return;
    }

    /* Switch to pairing mode for new device registration */
    g_verify_mode = 1;  /* Pairing mode requires camera-side confirmation */
    
    /* Generate random verification code for secure pairing */
    srand((unsigned int)time(NULL));
    g_verify_data = (uint16_t)(rand() % 10000);
    
    /* Start BLE connection in discovery mode (scan for any compatible camera) */
    int res = connect_logic_ble_connect(false);  /* is_reconnecting = false */
    if (res == 0) {
        ui_show_message("BLE Connected\nPress camera pair button", M5_COLOR_CYAN, 2000);
        
        /* Establish DJI protocol connection in pairing mode */
        res = connect_logic_protocol_connect(
            g_device_id,
            g_mac_addr_len,
            g_mac_addr,
            g_fw_version,
            g_verify_mode,
            g_verify_data,
            g_camera_reserved
        );
        
        if (res == 0) {
            ui_show_message("Manual Pair Success!", M5_COLOR_GREEN, 1500);
            /* Save new camera information for future auto-connect */
            store_camera_info();
            /* Enable real-time status monitoring */
            subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
        } else {
            ui_show_message("Manual Pair Failed", M5_COLOR_RED, 1500);
        }
    } else {
        ui_show_message("Manual BLE Failed", M5_COLOR_RED, 1500);
    }
}

/**
 * @brief Handle camera connection screen activation (button A on CONNECT).
 *
 * Now a thin dispatcher: the long-running BLE / protocol work is queued
 * onto the UI worker task so the main loop keeps polling buttons. The
 * user gets immediate visual feedback via the toast we set here.
 */
void ui_screen_connect(void) {
    ESP_LOGI(TAG, "Executing connect screen — dispatching to worker");

    connect_state_t state = connect_logic_get_state();
    if (state == PROTOCOL_CONNECTED) {
        ui_show_message("Already Connected!", M5_COLOR_GREEN, 1000);
        return;
    }
    if (state == BLE_NOT_INIT) {
        ui_show_message("BLE not ready", M5_COLOR_RED, 2000);
        return;
    }

    /* Show the user that we heard the press. The worker will flash its
     * own progressive messages ("BLE Connected...", "Reconnected!", etc.)
     * as it runs. */
    ui_show_message("Connecting...", M5_COLOR_CYAN, 1000);
    ui_dispatch_work(UI_WORK_UI_RECONNECT);
}

/* Original blocking body — now runs only on the worker task. */
static void ui_screen_connect_worker(void) {
    ESP_LOGI(TAG, "ui_screen_connect_worker: starting");

    connect_state_t state = connect_logic_get_state();
    if (state == PROTOCOL_CONNECTED) {
        ui_show_message("Already Connected!", M5_COLOR_GREEN, 1000);
        return;
    }
    if (state == BLE_NOT_INIT) {
        ui_show_message("BLE not ready", M5_COLOR_RED, 2000);
        return;
    }

    if (g_stored_camera.is_paired) {
        /* Camera already paired - attempt automatic reconnection */
        ESP_LOGI(TAG, "Attempting to reconnect to paired camera (verify_mode=0)");
        ui_show_message("Reconnecting...", M5_COLOR_CYAN, 500);
        
        /* Configure BLE to target the specific paired camera */
        ble_set_target_device(g_stored_camera.camera_name, g_stored_camera.camera_mac);
        
        load_stored_camera_info();
        g_verify_mode = 0;  /* Reconnection mode - no pairing required */
        
        /* Initiate BLE connection to paired device */
        int res = connect_logic_ble_connect(true);  /* is_reconnecting = true */
        if (res == 0) {
            ui_show_message("BLE Connected\nConnecting protocol...", M5_COLOR_BLUE, 1000);
            
            /* Establish DJI protocol connection using stored credentials */
            res = connect_logic_protocol_connect(
                g_device_id,
                g_mac_addr_len,
                g_mac_addr,
                g_fw_version,
                g_verify_mode,
                g_verify_data,
                g_camera_reserved
            );
            
            if (res == 0) {
                ui_show_message("Reconnected!", M5_COLOR_GREEN, 1000);
                /* Enable real-time camera status monitoring */
                subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
            } else {
                ui_show_message("Reconnect Failed\nTrying manual pairing...", M5_COLOR_ORANGE, 1500);
                /* Fallback to fresh pairing attempt */
                ui_try_manual_pairing();
            }
        } else {
            ui_show_message("BLE Failed\nTrying manual pairing...", M5_COLOR_ORANGE, 1500);
            /* Fallback to fresh pairing attempt */
            ui_try_manual_pairing();
        }
    } else {
        /* No paired camera found - initiate first-time pairing */
        ESP_LOGI(TAG, "First time pairing (verify_mode=1)");
        ui_show_message("Pairing Mode\nPress camera pair button", M5_COLOR_CYAN, 2000);
        
        g_verify_mode = 1;  /* Pairing mode requires user confirmation */
        
        /* Generate random verification code for secure pairing */
        srand((unsigned int)time(NULL));
        g_verify_data = (uint16_t)(rand() % 10000);
        
        /* Start BLE connection in discovery mode */
        int res = connect_logic_ble_connect(false);  /* is_reconnecting = false */
        if (res == 0) {
            ui_show_message("BLE Connected\nPairing...", M5_COLOR_BLUE, 1000);
            
            /* Establish DJI protocol connection in pairing mode */
            res = connect_logic_protocol_connect(
                g_device_id,
                g_mac_addr_len,
                g_mac_addr,
                g_fw_version,
                g_verify_mode,
                g_verify_data,
                g_camera_reserved
            );
            
            if (res == 0) {
                /* Successful pairing - save camera information for future use */
                store_camera_info();
                ui_show_message("Paired Successfully!", M5_COLOR_GREEN, 1500);
                g_pending_set_video_mode_after_connect = true;
                subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
                g_ui_state.current_screen = SCREEN_AUTO;
                g_ui_state.display_needs_update = true;
            } else {
                ui_show_message("Pairing Failed", M5_COLOR_RED, 1500);
            }
        } else {
            ui_show_message("BLE Connect Failed", M5_COLOR_RED, 1500);
        }
    }
}

/**
 * @brief Handle shutter screen activation
 * 
 * Controls photo capture and video recording based on current camera mode:
 * - Photo mode: Always takes a photo
 * - Video modes: Toggles recording start/stop
 * 
 * Requires active camera connection to function.
 */
void ui_screen_shutter(void) {
    ESP_LOGI(TAG, "Executing shutter screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    /* Check if camera status has been received for mode-aware operation */
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status not yet initialized, attempting command anyway");
        ui_show_message("Status Unknown\nTrying anyway...", M5_COLOR_ORANGE, 1000);
    }
    
    /* Determine shutter behavior based on current camera mode */
    camera_mode_t current_mode = (camera_mode_t)current_camera_mode;
    
    ESP_LOGI(TAG, "Current camera mode: %d, status: %d, recording: %s", 
             current_mode, current_camera_status, is_camera_recording() ? "yes" : "no");
    
    if (current_mode == CAMERA_MODE_PHOTO) {
        /* Photo mode - single shot capture */
        ESP_LOGI(TAG, "Taking photo in photo mode");
        record_control_response_frame_t* response = command_logic_start_record();
        if (response) {
            ui_show_message("Photo Taken", M5_COLOR_GREEN, 1000);
            free(response);
        } else {
            ui_show_message("Photo Failed", M5_COLOR_RED, 1500);
        }
    } else {
        /* Video modes - toggle recording state */
        bool is_recording = is_camera_recording();
        
        if (is_recording) {
            /* Stop current recording */
            ESP_LOGI(TAG, "Stopping recording in video mode");
            record_control_response_frame_t* response = command_logic_stop_record();
            /* Same retry watchdog as the AUTO-screen manual stop. The camera
             * sometimes ignores a single stop frame; the main loop will
             * re-send until status confirms stopped. */
            app_request_pending_stop();
            if (response) {
                ui_show_message("Recording Stopped", M5_COLOR_YELLOW, 1000);
                free(response);
            } else {
                ui_show_message("Stop Failed", M5_COLOR_RED, 1500);
            }
        } else {
            /* Start new recording */
            ESP_LOGI(TAG, "Starting recording in video mode");
            record_control_response_frame_t* response = command_logic_start_record();
            if (response) {
                ui_show_message("Recording Started", M5_COLOR_GREEN, 1000);
                free(response);
            } else {
                ui_show_message("Start Failed", M5_COLOR_RED, 1500);
            }
        }
    }
}

/**
 * @brief Handle camera mode switching screen activation
 * 
 * Cycles through common camera modes in sequence:
 * Video → Photo → Timelapse → Slow Motion → Video...
 * 
 * Requires active camera connection to function.
 */
void ui_screen_mode(void) {
    ESP_LOGI(TAG, "Executing mode screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    /* Determine next mode in cycling sequence */
    camera_mode_t current_mode = (camera_mode_t)current_camera_mode;
    camera_mode_t next_mode;
    const char* mode_name;
    
    /* Mode cycling logic - rotates through most commonly used modes */
    switch (current_mode) {
        case CAMERA_MODE_NORMAL:
            next_mode = CAMERA_MODE_PHOTO;
            mode_name = "Photo";
            break;
        case CAMERA_MODE_PHOTO:
            next_mode = CAMERA_MODE_TIMELAPSE;
            mode_name = "Timelapse";
            break;
        case CAMERA_MODE_TIMELAPSE:
            next_mode = CAMERA_MODE_SLOW_MOTION;
            mode_name = "Slow Motion";
            break;
        case CAMERA_MODE_SLOW_MOTION:
            next_mode = CAMERA_MODE_NORMAL;
            mode_name = "Video";
            break;
        default:
            next_mode = CAMERA_MODE_NORMAL;
            mode_name = "Video";
            break;
    }
    
    ESP_LOGI(TAG, "Switching from mode %d to mode %d (%s)", current_mode, next_mode, mode_name);
    
    camera_mode_switch_response_frame_t* response = command_logic_switch_camera_mode(next_mode);
    if (response) {
        char message[32];
        snprintf(message, sizeof(message), "Mode: %s", mode_name);
        ui_show_message(message, M5_COLOR_ORANGE, 1500);
        free(response);
    } else {
        ui_show_message("Mode Failed", M5_COLOR_RED, 1500);
    }
}

/**
 * @brief Handle camera sleep screen activation
 * 
 * Sends Camera Power Mode Settings command (CmdSet=0x00, CmdID=0x1A)
 * to put the camera into sleep mode for power conservation.
 * 
 * Requires active camera connection to function.
 */
void ui_screen_sleep(void) {
    ESP_LOGI(TAG, "Executing sleep screen");
    
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }
    
    ESP_LOGI(TAG, "Sending sleep command to camera");
    camera_power_mode_switch_response_frame_t* response = command_logic_power_mode_switch_sleep();
    if (response) {
        if (response->ret_code == 0x00) {
            ui_show_message("Camera Sleeping", M5_COLOR_CYAN, 1500);
            ESP_LOGI(TAG, "Camera successfully put to sleep");
        } else {
            ui_show_message("Sleep Failed", M5_COLOR_RED, 1500);
            ESP_LOGW(TAG, "Sleep command failed with ret_code: %d", response->ret_code);
        }
        free(response);
    } else {
        ui_show_message("Sleep Failed", M5_COLOR_RED, 1500);
        ESP_LOGE(TAG, "Failed to send sleep command");
    }
}

/**
 * @brief Auto Start/Stop screen — Button A toggles recording.
 *
 * If recording is active: stop immediately and reset motion state to Idle.
 * If idle: switch to video mode, start recording, and set motion state to
 * Moving so the 2.5-minute stop countdown begins once the vehicle is still.
 */
void ui_screen_auto(void) {
    connect_state_t state = connect_logic_get_state();
    if (state != PROTOCOL_CONNECTED) {
        ui_show_not_connected_message();
        return;
    }

    if (is_camera_recording()) {
        ESP_LOGI(TAG, "Auto: manual stop → DISARM");
        /* Audible cue first so the user can hear the press registered even
         * before the BLE round-trip to the camera completes. See SPEC.md
         * "Audible Cues" — pause = single low beep (700 Hz / 50 ms). */
        m5stickc_plus2_buzzer_beep(700, 50);
        (void)command_logic_stop_record();
        /* Camera sometimes ignores the first stop frame — arm the periodic
         * retry watchdog in app_main so the main loop re-issues stop_record
         * until status confirms the camera actually stopped. */
        app_request_pending_stop();
        motion_logic_force_idle();
        /* Disarm so motion can't immediately re-trigger start_record while the
         * user is still holding the device (the IMU sees finger movement and
         * would otherwise fire motion_just_started in ~300ms). */
        motion_logic_set_armed(false);
    } else {
        ESP_LOGI(TAG, "Auto: manual start → ARM");
        /* Unpause = single high beep (2500 Hz / 50 ms) — matches connect cue
         * pitch but shorter so the two events stay audibly distinct. */
        m5stickc_plus2_buzzer_beep(2500, 50);
        (void)command_logic_switch_camera_mode(CAMERA_MODE_NORMAL);
        vTaskDelay(pdMS_TO_TICKS(200));
        (void)command_logic_start_record();
        motion_logic_force_active();
        motion_logic_set_armed(true);
    }

    /* No toast — the pill itself (REC / WAITING / OFF / STOP m:ss) is the
     * source of truth, and a flash-then-redraw on every button press just
     * adds redraw flicker for a state the user can already see. */
    ui_update_auto_status_line_only();
}

/**
 * @brief Handle camera wake screen activation (button A on WAKE).
 *
 * Dispatches the BLE wake-broadcast + post-wake reconnect to the worker
 * task so the main loop's button polling stays responsive throughout the
 * 2 s broadcast and the 3 s post-wake settle period.
 */
void ui_screen_wake(void) {
    ESP_LOGI(TAG, "Executing wake screen — dispatching to worker");

    if (!g_stored_camera.is_paired) {
        ui_show_message("No Paired Camera", M5_COLOR_RED, 1500);
        ESP_LOGW(TAG, "No paired camera found for wake broadcast");
        return;
    }

    /* Immediate user feedback so the press registers; the worker will
     * overwrite this toast with progress messages as it runs. */
    ui_show_message("Wake Broadcasting...", M5_COLOR_YELLOW, 1000);
    ui_dispatch_work(UI_WORK_WAKE);
}

/* Original blocking body — runs only on the worker task. */
static void ui_screen_wake_worker(void) {
    ESP_LOGI(TAG, "ui_screen_wake_worker: starting");

    if (!g_stored_camera.is_paired) {
        ui_show_message("No Paired Camera", M5_COLOR_RED, 1500);
        ESP_LOGW(TAG, "No paired camera found for wake broadcast");
        return;
    }

    bool was_disconnected = (connect_logic_get_state() <= BLE_INIT_COMPLETE);

    ESP_LOGI(TAG, "Starting wake broadcast for paired camera");
    esp_err_t ret = ble_wake_camera(g_stored_camera.camera_mac);

    if (ret == ESP_OK) {
        ui_show_message("Wake Broadcast\nSent (2s)", M5_COLOR_YELLOW, 2000);
        ESP_LOGI(TAG, "Wake broadcast started successfully");

        /* If we were disconnected when sending wake, attempt reconnection after delay */
        if (was_disconnected) {
            ESP_LOGI(TAG, "Wake sent while disconnected, will attempt reconnection in 3 seconds");
            vTaskDelay(pdMS_TO_TICKS(3000));  /* Wait for camera to wake up */

            int reconnect_result = ui_perform_complete_reconnection(true);
            if (reconnect_result == 0) {
                ESP_LOGI(TAG, "Reconnection successful after wake broadcast");
            } else {
                ESP_LOGW(TAG, "Failed to reconnect after wake broadcast");
            }
        }
    } else {
        ui_show_message("Wake Failed", M5_COLOR_RED, 1500);
        ESP_LOGE(TAG, "Failed to start wake broadcast: %s", esp_err_to_name(ret));
    }
}