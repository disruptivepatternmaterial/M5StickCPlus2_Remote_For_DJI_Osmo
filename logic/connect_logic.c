/*
 * DJI Camera Remote Control - Connection Logic Layer
 * 
 * This file implements the complete connection management system for DJI camera
 * communication, handling both BLE and protocol-level connections.
 * 
 * Connection Flow:
 * 1. BLE Initialization: Set up ESP32 BLE stack
 * 2. Device Discovery: Scan for and connect to target camera
 * 3. Service Discovery: Find DJI communication characteristics
 * 4. Protocol Handshake: Establish DJI protocol connection
 * 5. Maintenance: Handle disconnections and reconnection attempts
 * 
 * State Management:
 * - BLE_NOT_INIT: Initial state before BLE initialization
 * - BLE_INIT_COMPLETE: BLE ready, no connection
 * - BLE_SEARCHING: Actively scanning for cameras
 * - BLE_CONNECTED: BLE link established, protocol pending
 * - PROTOCOL_CONNECTED: Full connection, ready for commands
 * - BLE_DISCONNECTING: Graceful disconnection in progress
 * 
 * Connection Types:
 * - Pairing (verify_mode=1): Initial camera registration
 * - Reconnection (verify_mode=0): Automatic connection to known camera
 * - Wake-up: BLE advertising to rouse sleeping cameras
 * 
 * Error Handling:
 * - Automatic reconnection attempts on unexpected disconnection
 * - Timeout management for all connection phases
 * - State restoration on connection failures
 * 
 * Hardware: M5StickC Plus2 with ESP32 BLE capabilities
 * Protocol: DJI proprietary communication protocol
 * 
 * Based on DJI SDK implementation
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

#include "ble.h"
#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

/* BLE connection wait parameters.
 * The poll interval and maximum count together define how long we wait for
 * the BLE stack to confirm a connection and for GATT handle discovery.
 * Each phase is capped at BLE_WAIT_POLLS × BLE_POLL_INTERVAL_MS milliseconds. */
#define BLE_POLL_INTERVAL_MS    100U   /* ms between each BLE readiness poll   */
#define BLE_WAIT_POLLS          100U   /* 100 × 100 ms = 10 s per phase        */

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_CONNECT"

static void connect_debug_log(const char *hypothesis_id,
                              const char *location,
                              const char *message,
                              const char *data_json) {
    static uint32_t seq = 0U;
    char buf[256];
    int64_t ts_ms = esp_timer_get_time() / 1000;
    seq++;
    snprintf(buf, sizeof(buf),
             "{\"sessionId\":\"2204a3\",\"id\":\"log_%lld_%lu\",\"timestamp\":%lld,"
             "\"location\":\"%s\",\"message\":\"%s\",\"data\":%s,"
             "\"runId\":\"connect-pre\",\"hypothesisId\":\"%s\"}",
             (long long)ts_ms, (unsigned long)seq, (long long)ts_ms,
             location, message, data_json ? data_json : "{}", hypothesis_id);
    printf("DBGJSON %s\n", buf);
}

/* Global connection state tracking
 * Manages the current state of BLE and protocol connections
 * Used throughout the system to determine available operations
 */
static connect_state_t connect_state = BLE_NOT_INIT;
static volatile bool s_unexpected_disconnect_pending = false;
/** Prevents overlapping BLE connect + protocol work (startup + UI + background). */
static volatile bool s_ble_connect_busy = false;

/**
 * @brief Get current connection state
 * 
 * Returns the current state of the connection system, allowing other
 * components to determine what operations are available and respond
 * appropriately to connection status changes.
 * 
 * @return connect_state_t Current connection state
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection events
 * 
 * Callback function triggered when the BLE connection to the camera is lost.
 * Implements sophisticated disconnection handling based on current state:
 * 
 * - Expected disconnections: Clean state reset
 * - Unexpected disconnections: Automatic reconnection attempt
 * - Failed reconnections: Graceful fallback to disconnected state
 * 
 * The function attempts one automatic reconnection for unexpected disconnections
 * to maintain seamless operation during temporary connection issues.
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            /* Already searching - no action needed */
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            ESP_LOGI(TAG, "Normal disconnection process.");
            /* Expected disconnection - clean state reset */
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            /* Do not block the BLE callback task. Reconnection is handled elsewhere. */
            char dbg_data[64];
            snprintf(dbg_data, sizeof(dbg_data), "{\"state\":%d}", (int)connect_state);
            // #region agent log: H4 unexpected_disconnect
            connect_debug_log("H4", "connect_logic.c:unexpected_disconnect",
                              "unexpected_disconnect", dbg_data);
            // #endregion
            ESP_LOGW(TAG, "Unexpected disconnection from state: %d", connect_state);
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            s_unexpected_disconnect_pending = true;
            ESP_LOGI(TAG, "Marked reconnect request for main loop handling.");
            break;
        }
    }
}

bool connect_logic_consume_unexpected_disconnect(void) {
    if (!s_unexpected_disconnect_pending) {
        return false;
    }
    s_unexpected_disconnect_pending = false;
    return true;
}

/**
 * @brief Initialize BLE subsystem for camera communication
 * 
 * Performs one-time initialization of the ESP32 BLE stack and prepares
 * the system for camera connections. This must be called before any
 * connection attempts.
 * 
 * Initialization includes:
 * - ESP32 BLE controller and host setup
 * - GATT client profile registration
 * - Service and characteristic UUID configuration
 * - Connection parameter setup
 * 
 * @return 0 on success, -1 on failure
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* Initialize ESP32 BLE stack with DJI camera service configuration */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;
    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Connect to BLE device
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 
 * If connection fails, returns error and resets connection state.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_connect(bool is_reconnecting) {
    const char *fail_reason = NULL;
    int fail_err = 0;
    char dbg_data[120];

    if (s_ble_connect_busy) {
        ESP_LOGW(TAG, "BLE connect already in progress — ignored");
        return -1;
    }
    if (connect_state == BLE_CONNECTED || connect_state == PROTOCOL_CONNECTED) {
        ESP_LOGW(TAG, "Already connected — disconnect before a new BLE session");
        return -1;
    }
    s_ble_connect_busy = true;

    connect_state = BLE_SEARCHING;

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    snprintf(dbg_data, sizeof(dbg_data),
             "{\"reconnecting\":%d,\"state\":%d}",
             is_reconnecting ? 1 : 0, (int)connect_state);
    // #region agent log: H2 ble_connect_start
    connect_debug_log("H2", "connect_logic.c:ble_connect_start",
                      "ble_connect_start", dbg_data);
    // #endregion

    /* 2. Start scanning and attempt connection */
    ble_set_reconnecting(is_reconnecting);
    ret = ble_start_scanning_and_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning and connect, error: 0x%x", ret);
        fail_reason = "scan_start";
        fail_err = (int)ret;
        goto connect_fail;
    }

    /* 3. Wait up to 10 seconds for the BLE connection to be established */
    ESP_LOGI(TAG, "Waiting up to %us for BLE to connect...", (BLE_WAIT_POLLS * BLE_POLL_INTERVAL_MS) / 1000U);
    bool connected = false;
    for (uint32_t i = 0; i < BLE_WAIT_POLLS; i++) {
        if (s_ble_profile.connection_status.is_connected) {
            ESP_LOGI(TAG, "BLE connected successfully");
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(BLE_POLL_INTERVAL_MS));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out after %ums", BLE_WAIT_POLLS * BLE_POLL_INTERVAL_MS);
        fail_reason = "connect_timeout";
        goto connect_fail;
    }

    /* 4. Wait up to 10 seconds for GATT characteristic handle discovery to complete */
    ESP_LOGI(TAG, "Waiting up to %us for characteristic handle discovery...", (BLE_WAIT_POLLS * BLE_POLL_INTERVAL_MS) / 1000U);
    bool handles_found = false;
    for (uint32_t i = 0; i < BLE_WAIT_POLLS; i++) {
        if (s_ble_profile.handle_discovery.notify_char_handle_found &&
            s_ble_profile.handle_discovery.write_char_handle_found) {
            ESP_LOGI(TAG, "Required characteristic handles found");
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(BLE_POLL_INTERVAL_MS));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within %ums timeout", BLE_WAIT_POLLS * BLE_POLL_INTERVAL_MS);
        fail_reason = "handles_timeout";
        goto connect_fail;
    }

    /* 5. Register notification */
    ret = ble_register_notify(s_ble_profile.conn_id, s_ble_profile.notify_char_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify, error: %s", esp_err_to_name(ret));
        fail_reason = "notify_fail";
        fail_err = (int)ret;
        goto connect_fail;
    }

    /* Update state to BLE connected */
    connect_state = BLE_CONNECTED;

    /* Short settle delay so LED / status updates can catch up (was 2000 ms; blocked main task). */
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "BLE connect successfully");
    snprintf(dbg_data, sizeof(dbg_data),
             "{\"reconnecting\":%d}", is_reconnecting ? 1 : 0);
    // #region agent log: H2 ble_connect_ok
    connect_debug_log("H2", "connect_logic.c:ble_connect_ok",
                      "ble_connect_ok", dbg_data);
    // #endregion
    s_ble_connect_busy = false;
    return 0;

connect_fail:
    connect_state = BLE_INIT_COMPLETE;
    s_ble_connect_busy = false;
    snprintf(dbg_data, sizeof(dbg_data),
             "{\"reason\":\"%s\",\"err\":%d,\"reconnecting\":%d}",
             fail_reason ? fail_reason : "unknown", fail_err, is_reconnecting ? 1 : 0);
    // #region agent log: H2 ble_connect_fail
    connect_debug_log("H2", "connect_logic.c:ble_connect_fail",
                      "ble_connect_fail", dbg_data);
    // #endregion
    return -1;
}

/**
 * @brief Disconnect BLE connection
 * 
 * Attempt to disconnect from BLE device.
 * 
 * @return int Returns 0 on success, -1 on failure
 */
int connect_logic_ble_disconnect(void) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;
    
    ESP_LOGI(TAG, "Disconnecting camera...");

    // Call BLE layer's ble_disconnect function
    esp_err_t ret = ble_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera, BLE error: %s", esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera disconnected successfully");
    return 0;
}

/**
 * @brief Establish DJI protocol connection with camera
 * 
 * Performs the complete DJI protocol handshake sequence over the established
 * BLE connection. This involves a complex bidirectional authentication process
 * that varies depending on whether this is a new pairing or reconnection.
 * 
 * Protocol Handshake Sequence:
 * 1. Send connection request with device credentials
 * 2. Handle camera response (may be response or command frame)
 * 3. Wait for camera's connection command with verification
 * 4. Send final connection response to complete handshake
 * 
 * Verification Modes:
 * - verify_mode=0: Reconnection to previously paired camera
 * - verify_mode=1: New pairing requiring camera-side confirmation
 * - verify_mode=2: Camera verification response
 * 
 * @param device_id Unique device identifier for this remote
 * @param mac_addr_len Length of MAC address (typically 6)
 * @param mac_addr Device MAC address for protocol identification
 * @param fw_version Firmware version for compatibility checking
 * @param verify_mode Authentication mode (0=reconnect, 1=pair)
 * @param verify_data Random verification code for security
 * @param camera_reserved Camera-specific identifier
 * @return 0 on successful protocol connection, -1 on failure
 */
int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    const char *result_reason = "unknown";
    int result_code = 0;
    char dbg_data[140];

    ESP_LOGI(TAG, "%s: Starting protocol connection", __FUNCTION__);
    uint16_t seq = generate_seq();

    snprintf(dbg_data, sizeof(dbg_data),
             "{\"verify_mode\":%u,\"verify_data\":%u}",
             (unsigned)verify_mode, (unsigned)verify_data);
    // #region agent log: H3 protocol_start
    connect_debug_log("H3", "connect_logic.c:protocol_start",
                      "protocol_start", dbg_data);
    // #endregion

    /* Construct DJI protocol connection request frame */
    connection_request_command_frame connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    memcpy(connection_request.mac_addr, mac_addr, mac_addr_len);


    // STEP1: Send connection request command to camera
    ESP_LOGI(TAG, "Sending connection request to camera...");
    CommandResult result = send_command(0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, 5000);

    /**** Connection issue: camera may return either response frame or command frame ****/

    if (result.structure == NULL) {
        // If a command frame is sent, execute this block of code

        // Directly call data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        
        // If != OK, it means no message was received, timeout occurred
        
        // Otherwise, GOTO wait_for_camera_command label
        void *parse_result = NULL;
        size_t parse_result_length = 0;
        uint16_t received_seq = 0;
        esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 5000, &received_seq, &parse_result, &parse_result_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout or error waiting for camera connection command, GOTO Failed.");
            result_reason = "wait_cmd_timeout";
            result_code = (int)ret;
            connect_logic_ble_disconnect();
            goto protocol_fail;
        }
        // If data is received, skip parsing camera response and directly enter STEP3
        goto wait_for_camera_command;
    }

    // STEP2: Parse the response returned from camera
    connection_request_response_frame *response = (connection_request_response_frame *)result.structure;
    if (response->ret_code != 0) {
        ESP_LOGE(TAG, "Connection handshake failed: unexpected response from camera, ret_code: %d", response->ret_code);
        result_reason = "handshake_ret";
        result_code = response->ret_code;
        free(response);
        connect_logic_ble_disconnect();
        goto protocol_fail;
    }

    ESP_LOGI(TAG, "Handshake successful, waiting for the camera to actively send the connection command frame...");
    free(response);

    // STEP3: Wait for camera to send connection request
wait_for_camera_command:
    void *parse_result = NULL;
    size_t parse_result_length = 0;
    uint16_t received_seq = 0;
    esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);

    if (ret != ESP_OK || parse_result == NULL) {
        ESP_LOGE(TAG, "Timeout or error waiting for camera connection command");
        result_reason = "wait_cmd_timeout";
        result_code = (int)ret;
        connect_logic_ble_disconnect();
        goto protocol_fail;
    }

    // Parse the connection request command sent by camera
    connection_request_command_frame *camera_request = (connection_request_command_frame *)parse_result;

    if (camera_request->verify_mode != 2) {
        ESP_LOGE(TAG, "Unexpected verify_mode from camera: %d", camera_request->verify_mode);
        result_reason = "verify_mode";
        result_code = camera_request->verify_mode;
        free(parse_result);
        connect_logic_ble_disconnect();
        goto protocol_fail;
    }

    if (camera_request->verify_data == 0) {
        ESP_LOGI(TAG, "Camera approved the connection, sending response...");

        // Construct connection response frame
        connection_request_response_frame connection_response = {
            .device_id = device_id,
            .ret_code = 0,
        };
        memset(connection_response.reserved, 0, sizeof(connection_response.reserved));
        connection_response.reserved[0] = camera_reserved;

        ESP_LOGI(TAG, "Constructed connection response, sending...");

        // STEP4: Send connection response frame
        send_command(0x00, 0x19, ACK_NO_RESPONSE, &connection_response, received_seq, 5000);

        // Set connection state to protocol connected
        connect_state = PROTOCOL_CONNECTED;

        ESP_LOGI(TAG, "Connection successfully established with camera.");
        free(parse_result);
        result_reason = "success";
        result_code = 0;
        goto protocol_success;
    } else {
        ESP_LOGW(TAG, "Camera rejected the connection, closing Bluetooth link...");
        result_reason = "camera_reject";
        result_code = camera_request->verify_data;
        free(parse_result);
        connect_logic_ble_disconnect();
        goto protocol_fail;
    }

protocol_success:
    snprintf(dbg_data, sizeof(dbg_data),
             "{\"result\":\"%s\",\"code\":%d}", result_reason, result_code);
    // #region agent log: H3 protocol_result
    connect_debug_log("H3", "connect_logic.c:protocol_result",
                      "protocol_result", dbg_data);
    // #endregion
    return 0;

protocol_fail:
    snprintf(dbg_data, sizeof(dbg_data),
             "{\"result\":\"%s\",\"code\":%d}", result_reason, result_code);
    // #region agent log: H3 protocol_result
    connect_debug_log("H3", "connect_logic.c:protocol_result",
                      "protocol_result", dbg_data);
    // #endregion
    return -1;
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}