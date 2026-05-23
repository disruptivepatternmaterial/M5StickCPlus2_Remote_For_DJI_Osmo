idf_component_register(SRCS "app_main.c" 
                            "ui.c"
                            "icons.c"
                            "../utils/crc/custom_crc16.c" 
                            "../utils/crc/custom_crc32.c"
                            "../protocol/dji_protocol_parser.c"
                            "../protocol/dji_protocol_data_processor.c"
                            "../protocol/dji_protocol_data_descriptors.c"
                            "../protocol/dji_protocol_data_structures.c"
                            "../ble/ble.c"
                            "../data/data.c"
                            "../logic/connect_logic.c"
                            "../logic/command_logic.c"
                            "../logic/motion_logic.c"
                            "../logic/status_logic.c"
                            "../logic/enums_logic.c"
                            "../logic/light_logic.c"
                            "../gps/gps.c"
                            "m5stickc_plus2_hal.c"
                            "m5stickc_plus11_hal.c"
                            "m5sticks3_hal.c"
                            "m5atoms3_hal.c"
                    PRIV_REQUIRES bt nvs_flash led_strip esp_lcd driver
                    INCLUDE_DIRS "." "../utils/crc" "../protocol" "../ble" "../data" "../logic" "../gps")

# Device selection: Plus2 (default), Plus 1.1, M5 StickS3, or M5 AtomS3 (mutually exclusive)
# PlatformIO:
#   m5stickc_plus11 → -DM5STICKC_PLUS_11_BUILD=ON
#   m5sticks3       → -DM5STICKS3_BUILD=ON
#   m5atoms3        → -DM5ATOMS3_BUILD=ON
option(M5STICKS3_BUILD "Build for M5 StickS3 (ESP32-S3)" OFF)
option(M5STICKC_PLUS_11_BUILD "Build for M5Stick C Plus 1.1 instead of Plus2" OFF)
option(M5ATOMS3_BUILD "Build for M5 AtomS3 (ESP32-S3FN8)" OFF)
if(M5ATOMS3_BUILD)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5ATOMS3)
elseif(M5STICKS3_BUILD)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKS3)
elseif(M5STICKC_PLUS_11_BUILD)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKC_PLUS_11)
else()
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKC_PLUS2)
endif()
