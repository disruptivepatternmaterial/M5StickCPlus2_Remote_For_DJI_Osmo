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
                    PRIV_REQUIRES bt nvs_flash led_strip esp_lcd driver
                    INCLUDE_DIRS "." "../utils/crc" "../protocol" "../ble" "../data" "../logic" "../gps")

# Device selection: Plus2 (default) or Plus 1.1 (DC=23, RST=18)
# For PlatformIO: use env m5stickc_plus11 and board_build.cmake_extra_args = -DM5STICKC_PLUS_11_BUILD=ON
option(M5STICKC_PLUS_11_BUILD "Build for M5Stick C Plus 1.1 instead of Plus2" OFF)
if(M5STICKC_PLUS_11_BUILD)
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKC_PLUS_11)
else()
  target_compile_definitions(${COMPONENT_LIB} PUBLIC M5STICKC_PLUS2)
endif()
