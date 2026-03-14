## Description

![PXL_20250820_213936612](https://github.com/user-attachments/assets/153c29c2-81bd-451d-8bd2-8c7f80474369)

M5StickC Plus2 remote control for DJI Osmo Action 5 Pro cameras.

Switch modes, start/stop recording, capture photo, sleep, wake, automatic reconnection, and GPIO control. 

**Motion-triggered dashcam**: Start/stop recording on movement (MPU6886), with Auto Start/Stop screen and GPS telemetry (placeholder until module wired). See **[SPEC.md](SPEC.md)** for the full specification (mode, flow, GPS, screen).

More info here: https://serialhobbyism.com/open-source-diy-remote-for-dji-osmo-action-5-pro-cameras

Demo on YouTube: https://youtube.com/shorts/t92D7x2sBuA?feature=share

## Hardware Specifications

The M5StickC Plus2 features:
- **MCU**: ESP32-PICO-V3-02 (Dual-core ESP32, 240MHz)
- **Display**: 1.14" TFT LCD (135x240 pixels, ST7789V2 driver)
- **Flash**: 4MB
- **PSRAM**: None
- **Buttons**: 3 (Button A/Home, Button B/Side, Power)
- **IMU**: MPU6886 (6-axis)
- **RTC**: BM8563
- **PMU**: None (Plus2). M5StickC Plus 1.1 uses AXP192.
- **Buzzer**: Built-in
- **IR Transmitter**: Built-in
- **LED**: Internal red LED
- **Grove Port**: For external GPS module connection

## Pin Mappings

### Display (ST7789V2)
- MOSI: GPIO 15
- SCLK: GPIO 13
- CS: GPIO 5
- DC: GPIO 14
- RST: GPIO 12
- Backlight: GPIO 27

### Buttons
- Button A (Home): GPIO 37
- Button B (Side): GPIO 39
- Power Button: GPIO 35

### I2C (IMU, RTC, PMU)
- SDA: GPIO 21
- SCL: GPIO 22

### Grove Port (UART for GPS)
- TX: GPIO 32
- RX: GPIO 33

### Other
- Internal LED: GPIO 10
- Buzzer: GPIO 2
- IR: GPIO 9
- Power Enable: GPIO 4

## Supported devices

- **M5Stick C Plus2** (default): LCD DC=14, RST=12.
- **M5Stick C Plus 1.1**: LCD DC=23, RST=18 (same resolution 240×135). Use when building for Plus 1.1.

## Building the Project

### Rule: Build and flash before claiming anything works

**Code changes are not verified until you build and flash the firmware to the device.** If you edit code (or an agent does) without building and uploading, the device is still running the previous firmware. Always run a full build and upload after making changes, and confirm the build/upload succeeds, before testing or iterating further.

### One-command build and upload (recommended)

From the repo root:

- **M5Stick C Plus 1.1**:  
  `./scripts/build_and_upload_plus11.sh`
- **M5Stick C Plus 2**:  
  Use PlatformIO directly (see below) or add a similar script if needed.

Options: `--no-upload` to build only; `--clean` to clean before building.

**If `pio` is not on your PATH** (e.g. in Cursor, CI, or some terminals), run the same steps using the PlatformIO venv:

```bash
# Plus 1.1: build and upload
"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11
"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11 -t upload
```

Or run the script; it auto-detects `pio` from PATH, then `~/.platformio/penv/bin/pio`, then `python3 -m platformio`.

**Capture serial to a file (no TTY needed):** With the device connected, run `./scripts/capture_logs.sh` or `python3 scripts/capture_serial.py [seconds] [output.log]` (default 40s → `.cursor/debug-7ee220.log`). Requires `pyserial` (`pip install pyserial`). Use this to capture logs for debugging or to provide runtime evidence to an agent; see CONTRIBUTING.md "Capture device logs for debugging".

### Prerequisites
1. Install ESP-IDF v5.0 or later (or use PlatformIO with `espidf` framework).
2. Set up the ESP-IDF environment (for idf.py builds):
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```
   Or on Windows:
   ```cmd
   %IDF_PATH%\export.bat
   ```

### Build with PlatformIO (recommended)

Use the environment that matches your device:

- **M5Stick C Plus2**:  
  `pio run -e m5stickc_plus2`
- **M5Stick C Plus 1.1**:  
  `pio run -e m5stickc_plus11`

Upload and monitor:

```bash
pio run -e m5stickc_plus2 -t upload    # or m5stickc_plus11
pio device monitor
```

Device selection is done by choosing the env; no code or config edits needed.

Note: PlatformIO currently uses `board = m5stick-c` as a stand-in for Plus2/Plus 1.1 in this project.

If `main/m5stickc_plus11_hal.c` is missing (e.g. first clone), generate it from the Plus2 HAL:

```bash
python3 scripts/generate_plus11_hal.py
```

### Build with ESP-IDF (idf.py)

#### IMPORTANT: Set the correct target first!

1. Clean any previous build:
   ```bash
   idf.py fullclean
   ```

2. **CRITICAL**: Set target to ESP32 for M5StickC Plus2:
   ```bash
   idf.py set-target esp32
   ```
   
   Note: M5StickC Plus2 uses ESP32-PICO-V3-02, which is a standard ESP32 chip.

3. Copy the M5StickC Plus2 specific configuration:
   ```bash
   cp sdkconfig.defaults.m5stickc_plus2 sdkconfig.defaults
   ```
   Or on Windows:
   ```cmd
   copy sdkconfig.defaults.m5stickc_plus2 sdkconfig.defaults
   ```

4. **Device selection (idf.py)**  
   By default the build is for **Plus2**. To build for **M5Stick C Plus 1.1** (LCD DC=23, RST=18), pass the CMake option:
   ```bash
   idf.py -DM5STICKC_PLUS_11_BUILD=ON build
   ```

5. Configure the project (optional):
   ```bash
   idf.py menuconfig
   ```

6. Build the project (if not already built in step 4):
   ```bash
   idf.py build
   ```

7. Flash to device:
   ```bash
   idf.py -p COM[X] flash monitor
   ```
   Replace `COM[X]` with your device's serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

## Features

### Display
The M5StickC Plus2's display shows:
- Boot status and initialization progress
- BLE connection status
- GPS status
- Camera connection status

### Button Functions
- **Button A (GPIO 37)**: Main control button - single press for commands, long press for reconnection
- **Button B (GPIO 39)**: Status display and secondary functions
- **Power Button (GPIO 35)**: Power management

### LED Indicators
The internal LED (GPIO 19) provides visual feedback:
- Different colors/patterns indicate various states
- Controlled via the light_logic module

## Troubleshooting

### Blank or corrupt screen (most common cause: wrong build for hardware)

**The Plus2 and Plus 1.1 use different LCD pins (DC and RST). Flashing a Plus2 build onto a Plus 1.1 (or vice versa) drives the wrong display pins and produces a blank or corrupted screen.**

- **Plus2**: DC = GPIO 14, RST = GPIO 12 → build with `pio run -e m5stickc_plus2` (default)
- **Plus 1.1**: DC = GPIO 23, RST = GPIO 18 → build with `pio run -e m5stickc_plus11` (or `./scripts/build_and_upload_plus11.sh`)

If the screen is blank after flashing, always check you built for the correct device first. If you are using `idf.py` for Plus 1.1, use:
```bash
idf.py -DM5STICKC_PLUS_11_BUILD=ON build
```
(Default is Plus2.)

Capture serial output from power-on with `./scripts/capture_logs.sh` to see whether HAL, BLE, or data-layer init failed — these early failures also result in a blank screen.

### Display Issues
- Ensure the display driver (ST7789) is properly initialized
- Check backlight path for your board:
  - Plus2: GPIO backlight pin is enabled
  - Plus 1.1: AXP192 LDO2 backlight is enabled
- Verify SPI pins are correctly configured

### Button Not Responding
- M5StickC Plus2 buttons are active low (pressed = 0)
- Internal pull-ups are enabled
- Check GPIO 37 (Button A), GPIO 39 (Button B), GPIO 35 (Power)

### Device not connecting / "No longer connects"

If the device powers on and shows the UI but never connects to the camera:
1. Capture serial logs: run `./scripts/capture_logs.sh` with the device plugged in, then attempt connect/reconnect.
2. Look for: "BLE connection timed out", "Characteristic handles not found", "Protocol connect failed", or "No paired camera found in storage".
3. If NVS was erased (full flash erase, partition change, or first flash of new build) the device has no stored camera data and requires manual pairing via the CONNECT screen.
4. Ensure the camera is powered on, discoverable, and within BLE range.

### Build Errors
- Ensure ESP-IDF v5.0+ is installed
- Target must be set to esp32
- Clean build directory if switching from another target
- If `pio` is not found, use `"$HOME/.platformio/penv/bin/pio"` or run `./scripts/build_and_upload_plus11.sh` (script resolves `pio` for you)

## Power Management
- **M5StickC Plus2**: uses GPIO power hold (`M5_PWR_EN_PIN = GPIO4`) and GPIO backlight control.
- **M5StickC Plus 1.1**: uses AXP192 (I2C `0x34`) for TFT backlight control (LDO2).

## Additional Resources
- [M5StickC Plus2 Documentation](https://docs.m5stack.com/en/core/M5StickC%20Plus2)
- [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [ESP-IDF Programming Guide (ESP32)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
