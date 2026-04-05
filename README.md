# M5Stick Remote for DJI Osmo Action

Open firmware for an M5Stick-based BLE remote focused on dashcam-style use with DJI Osmo Action cameras.

The project target is a motion-driven flow:

- movement starts -> wake/connect camera -> set video mode -> start recording
- movement stops for a timeout -> stop recording -> sleep camera

The authoritative feature behavior is in `SPEC.md`.

## Current Focus

This repo is actively being developed toward a reliable "install it and trust it" remote.

Current work is centered on:

- stable BLE reconnect and command sequencing
- motion-driven start/stop recording logic
- board-specific HAL support across Plus2, Plus 1.1, and StickS3
- GPS path (stub on Plus builds, NMEA on StickS3)
- cleaner on-device status UI during auto start/stop operation

## Supported Hardware Targets

- `M5StickC Plus2` (ESP32)
- `M5StickC Plus 1.1` (ESP32, different LCD control pins)
- `M5 StickS3` (ESP32-S3, BMI270, Unit GPS v1.1 on Grove PORT.A)

Build target selection is done through PlatformIO environments in `platformio.ini`.

## What This Firmware Controls

Through DJI BLE protocol support implemented in this project:

- connect / reconnect
- wake / sleep camera
- switch camera mode
- start / stop recording
- periodic GPS data push when fix is available

Some camera settings are read-only over BLE and must be set once on the camera UI (details in `SPEC.md`).

## Repository Layout

- `main/` app entry point, UI, board HAL integration
- `logic/` command, connection, motion, and UI state logic
- `ble/` BLE link and protocol transport
- `protocol/` DJI protocol parsing/building
- `gps/` GPS abstraction and board-specific behavior
- `scripts/` build/upload and serial log capture helpers
- `SPEC.md` required feature behavior and command constraints

## Build and Flash

Use PlatformIO with `framework = espidf`.

### Quick Commands

From repo root:

```bash
# M5Stick C Plus2
pio run -e m5stickc_plus2
pio run -e m5stickc_plus2 -t upload

# M5Stick C Plus 1.1
pio run -e m5stickc_plus11
pio run -e m5stickc_plus11 -t upload

# M5 StickS3
pio run -e m5sticks3
pio run -e m5sticks3 -t upload
```

If `pio` is not on `PATH`:

```bash
"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus2
```

### Helper Scripts

- `./scripts/build_and_upload_plus11.sh`
- `./scripts/build_and_upload_sticks3.sh`
- `./scripts/capture_logs.sh`
- `python3 scripts/capture_serial.py 40 .cursor/debug-7ee220.log`

## Device Notes

### Plus2 vs Plus 1.1 display pins

Plus2 and Plus 1.1 use different LCD `DC` and `RST` pins.  
Flashing the wrong build target commonly causes blank/corrupt display output.

### GPS behavior

- Plus2 / Plus 1.1: current GPS path is a stub
- StickS3: reads NMEA from Unit GPS v1.1 over UART

## Development Workflow

Recommended loop for every change:

1. edit
2. build
3. upload
4. capture serial logs during test
5. iterate

This keeps behavior claims tied to build output and runtime evidence.

## Project Status

This is a working in-progress firmware project, not a final release.

Planned release quality goals:

- reliable reconnect across camera power cycles
- robust motion-triggered recording flow without false stops
- clear status UI with minimal redraw overhead
- stable GPS push behavior on StickS3

## References

- [SPEC.md](SPEC.md)
- [M5StickC Plus2 docs](https://docs.m5stack.com/en/core/M5StickC%20Plus2)
- [M5 StickS3 docs](https://docs.m5stack.com/en/core/StickS3)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [Project article](https://serialhobbyism.com/open-source-diy-remote-for-dji-osmo-action-5-pro-cameras)
