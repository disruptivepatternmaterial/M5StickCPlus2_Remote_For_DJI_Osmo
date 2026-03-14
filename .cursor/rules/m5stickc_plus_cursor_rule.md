---
alwaysApply: true
description: Development rules for M5StickC Plus hardware (this repo)
---

# M5StickC Plus Development Rule

All firmware and code in this repository targets the M5StickC Plus family
(M5StickC Plus2 and/or Plus 1.1).

When generating code, the assistant MUST follow the actual hardware
specifications of these devices and must NOT invent capabilities or
hardware. See https://docs.m5stack.com/en/core/m5stickc_plus and the
README pin mappings.

## Device constraints

Target hardware: M5StickC Plus 1.1

Key components:

- CPU: ESP32-PICO-V3-02 (Plus2) / ESP32-PICO-D4 (Plus 1.1)
- Display: 1.14" TFT LCD, 240×135, driver ST7789V2 (Plus2) or ST7789 (Plus 1.1)
- Buttons: BtnA (GPIO 37), BtnB (GPIO 39), Power (GPIO 35)
- Battery: 120 mAh LiPo
- IMU: MPU6886 (I2C)
- Connectivity: WiFi (ESP32), Bluetooth Low Energy (ESP32 BLE)

Plus 1.1 differs from Plus2 in LCD pins (DC=23, RST=18 vs 14/12) and PMU (AXP192).

## Build and flash (mandatory before verification)

Code changes are **not** on the device until you build and upload. Do not assume fixes or features work until you have run a successful build and flash.

- **Plus 1.1**: From repo root run `./scripts/build_and_upload_plus11.sh`, or if `pio` is not on PATH: `"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11` then `"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11 -t upload`.
- After making firmware changes, run build and upload and confirm both succeed before testing or iterating further.

## Runtime logs for debugging

To fix bugs with evidence (not guesses): get serial output from the device. In many environments the agent cannot open the USB serial port. The user should run **with the device connected**: `./scripts/capture_logs.sh` (or `python3 scripts/capture_serial.py 40 .cursor/debug-7ee220.log`). That writes to `.cursor/debug-7ee220.log`. The agent then reads that file to see FLOW lines, panics, and errors. Do not iterate on firmware fixes without either (a) runtime log evidence from this capture, or (b) explicit user confirmation of behavior.

## Required software framework (this repo)

This repository uses **ESP-IDF** (v5.0+), not Arduino.

- Build: PlatformIO with `framework = espidf`, or `idf.py`.
- Hardware abstraction: `main/m5stickc_plus2_hal.c` and `main/m5stickc_plus11_hal.c` provide display, buttons, IMU (MPU6886), and power. Use these HAL APIs; do not introduce Arduino or M5Stack library calls (e.g. no M5.begin(), M5.Lcd, M5.IMU).
- Display: HAL functions (e.g. `m5stickc_plus2_display_*`). Resolution 240×135, landscape. Not touch enabled.
- Buttons: HAL (e.g. `m5stickc_plus2_button_a_pressed()`). Power button used for shutdown hold detection.
- IMU: `m5stickc_plus2_imu_init()`, `m5stickc_plus2_imu_read_accel()` from the HAL.

Do not suggest Arduino, M5StickCPlus library, or TFT_eSPI for this codebase.

## GPIO constraints

GPIO usage must match the README pin mappings. Do not assume arbitrary pins;
Plus2/Plus 1.1 expose limited GPIO (display, I2C, Grove UART for GPS, etc.).

## Power constraints

Battery capacity is small (120 mAh). Avoid tight polling loops, unnecessary
WiFi scanning; prefer sleep when possible.

## Bluetooth rules

BLE only. Prefer BLE over classic Bluetooth.

## Prohibited behavior

Do NOT:

- invent hardware features
- assume a touchscreen
- assume GPS hardware is present (placeholder/stub is allowed per SPEC.md)
- assume a camera on the stick
- assume large RAM or storage
- use Arduino or M5Stack libraries in this repo
- use libraries incompatible with ESP32 / ESP-IDF

## If uncertain

If a requested feature exceeds the capabilities of the device or conflicts
with the SPEC (motion-triggered dashcam, Auto screen, etc.), explain the
limitation instead of inventing a solution.
