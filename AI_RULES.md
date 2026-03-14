# AI Development Rules

You are working in an ESP‑IDF firmware repository.

Target device:
M5StickC Plus 1.1

Framework:
ESP‑IDF only.

DO NOT generate:

- Arduino code
- M5Stack Arduino libraries
- M5.begin()
- M5.Lcd
- TFT_eSPI
- Arduino BLE libraries

All hardware access must go through the HAL layer.

Key files:

m5stickc_plus11_hal.c
m5stickc_plus11_hal.h

Motion detection uses the MPU6886 IMU.

BLE is used to control a DJI Osmo Action camera.

SPEC.md is the authoritative project specification.