# AI Code Contract

Before generating or modifying code, follow these rules.

## Framework

This repository uses **ESP-IDF only**.

Do NOT generate:

- Arduino code
- Arduino libraries
- M5Stack Arduino APIs
- TFT_eSPI
- M5.begin()
- M5.Lcd

If code contains these, it is incorrect.

## Hardware

Target device: **M5StickC Plus 1.1**

Key components:

- ESP32-PICO
- MPU6886 IMU
- ST7789 display
- BLE communication

## Architecture

Hardware access must go through the HAL layer.

Files:
main/m5stickc_plus11_hal.c
main/m5stickc_plus11_hal.h


Do not bypass the HAL.

## Project goal

Control a **DJI Osmo Action camera** via BLE.

Motion detection from the **MPU6886** triggers:

motion start →
wake camera →
switch video mode →
start record

motion stop →
stop record →
sleep camera

## Source of truth

If there is conflict:

1. SPEC.md
2. .cursor/rules
3. This contract


