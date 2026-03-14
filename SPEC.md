# PROJECT SPEC (AUTHORITATIVE)

This file defines the REQUIRED behavior of this firmware.
If generated code conflicts with this spec, the spec is correct.

Framework: ESP-IDF (v5+)
Build system: PlatformIO
Framework setting in platformio.ini:

framework = espidf

Arduino is NOT used in this repository.

Hardware Target:
M5StickC Plus 1.1

Primary Feature:
Motion‑triggered dashcam remote for DJI Osmo Action cameras.

Core Flow:

motion detected →
BLE wake camera →
set video mode (0x01) →
start record

still timeout →
stop record →
sleep camera

Sensors:
MPU6886 IMU used for motion detection.

GPS:
Currently a placeholder returning fixed Switzerland coordinates for testing.
Later replaced with real NMEA GPS driver.

UI:

Default screen: Auto Start/Stop

Screen states:
Idle – No recording
Moving – Recording
Still – Stop countdown

The rest of the repository implements this behavior.