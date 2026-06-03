# PROJECT SPEC (AUTHORITATIVE)

This file defines the REQUIRED behavior of this firmware.
If generated code conflicts with this spec, the spec is correct.

Framework, hardware, and build rules are in `.cursor/rules/project.mdc`.

---

## Primary Feature

Motion-triggered dashcam remote for DJI Osmo Action cameras via BLE.

## Core Flow

| Trigger | Action |
|---------|--------|
| Motion detected | If not connected: BLE wake + reconnect. Once connected: **set Video mode (0x01)** → short delay → **start recording**. |
| Still for timeout | **Stop recording** and keep the camera awake for the next motion event. |

Sequence: wake/reconnect (if needed) → set mode → shutter start → … → shutter stop.

### Stop-record retry watchdog (added 2026-05-26)

The DJI Osmo Action occasionally ignores a one-shot `stop_record` over BLE
and keeps recording silently. To make stop reliable, every stop-record call
site MUST also call `app_request_pending_stop()` (declared in
[`main/app_main.h`](main/app_main.h)). The main loop in
[`main/app_main.c`](main/app_main.c) then re-issues `command_logic_stop_record()`
on a 1.5 s cadence until `is_camera_recording()` returns false, or until a
retry budget of 5 attempts (≈ 7.5 s worst case) is exhausted. The budget
exhaustion is logged at `ESP_LOGE` level (`FLOW: pending_stop GAVE UP …`).

Asymmetric to start_record on purpose: a missed start gets re-fired by the
next motion event in the dashcam flow, but a missed stop leaves the camera
recording silently — the expensive failure mode worth defending against.

The AUTO reconnect path also arms motion and sends `start_record` after setting
Video mode. This avoids depending on a connection-edge race in the main loop:
after a successful reconnect, motion start/stop behavior is active even if the
connection state changed inside the UI worker.

## Camera Mode

- Protocol value: `0x01` (Video).
- In code: `CAMERA_MODE_NORMAL` (see `logic/enums_logic.h`).
- Sent via `command_logic_switch_camera_mode(CAMERA_MODE_NORMAL)` before `command_logic_start_record()`.

**Shooting parameters** (sent by this firmware where the DJI BLE protocol allows):

The DJI BLE protocol as implemented exposes only the following **writable** commands:

| CmdSet | CmdID | Command |
|--------|-------|---------|
| 0x1D   | 0x03  | Record start / stop |
| 0x1D   | 0x04  | Camera mode switch |
| 0x1D   | 0x05  | Status subscription |
| 0x00   | 0x11  | Key report |
| 0x00   | 0x17  | GPS data push |
| 0x00   | 0x19  | Connection request |
| 0x00   | 0x1A  | Power mode (sleep/wake) |

All other camera settings (resolution, FPS, EIS, FOV, bit rate, loop recording,
screen timeout, video quality priority) are received as **read-only** status fields
via the 0x1D:0x02 status push. There is no BLE command to set them remotely.

**⚠ Protocol gaps — configure once on the camera:**

- **Loop Recording**: Enable on the camera; set segment length to 3–5 min (dashcam-style overwrite). The remote cannot set this.
- **Resolution**: Set 4K 16:9 @ 30 fps or 1080p 16:9 @ 30 fps on the camera. The remote cannot set this.
- **Stabilization**: Set RockSteady (EIS mode 1) on the camera. Avoid HorizonSteady for dashcam use. The remote cannot set this.
- **FOV**: Set Wide or Natural Wide on the camera. The remote cannot set this.
- **Bit rate**: Set High on the camera. The remote cannot set this.
- **Video Quality Priority**: Set Off on the camera (reduces heat on long runs). The remote cannot set this.
- **Screen**: Set Auto-off or short delay on the camera (saves power when USB-powered). The remote cannot set this.

## Motion Detection

- **Sensor**: MPU6886 (M5StickC Plus / Plus 1.1, M5 AtomS3) or BMI270 (M5 StickS3) via HAL — `m5stickc_plus2_imu_init()`, `m5stickc_plus2_imu_read_accel()`.
- **Logic**: `logic/motion_logic.c` — states IDLE / MOVING / COUNTDOWN.
- **API**: `motion_logic_just_started()` / `motion_logic_just_stopped()` drive `app_main.c`.
- **Timeout**: configurable; 15 s for testing, 5 min for production.

## GPS

- **M5StickC Plus / Plus 1.1**: Placeholder stub returning fixed Switzerland coordinates.
  `gps_has_fix()` → true, `gps_get_data()` → stub coords.
- **M5 StickS3 + Unit GPS v1.1**: Real UART/NMEA on Grove PORT.A (ESP TX→Yellow G9, ESP RX←White G10), 115200 8N1. `gps_has_fix()` / `gps_get_data()` reflect parsed GGA/RMC when a fix is present.
- **M5 AtomS3 + Unit GPS v1.1**: Real UART/NMEA on HY2.0 PORT.A (ESP TX→Yellow G2, ESP RX←White G1), 115200 8N1. Same parser as StickS3, gated by the `GPS_USE_REAL_UART` compile-time flag in `gps/gps.h`.
- **Push**: Every 1 s when connected and fix available, via `command_logic_push_gps_data()` (CmdSet 0x00, CmdID 0x17).

## Auto Start/Stop Screen

- Default screen after camera connects.
- Status line shows: **Idle** / **Moving – Recording** / **Still – Stop in M:SS**.
- Same style as other screens (icon, name, status line, nav dots).
- Only the status line redraws, and only when content changes — no full-screen redraw loop.

## Board variants

| Board                | Display       | Buttons                                    | UI shape                                |
|----------------------|---------------|--------------------------------------------|-----------------------------------------|
| M5StickC Plus2       | 240×135       | A + B + dedicated PWR                      | Full 6-screen icon UI                   |
| M5StickC Plus 1.1    | 240×135       | A + B + dedicated PWR                      | Full 6-screen icon UI                   |
| M5 StickS3           | 240×135       | A (G11) + B (G12); no PWR (M5PM1 shutdown) | Full 6-screen icon UI                   |
| M5 AtomS3 (variant)  | 128×128       | Single screen-button (G41)                 | Two-screen UI: BT (CONNECT) + AUTO only |

The AtomS3 variant is documented in detail under
[`docs/ATOMS3_MIGRATION_SPEC.md`](docs/ATOMS3_MIGRATION_SPEC.md), including:

- HAL pin map (R1)
- Single-button input model (D-004): short press cycles BT↔AUTO, long press
  is the contextual action (reconnect on BT, manual recording toggle on AUTO).
- Two-screen reduced UI for the 128×128 panel (D-005).
- GPS UART pinout for HY2.0 PORT.A: TX=G2, RX=G1.

## Summary Checklist

- [ ] Motion: MPU6886 → motion_logic → `just_started` / `just_stopped` drive app_main.
- [ ] Flow: wake → set Video (0x01) → start record; on still timeout → stop record → sleep.
- [ ] Mode: `CAMERA_MODE_NORMAL` (0x01) sent before start record.
- [ ] GPS: Plus builds use stub; StickS3 + AtomS3 use NMEA from Unit GPS v1.1; push when connected.
- [ ] Auto screen: status line (idle / moving / still–countdown); default after connect; no excessive redraws.


## Related Code that will provide ideas and information
- https://github.com/rhoenschrat/DJI-Remote
- https://github.com/dji-sdk/Osmo-GPS-Controller-Demo
- https://github.com/xaionaro-go/djictl
- https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit?srsltid=AfmBOoowTnRHHiPWqb0e6R41u3HLd33-PyM8mvr4PGcRLx7NE6iTvOV8
- https://docs.m5stack.com/en/core/m5stickc_plus
- When downloading programs to the device, it is recommended to select one of the following serial baud rates. Using other speeds may cause the program to fail to download correctly. 1500000 bps / 750000 bps / 500000 bps / 250000 bps / 115200 bps
- Added sleep and wake functions, version changed to v1.1
