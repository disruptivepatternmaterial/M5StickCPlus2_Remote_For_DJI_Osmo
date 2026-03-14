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
| Still for timeout | **Stop recording** → short delay → **sleep camera**. |

Sequence: wake (if needed) → set mode → shutter start → … → shutter stop → sleep.

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

- **Sensor**: MPU6886 IMU via HAL (`m5stickc_plus2_imu_init()`, `m5stickc_plus2_imu_read_accel()`).
- **Logic**: `logic/motion_logic.c` — states IDLE / MOVING / COUNTDOWN.
- **API**: `motion_logic_just_started()` / `motion_logic_just_stopped()` drive `app_main.c`.
- **Timeout**: configurable; 15 s for testing, 5 min for production.

## GPS

- **Now**: Placeholder stub returning fixed Switzerland coordinates.
  `gps_has_fix()` → true, `gps_get_data()` → stub coords.
- **Later**: Replace `gps/gps.c` with real UART/NMEA driver (M5Stack GPS Module v2.0 on Grove port).
- **Push**: Every 1 s when connected and fix available, via `command_logic_push_gps_data()` (CmdSet 0x00, CmdID 0x17).

## Auto Start/Stop Screen

- Default screen after camera connects.
- Status line shows: **Idle** / **Moving – Recording** / **Still – Stop in M:SS**.
- Same style as other screens (icon, name, status line, nav dots).
- Only the status line redraws, and only when content changes — no full-screen redraw loop.

## Summary Checklist

- [ ] Motion: MPU6886 → motion_logic → `just_started` / `just_stopped` drive app_main.
- [ ] Flow: wake → set Video (0x01) → start record; on still timeout → stop record → sleep.
- [ ] Mode: `CAMERA_MODE_NORMAL` (0x01) sent before start record.
- [ ] GPS: placeholder stub; push when connected; replace with NMEA when hardware wired.
- [ ] Auto screen: status line (idle / moving / still–countdown); default after connect; no excessive redraws.


## Related Code that will provide ideas and information
- https://github.com/rhoenschrat/DJI-Remote
- https://github.com/dji-sdk/Osmo-GPS-Controller-Demo
- https://github.com/xaionaro-go/djictl
- https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit?srsltid=AfmBOoowTnRHHiPWqb0e6R41u3HLd33-PyM8mvr4PGcRLx7NE6iTvOV8
- https://docs.m5stack.com/en/core/m5stickc_plus
- When downloading programs to the device, it is recommended to select one of the following serial baud rates. Using other speeds may cause the program to fail to download correctly. 1500000 bps / 750000 bps / 500000 bps / 250000 bps / 115200 bps
- Added sleep and wake functions, version changed to v1.1
