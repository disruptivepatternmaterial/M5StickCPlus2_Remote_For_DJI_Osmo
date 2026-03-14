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

- **Loop Recording**: Enable; segment length 3–5 min (dashcam-style overwrite).
- **Resolution**: 4K (16:9) @ 30 fps or 1080p (16:9) @ 30 fps.
- **Stabilization**: RockSteady (avoid HorizonSteady for dashcam).
- **FOV**: Wide or Natural Wide.
- **Bit rate**: High.
- **Video Quality Priority**: Off for long runs (reduces heat).
- **Screen**: Auto off or short delay (saves power when USB-powered).

If the protocol does not expose a given setting, document the gap and the user
configures it once on the camera; otherwise this firmware sets it on connect or
before start record.

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
