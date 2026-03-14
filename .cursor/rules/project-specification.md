# M5StickC DJI Osmo Remote — Specification

This document is the single source of truth for the **motion-triggered dashcam** behaviour, camera mode, GPS, and Auto Start/Stop UI. It exists so the same requirements are not repeated in every conversation.

---

## Implementation status and gaps

The spec is **not** fully implemented or verified. Below is what the spec requires and where to implement or verify it.

| Spec requirement | Where it should live | Status / gap |
|------------------|----------------------|--------------|
| **Motion detection** (MPU6886 → motion logic) | HAL: IMU init + read accel. `logic/motion_logic.c`: sample, thresholds, IDLE/MOVING/COUNTDOWN, `just_started` / `just_stopped`. `main/app_main.c`: call `motion_logic_init()`, `motion_logic_update()` every 100 ms. | Code exists in `main/m5stickc_plus2_hal.c`, `main/m5stickc_plus11_hal.c`, `logic/motion_logic.c`, `app_main.c`. **Verify:** IMU init succeeds on device; motion triggers in car. |
| **Flow: wake → set Video mode → start record** | `app_main.c`: on `motion_logic_just_started()` set pending; if disconnected call `connect_logic_ble_wakeup()` + `ui_attempt_background_reconnection()`. When connected and pending: `command_logic_switch_camera_mode(CAMERA_MODE_NORMAL)` then `command_logic_start_record()`. | Code exists. **Verify:** Camera actually switches to Video (0x01) and starts recording; mode switch uses CmdSet/CmdID correct for Action 6 (currently 0x1D/0x04). |
| **Flow: still timeout → stop record → sleep** | `app_main.c`: on `motion_logic_just_stopped()` and `s_is_recording`: `command_logic_stop_record()`, delay, `command_logic_power_mode_switch_sleep()`. `motion_logic.c`: MOTION_STOP_TIMEOUT_MS (e.g. 15 s test / 5 min prod). | Code exists. **Verify:** Camera stops and sleeps; timeout value is correct. |
| **Mode = Video 0x01** | `logic/enums_logic.h`: `CAMERA_MODE_NORMAL = 0x01`. `command_logic.c`: `command_logic_switch_camera_mode(mode)` sends `.mode = mode`. | Code exists. **Verify:** Protocol command (0x1D/0x04?) is the right one for “set mode” on Osmo Action 6. |
| **GPS** | `gps/gps.c`: placeholder; stub Switzerland so `gps_has_fix()` true, `gps_get_data()` returns coords. `app_main.c`: every 1 s if fix and connected, build frame and call `command_logic_push_gps_data()`. Protocol: CmdSet 0x00, CmdID 0x17. | Code exists. **Verify:** Frame format matches camera; real NMEA driver when hardware wired. |
| **Auto Start/Stop screen** | `main/ui.h`: `SCREEN_AUTO`, add to `screen_info`. `main/ui.c`: draw status line (Idle / Moving–Recording / Still–Stop in M:SS); `ui_update_auto_status_line_only()` only when content changes; on connect success set `current_screen = SCREEN_AUTO`. | Code exists. **Verify:** Default screen after connect; no full-screen redraw loop; status reflects real state. |

**Checklist (implement and verify):**

- [ ] Motion: MPU6886 init and read in HAL; motion_logic wired; `just_started` / `just_stopped` drive app_main.
- [ ] Flow: wake (if needed) → set Video (0x01) → start record; on still timeout → stop record → sleep.
- [ ] Mode: `CAMERA_MODE_NORMAL` (0x01) sent before start record; protocol command verified for Action 6.
- [ ] GPS: placeholder stubbed to Switzerland; push when connected; replace with NMEA when module wired.
- [ ] Auto screen: status line (idle / moving–recording / still–countdown); default after connect; no excessive redraws.

---

## 1. Goal

Use the M5StickC (Plus2 or Plus 1.1) to turn a DJI Osmo Action 6 (or compatible) into a **motion-triggered dashcam**:

- **Start recording** when the **MPU6886** detects movement (e.g. car moving).
- **Stop recording** after a **period of no movement** (configurable; e.g. 5 minutes for production, 15 seconds for testing).
- Use **sleep/wake** so the camera is woken when motion starts and put to sleep when recording stops.

The remote sends **wake** → **set correct mode** → **shutter (start record)** when motion starts, and **shutter (stop record)** → **sleep** when motion has been still for the configured timeout.

---

## 2. Flow (what the firmware does)

| Trigger              | Action |
|----------------------|--------|
| Motion detected      | If camera not connected: BLE wake + background reconnection. Once connected: **switch camera to Video mode** → short delay → **start recording**. |
| Still for timeout    | **Stop recording** → short delay → **send sleep command** to camera. |

So: **wake (if needed) → set mode → shutter start → (later) shutter stop → sleep**. The “right mode” (see below) must be set before starting the record.

---

## 3. Camera mode — exact value and camera setup

### 3.1 Mode we send over BLE

The remote must put the camera in **Video** mode before starting recording. In this project:

- **Protocol mode value**: `0x01` (**Video**).
- **In code**: `CAMERA_MODE_NORMAL` (see `logic/enums_logic.h`).
- **When**: Sent via `command_logic_switch_camera_mode(CAMERA_MODE_NORMAL)` immediately before `command_logic_start_record()` when motion-triggered recording starts.

So the “right mode” for dashcam is **Video mode, protocol byte 0x01**. Any change to the mode used for auto start/stop must stay consistent with this (and with the camera’s mode list in the protocol).

### 3.2 Camera-side settings (Loop Recording, resolution, etc.)

The **exact** checklist for how to set up the **camera itself** (Loop Recording, resolution, FOV, storage, etc.) is **not** in this repo; it is in:

- **Osmo Action 6 Dashcam Setup plan**: `.cursor/plans/osmo_action_6_dashcam_setup_997be9cb.plan.md` (or the current plan file with that content).

That plan describes:

- Enabling **Loop Recording** in **Video** mode (Control Center).
- Recommended video settings (e.g. 4K or 1080p @ 30 fps, RockSteady, segment length).
- Mount, power, storage, and workflow.

The firmware only sends **mode = Video (0x01)** and **start/stop record**; the user must configure the camera per that plan (Loop Recording on, desired resolution, etc.).

---

## 4. GPS

- **Goal**: Send **GPS data to the camera** (for telemetry/metadata), in the same spirit as [rhoenschrat/DJI-Remote](https://github.com/rhoenschrat/DJI-Remote).
- **Now**: GPS is a **placeholder**. No physical module yet; code is structured so a real driver can be dropped in later.
- **Testing**: The placeholder **stubs a fix in Switzerland** (e.g. Zurich) so that:
  - `gps_has_fix()` returns true,
  - `gps_get_data()` returns stub coordinates,
  - The main loop can exercise **GPS push** to the camera when connected.
- **Later**: When the GPS module is wired (e.g. M5Stack GPS Module v2.0 on the Plus 1.1), replace the placeholder in `gps/gps.c` with a real UART/NMEA driver; the rest of the app (push interval, protocol, `command_logic_push_gps_data()`) stays the same.

---

## 5. Auto Start/Stop screen

- **Purpose**: One of the main screens; used to **test and monitor** motion-triggered behaviour.
- **Behaviour**:
  - Shows whether the **movement sensor has triggered a start** (e.g. “Moving - Recording”) or the device is **still** and **counting down** to stop/sleep (e.g. “Still - Stop in M:SS”), or **idle** (e.g. “Idle - No recording”).
- **Style**: Same pattern as other screens (icon, name, status line, nav dots). No full-screen redraw loop; only the status line is updated, and only when the displayed content changes.
- **Default**: When the camera connects (first time or reconnect), the UI switches to this screen so the user lands on Auto Start/Stop when ready to drive.

---

## 6. Summary checklist (for implementers)

- [ ] Motion: MPU6886 → motion logic → `motion_logic_just_started()` / `motion_logic_just_stopped()`.
- [ ] Flow: wake (if needed) → set **Video mode (0x01)** → start record; on still timeout → stop record → sleep.
- [ ] Mode in code: `CAMERA_MODE_NORMAL` (0x01); camera UI/settings per **plan** (Loop Recording, etc.).
- [ ] GPS: placeholder stubbed to Switzerland; push when connected and “fix”; replace with real NMEA when hardware is wired.
- [ ] Auto Start/Stop screen: status line only (idle / moving–recording / still–countdown), no excessive redraws; default screen after connection.

# Contributing

## Build and flash before verifying

**Do not assume code changes are on the device until you have successfully built and flashed.**

- After any code change, run a full **build** and **upload** to the target device.
- Confirm both steps succeed (exit code 0) before testing behavior or iterating on fixes.
- If you are automating (e.g. Cursor, CI), use the same commands; see README for when `pio` is not on PATH.

### Commands

- **Plus 1.1**: From repo root, `./scripts/build_and_upload_plus11.sh`
- **Without `pio` on PATH**:  
  `"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11` then  
  `"$HOME/.platformio/penv/bin/pio" run -e m5stickc_plus11 -t upload`

### Capture device logs for debugging

To get runtime evidence (serial output) so an agent or developer can analyze failures:

1. Connect the device via USB.
2. From repo root run: `./scripts/capture_logs.sh` or `python3 scripts/capture_serial.py 40 .cursor/debug-7ee220.log`
3. Reproduce the issue (reconnect, pairing, etc.) while the script is running, or let it capture for the default 40s.
4. The log is written to `.cursor/debug-7ee220.log`. Share that file or its contents for analysis.

Requires `pyserial` (`pip install pyserial`). If the serial port is not found, ensure the device is connected and the port is not in use by another monitor.

### For AI agents / automation

1. Resolve PlatformIO: try `pio`, then `$HOME/.platformio/penv/bin/pio`, then `python3 -m platformio`.
2. Run build and upload (with network permission for build; full/permission for upload if USB access is needed).
3. Do not iterate on code fixes without at least one successful build and upload in the same session.
4. To obtain runtime logs: the agent cannot open the USB serial port in some environments. Ask the user to run `./scripts/capture_logs.sh` with the device connected, then read `.cursor/debug-7ee220.log` (or the path the user used) to analyze behavior.
