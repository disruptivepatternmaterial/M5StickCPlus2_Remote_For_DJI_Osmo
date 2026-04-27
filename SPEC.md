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
| Still for timeout | **Stop recording**. Camera is **kept awake** (not put to BLE sleep) so the next motion event can restart recording immediately. |

Sequence: wake (if needed) → set mode → shutter start → … → shutter stop → (idle, link kept alive) → next motion → set mode → shutter start.

> ⚠️ **Why we don't sleep the camera on motion timeout:** the Osmo Action does not
> reliably wake from a BLE-only request after `power_mode = 0x03` (sleep) — once
> asleep it requires a physical button press, which would defeat motion-triggered
> auto-restart. Manual sleep is still available from the SLEEP screen.

## Camera Mode

- Protocol value: `0x01` (Video).
- In code: `CAMERA_MODE_NORMAL` (see `logic/enums_logic.h`).
- Sent via `command_logic_switch_camera_mode(CAMERA_MODE_NORMAL)` before `command_logic_start_record()`.

## Required Camera Setup

The firmware can start/stop recording and switch the camera into Video mode, but it cannot
configure the camera's storage behavior over BLE. Configure these once on the camera before
using the remote as a dashcam:

- **Loop Recording**: enabled. This is required for unattended dashcam use; normal Video mode
  eventually fills the card and the camera reports **Insufficient Storage**.
- **Loop segment length**: 3-5 minutes. Shorter segments are easier to copy, recover, and overwrite.
- **Pre-Rec**: disabled unless there is a specific reason to keep it. It increases storage pressure.
- **Storage target**: microSD card. On models with internal storage, do not rely on internal storage
  for unattended dashcam recording.
- **microSD card**: UHS-I U3 / V30 or better, preferably 256 GB or 512 GB from DJI's recommended list.
- **Format**: format the card in the camera after offloading anything important.
- **Codec**: use Efficiency / HEVC if the NAS and playback tools support it; it reduces file size.
- **Resolution / frame rate**: start with 4K/30 or 1080p/30 while validating the setup. Higher frame
  rates fill storage faster and increase heat.

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

- **Sensor**: MPU6886 (M5StickC Plus / Plus 1.1) or BMI270 (M5 StickS3) via HAL — `m5stickc_plus2_imu_init()`, `m5stickc_plus2_imu_read_accel()`.
- **Logic**: `logic/motion_logic.c` — states IDLE / MOVING / COUNTDOWN.
- **API**: `motion_logic_just_started()` / `motion_logic_just_stopped()` drive `app_main.c`.
- **Timeout**: configurable; 15 s for testing, 2.5 min for production.

## GPS

- **M5StickC Plus / Plus 1.1**: Placeholder stub returning fixed Switzerland coordinates.
  `gps_has_fix()` → true, `gps_get_data()` → stub coords.
- **M5 StickS3 + Unit GPS v1.1**: Real UART/NMEA on Grove PORT.A (ESP TX→Yellow G9, ESP RX←White G10), 115200 8N1 default. Firmware auto-falls back to 9600 if no NMEA `$` is seen within 4 s (older Unit GPS / re-flashed modules). `gps_has_fix()` / `gps_get_data()` reflect parsed GGA/RMC when a fix is present.
- **Push**: Every 1 s when connected and fix available, via `command_logic_push_gps_data()` (CmdSet 0x00, CmdID 0x17).

## Footage Offload

Video files are not transferred over the BLE protocol used by this firmware. BLE is used for
control commands and GPS metadata push only.

Supported offload paths:

- **Camera loop recording**: primary storage-management strategy. The camera overwrites old
  loop segments locally so unattended recording does not stop with **Insufficient Storage**.
- **USB file transfer**: power on the camera, connect USB-C, choose **Transfer File / USB Transfer**
  on the camera, then copy `DCIM` to the NAS from the attached computer. The camera cannot
  record while transferring files.
- **microSD card reader**: fastest and most reliable bulk import path. Remove the card and import
  from a reader attached to the NAS or a computer.
- **DJI Mimo / phone bridge**: download in DJI Mimo or via Android USB/OTG, then sync the phone
  folder to the NAS with a separate sync tool.
- **Native NAS upload**: only consider this if the specific camera model and current DJI Mimo app
  expose a Camera Cloud Service / NAS / SMB feature. This is model- and app-version-dependent and
  is not controlled by this firmware.

## Auto Start/Stop Screen

- Default screen after camera connects.
- Status line shows: **Idle** / **Moving – Recording** / **Still – Stop in M:SS**.
- Same style as other screens (icon, name, status line, nav dots).
- Only the status line redraws, and only when content changes — no full-screen redraw loop.

## Summary Checklist

- [ ] Motion: MPU6886 → motion_logic → `just_started` / `just_stopped` drive app_main.
- [ ] Flow: wake/connect if needed → set Video (0x01) → start record; on still timeout → stop record and keep camera awake.
- [ ] Mode: `CAMERA_MODE_NORMAL` (0x01) sent before start record.
- [ ] Camera setup: Loop Recording enabled on camera; normal Video mode is not acceptable for unattended dashcam use.
- [ ] Offload: video files are copied by USB, card reader, phone bridge, or model-specific native NAS upload — not by BLE.
- [ ] GPS: Plus builds use stub; StickS3 uses NMEA from Unit GPS v1.1; push when connected.
- [ ] Auto screen: status line (idle / moving / still–countdown); default after connect; no excessive redraws.


## Related Code that will provide ideas and information
- https://github.com/rhoenschrat/DJI-Remote
- https://github.com/dji-sdk/Osmo-GPS-Controller-Demo
- https://github.com/xaionaro-go/djictl
- https://shop.m5stack.com/products/m5stickc-plus-esp32-pico-mini-iot-development-kit?srsltid=AfmBOoowTnRHHiPWqb0e6R41u3HLd33-PyM8mvr4PGcRLx7NE6iTvOV8
- https://docs.m5stack.com/en/core/m5stickc_plus
- When downloading programs to the device, it is recommended to select one of the following serial baud rates. Using other speeds may cause the program to fail to download correctly. 1500000 bps / 750000 bps / 500000 bps / 250000 bps / 115200 bps
- Added sleep and wake functions, version changed to v1.1
