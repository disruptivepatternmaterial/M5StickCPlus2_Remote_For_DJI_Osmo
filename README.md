# M5Stick Remote for DJI Osmo Action

Open firmware for an M5Stick-based BLE remote focused on dashcam-style use with DJI Osmo Action cameras.

The core behavior this project is driving toward:

- movement starts -> wake/connect camera -> set video mode -> start recording
- movement stops for a timeout -> stop recording, while keeping the camera awake for the next motion event

`SPEC.md` is the authoritative feature definition.

## Current Focus

This repo is in active development toward a reliable "install it and trust it" remote.

Current work is centered on:

- stable BLE reconnect and command sequencing
- motion-driven start/stop recording logic
- board-specific HAL support across Plus2, Plus 1.1, and StickS3
- GPS path (stub on Plus builds, NMEA on StickS3)
- cleaner on-device status UI during auto start/stop operation

## What I'm Working On Right Now

- hardening reconnect behavior when the camera was manually power-cycled
- reducing false stop/start transitions from motion noise
- making the auto start/stop status screen update only when state text changes
- validating StickS3 GPS fix handling before each push interval

## Current Limitations

- camera settings like resolution/FPS/EIS/FOV are not writable through the implemented BLE command set
- camera storage behavior is not writable through BLE; Loop Recording must be enabled on the camera
- video files cannot be transferred over this firmware's BLE control link
- Plus2 and Plus 1.1 still use a GPS stub path; real NMEA path is currently StickS3-only
- wrong board target selection still causes the most common bring-up issue (blank/corrupt screen)

## Required Camera Setup

Before using the remote unattended, configure the DJI camera itself:

- Enable **Loop Recording** in Video mode. Plain Video mode will eventually fill the card and show
  **Insufficient Storage**.
- Set loop segments to about 3-5 minutes.
- Disable **Pre-Rec** unless it is specifically needed.
- Record to the microSD card, not internal storage on models that have both.
- Use a UHS-I U3 / V30 or better microSD card, preferably 256 GB or 512 GB.
- Format the card in the camera after copying off any footage you want to keep.
- Use HEVC / Efficiency if your NAS and playback tools support it.

The remote can switch the camera to Video mode and start/stop recording, but it cannot set Loop
Recording, resolution, codec, EIS, FOV, or other camera-side shooting/storage settings.

## Getting Footage to a NAS

The BLE protocol used here is for camera control and GPS metadata. It does not expose video files.

Practical offload options:

- **USB transfer**: connect the powered-on camera to a computer or NAS helper over USB-C, choose
  **Transfer File / USB Transfer** on the camera, then copy `DCIM` to the NAS. The camera cannot
  record while transferring.
- **microSD reader**: fastest and most reliable bulk import path.
- **DJI Mimo / phone bridge**: download to a phone, then sync that folder to the NAS.
- **Native NAS upload**: use only if your exact Osmo Action model and DJI Mimo version expose a
  Camera Cloud Service / NAS / SMB feature. This is not controlled by this firmware.

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
