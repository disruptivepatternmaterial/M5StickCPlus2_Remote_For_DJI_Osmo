# AtomS3 Migration Spec

Board-specific technical spec for the M5AtomS3 variant of this firmware.
This file is **authoritative** for AtomS3-specific pin choices, init
sequences, UI layout, and input semantics. The repo-wide behavior
contract still lives in [`SPEC.md`](../SPEC.md).

References cited as `R1`–`R10` map to
[`ATOMS3_REFERENCE_GROUNDING.md`](ATOMS3_REFERENCE_GROUNDING.md).

## Hardware target

- M5AtomS3 (SKU C123): ESP32-S3FN8, 8 MB flash, **no PSRAM**.
- 0.85" IPS LCD, 128 × 128, GC9107 (vendor ref `R1`; espressif BSP uses
  `gc9a01` driver, which speaks the same SPI command subset — `R5`/`R6`).
- 1× programmable button under the screen (the screen IS the button).
- 1× MPU6886 IMU on the dedicated I2C bus.
- 1× HY2.0-4P (Grove) port for external Unit (we use Unit GPS here).

## Pin map

| Function       | GPIO  | Notes                                                                  |
|----------------|-------|------------------------------------------------------------------------|
| LCD MOSI       | 21    | `R1`/`R5`                                                              |
| LCD SCK        | 17    |                                                                        |
| LCD CS         | 15    |                                                                        |
| LCD DC (RS)    | 33    |                                                                        |
| LCD RST        | 34    |                                                                        |
| LCD BL         | 16    | PWM via LEDC                                                           |
| Screen button  | 41    | Active-low, internal pull-up. `R1`                                     |
| MPU6886 SDA    | 38    | I2C 0x68. `R10` confirms shared-with-Atomic-Base on the back I2C.      |
| MPU6886 SCL    | 39    |                                                                        |
| Grove (HY2.0)  | G2/G1 | Yellow=G2, White=G1 (M5 docs). Used for Unit GPS UART.                 |

Unit GPS wiring on AtomS3 HY2.0:

- Module RX  ← ESP TX = G2 (Yellow)
- Module TX  → ESP RX = G1 (White)

## Display

- Resolution: 128 × 128 logical, 16 bpp RGB565.
- Color order: BGR (per `R4` panel config; same workaround as StickS3).
- Init: SPI panel IO + ST7789-style init via `esp_lcd_new_panel_st7789`
  is used for first bring-up; if color/orientation is wrong, switch to
  the `esp_lcd_gc9a01` managed component (`R6`).
- Boot splash: existing `logo_bitmap` rendered at the center of 128 × 128
  for ≥ 1 s before the main UI takes over.

## Input

- Single button on GPIO 41 (active-low, pull-up).
- Behavior:
  - **Short press (< 1500 ms)**: AUTO record/arm toggle.
    - If recording: stop recording, force motion idle, and disarm motion start.
    - If idle: switch to Video, start recording, force motion active, and arm
      motion start/stop.
  - **Long press (≥ 1500 ms)**: connect/reconnect/pair.
- The button-driven path is the only operator input. No accelerometer
  gestures replace these for AtomS3.

## UI

Text-first two-screen operator UI for 128 × 128. References:
[Pebble system fonts](https://developer.repebble.com/guides/app-resources/system-fonts/) (R11),
[BRUTAL Pebble watchface](http://irek.gabr.pl/brutal.html) (R12),
[Qt embedded GUI principles](https://www.qt.io/blog/introducing-qts-gui-design-skill-design-for-developers-in-agentic-workflows) (R13),
[5x5 micro-typography](https://dev.to/_d916d77be80d376e49d8e/5x5-pixel-font-for-tiny-screens-the-complete-developer-guide-to-micro-typography-22d1) (R14).

Decisions: D-008 (no icons / dots / instruction strip), D-009 (3-tier scale),
D-010 (M5_TRUE_* colors), D-011 (one accent per state), D-012 (no full-screen
clear during steady state), D-013 (hard-clipped text, no wrap).

### Type scale

| Tier  | Pixels | Use                                                |
|-------|--------|----------------------------------------------------|
| HERO  | 24 px  | Reserved for rare short alerts only                |
| VALUE | 16 px  | Reserved; not used for the AtomS3 camera timer     |
| LABEL | 8 px   | Top tag (`DASH`), timer, bottom hint, boot label   |

### Color rules

- Background: black.
- Primary text: white.
- Persistent chrome uses the same visual language as the trigger4p remote:
  - top band: `DASH` label + link icon only when protocol-connected
  - main area: emoji state icon (`📡`, `🤝`, `📸`, `📷`)
  - bottom band: button/GPS hint
- AtomS3 does not use full-screen progress/error toasts. Connection attempts,
  reconnect failures, and pairing progress remain in logs; the screen stays in
  the persistent chrome and changes state through icons.

### Layout

```
CONNECT screen                     AUTO screen
+----------------+                +----------------+
|DASH        [🔗]|  top band      |DASH        [🔗]|  top band
|                |                |                |
|      📡        |  finding       |      📷        |  idle camera
|      🤝        |  pairing       |      📸        |  recording
|      📷        |  idle          |    00:23       |  small elapsed time
|                |                |                |
|HOLD=PAIR       |  hint          |TAP=REC/STOP    |  hint
+----------------+                +----------------+
```

Boot splash renders the shared zoomed logo plus `Dash Cam` in the small label
tier so the text fits the 128 px panel.

## GPS

- Enable the existing UART NMEA parser path that already runs on StickS3
  (no new parser code).
- AtomS3 GPS UART pins: `TX=G2`, `RX=G1`.
- Display only `FIX/NO FIX` and satellite count to fit 128 × 128.

## Power

- Powered via USB-C only (5 V → 3.3 V via SY8089). No battery, no PMIC,
  no power-button hold-to-shutdown gesture.
- The "shutdown" UI flow that exists for the Stick targets is `#ifdef`-out
  on AtomS3 builds.

## Differences vs StickS3

| Topic           | StickS3                          | AtomS3                                         |
|-----------------|----------------------------------|------------------------------------------------|
| Display         | ST7789, 240 × 135                | GC9107, 128 × 128                              |
| IMU             | BMI270                           | MPU6886                                        |
| Buzzer          | none (no-op stubs)               | none (no-op stubs)                             |
| Buttons         | A (G11), B (G12)                 | single screen button (G41)                     |
| Power mgmt      | M5PM1 PMIC                       | none (USB only)                                |
| GPS UART pins   | TX=G9, RX=G10                    | TX=G2, RX=G1                                   |
| PSRAM           | 8 MB OPI PSRAM                   | none                                           |

## Out of scope (for this migration)

- IR transmitter
- WS2812 (AtomS3 has none)
- Battery / sleep flows
- Multi-color icon theme — we keep the same RGB565 paths but drop the
  6-screen icon set.

## Open questions / risk

- GC9107 vs GC9A01 init sequence: if first-bring-up shows scrambled
  color or window offset, switch to the `esp_lcd_gc9a01` managed
  component. This is a known compatibility area (`R5`/`R6`).
- AtomS3 BLE without PSRAM: the StickS3 sdkconfig has SPIRAM enabled.
  AtomS3 has none. We disable SPIRAM in the AtomS3 sdkconfig defaults
  and watch for BLE stack RAM pressure.
