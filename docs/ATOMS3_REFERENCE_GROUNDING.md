# AtomS3 Reference Grounding

Approved reference set for the M5AtomS3 migration. Each reference has an ID
(`R1`–`R10`); decisions and implementation notes elsewhere in this repo
should cite by ID so we can audit grounding later. Approved by user on
2026-05-22.

| ID  | Reference                                                                                                                                                | Why it's relevant                                                                              |
|-----|----------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------|
| R1  | [m5stack/M5AtomS3](https://github.com/m5stack/M5AtomS3)                                                                                                   | Vendor reference for AtomS3 hardware (display, button, IMU) — pin map and example sketches.   |
| R2  | [m5stack/M5AtomS3-UserDemo](https://github.com/m5stack/M5AtomS3-UserDemo)                                                                                 | Vendor ESP-IDF/PlatformIO build & flash flow for AtomS3 + AssetPool partition pattern.        |
| R3  | [m5stack/M5Unified](https://github.com/m5stack/M5Unified)                                                                                                 | Confirms AtomS3 is a supported board family; canonical board-detection patterns.              |
| R4  | [m5stack/M5GFX](https://github.com/m5stack/M5GFX)                                                                                                         | Vendor display driver/init sequences for AtomS3's 0.85" panel; reference for color order.     |
| R5  | [espressif/esp-bsp m5_atom_s3](https://github.com/espressif/esp-bsp/tree/master/bsp/m5_atom_s3)                                                           | Espressif BSP showing AtomS3 init via gc9a01 driver and standard esp_lcd path.                |
| R6  | [espressif/esp_lcd_gc9a01](https://github.com/espressif/esp-bsp/blob/master/components/lcd/esp_lcd_gc9a01/README.md)                                      | Managed component used by the BSP for the AtomS3 display.                                     |
| R7  | [ESP-IDF BLE security GATTC example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble_50/ble50_security_client/main/ble50_sec_gattc_demo.c) | Reference BLE GATTC patterns for ESP-IDF on ESP32-S3 (existing repo also follows this style). |
| R8  | [m5stack/M5UnitUnified](https://github.com/m5stack/M5UnitUnified)                                                                                         | Vendor pattern for Unit-level peripheral wiring (Unit GPS, etc.).                             |
| R9  | [m5stack/m5-docs Unit GPS](https://github.com/m5stack/m5-docs/blob/master/docs/en/unit/gps.md)                                                            | AT6558/Unit GPS NMEA reference; behavioral grounding for the GPS path on AtomS3.              |
| R10 | [m5stack/M5Atomic-Motion](https://github.com/m5stack/M5Atomic-Motion)                                                                                     | Vendor source for Atomic Base I2C wiring on AtomS3 (`SDA=38`, `SCL=39`). Confirms IMU pins.   |
| R11 | [Pebble system fonts (Raster Gothic / system bitmap fonts)](https://developer.repebble.com/guides/app-resources/system-fonts/)                              | Condensed bitmap-font typography optimized for tiny monochromatic-style displays.             |
| R12 | [BRUTAL Pebble watchface design notes](http://irek.gabr.pl/brutal.html)                                                                                    | One-hero-per-screen design, three font sizes max, 5 px gaps between rows for readability.     |
| R13 | [Qt GUI Design Skill — embedded device principles](https://www.qt.io/blog/introducing-qts-gui-design-skill-design-for-developers-in-agentic-workflows)     | Fixed-pixel mental model; three independent state cues (color + shape + text); modular scale. |
| R14 | [5x5 Pixel Font for Tiny Screens (DEV.to)](https://dev.to/_d916d77be80d376e49d8e/5x5-pixel-font-for-tiny-screens-the-complete-developer-guide-to-micro-typography-22d1) | Mixed-size UIs: tiny font for labels/units, large font for the primary value.                 |

References R1–R10 are frozen at user approval on 2026-05-22 (Atom S3 hardware grounding).
References R11–R14 are added 2026-05-22 (Atom S3 UX/UI grounding) per the redesign approval; same usage policy applies.

## Usage policy

- When taking a non-trivial design decision specific to AtomS3 (pin choice, init
  sequence, UI layout, button semantics), cite at least one of `R1`–`R10` in
  [`ATOMS3_DECISIONS.md`](ATOMS3_DECISIONS.md) and in the implementation log
  in [`ATOMS3_IMPLEMENTATION_LOG.md`](ATOMS3_IMPLEMENTATION_LOG.md).
- If a decision is not backed by `R1`–`R10`, label it `unsupported-by-references`
  and either find a new reference (and add it here as `R11+`) or revert.
- This list is frozen at the current count (R1–R14) until the user approves
  an extension. R11–R14 were added with the 2026-05-22 redesign approval.
