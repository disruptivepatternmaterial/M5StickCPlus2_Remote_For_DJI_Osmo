# AtomS3 Implementation Log

Append-only log of changes to the AtomS3 variant. One bullet per change
wave; date the entry; link to commit / file changes; reference the
decision IDs from [`ATOMS3_DECISIONS.md`](ATOMS3_DECISIONS.md).

## 2026-05-22 — Initial scaffolding

- Added the AtomS3 documentation set:
  - [`docs/ATOMS3_REFERENCE_GROUNDING.md`](ATOMS3_REFERENCE_GROUNDING.md)
  - [`docs/ATOMS3_MIGRATION_SPEC.md`](ATOMS3_MIGRATION_SPEC.md)
  - [`docs/ATOMS3_DECISIONS.md`](ATOMS3_DECISIONS.md)
  - [`docs/ATOMS3_TEST_PLAN.md`](ATOMS3_TEST_PLAN.md)
  - [`docs/ATOMS3_TEST_EVIDENCE.md`](ATOMS3_TEST_EVIDENCE.md)
  - [`docs/ATOMS3_IMPLEMENTATION_LOG.md`](ATOMS3_IMPLEMENTATION_LOG.md)
- Decision references in scope: D-001, D-005.
- No code changes yet.

## 2026-05-22 — First code wave

- Added the M5AtomS3 HAL files (refs R1, R4, R5, R6, R10):
  - [`main/m5atoms3_hal.h`](../main/m5atoms3_hal.h)
  - [`main/m5atoms3_hal.c`](../main/m5atoms3_hal.c) — ST7789-driver-based 128×128
    init (D-002), MPU6886 IMU (D-007 family), buzzer no-op stubs, single-button
    handling on G41.
- Wired the `m5atoms3` build target through:
  - [`main/CMakeLists.txt`](../main/CMakeLists.txt) (`-DM5ATOMS3_BUILD=ON` →
    `M5ATOMS3` compile flag)
  - [`platformio.ini`](../platformio.ini) (`[env:m5atoms3]`)
  - [`sdkconfig.defaults.m5atoms3`](../sdkconfig.defaults.m5atoms3) — SPIRAM
    disabled (D-003).
- Refactored GPS to a board-agnostic `GPS_USE_REAL_UART` switch and added
  AtomS3 UART pin mapping (TX=G2, RX=G1) — refs R8, R9 (D-007).
- Added board-aware include guards and AtomS3 paths in:
  - [`main/ui.c`](../main/ui.c) — 128×128 layout coordinates, two-screen
    `ui_next_screen`, dot-strip suppression, AtomS3-friendly instruction text,
    AUTO pill / GPS row tuned for 128×128 (D-005).
  - [`main/app_main.c`](../main/app_main.c) — single-button short/long-press
    state machine (D-004); legacy A/B path preserved for Stick targets.
  - [`gps/gps.h`](../gps/gps.h), [`gps/gps.c`](../gps/gps.c) — board-agnostic
    UART path.
- Added build/upload helper:
  - [`scripts/build_and_upload_atoms3.sh`](../scripts/build_and_upload_atoms3.sh)
- Updated operator docs: [`SPEC.md`](../SPEC.md), [`README.md`](../README.md).
- Status: code edits saved to disk only. No build has been run yet from this
  conversation; the user will run `./scripts/build_and_upload_atoms3.sh`
  against the device on heliotroperidge once they confirm the target.

## 2026-05-22 — First-bring-up fixes + UI redesign

- First bring-up surfaced four issues. All addressed in this wave:
  1. **Build broke**: `m5stickc_plus2_hal.c` was guarded as
     `!M5STICKC_PLUS_11 && !M5STICKS3` — its body still compiled for AtomS3
     and used the ESP32-only `VSPI_HOST` constant. Guard now also excludes
     `M5ATOMS3`. ([`main/m5stickc_plus2_hal.c`](../main/m5stickc_plus2_hal.c))
  2. **Screen orientation**: panel rendered upside-down. Fixed in HAL by
     toggling both axes of `esp_lcd_panel_mirror(true, true)` while leaving
     the (2, 1) gap unchanged. ([`main/m5atoms3_hal.c`](../main/m5atoms3_hal.c))
  3. **Boot logo**: the existing 231×87 bitmap could not fit on 128×128 and
     looked broken when clipped. Added a 2:1 downsample
     (`atoms3_draw_logo_downsampled`) so the user-required boot graphic
     stays visible at ~116×44. ([`main/m5atoms3_hal.c`](../main/m5atoms3_hal.c))
  4. **UI was bad**: text overflow, icons, blinking, gross colors. Rewrote
     the AtomS3 render path. See decisions D-008..D-013 and references
     R11..R14.
- New AtomS3 renderer in [`main/ui.c`](../main/ui.c):
  - `atoms3_render(force_full)` — diff-based partial repaint with a per-region
    cache (hero text + color, value text, hint text, conn dot color).
  - `atoms3_compose_bt` / `atoms3_compose_auto` / `atoms3_compose_hint` —
    state → text/color mappers using `M5_TRUE_*` colors throughout.
  - `atoms3_clip_text` — hard truncation so labels never wrap on 128×128.
  - `atoms3_draw_line[_centered]` — clear-row-then-print primitive that
    keeps changing-length labels free of ghost pixels.
- Both `ui_update_display()` and `ui_update_auto_status_line_only()` now
  short-circuit to `atoms3_render()` on AtomS3, so main-loop call cadence
  is unchanged but the screen no longer full-clears every redraw.
- Decisions in scope: D-008 (no icons / dots / instruct strip), D-009 (3-tier
  scale), D-010 (`M5_TRUE_*` colors), D-011 (one accent per state),
  D-012 (no full-screen clear during steady state), D-013 (hard-clipped text).
- Status: code saved + lints clean. Build/upload to be run from the user's
  shell on heliotroperidge: `./scripts/build_and_upload_atoms3.sh`.
