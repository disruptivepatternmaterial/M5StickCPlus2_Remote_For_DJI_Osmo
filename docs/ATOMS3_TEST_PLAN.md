# AtomS3 Test Plan

L3 firmware test plan for the M5AtomS3 variant. Captured runs go in
[`ATOMS3_TEST_EVIDENCE.md`](ATOMS3_TEST_EVIDENCE.md). All tests run with
the AtomS3 USB-attached to the dev workstation and the camera
power-on within 1 m.

## Tier and gate

- Tier: L3 (shipping firmware change on a deployed-class device).
- Pass gate: every required test below has captured evidence in
  [`ATOMS3_TEST_EVIDENCE.md`](ATOMS3_TEST_EVIDENCE.md), referenced by
  test ID.
- Captured evidence files live in `.cursor/debug-*.log` (gitignored)
  and the relevant excerpt is pasted into `ATOMS3_TEST_EVIDENCE.md`
  with the test ID.

## Required tests

### T-001 — First boot, no camera nearby

**Steps**
1. Flash AtomS3 with current firmware.
2. Open serial at 115200, capture for 30 s.

**Pass criteria**
- Serial shows `M5ATOMS3_HAL: M5 AtomS3 hardware initialized successfully`
  or equivalent.
- No panic / WDT reset within 30 s.
- LCD shows boot logo at least briefly, then transitions to the `BT` screen.

### T-002 — BT screen states

**Steps**
1. From T-001's running device, observe BT label.
2. Power on paired DJI camera.
3. Observe BT state transitions over the next 60 s.

**Pass criteria**
- BT label cycles `OFFLINE → CONNECTING → ONLINE` and reaches `ONLINE`.
- No SPI/display assert during the transitions.

### T-003 — Single-button: short press cycles screens

**Steps**
1. From T-002's running device, short-press (< 600 ms) the screen button.

**Pass criteria**
- Screen cycles `BT → AUTO → BT` on each short press.
- Each transition completes in < 250 ms.
- No stuck/blank frames.

### T-004 — AUTO motion-driven recording

**Steps**
1. From `AUTO` screen with camera connected, gently shake AtomS3 to
   trigger motion.
2. Hold still for 30 s after shaking stops.

**Pass criteria**
- Camera transitions to recording within ~2 s of motion onset.
- AUTO screen shows `REC` and a counting timer.
- Recording stops once motion stops + countdown elapses.

### T-005 — Manual override: long-press while AUTO

**Steps**
1. On `AUTO` screen, long-press (≥ 1500 ms) the screen button.

**Pass criteria**
- Recording state toggles (start if stopped, stop if started).
- AUTO screen reflects the new state within 1 s.

### T-006 — Manual reconnect: long-press while BT

**Steps**
1. Disconnect camera (power off camera or move out of range).
2. Wait until BT label shows `OFFLINE`.
3. Power camera back on.
4. On the `BT` screen, long-press (≥ 1500 ms) the screen button.

**Pass criteria**
- Reconnect attempt is dispatched (serial logs show
  `LOGIC_CONNECT: Attempting to reconnect ...`).
- BT label progresses to `CONNECTING` and then `ONLINE` within 30 s.

### T-007 — GPS fix indication

**Steps**
1. With Unit GPS connected and an open-sky fix available, observe
   AUTO screen GPS line.

**Pass criteria**
- GPS line transitions from `NO FIX` to `FIX <n>` within 90 s of
  attaching antenna outdoors.
- No UI redraw stalls during the transition.

### T-008 — No SPI/display assert under load

**Steps**
1. Run device for 5 minutes with camera connected and GPS active.
2. Force one disconnect (camera off) and one reconnect.

**Pass criteria**
- No `assert failed: spi_device_release_bus` or `bus_lock` errors
  in serial output.
- Display continues to update across the disconnect/reconnect.

## Optional tests

### T-O-001 — Color order check

If T-001 shows wrong colors (RED/BLUE swap, etc.), capture a photo
and link it in `ATOMS3_TEST_EVIDENCE.md` so we can compare against
the BGR workaround in StickS3.
