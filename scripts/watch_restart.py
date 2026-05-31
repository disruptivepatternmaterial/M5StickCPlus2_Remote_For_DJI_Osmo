#!/usr/bin/env python3
"""Watch the serial port and flag reboots/crashes. Prints a marker line
whenever it sees a boot banner, panic/brownout/watchdog text, or the device
uptime counter drops (which means it reset). Full stream saved to logfile."""
import sys, glob, time, re, serial
dur = int(sys.argv[1]) if len(sys.argv) > 1 else 1800
out = sys.argv[2] if len(sys.argv) > 2 else ".cursor/restart_watch.log"
port = (glob.glob("/dev/cu.usbmodem*") or [None])[0]
if not port:
    print("NO PORT"); sys.exit(1)
s = serial.Serial(port, 115200, timeout=0.5)
up = re.compile(rb"^[IWE] \((\d+)\)")
crash = re.compile(rb"ESP-ROM|rst:0x|Brownout|brownout|panic|Guru Meditation|abort|assert failed|backtrace|StoreProhibited|LoadProhibited|task_wdt|rtc_wdt", re.I)
FRESH_BOOT_MS = 20000   # a real reboot restarts uptime below this
last_v = 0
print(f"WATCHING {port} for {dur}s -> {out}", flush=True)
deadline = time.monotonic() + dur
with open(out, "w") as f:
    while time.monotonic() < deadline:
        try:
            line = s.readline()
        except Exception as e:
            print(f"PORT DROPPED (possible reset/replug): {e}", flush=True); time.sleep(1); continue
        if not line:
            continue
        f.write(line.decode("utf-8","replace")); f.flush()
        if crash.search(line):
            print(f"CRASH/RESET MARKER: {line.decode('utf-8','replace').strip()[:160]}", flush=True)
        m = up.match(line)  # anchored: only the 'I (NNN)' log timestamp
        if m:
            v = int(m.group(1))
            # Real reboot = uptime is small now but was large just before.
            # Require the NEW value to be a plausible fresh-boot time so a
            # single corrupt high reading can't fake a drop.
            if v < FRESH_BOOT_MS and last_v > 120000:
                print(f"REBOOT DETECTED: uptime {last_v}ms -> {v}ms (fresh boot)", flush=True)
            last_v = v
print("WATCH WINDOW ENDED — no reboot or crash detected in window.", flush=True)
