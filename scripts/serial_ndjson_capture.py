#!/usr/bin/env python3
"""Append GPS debug NDJSON lines from StickS3 USB serial to Cursor debug log."""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
import time

LOG = pathlib.Path(".cursor/debug-ndjson.log")

_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def extract_ndjson(line: str) -> str | None:
    """ESP_LOGI('GPS_NDJSON', '{...}') → raw JSON substring."""
    if "df0fdc" not in line:
        return None
    clean = _ANSI.sub("", line)
    i = clean.find("{")
    if i < 0:
        return None
    tail = clean[i:]
    j = tail.rfind("}")
    if j < 0:
        return None
    return tail[: j + 1]


def list_candidate_ports() -> list[str]:
    cu = pathlib.Path("/dev")
    out: list[str] = []
    if not cu.is_dir():
        return out
    skip = ("Bluetooth-Incoming-Port", "debug-console")
    for p in sorted(cu.glob("cu.*")):
        name = p.name
        if any(s in name for s in skip):
            continue
        if "usb" in name.lower() or "wchusb" in name.lower() or "serial" in name.lower():
            out.append(str(p))
    return out


def main() -> None:
    try:
        import serial  # type: ignore
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    ap = argparse.ArgumentParser(description="Capture DF0fdc NDJSON lines from ESP32 USB serial.")
    ap.add_argument("--port", default=None, help="/dev/cu.*")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--auto", action="store_true", help="Pick single matching /dev/cu.usb* port")
    ap.add_argument("--duration", type=float, default=0.0, help="Seconds then exit (0 = forever)")
    ap.add_argument("--clear-log", action="store_true", help="Truncate log before capture")
    args = ap.parse_args()

    port = args.port
    candidates = list_candidate_ports()
    if args.auto or port is None:
        if port is None:
            if len(candidates) == 1:
                port = candidates[0]
                print(f"Using port {port}", flush=True)
            elif len(candidates) == 0:
                print("No USB serial ports found under /dev/cu.*", file=sys.stderr)
                sys.exit(1)
            else:
                print("Multiple ports; pick one with --port:", file=sys.stderr)
                for c in candidates:
                    print(c, file=sys.stderr)
                sys.exit(1)

    LOG.parent.mkdir(parents=True, exist_ok=True)
    if args.clear_log and LOG.exists():
        LOG.write_text("", encoding="utf-8")

    ser = serial.Serial(port, args.baud, timeout=0.25)
    print(f"Logging JSON lines containing sessionId df0fdc to {LOG}", flush=True)
    deadline = (time.monotonic() + args.duration) if args.duration and args.duration > 0 else None
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            break
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        payload = extract_ndjson(line) if "GPS_NDJSON" in line or "df0fdc" in line else None
        if payload is None and line.startswith("{") and "df0fdc" in line:
            payload = line
        if payload:
            with LOG.open("a", encoding="utf-8") as f:
                f.write(payload + "\n")
            print(payload, flush=True)


if __name__ == "__main__":
    main()
