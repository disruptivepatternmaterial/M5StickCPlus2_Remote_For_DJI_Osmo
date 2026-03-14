#!/usr/bin/env python3
"""
Capture serial output from the device to a log file. No TTY required.
Usage: python3 scripts/capture_serial.py [duration_sec] [output_path]
Default: 40 seconds, .cursor/debug-7ee220.log
Port: auto-detect first /dev/cu.usbserial* or /dev/ttyUSB* (115200 baud).
"""
import sys
import glob
import time

def find_port():
    for pattern in ["/dev/cu.usbserial*", "/dev/cu.SLAB*", "/dev/ttyUSB*", "/dev/ttyACM*"]:
        ports = glob.glob(pattern)
        if ports:
            return sorted(ports)[0]
    return None

def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    out_path = sys.argv[2] if len(sys.argv) > 2 else ".cursor/debug-7ee220.log"
    port = find_port()
    if not port:
        print("No serial port found (looked for cu.usbserial*, ttyUSB*, ttyACM*)", file=sys.stderr)
        sys.exit(1)
    try:
        import serial
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    ser = serial.Serial(port, 115200, timeout=0.5)
    print(f"Capturing {duration}s from {port} -> {out_path}", file=sys.stderr)
    with open(out_path, "w") as f:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            try:
                line = ser.readline()
            except (OSError, serial.SerialException) as e:
                print(f"Read error (device disconnected?): {e}", file=sys.stderr)
                break
            if line:
                text = line.decode("utf-8", errors="replace")
                f.write(text)
                f.flush()
    try:
        ser.close()
    except Exception:
        pass
    print("Done.", file=sys.stderr)

if __name__ == "__main__":
    main()
