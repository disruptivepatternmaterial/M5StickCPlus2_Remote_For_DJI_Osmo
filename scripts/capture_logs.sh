#!/usr/bin/env bash
# Capture device serial output to the standard debug log path.
# Run from repo root with the device connected via USB.
# Usage: ./scripts/capture_logs.sh [duration_sec]
# Default: 40 seconds -> .cursor/debug-7ee220.log
# Requires: pyserial (pip install pyserial)
set -e
cd "$(dirname "$0")/.."
DURATION="${1:-40}"
LOG_PATH=".cursor/debug-7ee220.log"
python3 scripts/capture_serial.py "$DURATION" "$LOG_PATH"
echo "Log written to $LOG_PATH — use this file for debugging or share with the agent."
