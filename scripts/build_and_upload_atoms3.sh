#!/usr/bin/env bash
# Build and upload firmware for M5 AtomS3 (ESP32-S3FN8).
# Run from repo root: ./scripts/build_and_upload_atoms3.sh [--no-upload] [--clean]
#
# Resolves pio from PATH, ~/.platformio/penv/bin/pio, or python3 -m platformio.
#
# AtomS3 USB-CDC download mode (same caveat as StickS3, ref M5 docs):
#   1. Press and HOLD the screen-button on the AtomS3.
#   2. Press Reset (or briefly disconnect/reconnect USB while holding).
#   3. Wait until the internal green LED lights, then release.
#   4. Run this script immediately.

set -e
cd "$(dirname "$0")/.."

if command -v pio &>/dev/null; then
  PIO=pio
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
  PIO="$HOME/.platformio/penv/bin/pio"
else
  PIO="python3 -m platformio"
fi
echo "Using: $PIO"

UPLOAD=1
CLEAN=0
for arg in "$@"; do
  case "$arg" in
    --no-upload) UPLOAD=0 ;;
    --clean)     CLEAN=1 ;;
  esac
done

if [ "$CLEAN" -eq 1 ]; then
  echo "=== Clean (avoid stale build dir) ==="
  $PIO run -e m5atoms3 -t clean 2>/dev/null || true
else
  echo "=== Skipping clean (use --clean to clean first) ==="
fi

echo "=== Building for m5atoms3 ==="
$PIO run -e m5atoms3

if [ "$UPLOAD" -eq 1 ]; then
  echo "=== Uploading to device ==="
  echo "(If upload hangs: hold AtomS3 screen-button, tap Reset, release on green LED, then re-run.)"
  $PIO run -e m5atoms3 -t upload
else
  echo "=== Skipping upload (--no-upload) ==="
fi

echo "=== Done ==="
