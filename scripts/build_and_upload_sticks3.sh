#!/usr/bin/env bash
# Build and upload firmware for M5 StickS3 (ESP32-S3).
# Run from repo root: ./scripts/build_and_upload_sticks3.sh [--no-upload] [--clean]
#
# Resolves pio from PATH, ~/.platformio/penv/bin/pio, or python3 -m platformio.

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
  $PIO run -e m5sticks3 -t clean 2>/dev/null || true
else
  echo "=== Skipping clean (use --clean to clean first) ==="
fi

echo "=== Building for m5sticks3 ==="
$PIO run -e m5sticks3

if [ "$UPLOAD" -eq 1 ]; then
  echo "=== Uploading to device ==="
  $PIO run -e m5sticks3 -t upload
else
  echo "=== Skipping upload (--no-upload) ==="
fi

echo "=== Done ==="
