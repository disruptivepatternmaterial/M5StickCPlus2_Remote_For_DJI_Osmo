#!/usr/bin/env bash
# Build and upload firmware for M5Stick C Plus 1.1.
# Run from repo root: ./scripts/build_and_upload_plus11.sh [--no-upload] [--clean]
#
# IMPORTANT: Code changes are not on the device until this script succeeds.
# Resolves pio from PATH, ~/.platformio/penv/bin/pio, or python3 -m platformio
# so CI or automation (e.g. Cursor agent) can run without pio on PATH.
# See README "Build and flash (required for verification)" and CONTRIBUTING.md.

set -e
cd "$(dirname "$0")/.."

# Resolve PlatformIO executable
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
  $PIO run -e m5stickc_plus11 -t clean 2>/dev/null || true
else
  echo "=== Skipping clean (use --clean to clean first) ==="
fi

echo "=== Generating Plus 1.1 HAL (if needed) ==="
python3 scripts/generate_plus11_hal.py

echo "=== Building for m5stickc_plus11 ==="
$PIO run -e m5stickc_plus11

if [ "$UPLOAD" -eq 1 ]; then
  echo "=== Uploading to device ==="
  $PIO run -e m5stickc_plus11 -t upload
else
  echo "=== Skipping upload (--no-upload) ==="
fi

echo "=== Done ==="
