#!/usr/bin/env python3
"""
Generate main/m5stickc_plus11_hal.c from main/m5stickc_plus2_hal.c.
Plus 1.1 uses LCD DC=23, RST=18; same API, different guard and include.
Run from repo root: python3 scripts/generate_plus11_hal.py
"""
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "main" / "m5stickc_plus2_hal.c"
DST = REPO / "main" / "m5stickc_plus11_hal.c"

NEW_HEADER = '''/*
 * M5StickC Plus 1.1 Hardware Abstraction Layer (LCD: DC=23, RST=18).
 * Same API as Plus2 HAL; compiled only when M5STICKC_PLUS_11 is defined.
 */
#ifdef M5STICKC_PLUS_11

#include "m5stickc_plus11_hal.h"
'''

def main():
    text = SRC.read_text()
    # Find the Plus2 include; replace from start of file through that line
    marker = '#include "m5stickc_plus2_hal.h"'
    if marker not in text:
        raise SystemExit("Could not find include marker in source")
    start = text.find(marker)
    if 'm5stickc_plus11_hal.h' in text[:start]:
        raise SystemExit("Source already looks like Plus 1.1 HAL")
    text = NEW_HEADER + text[start + len(marker):]
    # Fix trailing #endif
    text = text.replace("#endif /* !M5STICKC_PLUS_11 */", "#endif /* M5STICKC_PLUS_11 */")
    DST.write_text(text)
    print(f"Written {DST} ({len(text)} chars)")

if __name__ == "__main__":
    main()
