/*
 * AtomS3 graphics shim — C-callable interface to M5GFX. See m5atoms3_gfx.cpp
 * for context, references, and the decision log entries (D-014..D-016).
 *
 * Only included when the AtomS3 build target is active. The Plus2/StickS3
 * paths keep using their own hand-rolled draw helpers in their respective
 * HAL .c files; this header exists so feature code (HAL, UI) can call M5GFX
 * from C without a sea of `#ifdef M5ATOMS3` blocks at every call site.
 */

#ifndef M5ATOMS3_GFX_H
#define M5ATOMS3_GFX_H

#ifdef M5ATOMS3

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int  atoms3_gfx_init(void);
void atoms3_gfx_set_brightness(uint8_t brightness);
int  atoms3_gfx_width(void);
int  atoms3_gfx_height(void);

void atoms3_gfx_clear(uint16_t color);
void atoms3_gfx_fill_rect(int x, int y, int w, int h, uint16_t color);
void atoms3_gfx_fill_circle(int cx, int cy, int r, uint16_t color);
void atoms3_gfx_draw_xbitmap(int x, int y, int w, int h,
                             const uint8_t *bitmap, uint16_t color);
/* MSB-first 1-bpp bitmap (use for logo_bitmap.h). */
void atoms3_gfx_draw_bitmap(int x, int y, int w, int h,
                            const uint8_t *bitmap, uint16_t color);
/* Color PNG (use for icons_png.h emoji icons). */
void atoms3_gfx_draw_png(int x, int y, const uint8_t *data, unsigned len);

/* Tier 1 = LABEL (small, proportional). Tier 2 = VALUE (medium). Tier 3 =
 * HERO (large). See docs/ATOMS3_MIGRATION_SPEC.md "Type scale" for the
 * design rationale. The shim picks the M5GFX font + size internally. */
void atoms3_gfx_print(int x, int y, const char *text, uint16_t color, int tier);
void atoms3_gfx_print_centered(int y, const char *text, uint16_t color, int tier);
void atoms3_gfx_print_centered_at(int cx, int y, const char *text, uint16_t color, int tier);
int  atoms3_gfx_tier_pixel_height(int tier);

void atoms3_gfx_erase_rect(int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif

#endif /* M5ATOMS3 */

#endif /* M5ATOMS3_GFX_H */
