/*
 * AtomS3 graphics shim — wraps M5GFX (LovyanGFX-based) with a C-callable API.
 *
 * Why a shim: the rest of this codebase is C, and M5GFX is C++. We instantiate
 * the M5GFX object in this translation unit and expose the small subset of
 * draw operations the AtomS3 UI needs as `extern "C"` functions. The HAL and
 * UI layers stay in C and just call atoms3_gfx_*().
 *
 * Decision log:
 *   D-014 — Replace the hand-rolled esp_lcd ST7789 path on AtomS3 with
 *           M5GFX. The previous bitmap-only renderer looked like a 1985
 *           calculator (operator feedback). M5GFX yields proportional
 *           fonts, anti-aliased rendering, and AtomS3-aware auto-detect.
 *   D-015 — All AtomS3 colors now use M5GFX RGB565 directly. The previous
 *           M5_TRUE_* / BGR-swap workarounds are no longer needed because
 *           M5GFX configures the panel correctly for its own color path.
 *   D-016 — Rotation 180° (mirrored both axes on the previous esp_lcd path)
 *           is expressed as `gfx.setRotation(2)` here, which both M5GFX docs
 *           (R3, R4) and the LovyanGFX ESP-IDF guide (R15) support natively.
 *
 * References (see docs/ATOMS3_REFERENCE_GROUNDING.md):
 *   R3  — m5stack/M5Unified (auto-detect patterns)
 *   R4  — m5stack/M5GFX (high-level API + AtomS3 panel support)
 *   R15 — lovyan03/LovyanGFX ESP-IDF integration (`boards.cmake/esp-idf.cmake`)
 *   R16 — M5Unified issue #42 (canonical pattern for using M5GFX from
 *         pure ESP-IDF / framework=espidf)
 */

#ifdef M5ATOMS3

#include <M5GFX.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "fonts/ibmplex_fonts.h"

extern "C" {
#include "m5atoms3_hal.h"
}

static const char *TAG = "ATOMS3_GFX";

/* The display instance. M5GFX::init() / begin() inspects the chip and
 * board pin map to pick the right panel driver (Panel_GC9107 for AtomS3). */
static M5GFX gfx;

/* ──────────────────────────────────────────────────────────────────────────
 * Lifecycle
 * ──────────────────────────────────────────────────────────────────────── */

extern "C" int atoms3_gfx_init(void) {
    if (!gfx.init()) {
        ESP_LOGE(TAG, "M5GFX init() returned false — panel auto-detect failed");
        return -1;
    }
    /* Rotation 2 = 180°. AtomS3 panel ribbon makes the unrotated default
     * read upside-down on the user's mounting (confirmed during first
     * bring-up on 2026-05-22). */
    gfx.setRotation(2);
    /* Black background; full brightness by default. The LCD-BL backlight is
     * managed by M5GFX once init() succeeds. */
    gfx.setBrightness(255);
    gfx.fillScreen(TFT_BLACK);
    /* Default text behavior: top-left datum, opaque BLACK background so we
     * don't have to clear-then-print every label individually. */
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextDatum(top_left);

    ESP_LOGI(TAG, "M5GFX init OK — %dx%d, rotation=2",
             (int)gfx.width(), (int)gfx.height());
    return 0;
}

extern "C" void atoms3_gfx_set_brightness(uint8_t b) {
    gfx.setBrightness(b);
}

extern "C" int atoms3_gfx_width(void)  { return (int)gfx.width(); }
extern "C" int atoms3_gfx_height(void) { return (int)gfx.height(); }

/* ──────────────────────────────────────────────────────────────────────────
 * Primitive fills
 * ──────────────────────────────────────────────────────────────────────── */

extern "C" void atoms3_gfx_clear(uint16_t color) {
    gfx.fillScreen(color);
}

extern "C" void atoms3_gfx_fill_rect(int x, int y, int w, int h, uint16_t color) {
    gfx.fillRect(x, y, w, h, color);
}

extern "C" void atoms3_gfx_fill_circle(int cx, int cy, int r, uint16_t color) {
    gfx.fillCircle(cx, cy, r, color);
}

/* Optional: draw the existing 1-bit logo bitmap with a tinted FG. M5GFX has
 * drawBitmap for 1bpp data — we wrap it so the C HAL can keep its existing
 * call site unchanged. */
extern "C" void atoms3_gfx_draw_xbitmap(int x, int y, int w, int h,
                                        const uint8_t *bitmap, uint16_t color) {
    if (bitmap == nullptr) return;
    gfx.drawXBitmap(x, y, bitmap, w, h, color);
}

/* MSB-first 1-bpp bitmap (logo_bitmap.h is generated MSB-first). drawXBitmap
 * is LSB-first and would scramble it, so the logo must use drawBitmap. */
extern "C" void atoms3_gfx_draw_bitmap(int x, int y, int w, int h,
                                       const uint8_t *bitmap, uint16_t color) {
    if (bitmap == nullptr) return;
    gfx.drawBitmap(x, y, bitmap, w, h, color);
}

/* Color PNG (icons_png.h), decoded at draw time. Shares the trigger4p icon
 * language: a satellite dish, handshake, camera, link, etc. */
extern "C" void atoms3_gfx_draw_png(int x, int y, const uint8_t *data, unsigned len) {
    if (data && len) gfx.drawPng(data, (uint32_t)len, x, y);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Text
 *
 * The HAL surface previously exposed scale-1 / scale-2 / scale-3 print calls
 * sized at multiples of 8 px tall. We map that onto M5GFX's setTextSize()
 * scaling, but pick a proportional font (Font2 / Font4) so the result looks
 * like the reference photos instead of like ZX-Spectrum bitmap output.
 * ──────────────────────────────────────────────────────────────────────── */

/* Internal: configure font + size given a "pixel height tier". The tiers
 * align with the 3-tier UI scale in docs/ATOMS3_MIGRATION_SPEC.md:
 *   tier 1 (LABEL, ~8 px)  → Font2 size 1   (≈ 16 px tall but proportional)
 *   tier 2 (VALUE, ~16 px) → Font2 size 2   (≈ 32 px tall — used for countdown)
 *   tier 3 (HERO,  ~24 px) → Font4 size 1   (≈ 26 px tall, bold proportional)
 *
 * Note: M5GFX's "Font2" / "Font4" are vector-rendered Adafruit-derived fonts;
 * they are anti-aliased and proportional, which is what we wanted. The hand-
 * rolled 8x8 font path is no longer used on AtomS3.
 */
static void atoms3_gfx_pick_font_for_tier(int tier) {
    gfx.setTextSize(1);
    switch (tier) {
        case 3:  gfx.setFont(&IBMPlexSans_SemiBold28pt7b); break;  /* HERO  */
        case 2:  gfx.setFont(&IBMPlexSans_SemiBold18pt7b); break;  /* VALUE */
        case 1:
        default: gfx.setFont(&IBMPlexSans_Medium9pt7b);    break;  /* LABEL */
    }
}

/* Transparent background: callers repaint the region first, so GFX-font glyphs
 * must not stamp an opaque box (matches the trigger4p shim). */
extern "C" void atoms3_gfx_print(int x, int y, const char *text,
                                 uint16_t color, int tier) {
    if (text == nullptr) return;
    atoms3_gfx_pick_font_for_tier(tier);
    gfx.setTextColor(color);
    gfx.setTextDatum(top_left);
    gfx.drawString(text, x, y);
}

extern "C" void atoms3_gfx_print_centered(int y, const char *text,
                                          uint16_t color, int tier) {
    if (text == nullptr) return;
    atoms3_gfx_pick_font_for_tier(tier);
    gfx.setTextColor(color);
    gfx.setTextDatum(top_center);
    gfx.drawString(text, gfx.width() / 2, y);
}

/* Horizontally centered on an arbitrary x. */
extern "C" void atoms3_gfx_print_centered_at(int cx, int y, const char *text,
                                             uint16_t color, int tier) {
    if (text == nullptr) return;
    atoms3_gfx_pick_font_for_tier(tier);
    gfx.setTextColor(color);
    gfx.setTextDatum(top_center);
    gfx.drawString(text, cx, y);
}

/* Erase a rectangular region (helper for partial-update paths so a previous
 * longer string doesn't leave ghost pixels behind). */
extern "C" void atoms3_gfx_erase_rect(int x, int y, int w, int h) {
    gfx.fillRect(x, y, w, h, TFT_BLACK);
}

/* Returns the pixel height of the current font tier. Used by the renderer to
 * size erase regions correctly. */
extern "C" int atoms3_gfx_tier_pixel_height(int tier) {
    atoms3_gfx_pick_font_for_tier(tier);
    return (int)gfx.fontHeight();
}

#endif /* M5ATOMS3 */
