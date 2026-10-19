// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Bare-metal dashboard: renders directly through graphics/lcd_bus.c (no Quantum
// Painter). QP's asset blobs (qgf images, qff fonts) are decoded by lcd_bus.

#include "graphics/display.h"
#include "lcd_bus.h"

#include <string.h>
#include <stdio.h>
#include "quantum.h"
#include "gpio.h"
#include "rtc/rtc.h"
#include "kb_eeconfig.h"   /* persisted backlight level (phase 4) */
#include "ak820pro.h"      /* LOOP_SITE profiler (LOOPGAP_INSTRUMENT) */
#include "indicators.h"   /* charge_is_charging + battery_is_absent */
#include "battery.h"      /* VDD for the debug page */
#include "power.h"        /* the battery row's bulb: power_lights_mode */
/* Also included further down beside the connection-state code, which is where
 * it historically lived; #pragma once makes the duplicate free. Hoisted because
 * the Fn+D debug page sits above that point and reads the battery and TX
 * counters through it. */
#include "bluetooth/ch582f_ajazz.h"
#include "health.h"        /* debug-page counters */

#ifdef AK820_DEV_HOOKS
#    include "loop_acct.h"   /* ACCT_NOW: the 16-bit system tick, 5.33 us */
/* The display probe (flash 2d, dev builds only): the worst time of each stage
 * of the Fn+D restore and of each housekeeping sub-draw, read over raw HID
 * (HC_DEVHOOK, hid_protocol.c). Two stalls measured on flash 2c are each one
 * of these: the page exit (32-33 ms, in the pump) and the settings overlay's
 * retirement (31 ms, in housekeeping). In other builds DPROBE() is just the
 * statement. */
enum {
    DP_RESTORE0  = 0,                         /* the 14 restore stages: 0..13 */
    DP_TEXT_SLOT = DP_RESTORE0 + 14,
    DP_STATUS, DP_LOCKS, DP_CLOCK, DP_BATT, DP_CONN,
    DP_TEXT_ICON, DP_BAND_CLEAR, DP_DEBUG_PAGE, DP_DBG_GLYPH,
    DP_N
};
static uint16_t dprobe_max[DP_N];             /* ticks */
static inline void dprobe_note(uint8_t i, uint16_t t0) {
    uint16_t d = (uint16_t)(ACCT_NOW() - t0);
    if (i < DP_N && d > dprobe_max[i]) dprobe_max[i] = d;
}
#    define DPROBE(i, stmt) do { uint16_t dp_t0_ = ACCT_NOW(); stmt; dprobe_note((uint8_t)(i), dp_t0_); } while (0)
#else
#    define DPROBE(i, stmt) do { stmt; } while (0)
#endif



#define PANEL_BKL       A16

#define PANEL_WIDTH     128
#define PANEL_HEIGHT    128

#define COL_BG          0x0000   // black background
#define COL_FG          0xFFFF   // white text

// Font blobs (Iosevka, mono 1bpp).
// Fonts and images now live in external flash and are DMA-drawn; these are
// ids into the index that flash_assets_init() reads at boot. The full 95-glyph
// atlases are used, so EMBED_CHARSET and its "letters silently draw nothing"
// trap are gone.
#define FONT_CLOCK      ASSET_IOSEVKA_REGULAR_30   // big clock
#define FONT_STATUS     ASSET_IOSEVKA_MEDIUM_20    // small status text
#define FONT_STATUS_ADV 10   /* 20px face cell advance. The firmware reads
                              * cell_w from the flash index at RUNTIME for
                              * ordinary draws; these named constants exist for
                              * the few places layout math needs the width at
                              * compile time -- they must match the PROVISIONED
                              * atlases (10x23 and 6x14 cells). */
/* NOW COZETTE, not Iosevka -- the filename is unchanged on purpose, because
 * mkraw.py assigns asset ids by sorted filename and renaming would shift every
 * later id. Iosevka at this ppem lost the quantisation lottery repeatedly: 'j'
 * shipped with a 2px stem against every other lowercase's 1px, 'A' had legs
 * that stepped at different rows, 'd' had a flat bowl top where 'b' was round,
 * and 'l' was near-indistinguishable from '1'. Cozette is a BITMAP font drawn
 * at 6x13 by hand, so there is no rasteriser to lose to. See mkbdfatlas.py. */
#define FONT_SMALL      ASSET_IOSEVKA_MEDIUM_13    // dense: 6px advance
#define FONT_SMALL_ADV  6

/* Align the text with the TRANSPORT ICON, not with the band.
 *
 * Centre the icon on the text's CAP BAND, not on its cell: the cell carries
 * descender space that is empty for most strings, so centring the cell pulls
 * the apparent centre down and the icon up.
 *
 * MEASURED from the shipped atlas, not assumed -- the numbers below moved when
 * the face was swapped from Iosevka to Cozette and the old ones survived in
 * this comment, which is what left the icon 1px high until 2026-09-02. To
 * re-measure after any font change, walk the atlas PNG and record the first
 * and last row carrying ink (255,255,255), skipping the magenta grid marker at
 * each cell's top-left:
 *
 *   13px face (Cozette, 6x14 cell): caps row 2, baseline row 9  -> centre 5.5
 *                                   descenders to row 12; Q's tail to row 10
 *   20px face (Iosevka, 10x23 cell): caps row 4, baseline row 18 -> centre 11
 *
 * draw_text_icon paints rows y..y+11 for all three icons, so the icon's centre
 * is y + 5.5. For the 13px face that makes the icon's y exactly the text's CELL
 * TOP (5.5 == 5.5); no offset of any kind is needed. */
/* Line pitch for the two-line case: the 13px cell is 14 rows and a glyph blit
 * paints its whole cell, so lines sit exactly one cell apart with no overlap.
 * The cell's 14th row is blank, which is the inter-line gap. */
#define TEXT_LINE_H     14
/* The transport icon sits beside LINE 0 only (it is 12px tall in a 14px row),
 * so line 1 starts at the panel edge and gains the gutter back -- 21 glyphs
 * (2 + 21*6 = 128) against line 0's 19. See DISPLAY_TEXT_MAX_L1.
 *
 * Both lines now end at the same column, so the last cell's ink lands at 126
 * (col 5 of the 6px cell is blank for everything but '4' and 'q', which use it).
 * Unlike the battery row this leaves no margin for the recessed bezel -- the
 * price of the extra characters, and only ever paid by a string long enough to
 * need every one of them. */
#define TEXT_X2         2

/* Single-line offsets: where the text CELL starts. The icon is then centred on
 * whichever face is in use -- see the cap-band figures above. */
#define TEXT_FONT_DY    8    /* 13px face */
#define TEXT_BIG_DY     2    /* 20px face */
/* (128 - TEXT_X - 2) / 10px advance = 11 glyphs in the big face. */
#define TEXT_BIG_MAX    11

// Bottom row: y position of the wireless status line.
#define STATUS_Y 105

/* volatile: read every GPT tick in ISR context (display_backlight_tick),
 * written from the main loop -- same discipline as bkl_level/ind_*. Benign
 * without it today (no caching loop), but the one exception invites the
 * refactor that silently caches it. (Audit C-2.) */
static volatile bool display_powered = true;
static bool display_paused  = false;   // true while the flash-animation player owns the bus
static bool debug_active    = false;   // true while the Fn+D debug page owns the panel
static uint8_t debug_exit_step  = 0;   // >0 while the dashboard is being restored
static uint8_t debug_clear_band = 0;   // >0 while the entry clear is banding down the panel
static bool    debug_big        = false; // the second Fn+D tap: the camera page (big digits)
static bool    locks_stage_first = true; // restore: the next lock-band pass forgets the (cleared) band
static bool splash_cleared = false;
static bool mac_mode = false;

uint32_t display_redraw_dashboard(uint32_t trigger_time, void *cb_arg);
void display_set_paused(bool paused) {
    display_paused = paused;
    if (!paused) display_redraw_dashboard(0, NULL);   // resume: full repaint
}

enum {
    CONN_MODE_WIRED = 0,
    CONN_MODE_BLUETOOTH,
    CONN_MODE_2_4G
};
static uint8_t connection_mode = CONN_MODE_WIRED;

bool display_get_power(void) {
    return display_powered;
}

// Backlight is event-driven: written here whenever the power state changes.
/* --- Backlight brightness -------------------------------------------------
 *
 * PANEL_BKL (A16) is a plain GPIO. Hardware PWM on it is IMPOSSIBLE, not merely
 * unused: the SN32F299 datasheet gives P0.16 the single alternate function
 * CT16B5_CAP0, a capture INPUT. So brightness is software PWM.
 *
 * The tick is CT16B3 (GPTD4) at a MEASURED 20000 Hz -- a dedicated timer, set up
 * in pwm_tick_init() in ak820pro.c. It used to run from a weak hook in the RGB
 * RGB row-scan ISR), which welded the switching rate to RGB_MATRIX_SPD_STEP:
 * retuning the LED field rate for the rainbow artifact silently retuned the
 * backlight into or out of flicker. THAT COUPLING IS GONE -- SPD_STEP no longer
 * affects this at all.
 *
 * Period is BKL_PWM_TICKS: 20000/48 = 417 Hz switching, floor 1/48 = 2.1%.
 * A dimmer floor needs a LONGER period, which lowers the switching rate; the
 * tick rate is now independent, so that trade is purely period-vs-floor.
 *
 * The levels are spaced perceptually, not linearly. Brightness perception is
 * roughly a power law, so an even duty spread wastes most of its steps at the
 * top where they are indistinguishable and gives nothing usable at the bottom,
 * which is the end that matters in a dark room. */
#define BKL_PWM_TICKS 48   /* 20000/48 = 417 Hz, floor 1/48 = 2.1%. Sized when
                            * the tick was losing 23% of its ticks to a priority
                            * inversion (an effective 15385 Hz -> 320 Hz). The
                            * tick is now a steady 20000, so there is headroom to
                            * lengthen this for a dimmer floor if wanted. */
/* 24 levels (was 10). Every entry is a distinct integer tick count -- the duty
 * resolution is only 0..48, so a longer table has to be checked for collapsed
 * steps, and this one has none.
 *
 * Ratios between adjacent steps run 1.09-1.17 above duty 6, against 1.5-1.78
 * for the old 10-level table: the jumps JD could see are gone. The bottom three
 * (1->2 = +100%, 2->3 = +50%, 3->4 = +33%) are NOT fixable at 48 ticks -- there
 * is no integer between them. Closing those needs a longer BKL_PWM_TICKS, which
 * lowers the switching rate; 96 ticks would give a 1% floor at 208 Hz, and that
 * is close enough to visible flicker that it was ruled out deliberately.
 * Flicker is a worse defect than a coarse first step.
 *
 * The last entry must equal BKL_PWM_TICKS so the top level is full duty;
 * C cannot _Static_assert on an array element, so it is stated here. */
static const uint8_t bkl_duty[] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
    12, 14, 16, 18, 20, 23, 26, 29, 33, 37, 42, 48,
};
#define BKL_MAX_LEVEL ((uint8_t)(sizeof(bkl_duty) / sizeof(bkl_duty[0])) - 1)
_Static_assert(sizeof(bkl_duty) == 24, "brightness UI documents 24 levels (0..23); resize deliberately");

#ifndef DISPLAY_BRIGHTNESS_DEFAULT
#    define DISPLAY_BRIGHTNESS_DEFAULT BKL_MAX_LEVEL
#endif

static volatile uint8_t bkl_level = DISPLAY_BRIGHTNESS_DEFAULT;
static volatile uint8_t bkl_phase = 0;
/* Power-save ceiling on the level (power.c), BKL_NO_CAP when off. A cap, not a
 * setting: the user's level, and its EEPROM copy, stay where they were. */
#define BKL_NO_CAP 0xFFu
static volatile uint8_t bkl_cap = BKL_NO_CAP;
/* A deep sleep's wake (sleep.c, plan section 4, step 9): dark until the clock
 * and the battery row have repainted, so the screen never lights on the slow
 * time or the level before the sleep's debit. */
static volatile bool bkl_wake_hold = false;
/* rtc.c's deep-sleep recovery used the PCF this pass: no more display work in it
 * (the budget, plan section 4). Cleared at the pass's end. */
static bool pass_deferred = false;

/* ISR context, called from pwm_tick_cb() in ak820pro.c -- the CT16B3/GPTD4 20 kHz
 * tick, which ak820pro.c owns because the indicator LEDs share it. Full and zero
 * duty are handled without toggling so a static backlight is not switched
 * needlessly. */
void display_backlight_tick(void) {
    uint8_t level = bkl_level;
    if (bkl_cap < level) level = bkl_cap;
    if (bkl_wake_hold) level = 0;
    uint8_t duty = display_powered ? bkl_duty[level] : 0;

    if (duty >= BKL_PWM_TICKS) {
        gpio_write_pin(PANEL_BKL, 1);
        return;
    }
    if (duty == 0) {
        gpio_write_pin(PANEL_BKL, 0);
        return;
    }

    uint8_t phase = bkl_phase + 1;
    if (phase >= BKL_PWM_TICKS) phase = 0;
    bkl_phase = phase;

    gpio_write_pin(PANEL_BKL, phase < duty);
}

uint8_t display_get_brightness_max(void) {
    return BKL_MAX_LEVEL;
}

uint8_t display_get_brightness(void) {
    return bkl_level;
}

void display_set_brightness(uint8_t level) {
    if (level > BKL_MAX_LEVEL) level = BKL_MAX_LEVEL;
    bkl_level = level;
    /* The raw setter deliberately does NOT persist -- the bootloader splash
     * forces max brightness through here, and persisting that would overwrite
     * the user's level on every flash cycle. The user-gesture paths below are
     * what persist. (The old "never persist" rule predated the
     * backing_store_pre_write_hook + coalesced-deferred-write machinery that
     * now makes a settled write safe -- phase 4.2.) */
}

void display_set_backlight_cap(uint8_t cap) {
    bkl_cap = cap;
}

uint8_t display_backlight_effective(void) {
    uint8_t level = bkl_level;
    if (bkl_cap < level) level = bkl_cap;
    if (bkl_wake_hold) level = 0;
    return display_powered ? level : 0;
}

void display_backlight_pin_off(void) {
    gpio_write_pin_low(PANEL_BKL);
}

uint8_t display_backlight_duty(void) {
    return bkl_duty[display_backlight_effective()];
}

void display_brightness_up(void) {
    if (bkl_level < BKL_MAX_LEVEL) display_set_brightness(bkl_level + 1);
    kb_eeconfig_set_lcd_brightness(bkl_level);
}

void display_brightness_down(void) {
    if (bkl_level > 0) display_set_brightness(bkl_level - 1);
    kb_eeconfig_set_lcd_brightness(bkl_level);
}

void display_set_power(bool on) {
    display_powered = on;
    /* The ISR hook drives the pin from here on; write it once so power-off takes
     * effect immediately rather than at the next tick. */
    gpio_write_pin(PANEL_BKL, on && (bkl_duty[bkl_level] > 0));
}

void display_toggle_power(void) {
    display_set_power(!display_powered);
}

static bool display_backlight_init(void) {
    /* Restore the persisted level (phase 4.2); a fresh/unset block keeps the
     * compile-time default. kb_eeconfig_init() ran earlier in post-init. */
    uint8_t lvl;
    if (kb_eeconfig_get_lcd_brightness(&lvl) && lvl <= BKL_MAX_LEVEL) bkl_level = lvl;
    gpio_set_pin_output(PANEL_BKL);
    gpio_write_pin(PANEL_BKL, display_powered); // initial state (on)
    return true;
}

// y position of the big clock (top of the glyphs).
/* Vertical budget, 128 rows total. A glyph blit paints its WHOLE cell including
 * background, so cells cannot overlap and a band costs exactly its cell height.
 *
 *   0..24    connection strip   25
 *   25..26   gap                 2
 *   27..54   text (2 lines)     28   two 7x14 cells, at 27 and 41
 *   55       gap                 1
 *   56..77   clock              22   Regular-30, CROPPED to its ink
 *   78..81   gap                 4
 *   82..104  lock band          23   20px face
 *   105..127 battery            23   20px face
 *
 * SPACING IS SET BY INK, NOT BY BANDS. The 20px cell carries 4 blank rows above
 * its caps and 4 below its baseline, so a band boundary is nowhere near where
 * the eye puts the edge. Measured ink-to-ink the rhythm is:
 *
 *   icons -> text   2
 *   text  -> clock  2
 *   clock -> lock   8
 *   lock  -> batt   8
 *
 * Those last two read as one visual gap each side of the lock row, and they
 * were 6 and 8 before -- visibly lopsided. Balancing them took THREE coupled
 * moves, not one: the lock row down 1, the battery down 1 (the 20px cell needs
 * all 23 rows, so the lock band cannot grow without pushing STATUS_Y), and the
 * clock up 1 to make the two gaps exactly equal rather than 7 and 8.
 *
 * THE CLOCK ATLAS IS CROPPED TO 15x22 AND THAT IS WHERE THE SPACE CAME FROM.
 * It was 15x34, cut for full ASCII with room for ascenders and descenders --
 * but the clock only ever draws 0-9 and ':', which use neither, so 12 of its
 * 34 rows were blank by construction (5 above the digits, 7 below). Cropping
 * to the measured ink freed those 12 rows AND cut 34 KB off the asset blob.
 * The glyphs themselves are untouched, pixel for pixel; only the cell moved.
 *
 * That is what paid for the 2px gaps and for keeping the battery percentage at
 * 20px. Before the crop the panel was full to the row and every one of these
 * was a trade against another.
 *
 * ⚠️ DESCENDERS ARE CLIPPED IN THAT ATLAS. g/p/q/y and friends lost their
 * tails, because only the clock and the Fn+D camera page draw with FONT_CLOCK,
 * and neither draws a descender (the camera page: digits . : % V < > -). If you
 * ever draw general text at this size, regenerate the atlas at its full 15x34
 * cell and hand these 12 rows back -- do not try to fix it by nudging offsets. */
#define CLOCK_Y 56
/* 23, not the clock cell's 22: the playback readout uses the 20px face, whose
 * cell is 23 rows, borrowing one row from the 4-row gap below. The clear rect
 * must cover everything either owner can draw, or switching between them
 * leaves a row behind -- the padlock mistake again. */
#define CLOCK_BAND_H 23

// Clock format: 1 = HH:MM:SS (per-second redraw of the changed cells), 0 = HH:MM.
#ifndef DISPLAY_CLOCK_SHOW_SECONDS
#    define DISPLAY_CLOCK_SHOW_SECONDS TRUE
#endif

// Forces a full clock+date repaint after the background is cleared.
static bool clock_force_repaint = true;
static uint16_t clock_gen_painted = 0;   /* rtc_clock_generation() as the clock last drew from it */

static void draw_status(bool force); // CH582F status: battery + channel digit
/* draw_status() is FOUR forced repaints in a trench coat. Stepping it as one
 * unit left a 44 ms stall on the debug-page exit even after the clear was
 * banded -- the clear was never the whole cost. Declared individually so the
 * restore can spend one main-loop pass on each. */
static void draw_conn_number(bool force);
static void draw_battery(bool force);
static void draw_locks(bool force);
static bool draw_locks_staged(bool first);   // Fn+D restore: one lock-band component per call
static void draw_text_slot(bool force);
static void draw_debug_page(void);   // Fn+D full-panel diagnostics
static bool debug_pump_glyph(void);  // one changed debug cell, non-blocking
static bool cam_pump_glyph(void);    // one changed camera-page cell, non-blocking
static bool debug_restore_step(void); // one stage of the post-debug dashboard repaint
static void draw_conn_row(void);      // three-transport strip, top row

/* ---- Playback position ----------------------------------------------------
 *
 * Replaces the clock while media is playing, and hands the band straight back
 * when it stops. Showing a FROZEN timer while paused would be worse than the
 * time of day, which is why only the playing state takes the band.
 *
 * The host pushes an absolute position every few seconds and the firmware
 * advances it locally on the 1 Hz tick in between. That is what makes it read
 * as a running timer rather than a value that jumps every poll -- and it costs
 * no extra panel work, because this band already repaints once a second for
 * the clock's seconds.
 *
 * It expires on its own (PLAYBACK_TIMEOUT_MS). A stuck timer counting up
 * forever after the agent dies or the machine sleeps is exactly the kind of
 * confidently-wrong readout the text slot's expiry exists to prevent.  */
#define PLAYBACK_TIMEOUT_MS 20000u
#define PLAYBACK_MAX        24

static bool     pb_active = false;
static bool     pb_ticking = false;
static uint16_t pb_pos = 0, pb_dur = 0;
static uint32_t pb_stamp = 0;

void display_set_playback(uint8_t state, uint16_t pos, uint16_t dur) {
    bool want = (state != 0);
    if (want != pb_active) {
        pb_active = want;
        clock_force_repaint = true;     // band changes owner: repaint it whole
    }
    pb_pos     = pos;
    pb_dur     = dur;
    pb_ticking = want;      // the host only reports a position while PLAYING
    pb_stamp   = timer_read32();
}

/* Freeze/unfreeze on the media key, the same optimistic trick the transport
 * icon uses. Without it the timer kept counting for up to a poll after a pause,
 * which is the one thing a paused timer must not do.
 *
 * Resume needs no guesswork about the position: it does not change while
 * paused, so the value already held is still correct. Only whether it should be
 * ADVANCING is in doubt, and the host settles that within a poll either way.
 *
 * Freezing rather than handing the band straight back to the clock is
 * deliberate. The keypress may have gone to a browser tab this host agent
 * cannot see, in which case the player is still going and the host will re-
 * assert it -- a frozen timer that resumes is a much smaller lie than the clock
 * flashing up and being replaced. */
void display_playback_key(void) {
    if (pb_active) pb_ticking = !pb_ticking;
}

void display_playback_tick(void) {
    if (!pb_active) return;
    if (timer_elapsed32(pb_stamp) > PLAYBACK_TIMEOUT_MS) {
        pb_active = false;
        clock_force_repaint = true;
        return;
    }
    if (!pb_ticking) return;    // paused: hold the value, do not advance it
    /* Do not run past the end: at a track boundary the host's next push is up
     * to a poll away, and a timer reading beyond the duration looks broken. */
    /* Saturate rather than wrap. With dur == 0 (a live stream, no known length)
     * the position would otherwise roll 65535 -> 0 after 18h12m and the timer
     * would silently restart at 0:00. */
    if (pb_dur) {
        if (pb_pos < pb_dur) pb_pos++;
    } else if (pb_pos < 0xFFFFu) {
        pb_pos++;
    }
}

static uint8_t fmt_hms(char *out, uint16_t sec) {
    uint16_t h = sec / 3600, m = (sec / 60) % 60, s = sec % 60;
    return (uint8_t)(h ? snprintf(out, 9, "%u:%02u:%02u", h, m, s)
                       : snprintf(out, 9, "%u:%02u", m, s));
}

/* Forward decls: the clock band shares the glyph queue defined further down. */
static void queue_line(uint8_t slot, uint16_t font, uint16_t x, uint16_t y,
                       const char *str);
static void band_clear(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
static bool gq_pending(void);
static void display_queue_discard(void);
static void shadow_mark_empty(uint8_t slot, uint16_t font, uint16_t x, uint16_t y);
static void retire_line(uint8_t slot);
/* Slot 2 of the glyph queue is the CLOCK BAND -- the clock or the playback
 * timer, whichever owns it (defined with the queue further down). */
#define GQ_SLOT_CLOCK 2

/* ---- Clock format: 24h / 12h + AM-PM / off ---------------------------------
 *
 * Display-only (plans/CLOCK-FORMAT-PLAN.md): the rtc module keeps the time,
 * this only changes how the band draws it. 12h drops the leading zero
 * (H:MM:SS, 7 or 8 cells) and adds a 6px-wide stacked A/M or P/M to the
 * right of the digits in light grey.
 *
 * The letters are rasterised in firmware from 5x7 bitmaps into a RAM tile
 * (see draw_ampm), not drawn from an atlas, for two reasons: the atlases
 * have no colour (a glyph blit paints its whole cell, white on black), and a
 * new atlas PNG would shift every later asset id and force the coordinated
 * rebuild + re-provision (fonts-assets.md). Two 6x14 Cozette cells stacked
 * would also be 28 rows in a 22-row face.
 *
 * The glyph lives OUTSIDE the glyph queue's shadow, so `ampm` is its own
 * tiny shadow: where it is and which letter, so it can be cleared before it
 * moves (7 <-> 8 cells at 9:59:59 -> 10:00:00), flips (noon, midnight) or
 * goes away (mode change). That is a handful of times a day; the per-second
 * path never touches it. Clearing it never invalidates the clock run's own
 * shadow, because the glyph starts exactly where the run ends and
 * rects_overlap() is half-open. Every WHOLE-band clear goes through
 * clock_band_clear() so the two shadows cannot disagree. */
static uint8_t clock_mode = DISPLAY_CLOCK_24H;
static struct { bool shown; uint16_t x; bool pm; } ampm = {false, 0, false};
/* A format change RELAYOUTS the band; it does not wipe it. The first version
 * forced a whole-band clear and the eight digits then arrived one per main-
 * loop pass -- a visible black flash, the same wipe-was-the-flicker lesson
 * the text band learned. Now the run moves through queue_line's diff path
 * (which clears only the columns it vacates) and the old digits stay up
 * until each cell is overwritten. Only an OWNER change (playback) still
 * clears the band whole, via clock_force_repaint. */
static bool clock_relayout = false;

#define AMPM_W        6     /* 1px bearing + 5px of ink                          */
#define AMPM_INK_W    5
#define AMPM_LETTER_H 7     /* Cozette capitals are 5x7 of ink                    */
#define AMPM_GAP      2     /* rows between the stacked letters                  */
#define AMPM_STACK_H  (2 * AMPM_LETTER_H + AMPM_GAP)   /* 16 of the face's 22 rows */
#define AMPM_Y_OFF    ((22 - AMPM_STACK_H) / 2)         /* centred on the digits: 3 */
/* 12h sits 2px left of centre: 8 cells + glyph is 126 of 128 columns, and the
 * recessed panel's bezel clips the outermost ones (display.md). */
#define AMPM_X_SHIFT  2
#define COL_AMPM      0xCE79                            /* #CCCCCC light grey, RGB565: a hair off white -- mid grey (0x8410) was near-illegible at 5x7 */
_Static_assert(AMPM_Y_OFF + AMPM_STACK_H <= CLOCK_BAND_H, "AM/PM stack must fit the clock band");
_Static_assert(8 * 15 + AMPM_W <= PANEL_WIDTH, "12:MM:SS plus the AM/PM glyph must fit the panel");

/* 5x7 capitals after Cozette's shapes; bit 4 is the leftmost column. */
static const uint8_t ampm_A[AMPM_LETTER_H] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
static const uint8_t ampm_P[AMPM_LETTER_H] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
static const uint8_t ampm_M[AMPM_LETTER_H] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};

/* Rasterise one letter into the 5x16 RAM tile at row `top`. */
static void ampm_rasterise(uint16_t *tile, const uint8_t *rows, uint8_t top) {
    for (uint8_t r = 0; r < AMPM_LETTER_H; r++)
        for (uint8_t c = 0; c < AMPM_INK_W; c++)
            if (rows[r] & (0x10u >> c)) tile[(top + r) * AMPM_INK_W + c] = COL_AMPM;
}

/* x is the glyph's cell origin (the run's end); y is the band top.
 *
 * ONE RAM tile, ONE transfer. The first version emitted the letters as ~28
 * lcd_fill_rect runs, the way the padlock is drawn -- and the profiled build
 * put that at ~24 ms for 160 pixels: every rectangle pays an 11-byte window
 * command sent BYTE BY BYTE through spiSend, and the ChibiOS call overhead
 * per byte dwarfs the pixels. A 5x16 tile is 160 bytes through lcd_blit_ram:
 * one window, one send. It also paints its own background, so a letter flip
 * (A -> P) needs no clear first. lcd_blit_ram is the CPU path (the SN32 DMA
 * cannot source RAM), which is fine at this size. */
static void draw_ampm(uint16_t x, uint16_t y, bool pm) {
    static uint16_t tile[AMPM_INK_W * AMPM_STACK_H];
    memset(tile, 0, sizeof(tile));                      /* black background */
    ampm_rasterise(tile, pm ? ampm_P : ampm_A, 0);
    ampm_rasterise(tile, ampm_M, AMPM_LETTER_H + AMPM_GAP);
    /* CPU pushes share SPI0 with the DMA blits and lcd_blit_ram does not
     * wait on its own. Every caller has just cleared (which waits), but the
     * padlock's reliance on caller ordering is not worth copying. Free when
     * the bus is idle. */
    lcd_blit_wait();
    lcd_blit_ram(tile, (uint16_t)(x + 1), (uint16_t)(y + AMPM_Y_OFF), AMPM_INK_W, AMPM_STACK_H);
}

/* The one way the clock band is wiped whole: an owner change (clock <->
 * playback) and the dashboard repaint. A format change does NOT come here --
 * it relayouts through the diff path (clock_relayout). band_clear only knows
 * the glyph queue's runs, so the AM/PM shadow is reset here, with the pixels. */
static void clock_band_clear(void) {
    band_clear(0, CLOCK_Y, PANEL_WIDTH, CLOCK_BAND_H);
    ampm.shown = false;
}

uint8_t display_get_clock_mode(void) {
    return clock_mode;
}

void display_set_clock_mode(uint8_t mode) {
    if (mode >= DISPLAY_CLOCK_MODE_COUNT) mode = DISPLAY_CLOCK_24H;
    if (mode == clock_mode) return;
    clock_mode = mode;
    /* The housekeeping pass paints a relayout on its next 100 ms tick rather
     * than waiting for the second edge. */
    clock_relayout = true;
}

static void draw_playback(void) {
    char buf[PLAYBACK_MAX + 1];
    uint8_t n = fmt_hms(buf, pb_pos);
    if (pb_dur) {
        buf[n++] = '/';
        n = (uint8_t)(n + fmt_hms(buf + n, pb_dur));
    }
    buf[n] = '\0';

    /* Same adaptive rule as the text slot: the big face unless the string is
     * too wide for the panel, which only happens once a duration reaches an
     * hour. A 20px cell is 23 rows against this band's 22, so it borrows one
     * row from the 4-row gap below -- covered by the band clear. */
    bool     big  = (n * FONT_STATUS_ADV) <= PANEL_WIDTH;
    uint16_t font = big ? FONT_STATUS : FONT_SMALL;
    uint8_t  adv  = big ? FONT_STATUS_ADV : FONT_SMALL_ADV;
    uint16_t x0   = (uint16_t)((PANEL_WIDTH - n * adv) / 2);
    uint16_t y    = big ? CLOCK_Y : (CLOCK_Y + 4);

    /* Ownership change (clock <-> playback): clear the whole band through
     * band_clear so every overlapping shadow is invalidated -- then tell the
     * slot it is KNOWN EMPTY at the new layout, or queue_line's unknown path
     * clears the full width a second time (profiled: ~8 ms per clear). */
    if (clock_force_repaint) {
        clock_band_clear();
        shadow_mark_empty(GQ_SLOT_CLOCK, font, x0, y);
    }

    /* Through the glyph queue (audit BW-2): the per-second case still diffs
     * to 1-2 glyphs against the shadow, and a RELAYOUT -- font or length
     * change moves x0/y, which queue_line detects as a move -- clears the old
     * run and queues the full new one WITHOUT blocking the main loop. The old
     * blocking version cost ~22 ms at every start/stop/hour boundary. */
    queue_line(GQ_SLOT_CLOCK, font, x0, y, buf);
}

/* Bootloader splash.
 *
 * Bootloader mode is indistinguishable from a dead board -- no RGB, dark LCD,
 * no typing, and no subtle indicator. That has cost real diagnostic time.
 *
 * The GC9107 holds its own GRAM and has its own supply, so an image drawn here
 * survives the MCU reset that follows. Whether the BACKLIGHT survives is the
 * open question: PANEL_BKL (A16) goes high-impedance on reset, so if the board
 * has no pull-up the panel keeps the picture but goes unlit. If that turns out
 * to be the case the fix is to re-assert A16 after the reset, before the ROM
 * jump -- but this version is a pure no-op on failure, which is why it is
 * worth trying first.
 *
 * Layout is measured, not eyeballed. Cozette is 6x14 and Iosevka-Medium-20 is
 * 10x23; every line clears the recessed bezel by >=10px (BATT_X0 is 5 for the
 * same reason). Groups are 8px apart with 2 rows of margin:
 *
 *        BOOTLOADER        20px   y 2
 *     Awaiting firmware    13px   y 33
 *     Unplug and replug    13px   y 55
 *     to exit bootloader   13px   y 69
 *       USB Device ID:     13px   y 91
 *         0C45:7140        20px   y 105
 */
/* --- Fn+D debug page ---------------------------------------------------------
 *
 * EXISTS FOR UNTETHERED USE. With the cable attached, ak820health.py reads all
 * of this and more over raw HID in ANY slider position (commit 4b86d95014 sends
 * raw-HID replies over USB in wireless mode; ak820health.py's docstring still
 * claims otherwise and is stale). So this page is NOT the only way to see these
 * counters -- it is the only way to see them with no host attached, which is
 * exactly when the CH582F link is carrying your keystrokes and is the thing you
 * would want to look at.
 *
 * The second reason is that BT -> cable is a BROWNOUT RESET (readme.md's
 * measured transition matrix; cable -> BT is the only flip that survives). So
 * plugging in to investigate a wireless session destroys the very counters you
 * went to read. You read this page with your eyes, so the reset never happens.
 *
 * Owns the whole panel while active: display_housekeeping_task() returns here
 * instead of drawing the dashboard, and the toggle repaints the dashboard on
 * the way out. 21 columns x 9 rows in the 6x14 face -- cramming is fine, you
 * are already leaning in when you press Fn+D. */
/* A full-screen clear is 128*128*2 = 32 KB streamed flash->panel, and
 * lcd_clear_rect() ends in lcd_blit_wait(): measured 43-44 ms with the main
 * loop parked, which count_ge_25ms_nonflash correctly calls a keystroke hazard.
 *
 * Clearing in BANDS fixes it without going asynchronous. 128x16 is 4 KB, about
 * 5.5 ms -- under the 10 ms leading indicator, let alone the 25 ms alarm -- and
 * one band per main-loop pass costs the same total wire time while never
 * blocking for longer than a band.
 *
 * The earlier async attempt (lcd_clear_rect_async, reverted) HUNG THE BOARD:
 * arming the DMA and returning meant routing around the pump's stuck-blit
 * recovery and letting the dashboard draw over a restore still in progress.
 * Banding needs neither. Every call here is the same synchronous
 * lcd_clear_rect() that has always worked; there are just more, smaller ones. */
#define DBG_CLEAR_BAND_H 16
#define DBG_CLEAR_BANDS  (PANEL_HEIGHT / DBG_CLEAR_BAND_H)
_Static_assert(PANEL_HEIGHT % DBG_CLEAR_BAND_H == 0, "clear bands must tile the panel exactly");

#define DBG_ROWS      9
#define DBG_ROW_H     14
#define DBG_COLS      (PANEL_WIDTH / FONT_SMALL_ADV)   /* 21 */
_Static_assert(DBG_ROWS * DBG_ROW_H <= PANEL_HEIGHT, "debug page must fit the panel");

/* Composed row text, and what is currently ON the panel. Split because the page
 * must NOT paint everything it computes: see draw_debug_page(). */
static char dbg_text[DBG_ROWS][DBG_COLS + 1];
static char dbg_shown[DBG_ROWS][DBG_COLS + 1];

/* COMPOSE ONLY -- no LCD I/O. Right-aligns a value under a fixed-width label:
 * "sent" + 711 -> "sent            711". Fixed columns beat a running printf
 * because the numbers change every second and a jumping column is much harder
 * to read at 6px than a still one. Always fills the full width, so a staged
 * blit overwrites the previous row completely and short values cannot leave
 * stale characters behind. */
static void dbg_row(uint8_t row, const char *label, const char *value) {
    if (row >= DBG_ROWS) return;
    char *line = dbg_text[row];
    uint8_t i = 0;
    while (label[i] && i < DBG_COLS) { line[i] = label[i]; i++; }
    uint8_t vn = 0;
    while (value[vn]) vn++;
    if (vn > DBG_COLS) vn = DBG_COLS;
    uint8_t start = (vn >= DBG_COLS) ? 0 : (uint8_t)(DBG_COLS - vn);
    while (i < start) line[i++] = ' ';
    for (uint8_t k = 0; k < vn && i < DBG_COLS; k++) line[i++] = value[k];
    while (i < DBG_COLS) line[i++] = ' ';
    line[DBG_COLS] = '\0';
}

/* u32 -> decimal, no printf.
 *
 * RETURNS THE END (the terminating NUL), like dbg_append, so the two compose.
 * It used to return the START, which silently ATE EVERY NUMBER THAT HAD A
 * SUFFIX: dbg_append(dbg_u32(p, 73), "%") wrote "73" and then overwrote it in
 * place with "%". The panel showed "ISR cpu  %", "up  m  s", "rowgap  ms r3" --
 * labels and units intact, values gone -- while bare numbers with nothing
 * appended (ISR/s, scan/s, BT drop) were correct, which is what made the
 * pattern legible. Both helpers must agree on which end they return. */
static char *dbg_u32(char *p, uint32_t v) {
    char tmp[11];
    uint8_t n = 0;
    if (v == 0) tmp[n++] = '0';
    while (v) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n) *p++ = tmp[--n];
    *p = '\0';
    return p;
}

static char *dbg_append(char *p, const char *s) {
    while (*s) *p++ = *s++;
    *p = '\0';
    return p;
}

static void dbg_compose(void);

/* Bit per row: this row's composed text differs from what is on the panel.
 * The pump drains it one GLYPH at a time; see debug_pump_glyph(). */
static uint16_t dbg_dirty = 0;

/* ⚠️ THIS PAGE ONCE MADE THE KEYBOARD UNTYPEABLE. Read before changing it.
 *
 * The first version drew all nine rows with lcd_draw_flash_text() on every
 * refresh. That call is SYNCHRONOUS and issues one LCD operation per glyph -- a
 * window command, a flash read and a DMA arm per 6x14 cell, almost all
 * overhead. Nine rows of 21 columns is ~189 of them: measured blit_gap_max_ms
 * 304 and seven unexplained >=25 ms main-loop stalls, and JD's typing
 * visibly collapsed the moment the page appeared. A page built to prove
 * keystrokes were not being lost was losing them.
 *
 * Note also that lcd_draw_flash_text_staged() -- which the comments around the
 * host-text band recommend for exactly this, and which the header declares --
 * DOES NOT EXIST. It has no definition anywhere in the tree; using it is a link
 * error. Do not reach for it on the strength of those comments.
 *
 * So this page does what the band actually does: compose in RAM here, and let
 * display_blit_pump() paint ONE CHANGED GLYPH PER MAIN-LOOP PASS with the
 * non-blocking lcd_draw_flash_glyph_try(). At ~340 Hz a full 189-cell repaint
 * settles in ~0.6 s and steady state is a handful of cells a second, because
 * only the digits that actually changed are repainted. No pass ever blocks. */
static bool dbg_force_recompose = false;

/* Recompose on the next tick regardless of the second edge, so hold-Fn+D shows
 * its zeros immediately instead of up to a second later. Composition is
 * RAM-only and painting stays incremental, so this costs nothing. */
void display_debug_refresh(void) {
    dbg_force_recompose = true;
}

static void draw_debug_page(void) {
    static uint32_t last_sec = UINT32_MAX;
    uint32_t sec = rtc_get_seconds();
    if (sec == last_sec && !dbg_force_recompose) return;
    last_sec = sec;
    dbg_force_recompose = false;

    dbg_compose();
    for (uint8_t r = 0; r < DBG_ROWS; r++)
        if (memcmp(dbg_text[r], dbg_shown[r], DBG_COLS) != 0)
            dbg_dirty |= (uint16_t)(1u << r);
}

/* One changed cell per call, non-blocking. Returns true if it owned this pass.
 * Called from display_blit_pump() at MAIN-LOOP rate -- the housekeeping tick is
 * 10 Hz and is the wrong clock for this, the same lesson the glyph queue's own
 * comment records. */
static bool debug_pump_glyph(void) {
    /* Clear first, one band per pass. No glyph may land before the panel is
     * blank or a later band would wipe it. */
    if (debug_clear_band) {
        lcd_clear_rect(0, (uint16_t)((debug_clear_band - 1) * DBG_CLEAR_BAND_H),
                       PANEL_WIDTH, DBG_CLEAR_BAND_H);
        if (++debug_clear_band > DBG_CLEAR_BANDS) debug_clear_band = 0;
        return true;
    }
    if (debug_big && cam_pump_glyph()) return true;
    if (!dbg_dirty) return false;
    for (uint8_t r = 0; r < DBG_ROWS; r++) {
        if (!(dbg_dirty & (1u << r))) continue;
        for (uint8_t c = 0; c < DBG_COLS; c++) {
            if (dbg_text[r][c] == dbg_shown[r][c]) continue;
            if (!lcd_draw_flash_glyph_try(FONT_SMALL, dbg_text[r][c],
                                          (uint16_t)(c * FONT_SMALL_ADV),
                                          (uint16_t)(r * DBG_ROW_H)))
                return true;   /* bus busy: keep the cell dirty, retry next pass */
            dbg_shown[r][c] = dbg_text[r][c];
            return true;
        }
        dbg_dirty &= (uint16_t)~(1u << r);   /* row fully painted */
    }
    return false;
}

/* --- Fn+D's camera page ----------------------------------------------------------
 * The pack and the level in text a camera can read across a room. The debug
 * page's 6x14 text is ~3 px tall on the security camera JD films battery runs
 * with (2026-10-02), and the moment worth filming -- the board dying on the
 * lights-off reserve -- is the one no RAM log survives. The second tap of Fn+D;
 * a third returns to the dashboard. It shares the debug page's machinery: the
 * banded clear on entry, the staged restore on exit, and one glyph per pass.
 *
 * In the CLOCK's font (15x22, the largest face), since 2026-10-06. It replaced
 * seven-segment digits drawn as rectangles, which JD found ugly and cryptic;
 * the atlas, cropped for the clock, still holds all 95 ASCII glyphs, and the
 * ones drawn here (digits . : % V < > -) have no descenders, which the crop cut
 * (see CLOCK_Y). The previews were rendered from the shipped atlases first.
 *
 *   line 1  the pack, "3.712V"; ">4.036V" / "<3.161V" at the top and bottom
 *           clamps (bounds, never numbers, as on the Batt row); "-.---V" none
 *   line 2  the level and, on battery, the time left to the cut at the
 *           recent draw (battery.c's energy count, Phase 2): "58% 31h",
 *           "100% 4d" from 100 h, "9% <1h" under one, "~" in place of the
 *           space while the average has under an hour; the level alone on USB,
 *           at the cut or before the first minute; "--%" unknown
 *   line 3  the board's clock, "14:03:27", for time-lapse video: every frame
 *           dates itself from the clock the timekeeper keeps right
 *   line 4  the raw 5C and its age in s, "5C 62@3" (the 10x23 status face)
 *
 * Each line is a FIXED grid of cells, so a changed character repaints only its
 * own cell (a glyph tile paints its background too, so a space clears one).
 * Text is centred by padding within the grid; each grid's width is chosen so
 * the usual length sits exactly centred (6 chars of volts, 3 of percent, 8 of
 * clock, 7 of 5C) and the rest are off by at most half a cell. Line 2 is 8
 * cells since Phase 2: "100% 51h" is eight (codex), and 8 x 15 px fits the
 * 128 px panel. */
#define CAM_BIG_ADV   15   /* FONT_CLOCK's advance (draw_clock's 8 * 15) */
#define CAM_LINES     4
#define CAM_MAX_COLS  11
typedef struct {
    uint16_t font;
    uint8_t  adv, cols, y;
} cam_line_t;
static const cam_line_t cam_lines[CAM_LINES] = {
    {FONT_CLOCK,  CAM_BIG_ADV,     8,  4},
    {FONT_CLOCK,  CAM_BIG_ADV,     8,  32},
    {FONT_CLOCK,  CAM_BIG_ADV,     8,  64},
    {FONT_STATUS, FONT_STATUS_ADV, 11, 98},
};
_Static_assert(8 * CAM_BIG_ADV <= PANEL_WIDTH && 11 * FONT_STATUS_ADV <= PANEL_WIDTH, "camera lines must fit the panel");
_Static_assert(98 + 23 <= PANEL_HEIGHT, "the 5C line must fit the panel");
static char    cam_text[CAM_LINES][CAM_MAX_COLS];
static char    cam_shown[CAM_LINES][CAM_MAX_COLS];
static uint8_t cam_dirty;   /* bit per line: text differs from the panel */

/* Blank both grids: the banded clear on entry leaves the panel blank, which is
 * what a grid of spaces looks like. */
static void cam_reset(void) {
    memset(cam_text, ' ', sizeof(cam_text));
    memset(cam_shown, ' ', sizeof(cam_shown));
    cam_dirty = 0;
}

/* COMPOSE ONLY: line `i` becomes `s`, centred in its grid. */
static void cam_set(uint8_t i, const char *s) {
    const uint8_t cols = cam_lines[i].cols;
    uint8_t n = 0;
    while (s[n] && n < cols) n++;
    uint8_t pad = (uint8_t)((cols - n) / 2u);
    char    line[CAM_MAX_COLS];
    memset(line, ' ', sizeof(line));
    memcpy(&line[pad], s, n);
    if (memcmp(line, cam_text[i], cols) != 0) {
        memcpy(cam_text[i], line, cols);
        cam_dirty |= (uint8_t)(1u << i);
    }
}

/* COMPOSE ONLY -- no LCD I/O. Once a second, from dbg_compose(). */
static void cam_compose(void) {
    char t[12], *p;

    uint8_t  med = battery_5c_rounded();
    uint16_t pk  = battery_pack_mv();
    p = t;
    if (battery_is_absent() || !pk) {
        p = dbg_append(t, "-.---");
    } else {
        if (med == 100u) *p++ = '>';
        else if (med == 0u) *p++ = '<';
        *p++ = (char)('0' + (pk / 1000u) % 10u);
        *p++ = '.';
        *p++ = (char)('0' + (pk / 100u) % 10u);
        *p++ = (char)('0' + (pk / 10u) % 10u);
        *p++ = (char)('0' + pk % 10u);
    }
    dbg_append(p, "V");
    cam_set(0, t);

    uint8_t lv = battery_level_pct();   /* 0xFF unknown */
    if (lv > 100u) {
        cam_set(1, "--%");
    } else {
        p = dbg_append(dbg_u32(t, lv), "%");
        uint16_t left = battery_time_left_min();
        if (left != 0xFFFFu) {
            /* "~" for the space while the average is young: "100%~51h" stays
             * eight cells. */
            *p++ = battery_avg_minutes() < 60u ? '~' : ' ';
            if (left < 60u) {
                dbg_append(p, "<1h");
            } else if (left < 100u * 60u) {
                dbg_append(dbg_u32(p, left / 60u), "h");
            } else {
                dbg_append(dbg_u32(p, (left + 720u) / 1440u), "d");
            }
        }
        cam_set(1, t);
    }

    char clk[9] = "--:--:--";
    rtc_time_t now;
    if (rtc_get_time(&now)) {
        const uint8_t f[3] = {now.hours, now.minutes, now.seconds};
        for (uint8_t i = 0; i < 3; i++) {
            clk[i * 3]     = (char)('0' + f[i] / 10u);
            clk[i * 3 + 1] = (char)('0' + f[i] % 10u);
        }
    }
    cam_set(2, clk);

    uint8_t  raw  = ch582_get_battery();
    uint32_t rage = ch582_battery_age_ms();
    p = dbg_append(t, "5C ");
    if (raw <= 100 && rage < 20000u) {
        p = dbg_u32(p, raw);
        p = dbg_append(p, "@");
        dbg_u32(p, rage / 1000u);
    } else {
        dbg_append(p, "--");
    }
    cam_set(3, t);
}

/* One changed cell per call, non-blocking, as debug_pump_glyph(). Returns true
 * if it owned this pass. */
static bool cam_pump_glyph(void) {
    for (uint8_t i = 0; i < CAM_LINES; i++) {
        if (!(cam_dirty & (1u << i))) continue;
        const cam_line_t *ln = &cam_lines[i];
        const uint16_t    x0 = (uint16_t)((PANEL_WIDTH - ln->cols * ln->adv) / 2u);
        for (uint8_t c = 0; c < ln->cols; c++) {
            if (cam_text[i][c] == cam_shown[i][c]) continue;
            if (!lcd_draw_flash_glyph_try(ln->font, cam_text[i][c], (uint16_t)(x0 + c * ln->adv), ln->y))
                return true;   /* bus busy: keep the cell dirty, retry next pass */
            cam_shown[i][c] = cam_text[i][c];
            return true;
        }
        cam_dirty &= (uint8_t)~(1u << i);   /* line fully painted */
    }
    return false;
}

static void dbg_compose(void) {
    char v[24], *p;

    /* Uptime -- the timebase for every since-boot counter below, and the thing
     * that says whether a reading is meaningful yet. */
    uint32_t up = timer_read32() / 1000u;
    p = v;
    if (up >= 3600u) { p = dbg_append(dbg_u32(p, up / 3600u), "h "); up %= 3600u; }
    p = dbg_append(dbg_u32(p, up / 60u), "m ");
    p = dbg_append(dbg_u32(p, up % 60u), "s");
    dbg_row(0, "Uptime", v);

    /* Row ISR occupancy. THE headline number: the row ISR is ~73% of this M0 at
     * idle, so it -- not the timer configuration -- sets the LED field rate,
     * the matrix scan rate and what is left for the main loop. A live
     * one-second window rather than a since-reset average, precisely so it can
     * be watched while changing RGB modes. */
    static uint32_t prev_entries = 0, prev_ticks = 0, prev_ms = 0;
    uint32_t entries, ticks;
    uint16_t tmin, tmax;
    health_isr_stats(&entries, &ticks, &tmin, &tmax);
    uint32_t now_ms = timer_read32();
    uint32_t d_ms   = now_ms - prev_ms;
    uint32_t d_ent  = entries - prev_entries;
    uint32_t d_tick = ticks - prev_ticks;
    uint32_t hz     = health_isr_tick_hz();

    if (prev_ms && d_ms && hz >= 1000u) {
        /* One row for both: "73% 3900". Occupancy and rate are read together --
         * the rate only means something against the share it costs -- and
         * merging them buys the row that stall>10 now occupies. Field rate is
         * the second number / 18.
         *
         * Scale by ticks-per-ms BEFORE multiplying by 100: at 73% of 187,500
         * ticks/s a full-precision d_tick * 100000 overflows u32. */
        p = dbg_append(dbg_u32(v, (d_tick / (hz / 1000u)) * 100u / d_ms), "% ");
        dbg_u32(p, d_ent * 1000u / d_ms);
        dbg_row(1, "ISR", v);
    } else {
        dbg_row(1, "ISR", "--");
    }
    prev_entries = entries; prev_ticks = ticks; prev_ms = now_ms;

    /* The two counters that can actually LOSE a keystroke.
     *   rowgap  -- worst interval between samples of one key row. A press lasts
     *              25-80 ms, so single digits is healthy and 25+ means a press
     *              can begin and end unseen. Read 156-169 ms before the
     *              2026-09-03 publish fix.
     *   stall   -- main-loop gaps >= 25 ms with flash EXCLUDED. Flash windows
     *              are understood and bounded; anything else at that length is
     *              not, and must stay at zero. This page itself put SEVEN on
     *              the counter in its first version. */
    uint8_t  grow = 0;
    uint16_t gap  = health_row_gap_max_ms(&grow);
    p = dbg_append(dbg_u32(v, gap), "ms r");
    dbg_u32(p, grow);
    dbg_row(2, "Rowgap", v);
    /* Two thresholds, and the difference matters. >=25 ms is the LOSS alarm:
     * 25 ms is the shortest keypress, so a stall that long can swallow one
     * whole. >=10 ms cannot lose anything by itself -- the key is still down
     * when the loop catches up -- so it is the LEADING indicator, and a rising
     * count there is the warning that arrives before a keystroke goes missing. */
    p = dbg_append(dbg_u32(dbg_append(v, "25:"), health_count_ge_25ms_nonflash()), " 10:");
    dbg_u32(p, health_count_ge_10ms());
    dbg_row(3, "Stall", v);

    /* HOW BIG, AND WHAT IT WAS. The first question after the alarm fires, and
     * until this row existed the panel could not answer it -- you needed a
     * cable to read loop_gap_max_mark. "flash" is a wear-levelling
     * consolidation: understood and bounded. "blit" during normal use is the
     * LCD, and today that means the Fn+D dismiss (~44 ms, known). Anything
     * else, or a mark you do not recognise, is worth reporting. */
    p = dbg_append(dbg_u32(v, health_loop_gap_max_ms()), "ms ");
    switch (health_loop_gap_max_mark()) {
        case LOOP_MARK_FLASH: dbg_append(p, "flash"); break;
        case LOOP_MARK_BLIT:  dbg_append(p, "blit");  break;
        case LOOP_MARK_I2C:   dbg_append(p, "i2c");   break;
        default:              dbg_append(p, "-");     break;
    }
    dbg_row(4, "Worst", v);

    /* NOT a per-key sampling rate: it counts matrix_scan() calls and the ISR
     * samples ~4 rows per call. rowgap above is the one that bounds loss. */
#ifdef DEBUG_MATRIX_SCAN_RATE
    dbg_u32(v, (uint32_t)get_matrix_scan_rate());
    dbg_row(5, "Scan/s", v);
#else
    dbg_row(5, "Scan/s", "n/a");
#endif

    /* CH582F link, SINCE BOOT -- these live in the CH582 driver, not health.c,
     * so health_reset() cannot clear them. drop is the one that matters: a
     * frame is abandoned only after CH582_TX_MAX_RETRIES consecutive timeouts,
     * so a non-zero drop over BT is a keystroke that never arrived. A high
     * tmout with drop 0 is just the 10 ms ACK deadline sitting at the module's
     * own turnaround -- nothing lost. See docs/wireless.md. */
    uint32_t sent, to, drop;
    ch582_tx_stats(&sent, &to, &drop);
    dbg_u32(v, drop);
    dbg_row(6, "BT Drop", v);
    p = dbg_u32(v, to);
    if (sent) { p = dbg_append(p, " "); p = dbg_u32(p, (to * 100u) / sent); dbg_append(p, "%"); }
    dbg_row(7, "BT T/O", v);   /* "352 49%" */

    /* Three readings, kept apart because they mean different things:
     *   - the module's last `5C` ("--" once it is 20 s stale: the module holds
     *     its last value forever);
     *   - the PACK in mV, from battery.c's trimmed mean of the last 64 reports -- or the
     *     clamp's bound, ">4036" at 100 and "<3161" at 0, never a number;
     *   - VDD, the SN32's rail. On battery that is a buck-boost's output pinned
     *     at ~3.90 V: it says which supply is on, not how full the pack is.
     * As "5C@age pack vdd", all mV: "59@4 3677 3901" is 5C 59 reported 4 s
     * ago, pack 3677 mV, VDD 3901 mV. Widest: "Batt" + "100@19 >4036 4790"
     * = 4 + 17 of 21. */
    uint8_t  batt = ch582_get_battery();
    uint32_t age  = ch582_battery_age_ms();
    uint8_t  med  = battery_5c_rounded();
    uint16_t pk   = battery_pack_mv();
    uint16_t mv   = battery_vdd_mv();
    /* "none" is the presence verdict, not a level: no pack fitted (indicators.c). */
    if (battery_is_absent()) {
        dbg_row(8, "Batt", "none");
    } else {
        if (batt <= 100 && age < 20000u) {
            p = dbg_u32(v, batt);
            p = dbg_append(p, "@");
            p = dbg_u32(p, age / 1000u);
        } else {
            p = dbg_append(v, "--");
        }
        p = dbg_append(p, " ");
        if (!pk) {
            p = dbg_append(p, "--");
        } else {
            if (med == 100u) p = dbg_append(p, ">");
            if (med == 0u)   p = dbg_append(p, "<");
            p = dbg_u32(p, pk);
        }
        if (mv) {
            p = dbg_append(p, " ");
            dbg_u32(p, mv);
        }
        dbg_row(8, "Batt", v);
    }

    /* The camera page draws none of the debug rows: its four lines are its
     * own (cam_compose), and a debug cell left non-blank would paint over them. */
    if (debug_big) {
        for (uint8_t r = 0; r < DBG_ROWS; r++) dbg_row(r, "", "");
        cam_compose();
    }
}

bool display_debug_active(void) {
    return debug_active;
}

void display_debug_toggle(void) {
    /* Refuse while the animation player owns the bus rather than drawing over
     * it: both would keep blitting and the panel would tear between them. */
    if (!debug_active && display_paused) return;
    /* A cycle: dashboard -> debug page -> camera page -> dashboard. The middle
     * step keeps the panel and only changes what is drawn on it. */
    if (debug_active && !debug_big) {
        debug_big        = true;
        debug_clear_band = 1;   /* the same banded clear as on entry */
        /* Blank the debug text NOW, not at the next compose: the clear takes
         * eight passes and the compose runs on the 10 Hz tick, so a stale dirty
         * row could otherwise be painted straight over the digits. */
        for (uint8_t r = 0; r < DBG_ROWS; r++) {
            memset(dbg_text[r], ' ', DBG_COLS);  dbg_text[r][DBG_COLS]  = '\0';
            memset(dbg_shown[r], ' ', DBG_COLS); dbg_shown[r][DBG_COLS] = '\0';
        }
        dbg_dirty = 0;
        cam_reset();   /* the clear leaves the panel blank: a grid of spaces */
        dbg_force_recompose = true;
        return;
    }
    debug_big    = false;
    debug_active = !debug_active;
    if (debug_active) {
        /* Hand the clear to the pump, one band per pass. This is a single call
         * from process_record_kb and cannot span passes, so it must not clear
         * the panel itself -- a full-screen clear here is the same 43 ms stall
         * the exit path was split up to avoid. */
        debug_clear_band = 1;
        /* A cleared panel is BLANK, which is exactly what a space paints. Seed
         * the shadow with spaces rather than NULs so the diff skips every blank
         * cell on the first paint -- about a third of the 189, and the page
         * fills in that much sooner. */
        memset(dbg_shown, ' ', sizeof(dbg_shown));
        for (uint8_t r = 0; r < DBG_ROWS; r++) dbg_shown[r][DBG_COLS] = '\0';
    } else {
        /* Hand the panel back in STAGES, not with one display_redraw_dashboard()
         * call. That function clears the whole panel and then blits the icons
         * and status synchronously: measured 66 ms on the main loop, which
         * count_ge_25ms_nonflash duly flagged as a stall long enough to lose a
         * keypress. Pressing Fn+D to dismiss a DIAGNOSTICS page must not put the
         * next keystroke at risk.
         *
         * Driven one step per main-loop pass from display_blit_pump(), so each
         * step stays well under the 25 ms threshold and the whole restore still
         * completes in a few milliseconds of wall time -- visually instant. */
        debug_exit_step = 1;
    }
}

void display_bootloader_splash(void) {
    /* Force the panel on and to full brightness -- the user may have been
     * running it at the dimmest step, and this is the one screen that must be
     * readable. */
    display_set_power(true);
    display_set_brightness(display_get_brightness_max());
    /* And lift the power ladder's cap, which a dark stage (or Minimal) left at
     * 0: the PWM tick would drive the pin low again on its next pass, and
     * nothing between here and the reset runs power_task() to lift it. */
    bkl_cap = BKL_NO_CAP;
    gpio_write_pin(PANEL_BKL, 1);   // and bypass the software PWM entirely

    display_queue_discard();
    lcd_clear_rect(0, 0, PANEL_WIDTH, PANEL_HEIGHT);

    lcd_draw_flash_text(FONT_STATUS, 14,   2, "BOOTLOADER");
    lcd_draw_flash_text(FONT_SMALL,  13,  33, "Awaiting firmware");
    lcd_draw_flash_text(FONT_SMALL,  13,  55, "Replug exits IF");
    lcd_draw_flash_text(FONT_SMALL,  10,  69, "firmware is intact");
    lcd_draw_flash_text(FONT_SMALL,  22,  91, "USB Device ID:");
    lcd_draw_flash_text(FONT_STATUS, 19, 105, "0C45:7140");

    /* Let the last blit drain before the caller resets the MCU, or the DMA is
     * cut mid-transfer and the panel keeps a half-drawn frame. */
    lcd_blit_wait();
}

void draw_clock(void) {
    clock_gen_painted = rtc_clock_generation();   /* a deep sleep's repaint hold reads it */
    if (pb_active) {
        /* Leaving playback needs the band cleared of the wider/other-sized
         * glyphs before the clock is laid back down; entering needs the
         * clock's cells gone. Both are handled by clock_force_repaint, which
         * display_set_playback() sets on every ownership change. */
        draw_playback();
        clock_force_repaint = false;
        return;
    }
    bool cleared = clock_force_repaint;
    if (cleared) clock_band_clear();
    clock_force_repaint = false;
    bool relayout  = clock_relayout;
    clock_relayout = false;

    /* Off: retire what is up (the run exactly, plus the AM/PM cell) and the
     * per-second path does no work. Playback still takes the band, handled
     * before we get here. */
    if (clock_mode == DISPLAY_CLOCK_OFF) {
        if (relayout) {
            retire_line(GQ_SLOT_CLOCK);
            if (ampm.shown) {
                band_clear(ampm.x, CLOCK_Y, AMPM_W, CLOCK_BAND_H);
                ampm.shown = false;
            }
        }
        return;
    }
    (void)relayout;   /* 24h <-> 12h: nothing to pre-clear; the diff path moves the run */

    rtc_time_t shown;
    bool valid = rtc_get_time(&shown);
    if (!valid) memset(&shown, 0, sizeof(shown));

    /* Date: "Sep 1, 2026" in the 20px face -- the 30px atlas holds only
     * digits and the colon. 12 cells of 10 px at most, in the same 23-row
     * cell the playback timer uses. Redraws only when the string changes,
     * i.e. at midnight; the font change to/from the digits goes through
     * queue_line's moved path (a different face cannot overlap-paint, so
     * that one transition wipes, exactly as playback does). */
    if (clock_mode == DISPLAY_CLOCK_DATE) {
        static const char *const months[12] = {
            "Jan", "Feb", "Mar", "Apr", "May", "Jun",
            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
        };
        char date_str[32];   /* oversized: the compiler cannot prove %u is short */
        if (valid && shown.month >= 1 && shown.month <= 12) {
            snprintf(date_str, sizeof(date_str), "%s %u, %u",
                     months[shown.month - 1], (unsigned)shown.day, (unsigned)shown.year);
        } else {
            snprintf(date_str, sizeof(date_str), "--- --, ----");
        }
        uint8_t  dn  = (uint8_t)strlen(date_str);
        if (dn > 12) dn = 12;
        date_str[dn] = '\0';
        uint16_t dx0 = (uint16_t)((PANEL_WIDTH - dn * FONT_STATUS_ADV) / 2);
        if (ampm.shown) {   /* leaving 12h: the glyph cell is outside any run */
            band_clear(ampm.x, CLOCK_Y, AMPM_W, CLOCK_BAND_H);
            ampm.shown = false;
        }
        if (cleared) shadow_mark_empty(GQ_SLOT_CLOCK, FONT_STATUS, dx0, CLOCK_Y);
        queue_line(GQ_SLOT_CLOCK, FONT_STATUS, dx0, CLOCK_Y, date_str);
        return;
    }

    bool     twelve = (clock_mode == DISPLAY_CLOCK_12H);
    bool     pm     = shown.hours >= 12;
    unsigned hh     = shown.hours;
    if (twelve) {
        hh %= 12;
        if (hh == 0) hh = 12;
    }

    char time_str[12];
#if DISPLAY_CLOCK_SHOW_SECONDS
    if (twelve) snprintf(time_str, sizeof(time_str), "%u:%02u:%02u",
                         hh, (unsigned)shown.minutes, (unsigned)shown.seconds);
    else        snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u",
                         hh, (unsigned)shown.minutes, (unsigned)shown.seconds);
#else
    if (twelve) snprintf(time_str, sizeof(time_str), "%u:%02u", hh, (unsigned)shown.minutes);
    else        snprintf(time_str, sizeof(time_str), "%02u:%02u", hh, (unsigned)shown.minutes);
#endif
    /* Through the glyph queue (audit BW-3): queue_line diffs against the
     * band's shadow, so the per-second case queues 1-2 glyphs exactly as the
     * old last_time[] compare did, and a force repaint (band_clear above
     * invalidated the shadow) queues all 8 -- painted by the pump at main-
     * loop rate instead of ~16 ms of blocking waits. */
    uint16_t adv = lcd_font_advance(FONT_CLOCK);
    if (!adv) return;
    uint8_t  n  = (uint8_t)strlen(time_str);   // 8/7 (with seconds) or 5/4
    uint16_t w  = (uint16_t)(n * adv + (twelve ? AMPM_W : 0));
    uint16_t x0 = (uint16_t)((PANEL_WIDTH - w) / 2);
    if (twelve) x0 = (x0 > AMPM_X_SHIFT) ? (uint16_t)(x0 - AMPM_X_SHIFT) : 0;
    uint16_t ax = (uint16_t)(x0 + n * adv);   // AM/PM cell origin: the run's end

    /* Retire a stale AM/PM glyph BEFORE the run is re-laid: when the run
     * grows from 7 to 8 cells the old glyph's columns fall inside the new
     * run, and the queue paints those cells asynchronously -- clearing after
     * the fact would erase a freshly painted digit. This clear does not
     * touch the run's shadow (half-open overlap, see above), so the per-cell
     * diff below still holds and the queue handles the moved run itself.
     * A letter flip (noon, midnight) needs no clear: the tile paints its
     * own background. */
    if (ampm.shown && (!twelve || ampm.x != ax)) {
        band_clear(ampm.x, CLOCK_Y, AMPM_W, CLOCK_BAND_H);
        ampm.shown = false;
    }
    /* After a whole-band clear the slot is KNOWN EMPTY at this layout; saying
     * so spares queue_line's unknown-path full-width clear (~8 ms). */
    if (cleared) shadow_mark_empty(GQ_SLOT_CLOCK, FONT_CLOCK, x0, CLOCK_Y);
    queue_line(GQ_SLOT_CLOCK, FONT_CLOCK, x0, CLOCK_Y, time_str);
    if (twelve && !ampm.shown) {
        /* The cell may hold the tail of the run's OLD layout (24h -> 12h
         * shifts the digits left, off the glyph's columns). Clear it AFTER
         * queue_line: the new run does not cover these columns, so this
         * cannot invalidate the fresh shadow -- before it, it would overlap
         * the old run and force the full-width unknown clear (the wipe). */
        band_clear(ax, CLOCK_Y, AMPM_W, CLOCK_BAND_H);
        draw_ampm(ax, CLOCK_Y, pm);
        ampm.shown = true;
        ampm.x     = ax;
        ampm.pm    = pm;
    } else if (twelve && ampm.pm != pm) {
        draw_ampm(ax, CLOCK_Y, pm);    /* the tile paints its own background */
        ampm.pm = pm;
    }
}

/* One stage of display_redraw_dashboard(), for the Fn+D exit path. Returns true
 * while more stages remain.
 *
 * Same work, same order, split so no single main-loop pass carries all of it.
 * The animation-resume path still calls display_redraw_dashboard() whole: it
 * exits on a timer rather than a keypress, so nobody is mid-word when it fires,
 * and splitting a shared path used by the flash player was not worth the risk
 * for that caller. If it ever shows up in blit_gap_max_ms, route it here too. */
static bool debug_restore_step(void) {
    uint8_t step = debug_exit_step;

    /* Anything queued belongs to the pre-clear frame; painting it afterwards
     * would scatter stale glyphs over the fresh dashboard. */
    if (step == 1) { display_queue_discard(); locks_stage_first = true; }

    if (step <= DBG_CLEAR_BANDS) {
        lcd_clear_rect(0, (uint16_t)((step - 1) * DBG_CLEAR_BAND_H),
                       PANEL_WIDTH, DBG_CLEAR_BAND_H);
    } else {
        switch (step - DBG_CLEAR_BANDS) {
            case 1:
                splash_cleared = true;
                lcd_draw_flash_image(mac_mode ? ASSET_APPLE_ICON_24X24 : ASSET_WINDOWS_ICON_24X24, 0, 0);
                break;
            case 2:
                draw_conn_row();
                clock_force_repaint = true;   /* draw_clock queues; cheap here */
                draw_clock();
                break;
            case 3: draw_conn_number(true); break;
            case 4: draw_battery(true);     break;
            case 5:
                /* Lock band, ONE component per pass. A label is up to four
                 * synchronous glyphs (~10 ms), and one draw_locks(true) with
                 * CAPS, WIN and FN all lit was 25-30 ms -- the last stall on
                 * the board long enough to lose a keypress (measured
                 * 2026-09-03: `worst 30ms blit` on every Fn+D exit). Stays on
                 * this stage until the band matches the lock state. */
                if (draw_locks_staged(locks_stage_first)) {
                    locks_stage_first = false;
                    return true;             /* painted one; the rest next pass */
                }
                break;                       /* nothing left: advance */
            default: draw_text_slot(true);  break;
        }
    }

    if (++debug_exit_step > DBG_CLEAR_BANDS + 6) { debug_exit_step = 0; return false; }
    return true;
}

uint32_t display_redraw_dashboard(uint32_t trigger_time, void *cb_arg) {
    splash_cleared = true;

    /* Anything still queued belongs to the pre-clear frame -- painting it
     * after this clear would scatter stale glyphs over the fresh dashboard
     * (the resume-from-animation path made that reachable). */
    display_queue_discard();

    // Clear background.
    lcd_clear_rect(0, 0, PANEL_WIDTH, PANEL_HEIGHT);

    // Full repaint: force the clock and date to redraw over the cleared screen.
    clock_force_repaint = true;

    // Mac/Windows icon (top-left).
    lcd_draw_flash_image(mac_mode ? ASSET_APPLE_ICON_24X24 : ASSET_WINDOWS_ICON_24X24, 0, 0);

    // Connection icon.
    draw_conn_row();

    draw_clock();
    draw_status(true);   // battery + channel digit over the cleared screen

    return 0; // one-shot
}

bool display_init_kb(void) {
    lcd_init();          // GC9107 bring-up, rotation 270

    // Splash logo, held until the deferred dashboard repaint below.
    lcd_clear_rect(0, 0, PANEL_WIDTH, PANEL_HEIGHT);
    // All art lives in external flash now, so nothing can be drawn until the
    // index is read. An unprovisioned keyboard therefore shows a BLANK panel and
    // says so on the console -- there is no embedded fallback left to draw with,
    // which is the whole point (it reclaimed ~32KB of firmware). Provision with:
    //   ak820ctl flash write 0x0CE0000 graphics/res/flash_assets.bin
    if (flash_assets_init()) {
        dprintf("[assets] index ok, %u entries\n", flash_assets_count());
        lcd_draw_flash_image(ASSET_SONIXQMK, 0, 0);
    } else {
        dprintf("[assets] NO VALID INDEX at 0x%06lX -- panel stays blank.\n"
                "[assets] provision with: ak820ctl flash write 0x%06lX flash_assets.bin\n",
                (unsigned long)FLASH_ASSET_BASE, (unsigned long)FLASH_ASSET_BASE);
    }

    display_backlight_init();

    /* Stored clock format, else the compile-time default (24h). Raw
     * assignment: the dashboard has not been drawn yet, so there is nothing
     * to relayout. */
    uint8_t cm;
    if (kb_eeconfig_get_clock_mode(&cm)) clock_mode = cm;

    bool res = display_init_user();
    if (res)
        defer_exec(1500, display_redraw_dashboard, NULL);

    return true;
}

__attribute__((weak)) bool display_init_user(void) {
    return true;
}

__attribute__((weak)) bool display_housekeeping_task_user(void) {
    return true;
}

#include "bluetooth/ch582f_ajazz.h"   // connection state enum + getters
extern uint8_t ch582_get_battery(void);

// General blink phase: true during the first half of each period_ms cycle, so any
// element can gate its visibility on it. period_ms == 0 means always on (solid).
bool display_blink(uint16_t period_ms) {
    if (!period_ms) return true;
    return (timer_read32() % period_ms) < (period_ms / 2u);
}

// Connection-digit blink rates.
#define CONN_BLINK_PAIRING_MS 200   // pairing: fast
#define CONN_BLINK_LINKING_MS 700   // linking/reconnecting: slow
/* Failed: a lazy pulse that reads as DORMANT rather than busy. The slot is still
 * the one you selected, so blanking the digit threw away real information -- but
 * solid already means "connected", so it cannot be reused here. Confirmed on
 * hardware: a connect to an unreachable slot gives `5B 33 33 36`, and the
 * following `5B 23` (idle) is ignored by the parser, so REJECTED persists
 * indefinitely and the digit stayed gone for good. */
#define CONN_BLINK_FAILED_MS 2000   // failed: very slow, "stopped" not "trying"

/* Connection strip: all three transports shown at once, ordered LEFT-TO-RIGHT TO
 * MATCH THE PHYSICAL SWITCH (2.4G / wired / BT as seen from the top of the
 * board), with the inactive two dimmed. You read it the way you read the switch
 * under your fingers instead of decoding a single glyph.
 *
 * It is ONE 76x24 asset per state, not three 24x24 icons composited at runtime,
 * because flash assets are RGB565 with colour baked in -- lcd_draw_flash_image()
 * has no tint parameter, so "dim" has to be authored, not computed. Baking whole
 * rows also means one blit and no alignment arithmetic. Generated by the snippet
 * in assets/ from the original three icons.
 *
 * This replaced the date, which was redundant next to a computer clock. */
#define CONN_ICON_X   29   /* 5px clear of the OS logo: it shows a different kind of thing */
#define CONN_ICON_W   79   /* 3x24 icons + gaps of 3 and 4 (BT nudged 1px right for optical balance) */
#define CONN_ICON_H   24   /* was implicit in CONN_ICON_W while the icon was square */
#define CONN_NUM_X    (CONN_ICON_X + CONN_ICON_W + 1)
#define CONN_NUM_W    12

// Paints the whole three-transport strip for the current mode.
static void draw_conn_row(void) {
    uint16_t id = (connection_mode == CONN_MODE_BLUETOOTH) ? ASSET_CONN_ROW_BT
                : (connection_mode == CONN_MODE_2_4G)      ? ASSET_CONN_ROW_24G
                                                           : ASSET_CONN_ROW_CABLE;
    lcd_draw_flash_image(id, CONN_ICON_X, 0);
}

// Channel digit next to the connection icon, blinking to show link state:
//   connected -> solid digit;  linking/reconnecting -> slow blink;
//   pairing -> fast blink;  failed -> very slow pulse;  idle / USB -> no digit.
// Driven every housekeeping tick (~10 Hz) so the blink animates; the redraw is
// self-guarded so it only touches the panel on a visible transition.
static void draw_conn_number(bool force) {
    uint8_t slot  = ch582_get_target_slot();               // 1-3, or 0
    char    digit = (slot >= 1 && slot <= 3) ? (char)('0' + slot) : 0;

    /* No digit in 2.4G: ch582_get_target_slot() reports 1 there so the rest of
     * the driver has something to work with, but there is exactly ONE dongle and
     * no slot concept at all -- a "1" implies "slot 1 of several" and is worse
     * than nothing. State is not lost: the icon strip shows the radio, and the
     * status band spells out every non-connected state in words. */
    if (connection_mode == CONN_MODE_2_4G) digit = 0;

    char     c      = 0;   // the digit this state wants to show (0 = none)
    uint16_t period = 0;   // blink period; 0 = solid
    switch (ch582_get_conn_state()) {
        case CH582_CONN_CONNECTED: c = digit; period = 0;                     break;
        case CH582_CONN_PAIRING:   c = digit; period = CONN_BLINK_PAIRING_MS; break;
        case CH582_CONN_LINKING:   c = digit; period = CONN_BLINK_LINKING_MS; break;
        case CH582_CONN_REJECTED:  c = digit; period = CONN_BLINK_FAILED_MS;  break;
        default:                   c = 0;                                     break;
    }
    char shown = (c && display_blink(period)) ? c : 0;     // current blink-phase visibility

    static char last_shown = -1; // force the first paint
    if (!force && shown == last_shown) return;
    last_shown = shown;

    lcd_clear_rect(CONN_NUM_X, 0, CONN_NUM_W + 1, CONN_ICON_H);
    if (shown) {
        char s[2] = {shown, 0};
        lcd_draw_flash_text(FONT_STATUS, CONN_NUM_X, 2, s);
    }
}

// Battery icon, bottom-left. Body + terminal nub, with a proportional fill.
// Geometry is inclusive coordinates (lcd_fill_rect takes x0,y0,x1,y1).
#define BATT_X0    5   /* was 2: the LCD is recessed, so the bezel clips the outermost pixels */
/* +4, not +3: rows 126-127 were dead space below the icon, which read as the
 * battery row floating. LCD_OFF_Y is a controller offset, not lost rows, so the
 * full 0..127 is visible. One row of bottom margin is kept deliberately -- the
 * LCD is recessed and the bezel clips the outermost pixels, which is the same
 * reason BATT_X0 is 5 rather than 0. */
#define BATT_Y0    (STATUS_Y + 5)   /* icon 110..121, centred on the 20px digits' ink (109..123) */
#define BATT_W     24
#define BATT_H     12
#define BATT_X1    (BATT_X0 + BATT_W - 1)
#define BATT_Y1    (BATT_Y0 + BATT_H - 1)
#define BATT_NUB_W 2
#define BATT_NUB_INSET 4          // nub is shorter than the body, top and bottom
// Inner (fillable) area, one pixel inside the 1px outline.
#define BATT_IN_X0 (BATT_X0 + 2)
#define BATT_IN_Y0 (BATT_Y0 + 2)
#define BATT_IN_X1 (BATT_X1 - 2)
#define BATT_IN_Y1 (BATT_Y1 - 2)
#define BATT_IN_W  (BATT_IN_X1 - BATT_IN_X0 + 1)

#define COL_BATT_OK    0x07E0   // green
#define COL_BATT_WARN  0xFFE0   // amber
#define COL_BATT_LOW   0xF800   // red
#define COL_BOLT       0xFFE0   // yellow, matching the padlock

extern bool charge_is_charging(void);   // ak820pro.c

/* 9x14 charging bolt, sat just right of the battery so the two read as one
 * unit. Drawn from horizontal runs of a rasterised polygon -- a hand-placed
 * zigzag of rectangles came out looking like the digit "4"; the diagonals only
 * read as a bolt once they step a pixel at a time.
 *
 * Colour already signals charging (the icon goes cyan), but that is a subtle cue
 * if you do not know the code, and the gap between icon and percentage was dead
 * space anyway. */
#define BOLT_X  (BATT_X0 + BATT_W + BATT_NUB_W + 4)
#define BOLT_Y  (BATT_Y0 - 1)

/* The bolt's horizontal runs, {x0, y0, x1, y1} from the slot's origin. */
static const uint8_t bolt_runs[][4] = {
    {6, 0, 6, 0}, {5, 1, 6, 2}, {4, 3, 5, 4}, {3, 5, 5, 5}, {3, 6, 8, 6}, {2, 7, 7, 7},
    {4, 8, 7, 8}, {4, 9, 6, 9}, {4, 10, 5, 10}, {3, 11, 4, 12}, {3, 13, 3, 13},
};

// Bottom row: battery icon (left) + percentage (right-aligned).
/* Bolt footprint: bolt_runs paint x+2..x+8, y+0..y+13. */
#define BOLT_W    9
#define BOLT_H    14

/* 9x9 "no battery" cross, drawn in the bolt's slot -- the two are mutually
 * exclusive by construction (a board with no pack can never be charging), so
 * they can share the rect without either needing to know about the other.
 * Two-pixel strokes, clamped at the edges: the bolt's diagonals needed the same
 * weight before they read as anything at arm's length. */
#define CROSS_WH 12
/* Geometric centring puts it a pixel high against the battery outline beside it,
 * because the outline's visual mass sits low of its own bbox (the nub is inset).
 * Nudged by eye on hardware rather than by arithmetic. */
#define CROSS_Y_NUDGE 1
#define CROSS_Y_OFF (((BOLT_H - CROSS_WH) / 2) + CROSS_Y_NUDGE)

/* The power mode's bulb (plans/POWER-MODE-ICONS-PLAN.md), in the same slot on
 * battery, where the bolt and the cross never show; on USB every mode is
 * Always on, and the slot is theirs. One bulb losing its light: lit with
 * rays (Always on), lit (Balanced), dimmed (Power saver), unlit (Lights out).
 * '#' is the outline or the lit glass, '*' the dimmed glass, '+' the screw
 * base. Row 0 is the slot's top, so the rays reach it and the bulb sits on
 * the battery's bottom row. */
#define BULB_W 13
#define BULB_H 14
static const char bulb_art[POWER_MODE_COUNT][BULB_H][BULB_W + 1] = {
    [POWER_MODE_FULL] = {
        "......#......", ".#.........#.", "..#.......#..", ".....###.....",
        "...#######...", "##.#######.##", "..#########..", "..#########..",
        "..#########..", "...#######...", "....#####....", ".....+++.....",
        ".....+++.....", "......+......",
    },
    [POWER_MODE_NORMAL] = {
        ".............", ".............", ".............", ".....###.....",
        "...#######...", "...#######...", "..#########..", "..#########..",
        "..#########..", "...#######...", "....#####....", ".....+++.....",
        ".....+++.....", "......+......",
    },
    [POWER_MODE_SAVER] = {
        ".............", ".............", ".............", ".....###.....",
        "...##***##...", "...#*****#...", "..#*******#..", "..#*******#..",
        "..#*******#..", "...#*****#...", "....#***#....", ".....+++.....",
        ".....+++.....", "......+......",
    },
    [POWER_MODE_MINIMAL] = {
        ".............", ".............", ".............", ".....###.....",
        "...##...##...", "...#.....#...", "..#.......#..", "..#.......#..",
        "..#.......#..", "...#.....#...", "....#...#....", ".....+++.....",
        ".....+++.....", "......+......",
    },
};
/* Starting values, to judge on the panel: mid grey (0x8410) was
 * near-illegible as 5x7 text here (COL_AMPM). Saver's glass is a filled area
 * and may survive it; try 0xA514 beside it. Lights out's outline is a 1 px
 * stroke, so it is lighter than the glass, and the missing fill is what tells
 * it from Saver. Never yellow (the bolt's, in the bolt's place) or red and
 * amber (a low battery, on this row). */
#define COL_BULB_BASE     0x8C71   /* #8C8C8C: the lit bulbs' screw base */
#define COL_BULB_DIM      0x8410   /* #808080: Saver's glass */
#define COL_BULB_DIM_BASE 0x4228   /* #464646: darker, so the grey glass does not run into it */
#define COL_BULB_OFF      0xA514   /* #A0A0A0: Lights out's outline */
/* Per mode: '#', '*', '+'. */
static const uint16_t bulb_cols[POWER_MODE_COUNT][3] = {
    [POWER_MODE_FULL]    = {COL_FG, 0, COL_BULB_BASE},
    [POWER_MODE_NORMAL]  = {COL_FG, 0, COL_BULB_BASE},
    [POWER_MODE_SAVER]   = {COL_FG, COL_BULB_DIM, COL_BULB_DIM_BASE},
    [POWER_MODE_MINIMAL] = {COL_BULB_OFF, 0, COL_BULB_DIM_BASE},
};
#define SLOT_NO_BULB 0xFFu

/* The slot is the union of the three: the bulb's rays make it 13 wide and
 * start it a row above the bolt, which moves down a row inside it to stay
 * where it was. Clearing less than the widest would leave the last columns of
 * whatever is taken down. Right edge x 47, so the row's text (at most 7
 * cells, from x 54: draw_battery's txt[8]) clears it. */
#define ICON_SLOT_W 13
#define ICON_SLOT_H 15
#define SLOT_Y      (BOLT_Y - 1)
#define BOLT_DY     (BOLT_Y - SLOT_Y)
_Static_assert(ICON_SLOT_W >= CROSS_WH && ICON_SLOT_W >= BOLT_W && ICON_SLOT_W >= BULB_W, "slot width");
_Static_assert(ICON_SLOT_H >= BOLT_H + BOLT_DY && ICON_SLOT_H >= BULB_H, "slot height");

static void slot_run(uint16_t *t, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t col) {
    for (uint8_t y = y0; y <= y1 && y < ICON_SLOT_H; y++)
        for (uint8_t x = x0; x <= x1 && x < ICON_SLOT_W; x++) t[y * ICON_SLOT_W + x] = col;
}

/* The bolt slot -- the bolt, the cross, the mode's bulb, or blank -- as ONE
 * RAM tile in ONE transfer (flash 2c). It was a clear and 11 lcd_fill_rect
 * runs for the bolt, 24 for the cross, each paying an 11-byte window command
 * sent byte by byte: 19 ms at every plug-in, measured on flash 2b and 2c alike
 * (the plug-in snapshots of 2026-10-11), the lesson draw_ampm learned first.
 * The tile paints its own background, so no clear goes first, and it is
 * always the whole slot, so none leaves pixels under another. The bulb is
 * rasterized from its art into the same tile: no runs, no second transfer.
 * bulb is a power_mode_t, or SLOT_NO_BULB. */
static void draw_bolt_slot(bool absent, bool chrg, uint8_t bulb) {
    static uint16_t tile[ICON_SLOT_W * ICON_SLOT_H];
    memset(tile, 0, sizeof(tile));                      /* black background */
    if (absent) {
        /* Two-pixel strokes, clamped at the edges, CROSS_Y_OFF down. */
        for (uint8_t i = 0; i < CROSS_WH; i++) {
            uint8_t y = (uint8_t)(BOLT_DY + CROSS_Y_OFF + i);
            uint8_t a = i;                              /* the "\" stroke */
            uint8_t b = (uint8_t)(CROSS_WH - 1 - i);    /* the "/" stroke  */
            slot_run(tile, a, y, (a < CROSS_WH - 1) ? (uint8_t)(a + 1) : a, y, COL_BATT_LOW);
            slot_run(tile, (b > 0) ? (uint8_t)(b - 1) : b, y, b, y, COL_BATT_LOW);
        }
    } else if (chrg) {
        for (uint8_t i = 0; i < sizeof(bolt_runs) / sizeof(bolt_runs[0]); i++)
            slot_run(tile, bolt_runs[i][0], (uint8_t)(bolt_runs[i][1] + BOLT_DY), bolt_runs[i][2],
                     (uint8_t)(bolt_runs[i][3] + BOLT_DY), COL_BOLT);
    } else if (bulb < POWER_MODE_COUNT) {
        for (uint8_t y = 0; y < BULB_H; y++)
            for (uint8_t x = 0; x < BULB_W; x++) {
                char c = bulb_art[bulb][y][x];
                if (c != '.') tile[y * ICON_SLOT_W + x] = bulb_cols[bulb][c == '#' ? 0 : c == '*' ? 1 : 2];
            }
    }
    /* CPU pushes share SPI0 with the DMA blits. lcd_blit_ram quiesces the bus
     * itself (bus_quiesce); the explicit wait is draw_ampm's pattern, and free
     * when the bus is idle. */
    lcd_blit_wait();
    lcd_blit_ram(tile, BOLT_X, SLOT_Y, ICON_SLOT_W, ICON_SLOT_H);
}
#define BATT_IN_H (BATT_IN_Y1 - BATT_IN_Y0 + 1)

/* draw_battery()'s pending components, one painted per call (flash 2c). */
#define BATT_DIRTY_OUTLINE 0x01u
#define BATT_DIRTY_BOLT    0x02u
#define BATT_DIRTY_TEXT    0x04u
#define BATT_DIRTY_BAR     0x08u
#define BATT_DIRTY_ALL     0x0Fu
static uint8_t batt_dirty = 0;
static uint8_t txt_done   = 0;       /* glyphs of the current text painted */
static bool    txt_placed = false;   /* its x is set and its vacated cells cleared */
/* At most ONE component per main-loop pass. display_housekeeping_task() runs
 * from the RTC second edge AND the 10 Hz tick, and the deferred dashboard
 * redraw (draw_status) runs before both, so two or three can land in one pass
 * (codex, flash 2c finding 1 and its verification). display_pass_end(), the
 * last thing housekeeping_task_kb() does, clears this once a pass. */
static bool    batt_painted_this_pass = false;
#define BATT_TXT_CHUNK   3           /* glyphs per call: 10x23 is ~2.5 ms each */
static uint8_t batt_last_bulb = SLOT_NO_BULB;   /* the mode's bulb draw_battery() last wanted */

/* A bulb is up and the mode it shows has changed (Fn+B, or the low cap): the
 * 10 Hz tick calls draw_battery() for it, so the bulb follows Fn+B's overlay
 * rather than the next RTC second (codex, power-icons review, note 3). Only a
 * change of the lights' mode triggers it: Fn+B, or the low cap set or
 * cleared, which a plug-in at the cap does once (codex's verify). CHRG alone
 * never moves that mode, so an unsmoothed CHRG cannot repaint the slot faster
 * than before; a call made for a mode change samples it as any call does. */
static bool batt_bulb_stale(void) {
    return batt_last_bulb != SLOT_NO_BULB && (uint8_t)power_lights_mode() != batt_last_bulb;
}

static void draw_battery(bool force) {
    static bool     last_chrg  = false;
    static uint16_t last_fw    = 0;            /* fill bar width on the panel, px */
    static uint16_t last_col   = 0;            /* ...and its colour               */
    static uint16_t last_txt_x = PANEL_WIDTH;  /* where the percent text starts   */
    static uint16_t last_txt_w = 0;            /* ...and how wide it is            */
    static bool     last_absent = false;       /* no pack fitted: cross, no number */
    static char     last_txt[8] = "\x01";      /* what the text slot says ("\x01": never drawn) */

    /* The level is battery.c's, from the CH582F's `5C` -- a voltmeter on the
     * pack -- in whole percent (tenths with BATTERY_SHOW_TENTHS). Never VDD, which this row used to show: on battery
     * it is a buck-boost's output, pinned at 3.90 V, as informative as the
     * stock firmware's hardcoded 100. "Low" at the bottom rather than a "0%"
     * that reads as dead while the board still runs. When nothing is known
     * yet the bar stays down and the slot says what is known instead. */
    battery_state_t st   = battery_state();
    uint8_t         batt = battery_level_pct();   /* 0xFF unknown */
    char            txt[8];
#ifdef BATTERY_SHOW_TENTHS
    uint16_t        pm   = battery_level_permille();
#endif
    if (batt == 0 && st == BATTERY_STATE_BATTERY) snprintf(txt, sizeof(txt), "Low");
#ifdef BATTERY_SHOW_TENTHS
    /* Calibration view: "85.9%". 100% stays whole. 6 glyphs at most is 60 px
     * of the slot's 78, and a redraw is one text blit per 0.1% change. */
    else if (batt <= 100 && pm < 1000u)           snprintf(txt, sizeof(txt), "%u.%u%%", pm / 10u, pm % 10u);
#endif
    else if (batt <= 100) {
        /* Hours of USE beside the level once they matter: under 10 h on
         * battery, with a mature average (plans/DEEP-SLEEP-BUILD-PLAN.md,
         * section 7): "18% 6h", "99% <1h", at most 7 cells. Rounded down. */
        uint16_t left = battery_time_left_min();
        if (st == BATTERY_STATE_BATTERY && left < 600u && battery_avg_minutes() >= 60u) {
            char wide[16];   /* the compiler cannot prove 7 fit: composed wide, then copied */
            if (left < 60u) snprintf(wide, sizeof(wide), "%u%% <1h", batt);
            else            snprintf(wide, sizeof(wide), "%u%% %uh", batt, (unsigned)(left / 60u));
            memcpy(txt, wide, sizeof(txt) - 1u);
            txt[sizeof(txt) - 1u] = '\0';
        } else {
            snprintf(txt, sizeof(txt), "%u%%", batt);
        }
    }
    /* "Charge", not "Charging": at 10 px a cell that is 80 px against the 76
     * between the icon slot and the bezel margin, and it would start inside
     * the slot that a bolt toggle clears. */
    else if (st == BATTERY_STATE_CHARGING)        snprintf(txt, sizeof(txt), "Charge");
    else if (st == BATTERY_STATE_USB || st == BATTERY_STATE_FULL) snprintf(txt, sizeof(txt), "USB");
    else txt[0] = '\0';
    bool    chrg   = charge_is_charging();
    /* Latches false-to-true at most once per boot (indicators.c), so this costs
     * one repaint, not a flicker. */
    bool    absent = battery_is_absent();
    /* With no pack fitted the charger's state is meaningless, and on a charger
     * that cycles into an empty output it TOGGLES about once a second. Left
     * live it would retrigger this whole function -- repainting the cross and
     * re-blitting the 7-glyph label, ~11 ms of synchronous LCD work -- every
     * cycle, forever, for no change on the panel. Freeze it. */
    if (absent) chrg = false;

    // Charging state changes the bolt, so it has to retrigger a redraw as well
    // -- the level alone is not enough. Same for presence: it owns the same rect
    // as the bolt and suppresses the number.
    if (absent) snprintf(txt, sizeof(txt), "No Batt");
    bool same_txt = strcmp(txt, last_txt) == 0;

    /* ONE component per call (flash 2c). A plug-in changes the bolt, the text
     * and the bar at once, and painting all three in one pass made a 27 ms
     * pass, 19 ms of it here: over the 25 ms line where a keypress can be lost
     * (the plug-in snapshot of 2026-10-11; flash 2's plug-ins were 33 ms). The
     * bolt was a clear and 11 synchronous rects (one RAM tile since, which was
     * the real fix: draw_bolt_slot), the text one synchronous glyph each
     * (lcd_draw_flash_text_staged() never existed; see draw_debug_page),
     * and the bar a clear and a rect. What changed is marked in batt_dirty, and
     * each call paints the first mark: the outline (after a clear), the bolt,
     * the text, then the bar. The housekeeping tick calls back while marks
     * remain, so a plug-in settles in three ticks, ~300 ms. The Fn+D restore's
     * battery stage and draw_status() now paint only the outline themselves. */
    if (force) {
        /* The callers (draw_status, the Fn+D restore) have just cleared the
         * panel: nothing of ours is on it. */
        last_fw = 0; last_txt_x = PANEL_WIDTH; last_txt_w = 0;
        txt_done = 0; txt_placed = false;
        batt_dirty = BATT_DIRTY_ALL;
    }
    /* The bar the panel should show. Marked by the fill it would paint, not by
     * the level: at 0% the supply alone moves "Low"'s red sliver (codex, flash
     * 2c finding 3), and a level step inside one column needs no paint. */
    uint16_t want_fw = 0, want_col = 0;
    if (batt <= 100 && !absent) {
        // Proportional fill. Round up so any nonzero charge shows at least one
        // column -- an empty-looking icon at 3% would read as "dead".
        want_fw = (uint16_t)(((uint32_t)batt * BATT_IN_W + 99u) / 100u);
        if (want_fw > BATT_IN_W) want_fw = BATT_IN_W;
        /* "Low" keeps a red sliver: the text glyphs cannot be recoloured, and
         * an empty white outline beside it would not read as a warning. */
        if (want_fw == 0 && batt == 0 && st == BATTERY_STATE_BATTERY) want_fw = 2;
        want_col = (batt <= 20) ? COL_BATT_LOW
                 : (batt <= 50) ? COL_BATT_WARN
                                : COL_BATT_OK;
    }
    /* The mode's bulb: on battery only, where the ladder runs; on USB every
     * mode is Always on and the slot is the bolt's, or blank. Saver's while
     * the low cap holds the lights to its stages (power_lights_mode). */
    uint8_t bulb = (!absent && !chrg && battery_on_battery()) ? (uint8_t)power_lights_mode() : SLOT_NO_BULB;
    /* Charging state changes the bolt, and presence and the bulb own the same
     * rect: one mark, one tile, one transfer, whichever changed. */
    if (chrg != last_chrg || absent != last_absent || bulb != batt_last_bulb) batt_dirty |= BATT_DIRTY_BOLT;
    if (!same_txt) { batt_dirty |= BATT_DIRTY_TEXT; txt_done = 0; txt_placed = false; }
    if (want_fw != last_fw || (want_fw && want_col != last_col)) batt_dirty |= BATT_DIRTY_BAR;
    last_chrg      = chrg;
    last_absent    = absent;
    batt_last_bulb = bulb;
    memcpy(last_txt, txt, sizeof(last_txt));
    if (!batt_dirty || batt_painted_this_pass) return;
    batt_painted_this_pass = true;

    /* Repaint only what moved. This used to clear the whole 128x22 strip -- a
     * 5.6 KB DMA, ~7.6 ms with the main loop parked -- on every percent tick
     * and then redraw all of it, which with the four synchronous glyphs on top
     * came to ~20 ms: three milliseconds under the line where a keypress can
     * be lost (docs/display.md). None of that clearing was needed. The outline
     * never changes, a glyph blit overpaints its whole cell, and the bar and
     * the bolt have known rects -- so the outline is painted once, after a
     * forced repaint's clear, and everything else clears exactly the pixels it
     * vacates. */
    if (batt_dirty & BATT_DIRTY_OUTLINE) {
        batt_dirty &= (uint8_t)~BATT_DIRTY_OUTLINE;
        uint16_t outline = COL_FG;
        // Body outline: four 1px edges rather than a filled rect, so the
        // interior stays background and the fill below can be drawn
        // independently.
        lcd_fill_rect(BATT_X0, BATT_Y0, BATT_X1, BATT_Y0, outline);   // top
        lcd_fill_rect(BATT_X0, BATT_Y1, BATT_X1, BATT_Y1, outline);   // bottom
        lcd_fill_rect(BATT_X0, BATT_Y0, BATT_X0, BATT_Y1, outline);   // left
        lcd_fill_rect(BATT_X1, BATT_Y0, BATT_X1, BATT_Y1, outline);   // right
        // Terminal nub on the right.
        lcd_fill_rect(BATT_X1 + 1, BATT_Y0 + BATT_NUB_INSET,
                      BATT_X1 + BATT_NUB_W, BATT_Y1 - BATT_NUB_INSET, outline);
        return;
    }

    /* Outline stays white and the fill stays level-coloured even while
     * charging: the bolt carries that signal, so the two are orthogonal. The
     * earlier version recoloured the whole icon cyan, which hid the level
     * exactly when "15% and charging" is the most useful thing to know. */
    if (batt_dirty & BATT_DIRTY_BOLT) {
        /* One rect, four kinds of state, one transfer (draw_bolt_slot). */
        draw_bolt_slot(absent, chrg, bulb);
        batt_dirty &= (uint8_t)~BATT_DIRTY_BOLT;
        return;
    }

    /* The text slot: the level, "Low", a state word while the level is
     * unknown, or -- with no pack -- "No Batt", which at 10px/cell is 70px
     * against the 76px between the icon slot and the bezel margin.
     *
     * Right-aligned, so a SHORTER string starts further right and would leave
     * its old leftmost cells behind ("100%" -> "99%", and "No Batt" -> "5%").
     * Clear exactly what is vacated; a longer string overpaints every old cell
     * itself. 20px face: the clock crop freed the rows the small one was forced
     * into. Cell 23 at STATUS_Y = 104..126, ink 108..122, row 127 margin.
     * PANEL_WIDTH - 4 and not - 1, because viewed from the right the bezel
     * hides the last couple of columns -- which was clipping the '%'.
     *
     * Painted BATT_TXT_CHUNK glyphs a call, since one glyph is ~2.5 ms of
     * synchronous LCD work and "No Batt" whole was ~17.5 ms (codex, flash 2c
     * finding 2). The font is monospaced (lcd_flash_text_width), so glyph i
     * sits at x + i * w / n. A text that changes mid-way restarts at its own
     * x: everything left of it is cleared, and the rest is overpainted. */
    if (batt_dirty & BATT_DIRTY_TEXT) {
        uint8_t n = (uint8_t)strlen(txt);
        if (!n) {
            if (last_txt_w) {
                lcd_clear_rect(last_txt_x, STATUS_Y, last_txt_w, lcd_font_height(FONT_STATUS));
                last_txt_x = PANEL_WIDTH; last_txt_w = 0;
            }
            txt_placed = false;
            batt_dirty &= (uint8_t)~BATT_DIRTY_TEXT;
            return;
        }
        if (!txt_placed) {
            uint16_t w     = lcd_flash_text_width(FONT_STATUS, txt);
            uint16_t x     = (uint16_t)(PANEL_WIDTH - 4 - w);
            bool     clear = x > last_txt_x;
            if (clear)
                lcd_clear_rect(last_txt_x, STATUS_Y, (uint16_t)(x - last_txt_x), lcd_font_height(FONT_STATUS));
            last_txt_x = x;
            last_txt_w = w;
            txt_placed = true;
            /* A shortening text's clear is this call's work; its glyphs start
             * on the next (codex, flash 2c verification: clear plus three
             * glyphs in one pass). */
            if (clear) return;
        }
        char    chunk[BATT_TXT_CHUNK + 1];
        uint8_t k = (uint8_t)(n - txt_done);
        if (k > BATT_TXT_CHUNK) k = BATT_TXT_CHUNK;
        memcpy(chunk, &txt[txt_done], k);
        chunk[k] = '\0';
        lcd_draw_flash_text(FONT_STATUS, (uint16_t)(last_txt_x + txt_done * (last_txt_w / n)), STATUS_Y, chunk);
        txt_done = (uint8_t)(txt_done + k);
        if (txt_done >= n) { txt_done = 0; txt_placed = false; batt_dirty &= (uint8_t)~BATT_DIRTY_TEXT; }
        return;
    }

    batt_dirty &= (uint8_t)~BATT_DIRTY_BAR;   /* the only mark left */
    /* With no level to fill (nothing known yet, or no pack, where a "0%" bar
     * next to the cross would state a flat battery and a missing one in the
     * same breath) want_fw is 0 and the bar comes down. A drop vacates columns
     * on the right; a rise, or a colour change at the 20/50 thresholds, is
     * covered by repainting the whole bar. */
    if (want_fw < last_fw)
        lcd_clear_rect((uint16_t)(BATT_IN_X0 + want_fw), BATT_IN_Y0, (uint16_t)(last_fw - want_fw), BATT_IN_H);
    if (want_fw > 0)
        lcd_fill_rect(BATT_IN_X0, BATT_IN_Y0, BATT_IN_X0 + want_fw - 1, BATT_IN_Y1, want_col);
    last_fw  = want_fw;
    last_col = want_col;
}

// --- Lock indicator band ---------------------------------------------------
//
// The clock (Regular-30, 34 tall at CLOCK_Y 49) ends at y82 and the battery row
// starts at STATUS_Y 106, leaving exactly 23px -- which is the Medium-20 cell
// height, so one row of status text fits precisely.
//
// A single padlock marks "something is locked"; fixed-position labels say which.
// Labels never reflow, so their positions are learnable. The band is left black
// whenever nothing is locked -- this board lives on a desk in a dark room, and a
// permanently-lit row of words would defeat the point.
//
// Labels are white because lcd_draw_flash_text() has no colour parameter: the
// atlases are RGB565 with colour baked in, and a glyph blit paints its whole
// cell including the black background (no transparency). Colour is therefore
// only available from lcd_fill_rect, which is what draws the padlock.
//
// Labelled "WIN" even though the underlying flag is keymap_config.no_gui (which
// technically disables Cmd on a Mac). Deliberate: GUI-lock exists so the Windows
// key cannot yank you out of a fullscreen game. On macOS, locking Cmd would
// disable most hotkeys and nobody would enable it on purpose -- so the label is
// for the only context where the feature is meaningful. The padlock conveys
// "disabled"; the label says which key.
#define LOCK_Y      82
#define LOCK_PAD_X  4
/* 16px padlock inside the 23px band -> LOCK_Y+3 .. LOCK_Y+18, which the clear
 * rect (STATUS_Y - LOCK_Y = 23) fully covers. It did NOT while the band was
 * 17px: the padlock ran to LOCK_Y+18 against a clear of LOCK_Y+16, leaving its
 * bottom two rows on screen forever once the locks cleared. */
#define LOCK_PAD_Y  (LOCK_Y + 3)

/* Layout invariants the panel budget depends on (phase 3.3). The padlock one
 * was a real bug: while the lock band was briefly 17px, the 16px padlock at
 * LOCK_Y+3 overhung the band's clear rect and stayed lit forever. */
_Static_assert(STATUS_Y - LOCK_Y >= 19, "lock band must contain the 16px padlock drawn at LOCK_Y+3");
_Static_assert(STATUS_Y + 23 <= 128, "battery band (20px face, 23-row cell) must fit the panel");
_Static_assert(CLOCK_Y + CLOCK_BAND_H <= LOCK_Y, "clock band must not overlap the lock band");
#define LOCK_CAP_X  20                    // "CAPS" 4 glyphs = 40px -> 20..59
#define LOCK_WIN_X  64                    // "WIN"  3 glyphs = 30px -> 64..93
#define LOCK_SLOT3_X 96                   // shared: "FN" 20px, "SCR" 30px -> 96..125
#define COL_PADLOCK 0xFFE0                // yellow

extern bool lock_state_caps(void);        // ak820pro.c
extern bool lock_state_gui(void);
extern bool lock_state_fn(void);
extern bool lock_state_scroll(void);

// 13x16 padlock. The first attempt was 11x14 with a 5px-wide flat-topped
// shackle over an 11px body, which read as a blob -- too narrow relative to the
// body and visibly square on top. This widens the shackle to 9px and steps the
// crown in two rows (5 wide, then 7) so it reads as an arch at this size; a
// single-pixel step is all the rounding 16px affords.
static void draw_padlock(uint16_t x, uint16_t y, uint16_t col) {
    lcd_fill_rect(x + 4, y + 0, x + 8,  y + 0,  col);   // crown, narrowest
    lcd_fill_rect(x + 3, y + 1, x + 9,  y + 1,  col);   // crown, stepping out
    lcd_fill_rect(x + 2, y + 2, x + 3,  y + 7,  col);   // shackle left post
    lcd_fill_rect(x + 9, y + 2, x + 10, y + 7,  col);   // shackle right post
    lcd_fill_rect(x + 0, y + 7, x + 12, y + 15, col);   // body
}

/* Per-slot diffing, for the same reason the text band does it: this used to
 * clear the whole band and repaint every label whenever ANY bit changed, so
 * pressing Fn visibly flickered CAPS and the padlock -- pixels that were not
 * changing. The four slots are at fixed, non-overlapping x positions, so each
 * can be left alone unless its own content moves. */
#define LOCK_PAD_W  13
#define LOCK_PAD_H  16
#define LOCK_TXT_H  23                    /* FONT_STATUS cell height */
#define LOCK_CAP_W  40                    /* "CAPS" 4 glyphs @10px            */
#define LOCK_WIN_W  30                    /* "WIN"  3 glyphs                  */
#define LOCK_SL3_W  30                    /* "SCR" 3; covers "FN" shrinking   */
/* draw_battery() no longer clears its whole strip, so nothing erases lock-band
 * pixels that stray below LOCK_Y + LOCK_TXT_H. Today the bands meet exactly at
 * STATUS_Y (82 + 23 = 105); keep it that way or restore the clear. */
_Static_assert(LOCK_Y + LOCK_TXT_H <= STATUS_Y, "lock band text cells must not reach into the battery strip");

static bool        shown_pad = false, shown_caps = false, shown_gui = false;
static const char *shown_sl3 = NULL;

/* The band has just been cleared: nothing of ours is on it. */
static void locks_forget(void) {
    shown_pad = shown_caps = shown_gui = false;
    shown_sl3 = NULL;
}

/* Paint whatever differs from what is on the panel. With `one` set, stop after
 * the first component painted, so a caller stepping the Fn+D restore never
 * spends more than one label -- up to four synchronous glyphs, ~10 ms -- in a
 * single main-loop pass. Returns true if anything was painted. */
static bool draw_locks_diff(bool one) {
    bool caps = lock_state_caps();
    bool gui  = lock_state_gui();
    bool fn   = lock_state_fn();
    bool scr  = lock_state_scroll();

    /* The padlock means "locked", so it tracks the lock states only. A held Fn
     * layer is not a lock -- it lights its slot without one. That stays correct
     * if Fn is ever made a toggle (TG/TT) rather than momentary. */
    bool pad = caps || gui || scr;

    /* Third slot is shared. Scroll Lock wins when actually set -- this board has
     * no Scroll Lock key and macOS effectively never sets it, so in practice the
     * slot is Fn, but the state is not thrown away on the rare occasion some
     * other host or app does set it. A pointer compare is enough: both are
     * string literals. */
    const char *slot3 = scr ? "SCR" : (fn ? "FN" : NULL);

    bool did = false;
    if (pad != shown_pad) {
        if (pad) draw_padlock(LOCK_PAD_X, LOCK_PAD_Y, COL_PADLOCK);
        else     lcd_clear_rect(LOCK_PAD_X, LOCK_PAD_Y, LOCK_PAD_W, LOCK_PAD_H);
        shown_pad = pad;
        did = true;
        if (one) return true;
    }
    if (caps != shown_caps) {
        if (caps) lcd_draw_flash_text(FONT_STATUS, LOCK_CAP_X, LOCK_Y, "CAPS");
        else      lcd_clear_rect(LOCK_CAP_X, LOCK_Y, LOCK_CAP_W, LOCK_TXT_H);
        shown_caps = caps;
        did = true;
        if (one) return true;
    }
    if (gui != shown_gui) {
        if (gui) lcd_draw_flash_text(FONT_STATUS, LOCK_WIN_X, LOCK_Y, "WIN");
        else     lcd_clear_rect(LOCK_WIN_X, LOCK_Y, LOCK_WIN_W, LOCK_TXT_H);
        shown_gui = gui;
        did = true;
        if (one) return true;
    }
    if (slot3 != shown_sl3) {
        /* Clear first: "SCR" -> "FN" would otherwise strand the third glyph,
         * since a glyph blit only paints its own cell. */
        lcd_clear_rect(LOCK_SLOT3_X, LOCK_Y, LOCK_SL3_W, LOCK_TXT_H);
        if (slot3) lcd_draw_flash_text(FONT_STATUS, LOCK_SLOT3_X, LOCK_Y, slot3);
        shown_sl3 = slot3;
        did = true;
    }
    return did;
}

/* Normal path (10 Hz housekeeping): everything that changed, in one call, so a
 * Caps press shows its padlock and its label on the same tick. `force` means
 * the caller has just cleared the screen and nothing is drawn. */
static void draw_locks(bool force) {
    if (force) locks_forget();
    draw_locks_diff(false);
}

/* Fn+D restore: ONE component per call; `first` forgets the freshly cleared
 * band. Returns true while a call painted something -- keep calling. */
static bool draw_locks_staged(bool first) {
    if (first) locks_forget();
    return draw_locks_diff(true);
}


// --- Host text slot --------------------------------------------------------
//
// One line pushed from the host over raw HID, drawn in the 24px band between
// the top row and the clock (y25..48). The firmware assigns no meaning to the
// text -- a script decides what it says and how to abbreviate it.
//
// Deliberately NOT scrolling in v1. A marquee would redraw at ~10 Hz and keep
// the flash->LCD DMA busy far more than the current roughly-once-a-second
// cadence, and DMA contention is exactly what the eeconfig-write freeze was
// about (see rgb_matrix_eeprom_flush_allowed in ak820pro.c). Truncating to what
// fits is also just less distracting to sit next to.
//
// The whole payload fits one raw-HID packet: 12 characters at a 10px advance
// fills the 128px band, against ~27 usable bytes per packet. So there is no
// framing, no offsets and no partial-render window to design around.
#define TEXT_Y        27
#define TEXT_H        (CLOCK_Y - TEXT_Y)     // 24px, clock starts at 49
#define TEXT_ICON_W   12
#define TEXT_ICON_GAP 2   /* tightened with the denser face: one more glyph */
#define TEXT_X        (TEXT_ICON_W + TEXT_ICON_GAP)
/* The per-line budgets in display.h (mirrored by MAXLEN in ak820text.py) are
 * derived facts; keep them provably in sync with the geometry. */
_Static_assert(DISPLAY_TEXT_MAX_L0 == (PANEL_WIDTH - TEXT_X) / FONT_SMALL_ADV, "line-0 budget vs icon gutter");
_Static_assert(DISPLAY_TEXT_MAX_L1 == (PANEL_WIDTH - TEXT_X2) / FONT_SMALL_ADV, "line-1 budget vs full width");

// Blank after this long with no update. Without it the panel would happily show
// last night's track forever if the host agent dies, the machine sleeps, or the
// cable is pulled -- and stale data presented confidently is worse than none.
#ifndef DISPLAY_TEXT_TIMEOUT_MS
#    define DISPLAY_TEXT_TIMEOUT_MS 180000u   /* 3 minutes */
#endif

#define COL_ICON_PLAY  0x07E0                 // green
#define COL_ICON_PAUSE 0xFFE0                 // amber
#define COL_ICON_STOP  0xF800                 // red

/* TWO lines. Line 0 is the primary (song title); line 1 is secondary (artist).
 * Kept as separate buffers rather than one wrapped string because the producer
 * knows the semantic split and we do not -- wrapping "Bohemian Rhapsody" mid
 * word would look worse than truncating it. */
#define TEXT_LINES 2
static char     text_buf[TEXT_LINES][DISPLAY_TEXT_MAX + 1] = {{0}};
static uint8_t  text_icon    = DISPLAY_ICON_NONE;
static uint32_t text_stamp   = 0;
static bool     text_present = false;
static bool     text_dirty   = false;

/* A message appeared in the band: the power ladder's Minimal mode, which keeps
 * the backlight off on battery, lights it for a moment after one, or the
 * message would go unseen and the key that caused it would seem broken.
 *
 * Noted when a message is first PAINTED, not when it is posted: a link state
 * that changes under an alert, or an alert posted while the boot animation
 * owns the panel, is first seen when the band shows it, and a wake started
 * at the post could be over by then. Each source bumps its generation when
 * its content is new -- a setting's overlay, an alert, a link state or slot,
 * host text that differs -- and draw_text_slot() notes a generation the first
 * time it paints one. A message the band returns to (a failed link's two
 * texts alternating, the host text after an overlay expires) is a generation
 * already painted, so a persistent one cannot hold the screen lit all night;
 * nor can a host keepalive, which changes nothing. The pair gesture is noted
 * continuously by conn_status_update() instead, for as long as it shows. */
enum { BAND_NONE, BAND_ALERT, BAND_PARAM, BAND_PAIR, BAND_LINK, BAND_TEXT, BAND_KINDS };
static uint16_t band_gen[BAND_KINDS];       /* bumped when a source's content is new */
static uint16_t band_painted[BAND_KINDS];   /* the generation last painted, per source */
static uint32_t message_at   = 0;
static bool     message_seen = false;

static void note_message(void) {
    message_at   = timer_read32();
    message_seen = true;
}

bool display_message_recent(uint32_t ms) {
    return message_seen && timer_elapsed32(message_at) < ms;
}

static void band_mark_painted(uint8_t kind) {
    if (kind == BAND_NONE || kind == BAND_PAIR) return;
    if (band_painted[kind] != band_gen[kind]) {
        band_painted[kind] = band_gen[kind];
        note_message();
    }
}

void display_set_text_line(uint8_t line, uint8_t icon, const char *s, uint8_t len) {
    if (line >= TEXT_LINES) return;
    /* Per-line budget: line 0 loses the icon gutter, line 1 does not. */
    uint8_t max = (line == 0) ? DISPLAY_TEXT_MAX_L0 : DISPLAY_TEXT_MAX_L1;
    if (len > max) len = max;
    char prev[DISPLAY_TEXT_MAX + 1];
    strcpy(prev, text_buf[line]);          // for the change check below
    uint8_t n = 0;
    for (uint8_t i = 0; i < len && s[i]; i++) {
        char c = s[i];
        text_buf[line][n++] = (c >= 0x20 && c < 0x7F) ? c : '?';
    }
    text_buf[line][n] = '\0';
    uint8_t new_icon = (line == 0) ? ((icon <= DISPLAY_ICON_STOP) ? icon : DISPLAY_ICON_NONE)
                                   : text_icon;

    /* Redraw only on a real change. The host re-pushes on a keepalive to correct
     * the optimistic play/pause guess (see process_record_kb), so identical
     * pushes are routine -- repainting on each one would run the flash->LCD DMA
     * for nothing, and that DMA is one half of the eeconfig-write freeze.
     *
     * text_stamp still advances on EVERY push: it is the liveness heartbeat
     * behind DISPLAY_TEXT_TIMEOUT_MS, and gating it on change would expire the
     * band mid-track whenever a track ran longer than the timeout.
     *
     * A push that brings an EXPIRED slot back is a change too, though its
     * content is not: expiry keeps the buffers, so the same text after a
     * silence would otherwise leave the band blank (codex, Minimal verify). */
    bool changed     = strcmp(text_buf[line], prev) != 0 || new_icon != text_icon;
    bool was_present = text_present;
    text_icon    = new_icon;
    text_stamp   = timer_read32();
    text_present = text_buf[0][0] || text_buf[1][0] || (text_icon != DISPLAY_ICON_NONE);
    if (changed || (text_present && !was_present)) {
        text_dirty = true;
        band_gen[BAND_TEXT]++;
    }
}

void display_set_text(uint8_t icon, const char *s, uint8_t len) {
    /* Legacy single-line entry point: line 0, and clear line 1 so a producer
     * that only knows about one line cannot leave a stale artist behind. The
     * clear is a change of its own: line 0 alone may be unchanged. */
    if (text_buf[1][0]) {
        text_buf[1][0] = '\0';
        text_dirty     = true;
        band_gen[BAND_TEXT]++;
    }
    display_set_text_line(0, icon, s, len);
}

/* Optimistic local flip when the user presses play/pause.
 *
 * The host poll is the authority, but it only runs every few seconds, and the
 * one moment the delay is annoying is when YOU pressed the key and are looking
 * at the screen. So guess immediately; the next host push overwrites it with
 * the truth, and a wrong guess self-corrects within one poll interval. That is
 * the crucial difference from tracking playback locally: this always converges.
 *
 * Deliberately does NOT touch text_stamp. The flip is a guess, not evidence the
 * host agent is still alive, so it must not extend the staleness window -- an
 * agent that died should still let the slot expire on schedule. */
void display_toggle_play_icon(void) {
    if (text_icon == DISPLAY_ICON_PLAY) {
        text_icon = DISPLAY_ICON_PAUSE;
    } else if (text_icon == DISPLAY_ICON_PAUSE) {
        text_icon = DISPLAY_ICON_PLAY;
    } else {
        return;                 /* nothing playing -> nothing to guess about */
    }
    text_dirty = true;
    band_gen[BAND_TEXT]++;
}

void display_clear_text(void) {
    text_buf[0][0] = '\0';
    text_buf[1][0] = '\0';
    text_icon    = DISPLAY_ICON_NONE;
    text_present = false;
    text_dirty   = true;
}

// 12x12 transport glyphs from rectangles -- colour is only available here, not
// from the font atlases (their colour is baked in and a glyph blit paints its
// whole cell). Play is a stepped triangle, the same trick as the padlock crown.
static void draw_text_icon_runs(uint16_t x, uint16_t y, uint8_t icon);
static void draw_text_icon(uint16_t x, uint16_t y, uint8_t icon) {
    DPROBE(DP_TEXT_ICON, draw_text_icon_runs(x, y, icon));
}

/* The transport icon as ONE RAM tile in ONE transfer (flash 2e). Drawn as
 * lcd_fill_rect runs, the play triangle was ~9 ms (the flash 2d probe, music
 * playing), and draw_text_slot() can draw it twice in a call (icon_clobbered):
 * that pair was most of the page exit's 28 ms and the overlay retirement's
 * 23 ms. Same shapes, same colours; the tile paints its own black background
 * over x..x+11, y..y+11, the icon gutter, which holds nothing else whenever an
 * icon is wanted (the overlay draws none; line 0's text starts at TEXT_X, and
 * line 1 starts below the icon in the two-line layout). */
#define TEXT_ICON_PX 12
static void icon_run(uint16_t *t, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t col) {
    for (uint8_t y = y0; y <= y1 && y < TEXT_ICON_PX; y++)
        for (uint8_t x = x0; x <= x1 && x < TEXT_ICON_PX; x++) t[y * TEXT_ICON_PX + x] = col;
}

static void draw_text_icon_runs(uint16_t x, uint16_t y, uint8_t icon) {
    static uint16_t tile[TEXT_ICON_PX * TEXT_ICON_PX];
    memset(tile, 0, sizeof(tile));                      /* black background */
    switch (icon) {
        case DISPLAY_ICON_PLAY:
            for (uint8_t i = 0; i < 6; i++)
                icon_run(tile, (uint8_t)(i * 2), i, (uint8_t)(i * 2 + 1), (uint8_t)(11 - i), COL_ICON_PLAY);
            break;
        case DISPLAY_ICON_PAUSE:
            icon_run(tile, 1, 0, 4, 11, COL_ICON_PAUSE);
            icon_run(tile, 7, 0, 10, 11, COL_ICON_PAUSE);
            break;
        case DISPLAY_ICON_STOP:
            icon_run(tile, 1, 0, 10, 11, COL_ICON_STOP);
            break;
        default:
            return;                                     /* no icon: draw nothing, as before */
    }
    lcd_blit_wait();                                    /* draw_ampm's pattern */
    lcd_blit_ram(tile, x, y, TEXT_ICON_PX, TEXT_ICON_PX);
}

/* --- Wireless status overlay ----------------------------------------------
 *
 * The icon strip says WHICH link and the channel digit says which slot, but
 * neither says what the link is DOING -- a blinking digit is only legible if you
 * already know that ~200 ms means pairing and ~700 ms means connecting. This
 * puts that in words.
 *
 * It borrows the host text band rather than adding a row: there is no spare
 * vertical space (top strip 0..24, this band 25..48, clock 49..82, locks 83..105,
 * battery 106..). Link events are transient and actionable while a track title
 * is ambient, so the overlay OUTRANKS host text while active and hands the band
 * straight back afterwards -- a title is displaced for seconds, never lost.
 *
 * CONNECTED is deliberately a brief confirmation, not a steady readout: the
 * solid digit already says "connected" indefinitely, so holding the words there
 * would permanently cost the music slot to say nothing new.
 *
 * ELEVEN characters, not DISPLAY_TEXT_MAX (12). Text is drawn from TEXT_X (16)
 * to leave the icon gutter, so 12 glyphs at a 10px advance would end at x=136 on
 * a 128px panel -- and the bezel clips the outermost columns besides. */
#define CONN_STATUS_HOLD_MS 3000u

/* The overlay draws no icon, so unlike the host slot it need not start at
 * TEXT_X (16). Drawing from 4 gives TWELVE glyphs (4 + 12*10 = 124) inside the
 * 128px panel while keeping the same 4px margin the battery row uses -- the LCD
 * is recessed and the bezel clips the outermost columns. */
#define CONN_STATUS_X   4
#define CONN_STATUS_MAX 12

/* Full period of the symptom/remedy alternation: half of it on each message.
 * Slow enough to finish reading, fast enough not to look frozen. */
#define CONN_STATUS_ALT_MS 3000

/* <0 = not holding; 0-100 = how far through BT_PAIR_HOLD_MS the hold is. */
static int16_t     pair_hint_pct = -1;
#define PAIR_BAR_CELLS 6
static char        conn_status_buf[CONN_STATUS_MAX + 1] = {0};

void display_set_pair_hint(int16_t pct) {
    pair_hint_pct = pct;
}

#ifdef PARAM_OVERLAY
static char     param_status_buf[CONN_STATUS_MAX + 1] = {0};
static uint32_t param_status_since                    = 0;
static bool     param_status_on                       = false;

void display_set_param_status(const char *s) {
    if (!s) {
        if (param_status_on) { param_status_on = false; text_dirty = true; }
        return;
    }
    strncpy(param_status_buf, s, CONN_STATUS_MAX);
    param_status_buf[CONN_STATUS_MAX] = '\0';
    param_status_since = timer_read32();
    param_status_on    = true;
    text_dirty       = true;
    band_gen[BAND_PARAM]++;
}
#endif
/* --- Boot alert -------------------------------------------------------------
 * A recovery the user should KNOW ABOUT (a watchdog reset) is silent by
 * design in the daily build -- no console -- so it announces itself here:
 * top of the band priority (below only an active pair gesture) for
 * ALERT_HOLD_MS, then the band goes back to normal. Set once at post-init. */
#define ALERT_HOLD_MS 60000u
static char     alert_buf[CONN_STATUS_MAX + 1] = {0};
static uint32_t alert_since = 0;
static bool     alert_on    = false;

void display_set_alert(const char *msg) {
    strncpy(alert_buf, msg, CONN_STATUS_MAX);
    alert_buf[CONN_STATUS_MAX] = '\0';
    alert_since = timer_read32();
    alert_on    = true;
    text_dirty  = true;
    band_gen[BAND_ALERT]++;
}

static bool alert_active(void) {
    if (alert_on && timer_elapsed32(alert_since) >= ALERT_HOLD_MS) {
        alert_on   = false;
        text_dirty = true;
    }
    return alert_on;
}

bool display_band_busy(void) {
    if (display_paused || debug_active || debug_exit_step) return true;   /* the band is not on the panel */
    if (pair_hint_pct >= 0) return true;
    if (alert_on && timer_elapsed32(alert_since) < ALERT_HOLD_MS) return true;
#ifdef PARAM_OVERLAY
    /* From the post, not the paint: an overlay the band has yet to draw is
     * one a notice would replace. */
    if (param_status_on && timer_elapsed32(param_status_since) < PARAM_OVERLAY_HOLD_MS) return true;
#endif
    return false;
}

static const char *conn_status_str   = NULL;   /* NULL = host text owns the band */
static uint8_t     conn_status_kind  = BAND_NONE;   /* conn_status_str's source */
static uint32_t    conn_status_since = 0;

static void conn_status_update(void) {
    /* In wired mode the CH582F is bypassed entirely, so its state is stale --
     * report nothing rather than whatever the last wireless session left. */
    uint8_t st = (connection_mode == CONN_MODE_WIRED)
                     ? (uint8_t)CH582_CONN_IDLE
                     : (uint8_t)ch582_get_conn_state();
    uint8_t slot = ch582_get_target_slot();

    static uint8_t last_st   = 0xFF;  /* 0xFF: force evaluation on the first tick */
    static uint8_t last_slot = 0xFF;

    /* `changed` means "conn_status_buf may now hold different text". Both the
     * state and the slot feed it: the state selects WHICH string is formatted
     * and the slot is embedded IN it. Tracking only the slot was a latent bug --
     * switching REJECTED -> PAIRING on the same slot rewrites the buffer while
     * its POINTER is unchanged, so a pointer compare alone would have left the
     * previous message on the panel. */
    bool changed = false;
    if (st != last_st) {
        last_st           = st;
        conn_status_since = timer_read32();
        changed           = true;
    }
    if (slot != last_slot) {
        last_slot = slot;
        changed   = true;
    }
    if (changed) band_gen[BAND_LINK]++;   /* even while hidden: first painted later */

    const char *want = NULL;

    /* The boot alert outranks everything except an active pair gesture: the
     * user must learn the board crashed and recovered, and a passive link
     * state or an RGB readout can wait a minute. */
    if (alert_active() && pair_hint_pct < 0) {
        if (conn_status_str != alert_buf) {
            conn_status_str = alert_buf;
            text_dirty      = true;
        }
        conn_status_kind = BAND_ALERT;
        return;
    }

#ifdef PARAM_OVERLAY
    /* Sits below the pair hint but above the link state: adjusting a setting is
     * an active gesture and wants immediate feedback, while a link state is
     * passive and will still be there in two seconds. */
    if (param_status_on) {
        if (timer_elapsed32(param_status_since) >= PARAM_OVERLAY_HOLD_MS) {
            param_status_on = false;
            text_dirty    = true;      /* expired: fall through and repaint */
        } else if (pair_hint_pct < 0) {
            if (conn_status_str != param_status_buf) {
                conn_status_str = param_status_buf;
                text_dirty      = true;
            }
            conn_status_kind = BAND_PARAM;
            return;
        }
    }
#endif

    /* Outranks every link state: during the hold the link state is whatever the
     * press kicked off (LINKING, or REJECTED if the slot is unreachable), and
     * showing "Link failed" while the user is mid-gesture is both wrong and
     * discouraging. The hold is what matters until it resolves. */
    if (pair_hint_pct >= 0) {
        note_message();   /* the gesture is in progress: keep the screen lit */
        /* A filling bar, not a static label: at 2 s a motionless message reads as
         * "nothing is happening" and invites letting go early, which is the exact
         * frustration a longer hold would otherwise cost. Progress makes the wait
         * legible, which is what makes 2 s strictly better than 1 s rather than
         * merely safer. */
        uint8_t filled = (uint8_t)((pair_hint_pct * PAIR_BAR_CELLS) / 100);
        if (filled > PAIR_BAR_CELLS) filled = PAIR_BAR_CELLS;
        char bar[CONN_STATUS_MAX + 1];
        int  n = 0;
        bar[n++] = 'P'; bar[n++] = 'a'; bar[n++] = 'i'; bar[n++] = 'r';
        bar[n++] = ':'; bar[n++] = ' ';
        for (uint8_t i = 0; i < PAIR_BAR_CELLS; i++)
            bar[n++] = (i < filled) ? '=' : ' ';
        bar[n] = '\0';

        if (conn_status_str != conn_status_buf || strcmp(conn_status_buf, bar) != 0) {
            strcpy(conn_status_buf, bar);
            conn_status_str = conn_status_buf;
            text_dirty      = true;
        }
        conn_status_kind = BAND_PAIR;
        return;
    }

    switch (st) {
        case CH582_CONN_PAIRING:
            if (connection_mode == CONN_MODE_2_4G) {
                want = "Pairing 2.4G";     /* the dongle pairs; no advertised name */
            } else {
                /* Show the EXACT advertised name, slot digit included, because
                 * the whole point is telling you what to look for in the phone's
                 * list. The module appends the slot to a stored prefix, giving
                 * "AK820 5.1-1". The "5.1" is Bluetooth-version marketing and
                 * means nothing -- but it IS what the host displays, so putting
                 * a tidier name here would be a lie.
                 *
                 * To actually rename it you would have to send 0xA9 (which this
                 * port never does) with framing documented only as "AK820 5.1-$",
                 * writing to module storage THAT CANNOT BE READ BACK. Verify the
                 * framing on the wire before attempting it. */
                if (display_blink(CONN_STATUS_ALT_MS)) {
                    /* The name alone is cryptic; the verb makes the two halves
                     * read as one sentence: "Pair with:" -> "AK820 5.1-2". */
                    want = "Pair with:";
                } else {
                    if (slot >= 1 && slot <= 3)
                        snprintf(conn_status_buf, sizeof(conn_status_buf), "AK820 5.1-%u", slot);
                    else
                        snprintf(conn_status_buf, sizeof(conn_status_buf), "AK820 5.1");
                    want = conn_status_buf;
                }
            }
            break;
        case CH582_CONN_LINKING:   want = "Connecting"; break;
        /* 0x36 is NOT in CH582F_PROTOCOL.md's 5B state table -- the "host
         * refused" reading is disassembly guesswork, never seen on the wire. So
         * describe the OBSERVABLE (no working link) and claim nothing about the
         * cause: "Refused" or "Not paired" would both assert more than is known.
         * This state also draws no digit, so without a word here the panel says
         * nothing at all. */
        case CH582_CONN_REJECTED:
            /* Alternate the symptom with the REMEDY. A failed link is the one
             * state where the panel can say something actionable, and we know
             * WHICH slot failed -- so name the single key rather than listing
             * Fn+Q/W/E and making the reader pick (which is 13 chars anyway,
             * one over budget).
             *
             * Deliberately NOT done for a long-lived LINKING. That state is
             * ambiguous: it is equally a dropped `5B 32` on a link that is
             * actually UP, and telling someone to hold the slot key there would
             * tear down a working connection to fix a display bug.
             *
             * The advice survives even if the 0x36 reading is wrong: re-pairing
             * the slot is a sane remedy for any stuck wireless state, which is
             * more than can be said for the label. */
            if (display_blink(CONN_STATUS_ALT_MS)) {
                want = "Link failed";
            } else {
                /* Stock keymap: Fn+Q/W/E select BT slots 1-3, Fn+R the dongle. */
                char key = (connection_mode == CONN_MODE_2_4G) ? 'R'
                         : (slot == 1) ? 'Q' : (slot == 2) ? 'W' : (slot == 3) ? 'E' : 0;
                if (key) {
                    /* Name the duration: the hold is BT_PAIR_HOLD_MS (2 s) and
                     * now fires under the finger, so "2s" is a promise the
                     * firmware actually keeps. 12 chars exactly. */
                    snprintf(conn_status_buf, sizeof(conn_status_buf), "Hold Fn+%c 2s", key);
                } else {
                    snprintf(conn_status_buf, sizeof(conn_status_buf), "Fn+Q/W/E");
                }
                want = conn_status_buf;
            }
            break;
        case CH582_CONN_CONNECTED:
            if (timer_elapsed32(conn_status_since) < CONN_STATUS_HOLD_MS)
                want = "Connected";
            break;
        default: break;
    }

    /* Pointer compare covers the literals; `changed` covers the formatted
     * buffer, whose pointer is stable while its contents are not. */
    if (want != conn_status_str || (changed && want == conn_status_buf)) {
        conn_status_str = want;
        text_dirty      = true;   /* overlay appeared, changed, or was released */
    }
    conn_status_kind = want ? BAND_LINK : BAND_NONE;
}

/* ---- Text band: draw a BOUNDED number of glyphs per pass -------------------
 *
 * lcd_draw_flash_text() issues one DMA blit PER GLYPH and spins on each, so a
 * two-line update was ~40 sequential blocking waits on the main loop -- the
 * same loop that scans the matrix. Measured at 53 ms, which at typing speed
 * (a character roughly every 125 ms) is ample to swallow a keystroke. Isolated
 * by experiment: with the host text producer stopped, four reboots produced
 * zero stalls over 15 ms; with it running, 53 ms on both boots.
 *
 * Most of that is per-glyph OVERHEAD, not data: a 6x14 glyph is 168 bytes, but
 * each one pays a window command, a flash read command, and a DMA arm.
 *
 * SUPERSEDED: the next design was to paint ONE LINE per housekeeping pass,
 * composed in RAM and blitted once (lcd_draw_flash_text_staged). That function
 * was declared and never defined (see draw_debug_page's note). The glyph queue
 * below is what the band uses.
 *
 * The clear rect and icon are drawn ONCE per repaint, on the first pass --
 * repeating them on every resume would erase the glyphs already painted. */
/* One LINE per housekeeping pass, composed in RAM and blitted once.
 *
 * The first attempt at this budgeted per GLYPH, which bounded the stall but
 * made every update crawl in letter by letter -- visibly worse, and it hit the
 * RGB readout too, where instant feedback is the whole point.
 *
 * SUPERSEDED, as above: lcd_draw_flash_text_staged() was to pay the per-glyph
 * setup once and blit a line whole. It never existed. The glyph queue below
 * bounds every pass instead, one glyph per main-loop pass. */
/* Glyph queue: up to two runs (line 0, line 1), painted ONE GLYPH PER MAIN-LOOP
 * ITERATION by display_blit_pump(). Each turn either arms a DMA or returns on a
 * single lcd_blit_busy() compare, so the band never blocks the matrix scan.
 *
 * At ~390 Hz a 20-glyph line completes in ~50 ms -- below the threshold where
 * anyone sees it arrive. The earlier one-glyph-per-HOUSEKEEPING-tick version
 * looked identical on paper and took 2 s, because 10 Hz was the wrong clock.
 * Granularity was never the problem; the tick was. */
#define GQ_RUNS  3                      /* line 0, line 1, clock band      */
/* Slot 2 (GQ_SLOT_CLOCK, defined above) carries the clock band. Routing it
 * through the same queue+shadow machinery is what removed the blocking full
 * repaints (audit BW-2/3: playback relayout ~22 ms, clock force repaint
 * ~16 ms, both on the loop that scans the matrix). */
/* Longest single run any slot can queue (the playback buffer). Text callers
 * clamp to their own budgets before reaching queue_line. */
#define GQ_LINE_MAX PLAYBACK_MAX
#define GQ_MAX  (DISPLAY_TEXT_MAX_L0 + DISPLAY_TEXT_MAX_L1 + GQ_LINE_MAX)

/* The queue holds INDIVIDUAL GLYPHS, not runs, because a repaint normally
 * touches only the cells whose character actually changed -- "Bright 25%" to
 * "Bright 26%" is one blit, not eleven. */
static struct { uint16_t font, x, y; char c; } gq[GQ_MAX];
static uint8_t gq_n = 0, gq_i = 0;

/* What is CURRENTLY on the panel, so the next repaint can diff against it. */
static struct {
    uint16_t font, x, y;
    char     s[GQ_LINE_MAX + 1];
    bool     valid;
} shadow[GQ_RUNS];

static uint8_t last_icon_drawn = DISPLAY_ICON_NONE;
static uint16_t last_icon_y    = 0;

#define TEXT_ICON_H 12          /* draw_text_icon paints rows y..y+11 */
_Static_assert(TEXT_ICON_H == TEXT_ICON_PX && TEXT_ICON_W == TEXT_ICON_PX, "the icon tile is the icon's whole box");

static bool icon_clobbered = false;

static bool rects_overlap(uint16_t ax, uint16_t ay, uint16_t aw, uint16_t ah,
                          uint16_t bx, uint16_t by, uint16_t bw, uint16_t bh) {
    return ax < bx + bw && bx < ax + aw && ay < by + bh && by < ay + ah;
}

/* EVERY clear in this band goes through here, and it is the invariant the whole
 * diffing scheme rests on: a shadow may only claim "already painted" for pixels
 * that are still on the panel.
 *
 * The band has three overlapping owners in a 14px-wide gutter -- the transport
 * icon (x 0..11), the overlay (x 4.., it starts left of TEXT_X to buy a twelfth
 * character) and line 1 (x 2..). So one owner's cleanup routinely erases
 * another's pixels, and a diffing renderer will then never repaint them,
 * because its shadow still says they are there. That is not an edge case; it
 * happened twice, and both times it was permanent damage rather than a flicker:
 *
 *   - the retiring overlay's clear (x 4..) ate the icon, leaving columns 0..3
 *   - the icon block's gutter clear (full band height) ate the first two
 *     glyphs of line 1, which never came back
 *
 * Rather than hand-patching each pair as it is discovered, any clear now
 * invalidates every shadow it touches, so the next pass repaints them. Add a
 * new owner to this band and it is covered automatically.
 *
 * ⚠️ DO NOT "FIX" THE ICON/OVERLAP BY DRAWING THE ICON AFTER THE TEXT. That
 * breaks the other direction: entering the overlay, the icon's clear erases the
 * overlay's first glyph at x=4. Icon-first is required. */
static void band_clear(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    DPROBE(DP_BAND_CLEAR, lcd_clear_rect(x, y, w, h));

    if (last_icon_drawn != DISPLAY_ICON_NONE &&
        rects_overlap(x, y, w, h, 0, last_icon_y, TEXT_ICON_W, TEXT_ICON_H))
        icon_clobbered = true;

    for (uint8_t i = 0; i < GQ_RUNS; i++) {
        if (!shadow[i].valid) continue;
        uint16_t adv = lcd_font_advance(shadow[i].font);
        uint16_t hh  = lcd_font_height(shadow[i].font);
        if (!adv || !hh) continue;
        if (rects_overlap(x, y, w, h, shadow[i].x, shadow[i].y,
                          (uint16_t)(strlen(shadow[i].s) * adv), hh))
            shadow[i].valid = false;   /* forces a full repaint of that line */
    }
}

static void gq_reset(void) { gq_n = gq_i = 0; }
static bool gq_pending(void) { return gq_i < gq_n; }

bool display_bus_idle(void) {
    return !gq_pending() && !debug_exit_step && !lcd_blit_busy();
}

static void shadow_invalidate(void) {
    for (uint8_t i = 0; i < GQ_RUNS; i++) shadow[i].valid = false;
    last_icon_drawn = DISPLAY_ICON_NONE;
    ampm.shown      = false;   /* the AM/PM glyph is its own shadow */
}

static void gq_push(uint16_t font, uint16_t x, uint16_t y, char c) {
    if (gq_n >= GQ_MAX) return;
    gq[gq_n].font = font; gq[gq_n].x = x; gq[gq_n].y = y; gq[gq_n].c = c;
    gq_n++;
}

/* Called every main-loop iteration. Cheap by construction: one compare while
 * the DMA is running, one DMA arm when it is not. */
/* The bus stayed busy for this pass. A healthy blit completes in well under a
 * millisecond; one whose completion was lost never does. Give it a generous
 * grace, then run the bounded recovery -- teardown, counter, [lcd] console
 * line -- exactly as a blocking wait would. */
static void pump_bus_busy(bool *blocked, uint32_t *since) {
    if (!*blocked) {
        *blocked = true;
        *since   = timer_read32();
    } else if (timer_elapsed32(*since) > 50) {
        lcd_blit_wait();
        *blocked = false;
    }
}

void display_blit_pump(void) {
    static uint32_t blocked_since = 0;
    static bool     blocked       = false;
    static uint32_t last_heal     = 0;

    if (pass_deferred) return;   /* a deep sleep's recovery used the PCF this pass */

    /* A blit given up for good left pixels unpainted that the shadows claim
     * are there (codex review, 2026-09-23, finding 13). Repaint the whole
     * dashboard through the staged, keystroke-safe Fn+D restore. At most once
     * per 5 s, so a bus that keeps failing cannot become a repaint loop; the
     * flag waits while the dashboard is not showing. */
    if (!debug_active && !debug_exit_step && !display_paused &&
        timer_elapsed32(last_heal) > 5000 && lcd_blit_lost_take()) {
        last_heal       = timer_read32();
        debug_exit_step = 1;
        dprintf("[display] a blit was given up: repainting the dashboard\n");
    }

    /* The debug page owns the panel outright while active, so the band queue is
     * idle and there is no contention. One cell per pass, same budget as a
     * queued glyph -- and the same grace-then-recover on a busy bus, which it
     * used to retry forever (codex review, finding 12). */
    if (debug_active) {
        if (lcd_blit_busy()) { pump_bus_busy(&blocked, &blocked_since); return; }
        blocked = false;
        DPROBE(DP_DBG_GLYPH, debug_pump_glyph());
        return;
    }
    /* Restoring the dashboard after Fn+D: one stage per pass. Ahead of the
     * queue because stage 1 clears the panel and discards whatever is in it. */
#ifdef AK820_DEV_HOOKS
    if (debug_exit_step) {
        blocked = false;
        uint8_t st = debug_exit_step;   /* 1..14, before the stage advances it */
        DPROBE(DP_RESTORE0 + st - 1u, debug_restore_step());
        return;
    }
#else
    if (debug_exit_step) { blocked = false; debug_restore_step(); return; }
#endif

    if (gq_i >= gq_n || display_paused) {
        blocked = false;   /* stale grace must not carry into later work */
        return;
    }
    if (!lcd_draw_flash_glyph_try(gq[gq_i].font, gq[gq_i].c, gq[gq_i].x, gq[gq_i].y)) {
        /* Bus busy. Nothing on this path calls lcd_blit_wait() (the old
         * gq_flush did, as a side effect), so a wedged blit would park the
         * queue AND the deferring housekeeping pass forever, freezing every
         * band. pump_bus_busy() bounds it. */
        pump_bus_busy(&blocked, &blocked_since);
        return;                        /* try again next iteration */
    }
    blocked = false;
    if (++gq_i >= gq_n) gq_reset();
}

/* Throw away everything queued and forget the panel contents. For the two
 * moments the queue's contents become lies: a full-screen clear, and the
 * bootloader splash. NOT a flush -- the old synchronous gq_flush() drained up
 * to a full two-line repaint (~40 glyphs ~= 52 ms, the original keystroke-
 * eater number) on the loop that scans the matrix whenever a band transition
 * met another owner's redraw (audit BW-1). Nothing waits for the pump any
 * more; owners DEFER instead (see display_housekeeping_task). */
static void display_queue_discard(void) {
    gq_reset();
    shadow_invalidate();
}

/* Queue one line, drawing ONLY what differs from what is already on the panel.
 *
 * This is what the clock does, and it is why the clock never flickered while
 * this band did: a glyph blit paints its whole cell INCLUDING the background,
 * so overwriting a cell fully replaces it and no clear is needed. Clearing
 * first is what made a one-character change read as a flash -- the band spent
 * the paint time empty.
 *
 * Only a shorter string needs any clearing, and only of the cells it vacates. */
static void queue_line(uint8_t slot, uint16_t font, uint16_t x, uint16_t y,
                       const char *str) {
    if (slot >= GQ_RUNS) return;
    uint16_t adv = lcd_font_advance(font);
    uint16_t h   = lcd_font_height(font);
    if (!adv || !h) return;

    uint8_t n = (uint8_t)strlen(str);
    if (n > GQ_LINE_MAX) n = GQ_LINE_MAX;

    /* Two distinct non-diffable states, and conflating them was a real bug:
     *
     *   moved   -- the shadow is VALID but the run is elsewhere (font/x/y
     *              changed): the old cells are KNOWN, clear exactly them.
     *   unknown -- the shadow is INVALID (some clear overlapped this run and
     *              band_clear dropped it): the panel content here is UNKNOWN,
     *              and may be WIDER than the new run. Treating this as
     *              "nothing there" skipped the trailing-cell clear and left
     *              "Connectedg" on the panel (the retiring line-1 clear
     *              invalidates the overlay's shadow every time the band
     *              changes hands, so the shrink-by-one-cell transition never
     *              saw its vacated-cell clear). Clear the FULL band width at
     *              this line's rect -- the only safe answer to unknown. The
     *              cascade is convergent: invalidating a neighbour makes IT
     *              full-clear its own rect next paint, and the text-line
     *              rects don't mutually overlap. */
    bool unknown = !shadow[slot].valid;
    bool moved   = !unknown && (shadow[slot].font != font ||
                                shadow[slot].x != x || shadow[slot].y != y);
    uint8_t old_n = (moved || unknown) ? 0 : (uint8_t)strlen(shadow[slot].s);

    if (moved) {
        uint16_t oadv = lcd_font_advance(shadow[slot].font);
        uint16_t oh   = lcd_font_height(shadow[slot].font);
        uint16_t ox   = shadow[slot].x, oy = shadow[slot].y;
        uint16_t ow   = (uint16_t)(strlen(shadow[slot].s) * oadv);
        uint16_t nw   = (uint16_t)(n * adv);
        if (oadv && oh) {
            if (oy == y && oh == h) {
                /* Same row, same face (the clock format change, a playback
                 * relayout at the same size): the new cells will overwrite
                 * everything they cover, so clear ONLY the columns the old
                 * run vacates. The old digits stay up until each cell is
                 * repainted -- no black flash. */
                if (ox < x) band_clear(ox, oy, (uint16_t)(x - ox), oh);
                if (ox + ow > x + nw)
                    band_clear((uint16_t)(x + nw), oy, (uint16_t)(ox + ow - (x + nw)), oh);
            } else {
                band_clear(ox, oy, ow, oh);
            }
        }
    } else if (unknown) {
        band_clear(0, y, PANEL_WIDTH, h);
    }

    for (uint8_t i = 0; i < n; i++)
        if (moved || unknown || i >= old_n || shadow[slot].s[i] != str[i])
            gq_push(font, (uint16_t)(x + i * adv), y, str[i]);

    /* Cells the new string no longer occupies keep their old glyphs otherwise. */
    if (old_n > n)
        band_clear((uint16_t)(x + n * adv), y,
                   (uint16_t)((old_n - n) * adv), h);

    memcpy(shadow[slot].s, str, n);
    shadow[slot].s[n] = '\0';
    shadow[slot].font = font; shadow[slot].x = x; shadow[slot].y = y;
    shadow[slot].valid = true;
}

/* Declare a slot KNOWN EMPTY at a layout: the caller has just cleared the
 * whole band (every pixel the slot could own), so the next queue_line at
 * this font/x/y paints every cell WITHOUT the unknown-path full-width clear.
 * Only valid right after such a clear -- lying here is the stray-glyph bug. */
static void shadow_mark_empty(uint8_t slot, uint16_t font, uint16_t x, uint16_t y) {
    if (slot >= GQ_RUNS) return;
    shadow[slot].font  = font;
    shadow[slot].x     = x;
    shadow[slot].y     = y;
    shadow[slot].s[0]  = '\0';
    shadow[slot].valid = true;
}

/* Retire a line that is no longer shown, clearing only what it occupied. */
static void retire_line(uint8_t slot) {
    if (slot >= GQ_RUNS || !shadow[slot].valid) return;
    uint16_t adv = lcd_font_advance(shadow[slot].font);
    uint16_t h   = lcd_font_height(shadow[slot].font);
    if (adv && h)
        band_clear(shadow[slot].x, shadow[slot].y,
                       (uint16_t)(strlen(shadow[slot].s) * adv), h);
    shadow[slot].valid = false;
}

static void draw_text_slot(bool force) {
    // Expire before drawing, so a stale slot blanks itself without the host
    // having to send anything.
    if (text_present && timer_elapsed32(text_stamp) > DISPLAY_TEXT_TIMEOUT_MS) {
        text_present = false;
        text_dirty   = true;
    }
    if (!force && !text_dirty) return;
    /* A previous diff is still being pumped: computing a new one against a
     * half-painted panel would corrupt the shadows. Keep the dirty flag and
     * come back next tick -- the pump finishes a full line in ~50 ms at
     * main-loop rate, so the deferral is at most one 100 ms tick. */
    if (!force && gq_pending()) return;
    text_dirty = false;
    icon_clobbered = false;

    if (force) {
        /* A forced repaint follows a full-screen clear (the Fn+D restore's
         * clear bands; display_redraw_dashboard's lcd_clear_rect), so the band
         * is already blank, and nothing the callers draw between lands in it.
         * The shadows cannot be trusted, though: forget them, and each line's
         * first paint clears its own row. A band clear here as well was ~8 ms
         * of the page exit's 28 ms for nothing (flash 2d probe). */
        shadow_invalidate();
    }

    /* --- icon: only when it actually changes ------------------------------ */
    uint8_t  want_icon = DISPLAY_ICON_NONE;
    uint16_t icon_y    = 0;
    if (!conn_status_str && text_present && text_icon != DISPLAY_ICON_NONE) {
        want_icon = text_icon;
        /* Centre on the CAP band, not the cell -- see the measured figures at
         * the top of this file. The icon centre is y+5.5; the 13px cap band
         * centre is cell_top+5.5, so the icon's y IS the text's cell top.
         *
         * The 20px case cannot land exactly: its cap centre is cell_top+11
         * against the icon's +5.5, so the ideal y is fractional (x.5). It is
         * rounded up, leaving the icon 0.5px high -- the direction optical
         * centring prefers, and the only alternative is 0.5px low. */
        if (text_buf[1][0])                       icon_y = TEXT_Y;
        else if (strlen(text_buf[0]) <= TEXT_BIG_MAX)
                                                  icon_y = TEXT_Y + TEXT_BIG_DY + 5;
        else                                      icon_y = TEXT_Y + TEXT_FONT_DY;
    }
    if (want_icon != last_icon_drawn || icon_y != last_icon_y) {
        /* Clear only where the OLD icon actually was. This used to wipe the
         * full gutter for the whole band height, which reached down into
         * line 1 and took its first two glyphs with it. */
        if (last_icon_drawn != DISPLAY_ICON_NONE)
            band_clear(0, last_icon_y, TEXT_ICON_W, TEXT_ICON_H);
        if (want_icon != DISPLAY_ICON_NONE) draw_text_icon(0, icon_y, want_icon);
        last_icon_drawn = want_icon;
        last_icon_y     = icon_y;
        icon_clobbered  = false;   /* just painted it; only LATER clears matter */
    }

    /* --- text ------------------------------------------------------------- */
    band_mark_painted(conn_status_str ? conn_status_kind : text_present ? BAND_TEXT : BAND_NONE);
    if (conn_status_str) {
        queue_line(0, FONT_STATUS, CONN_STATUS_X, TEXT_Y + TEXT_BIG_DY, conn_status_str);
        retire_line(1);
    } else if (text_present && text_buf[1][0]) {
        queue_line(0, FONT_SMALL, TEXT_X,  TEXT_Y,               text_buf[0]);
        queue_line(1, FONT_SMALL, TEXT_X2, TEXT_Y + TEXT_LINE_H, text_buf[1]);
    } else if (text_present && text_buf[0][0]) {
        bool big = strlen(text_buf[0]) <= TEXT_BIG_MAX;
        queue_line(0, big ? FONT_STATUS : FONT_SMALL, TEXT_X,
                   TEXT_Y + (big ? TEXT_BIG_DY : TEXT_FONT_DY), text_buf[0]);
        retire_line(1);
    } else {
        retire_line(0);
        retire_line(1);
    }

    /* Repaint the icon if a text clear reached into it. Costs one blit, and
     * only on a transition that is already repainting -- in the steady state
     * nothing clears the gutter and this never fires. */
    if (icon_clobbered && want_icon != DISPLAY_ICON_NONE)
        draw_text_icon(0, icon_y, want_icon);
}

static void draw_status(bool force) {
    draw_conn_number(force);
    draw_battery(force);
    draw_locks(force);
    draw_text_slot(force);
}

/* Clock-sync plan 3.9: on an RTC second edge, run ONE extra pass of the
 * fully gated task below so the digits change within a main-loop pass
 * (~2.5 ms) of the tick instead of 0-100 ms later at the 10 Hz cadence.
 * Every gate (user hook, pause, splash, gq_pending, the last_shown_sec
 * latch) is inside display_housekeeping_task(); nothing is moved. Fires at
 * most once per RTC second, never per iteration. */
/* Clock-sync plan 3.6: boot phase acquisition waits for the splash to be
 * gone before it starts polling the PCF. */
bool display_splash_done(void) { return splash_cleared; }

#ifdef AK820_DEV_HOOKS
/* HC_DEVHOOK op 2: probe page 0 or 1 as 14 u16 microsecond values. */
void display_probe_fill(uint8_t page, uint8_t *out28) {
    memset(out28, 0, 28);
    for (uint8_t k = 0; k < 14; k++) {
        uint16_t i = (uint16_t)(page * 14u + k);
        if (i >= DP_N) break;
        uint64_t us = (uint64_t)dprobe_max[i] * 1000000u / CH_CFG_ST_FREQUENCY;
        if (us > 0xFFFFu) us = 0xFFFFu;
        out28[2 * k]     = (uint8_t)us;
        out28[2 * k + 1] = (uint8_t)(us >> 8);
    }
}

void display_probe_reset(void) { memset(dprobe_max, 0, sizeof(dprobe_max)); }
#endif

/* --- A deep sleep's wake (sleep.c; plans/DEEP-SLEEP-BUILD-PLAN.md, section 4,
 * steps 6 and 9) ---------------------------------------------------------------
 * At the wake: host content expired (no host refreshes it while the board
 * sleeps, and a 3 min text could otherwise come back hours later), the
 * backlight held dark. Once the clock is set and the level debited
 * (display_wake_corrected): both repaints scheduled, one component a pass
 * rather than at the second's or the 10 Hz cadence, and the hold released
 * when both are painted from the corrected state and the bus is idle (the
 * final DMA done, not just the queue empty). With no clock to wait for (off,
 * playback's band, Fn+D or the animation owning the panel), only the row. */
static bool     wake_painting     = false;
static bool     wake_clock_due    = false;
static bool     wake_batt_due     = false;
static uint16_t wake_clock_target = 0;

void display_defer_pass(void) {
    pass_deferred = true;
}

bool display_pass_deferred(void) {
    return pass_deferred;
}

void display_sleep_wake(void) {
    bkl_wake_hold = true;
    wake_painting = false;
    if (text_present) {
        text_present = false;
        text_dirty   = true;
    }
    if (pb_active) {
        pb_active           = false;
        clock_force_repaint = true;
    }
}

void display_wake_corrected(void) {
    wake_clock_target = rtc_clock_generation();
    wake_clock_due    = true;
    wake_batt_due     = true;
    wake_painting     = true;
}

static void wake_paint_step(void) {
    if (pass_deferred || !splash_cleared || display_paused || debug_active || debug_exit_step) return;
    if (gq_pending() || lcd_blit_busy()) return;   /* the pump finishes what is queued first */
    if (wake_clock_due) {
        wake_clock_due = false;
        draw_clock();
        return;
    }
    if (wake_batt_due || batt_dirty) {
        wake_batt_due = false;
        draw_battery(false);   /* one component; it self-guards on what changed */
    }
}

bool display_wake_repainted(void) {
    bool no_clock = !splash_cleared || display_paused || debug_active || clock_mode == DISPLAY_CLOCK_OFF || pb_active;
    bool clock_ok = no_clock || (!wake_clock_due && (int16_t)(clock_gen_painted - wake_clock_target) >= 0);
    return clock_ok && !wake_batt_due && !batt_dirty && display_bus_idle();
}

void display_wake_release(void) {
    bkl_wake_hold  = false;
    wake_painting  = false;
    wake_clock_due = wake_batt_due = false;
}

void display_pass_end(void) {
    batt_painted_this_pass = false;
    pass_deferred          = false;
}

void display_second_edge_task(void) {
    if (wake_painting) wake_paint_step();
    static uint32_t edge_seen = UINT32_MAX;
    uint32_t sec = rtc_get_seconds();
    if (sec != edge_seen) {
        edge_seen = sec;
        display_housekeeping_task();
    }
}

void display_housekeeping_task(void) {
    if (!display_housekeeping_task_user())
        return;
    if (pass_deferred) return;   /* a deep sleep's recovery used the PCF this pass */

    if (display_paused) return;   // animation owns the bus
    if (debug_active) { DPROBE(DP_DEBUG_PAGE, draw_debug_page()); return; }   // Fn+D owns the panel
    /* The Fn+D restore owns the panel too, one stage per main-loop pass from
     * display_blit_pump(). A 10 Hz tick landing inside it would paint into a
     * clear band still sweeping down, or run draw_locks(false) and collapse the
     * staged lock repaint back into one 25-30 ms call. It is over in ~40 ms;
     * every owner below self-guards and simply catches up on the next tick.
     *
     * ⚠️ THIS GATE IS SAFE ONLY BECAUSE display_blit_pump() IS CALLED FROM ITS
     * OWN SITE in housekeeping_task_kb() (ak820pro.c), not from here. The pump
     * is the only thing that advances debug_exit_step. Fold the pump into this
     * function -- the obvious tidy-up, both are display work -- and this line
     * blocks the one caller that can clear the flag: the restore never finishes,
     * the dashboard never comes back, and the main loop stays alive so the
     * watchdog does not save you. */
    if (debug_exit_step) return;

    if (splash_cleared) {
        /* A queued line is mid-paint: let the pump finish before any owner
         * draws or diffs. The pump completes a worst-case line in ~50 ms at
         * main-loop rate, so this defers at most one 100 ms tick -- where the
         * old synchronous gq_flush() here could block ~52 ms on the matrix-
         * scanning loop (audit BW-1). The 1 Hz clock latch below is only
         * reached when this passes, so a deferred second paints on the next
         * tick rather than being skipped. */
        if (gq_pending()) return;

        // Connection digit every tick (~10 Hz) so its blink animates; self-guarded.
        LOOP_SITE(LOOP_SITE_CONN, DPROBE(DP_CONN, draw_conn_number(false)));

        // Wireless status overlay: recompute before the slot is drawn so an
        // appearing/releasing overlay repaints on this same tick.
        LOOP_SITE(LOOP_SITE_STATUS, DPROBE(DP_STATUS, conn_status_update()));

        // Host text slot: self-guards, and expires itself on the same tick.
        LOOP_SITE(LOOP_SITE_TEXT, DPROBE(DP_TEXT_SLOT, draw_text_slot(false)));

        // Lock band: self-guards, only redraws when a lock state actually
        // changes. Needs the ~10 Hz tick rather than the 1 Hz clock path so a
        // Caps press shows up immediately.
        LOOP_SITE(LOOP_SITE_LOCKS, DPROBE(DP_LOCKS, draw_locks(false)));

        /* A forced clock-band repaint (owner or format change) paints on this
         * tick rather than up to a second later at the edge; the per-second
         * pass below then diffs to nothing. */
        if (clock_force_repaint || clock_relayout) LOOP_SITE(LOOP_SITE_CLOCKF, draw_clock());

        // Clock + battery only need refreshing once per RTC second.
        static uint32_t last_shown_sec = UINT32_MAX;
        uint32_t sec = rtc_get_seconds();
        if (sec != last_shown_sec) {
            last_shown_sec = sec;
            /* Advance the playback timer on the RTC second, so it runs at the
             * same cadence the band already repaints at -- no extra DMA. */
            display_playback_tick();
            LOOP_SITE(LOOP_SITE_CLOCK, DPROBE(DP_CLOCK, draw_clock()));
            LOOP_SITE(LOOP_SITE_BATT,  DPROBE(DP_BATT, draw_battery(false))); // self-guards, only draws on change
        } else if (batt_dirty || batt_bulb_stale()) {
            /* The rest of a battery repaint, one component per tick; or a new
             * mode's bulb (batt_bulb_stale). */
            LOOP_SITE(LOOP_SITE_BATT, DPROBE(DP_BATT, draw_battery(false)));
        }
    }
}

void display_draw_mac_logo(void) {
    mac_mode = true;
    if (splash_cleared && !display_paused)
        lcd_draw_flash_image(ASSET_APPLE_ICON_24X24, 0, 0);
}

void display_draw_windows_logo(void) {
    mac_mode = false;
    if (splash_cleared && !display_paused)
        lcd_draw_flash_image(ASSET_WINDOWS_ICON_24X24, 0, 0);
}

void display_draw_usb_logo(void) {
    connection_mode = CONN_MODE_WIRED;
    if (splash_cleared && !display_paused)
        draw_conn_row();
}

void display_draw_bluetooth_logo(void) {
    connection_mode = CONN_MODE_BLUETOOTH;
    if (splash_cleared && !display_paused)
        draw_conn_row();
}

void display_draw_2_4_g_logo(void) {
    connection_mode = CONN_MODE_2_4G;
    if (splash_cleared && !display_paused)
        draw_conn_row();
}
