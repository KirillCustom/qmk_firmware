// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#include <stdint.h>
#include <stdbool.h>
#include "rtc/rtc.h"

bool display_init_kb(void);
bool display_init_user(void);
void display_housekeeping_task(void);
/* Per main-loop pass: one extra housekeeping pass on an RTC second edge
 * (clock-sync plan 3.9). Call before display_blit_pump(). */
void display_second_edge_task(void);
/* The end of a main-loop pass: housekeeping_task_kb() calls it last. */
void display_pass_end(void);
#ifdef AK820_DEV_HOOKS
/* The display probe (dev builds; HC_DEVHOOK in hid_protocol.c). */
void display_probe_fill(uint8_t page, uint8_t *out28);
void display_probe_reset(void);
#endif
/* True once the boot splash has been cleared and the dashboard owns the panel. */
bool display_splash_done(void);

/* Paints one queued glyph per call. Must be driven from housekeeping_task_kb()
 * at MAIN-LOOP rate, not from the 10 Hz block -- see the glyph queue in
 * display.c for why that distinction is the whole fix. */
void display_blit_pump(void);

void display_set_power(bool on);
bool display_get_power(void);
void display_toggle_power(void);

/* Backlight brightness, 0..BKL_MAX_LEVEL (perceptually spaced; 0 = off).
 * Software PWM from the RGB row ISR -- see display.c. Not persisted. */
/* Backlight PWM tick. ISR context; called once per row-scan interrupt. */
void    display_backlight_tick(void);

uint8_t display_get_brightness(void);
void    display_set_brightness(uint8_t level);
/* Power-save ceiling on the backlight level, 0xFF for none; 0 turns the
 * backlight off. Does not change or persist the user's level. */
void    display_set_backlight_cap(uint8_t cap);
/* The level the panel is actually lit at: the user's, under any cap; 0 while
 * the display is powered down. For the battery log. */
uint8_t display_backlight_effective(void);
/* That level's PWM duty, 0..48 of 48: the backlight's draw is linear in it
 * (R1, 10-15). For battery.c's model. */
uint8_t display_backlight_duty(void);
void    display_brightness_up(void);
void    display_brightness_down(void);

/* Clock band format. Display-only: it changes how the band draws the time the
 * RTC module keeps, never the time itself. Persisted in kb_eeconfig by the
 * Fn+C gesture path (ak820pro.c); the setter here is raw, so init can restore
 * the stored value through it. */
enum display_clock_mode {
    DISPLAY_CLOCK_24H = 0,      /* HH:MM:SS                                   */
    DISPLAY_CLOCK_12H,          /* H:MM:SS + stacked grey AM/PM glyph          */
    DISPLAY_CLOCK_OFF,          /* band blank; the playback timer still uses it */
    DISPLAY_CLOCK_DATE,         /* "Sep 1, 2026" in the 20px face (appended 2026-09-01) */
    DISPLAY_CLOCK_MODE_COUNT    /* append only: the value is persisted */
};
void    display_set_clock_mode(uint8_t mode);
uint8_t display_get_clock_mode(void);

/* --- Host text slot -------------------------------------------------------
 * A single line the host pushes over raw HID, drawn in the band above the
 * clock. The firmware attaches no meaning to the text; it is whatever the host
 * script decided to send.
 *
 * icon: 0 none, 1 play, 2 pause, 3 stop. Deliberately an ICON ID rather than a
 * "media state" so other producers can reuse it without the name lying. */
/* Host text is drawn in the 14px face (7px advance) rather than the 20px one,
 * because this is the slot where cramming matters -- song titles.
 *
 * The two lines have DIFFERENT budgets, because only line 0 sits beside the
 * transport icon. Line 0 starts at TEXT_X (14) and gets (128 - 14) / 6 = 19;
 * line 1 starts at TEXT_X2 (2) and gets (128 - 2) / 6 = 21. The producer puts
 * the TITLE on line 1, so the title gets the wider of the two.
 *
 * The 6px advance is Cozette's OWN design width -- the atlas used to be set on
 * a 7px cell, which spent a column of letterspacing on every glyph for nothing.
 * Setting it natively bought three characters per line and tightened the word
 * shapes at the same time.
 *
 * The wireless status overlay deliberately stays on the 20px face: those strings
 * are short and want legibility, not density. */
#define DISPLAY_TEXT_MAX_L0 19   /* beside the icon gutter */
#define DISPLAY_TEXT_MAX_L1 21   /* full width */
#define DISPLAY_TEXT_MAX    DISPLAY_TEXT_MAX_L1   /* buffer size: the wider one */

enum display_text_icon {
    DISPLAY_ICON_NONE = 0,
    DISPLAY_ICON_PLAY,
    DISPLAY_ICON_PAUSE,
    DISPLAY_ICON_STOP,
};

void display_set_text(uint8_t icon, const char *s, uint8_t len);

/* Two-line variant. line 0 = primary (song title), line 1 = secondary (artist).
 * The icon belongs to the slot, not a line, so it is only read from line 0.
 * Setting line 0 via display_set_text() clears line 1, so a single-line
 * producer cannot strand a stale second line. */
void display_set_text_line(uint8_t line, uint8_t icon, const char *s, uint8_t len);
/* The slot's icon alone, its lines kept; a no-op when no text is shown. */
void display_set_text_icon(uint8_t icon);
void display_clear_text(void);
/* Read-only views of the text slot and the playback state (display.c), for
 * tests: no drawing, no state change. */
uint8_t display_get_text_line(uint8_t line, char *out, uint8_t max, uint8_t *icon, bool *present);
/* The shown title's lease left, in ms; 0 when none is shown or it is spent. */
uint32_t display_text_lease_left(void);
void    display_get_playback(bool *active, bool *ticking, uint16_t *pos, uint16_t *dur);

/* Draw the bootloader notice. Called just before the MCU reset that enters the
 * ROM bootloader; the panel retains the image across that reset. */
void display_bootloader_splash(void);

/* --- Fn+D debug page ---------------------------------------------------------
 * Full-panel diagnostics, toggled from process_record_kb.
 *
 * For UNTETHERED use: with the cable attached ak820health.py reads all of this
 * over raw HID in any slider position. This is the readout when no host is
 * attached -- and BT -> cable is a brownout reset, so plugging in to look
 * destroys the wireless session's counters. Toggling takes the panel over and
 * hands it back with a full dashboard repaint; refuses while the animation
 * player owns the bus. */
void display_debug_toggle(void);
bool display_debug_active(void);
/* A message was first painted in the band (an overlay, an alert, a link state
 * with words, new host text, the pair gesture) within the last `ms`: power.c's
 * Minimal mode lights the backlight for it. */
bool display_message_recent(uint32_t ms);
/* The band holds, or is about to show, an overlay, an alert or the pair
 * gesture, or another page owns the panel: a parameter overlay posted now
 * would replace one, or expire behind it unseen. power.c's low notice waits
 * for this to clear. */
bool display_band_busy(void);
/* Nothing queued for the panel and no transfer in flight (a sleep's
 * precondition); and the backlight pin driven low (a stopped 20 kHz tick
 * leaves it where it was). */
bool display_bus_idle(void);
/* A deep sleep's wake (sleep.c; plans/DEEP-SLEEP-BUILD-PLAN.md, section 4):
 * host content expired and the backlight held dark at the wake; the clock and
 * the battery row scheduled to repaint once corrected; the hold released by
 * sleep.c when they have (or at its bound). A pass in which rtc.c's recovery
 * used the PCF defers the rest of the display's work (display_defer_pass). */
void display_sleep_wake(void);
void display_wake_corrected(void);
bool display_wake_repainted(void);
void display_wake_release(void);
void display_defer_pass(void);
bool display_pass_deferred(void);
void display_backlight_pin_off(void);
/* Recompose the page on the next tick without waiting for the second edge, so a
 * counter reset shows its zeros immediately. No-op when the page is closed. */
void display_debug_refresh(void);

/* Playback position, drawn in place of the clock while media is PLAYING.
 * pos/dur are whole seconds; state 0 hands the band back to the clock.
 * The firmware advances pos itself once per second, so the host only has to
 * re-assert it occasionally -- see display_playback_tick(). */
void display_set_playback(uint8_t state, uint16_t pos, uint16_t dur);
/* The same with its own expiry: the lock-LED channel's, which refreshes far
 * less often than raw HID's poll (ch582f_ajazz.c). */
void display_set_playback_for(uint8_t state, uint16_t pos, uint16_t dur, uint32_t timeout_ms);
void display_playback_tick(void);
void display_playback_key(void);

/* Hold-to-pair feedback. A slot key was pressed and the pair threshold has not
 * elapsed yet, so the band says the hold is in progress. Without it the hold is
 * silent until it fires, and being 100ms short is indistinguishable from the
 * feature not working -- which reads as "the hold must be ~3s". */
void display_set_pair_hint(int16_t pct);
/* Boot alert: shown at near-top band priority for ~60 s (e.g. "WDT reset x1"). */
void display_set_alert(const char *msg);   /* 0-100 = progress, <0 = off */

#ifdef PARAM_OVERLAY
/* Transient parameter readout for the info band. Pass a string to show it for
 * PARAM_OVERLAY_HOLD_MS, or NULL to clear early. The CALLER owns the formatting;
 * display.c only stores and renders, which keeps the RGB API out of the graphics
 * layer and lets any producer use the slot. */
void display_set_param_status(const char *s);
#endif

/* Number of backlight steps (levels are 0..this). Perceptually spaced, so the
 * index is the meaningful figure, not the duty percentage. */
uint8_t display_get_brightness_max(void);

/* Flip play<->pause immediately on a local keypress; the next host update
 * corrects it. No-op when no transport icon is showing. */
void display_toggle_play_icon(void);

void display_draw_mac_logo(void);
void display_draw_windows_logo(void);
void display_draw_usb_logo(void);
void display_draw_bluetooth_logo(void);
void display_draw_2_4_g_logo(void);

// General blink phase for the LCD: true during the first half of each period_ms
// cycle (period_ms==0 => always on). Gate any drawn element on it to make it blink.
bool display_blink(uint16_t period_ms);
