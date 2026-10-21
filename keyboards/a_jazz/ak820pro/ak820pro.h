// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"
#include "watchdog_record.h"

/* Layer indices, shared between the keymaps and board code. Historically the
 * keymaps owned this enum and indicators.c hand-wrote the Fn mask as
 * (1<<1)|(1<<3) -- a silent-corruption coupling if layers were ever
 * rearranged. The mac/win dip switch selects the base layer, so per-key
 * remaps usually need doing on both bases. */
enum ak820pro_layers {
    WINBASE = 0,
    WINFN,
    MACBASE,
    MACFN,
};

// Keyboard-level custom keycodes handled in process_record_kb(). Based at
// QK_KB_0 (not SAFE_RANGE/QK_USER) so their values line up with VIA, which maps
// its customKeycodes[] to the QK_KB_0 range -- otherwise a keycode reassigned in
// VIA is stored as 0x7E00+i but the firmware's case is 0x7E40+i and never fires.
enum ak820pro_keycodes {
    SCR_TOG = QK_KB_0,     // toggle LCD backlight
    BT1,                   // Fn+Q: BT slot 1 (BT mode)
    BT2,                   // Fn+W: BT slot 2 (BT mode)
    BT3,                   // Fn+E: BT slot 3 (BT mode)
    BT24G,                 // Fn+R: 2.4G       (2.4G mode)
    BT_PAIR,               /* Pair the CURRENTLY-SELECTED profile (A6 51).
                            * UNBOUND in the default keymap as of 2026-08-29 --
                            * kept here because this enum is INDEX-MATCHED to
                            * via.json's customKeycodes[]; removing it would shift
                            * every later keycode and corrupt existing VIA keymaps.
                            * Assign it in VIA if you want it back.
                            *
                            * Dropped because it is redundant where it works and
                            * destructive where it is unique: holding Fn+Q/W/E
                            * already selects AND pairs a BT slot, while in 2.4G it
                            * is the only pairing key but drops a working dongle
                            * link -- and the dongle almost certainly cannot be put
                            * into pairing mode without vendor software, so the
                            * broadcast is half a handshake that never completes.
                            * Confirmed on hardware: pressing it killed the link,
                            * and toggling the slider restored it, which means the
                            * dongle never lost its bond. */
    // RGB-matrix control wrappers, exposed to VIA as custom keycodes. VIA's
    // built-in "Lighting" picker only offers the underglow RGB_* keycodes
    // (0x7820), which do nothing on this matrix-only board; these forward to the
    // rgb_matrix_* API so they can be assigned from VIA's Custom tab.
    RGBM_TOG,              // toggle
    RGBM_MOD,              // next effect
    RGBM_RMOD,             // previous effect
    RGBM_HUI,              // hue +
    RGBM_HUD,              // hue -
    RGBM_SAI,              // sat +
    RGBM_SAD,              // sat -
    RGBM_VAI,              // brightness +
    RGBM_VAD,              // brightness -
    RGBM_SPI,              // speed +
    RGBM_SPD,              // speed -
    ANIM_TOG,              // toggle the flash-animation player
    // NOTE: order here is index-matched to via.json customKeycodes[]. Append
    // only; inserting shifts every later keycode and silently breaks VIA maps.
    SCR_UP,                // LCD backlight brighter
    SCR_DN,                // LCD backlight dimmer
    CLK_MODE,              // Fn+C: clock band format, 24h -> 12h (AM/PM) -> off
    DBG_PAGE,              /* Fn+D: full-panel diagnostics. The panel is the only
                            * readout that works wirelessly -- raw HID needs the
                            * cable position, and BT -> cable brownout-resets the
                            * board, so a wireless session's counters die exactly
                            * when they become readable. */
    PWR_MODE,              // Fn+B: power mode, Always on -> Balanced -> Power saver -> Lights out (power.c)
    AK820PRO_SAFE_RANGE
};

/* Loop-stall attribution (see loop_gap_task in ak820pro.c). Which long
 * operation was in progress when the main loop stalled -- the three suspects
 * need entirely different fixes, so the probe reports the culprit, not just
 * the duration. */
#define LOOP_MARK_NONE  0
#define LOOP_MARK_FLASH 1
#define LOOP_MARK_BLIT  2
#define LOOP_MARK_I2C   3
extern volatile uint8_t loop_stall_mark;

/* Loop-stall SITE profiler (LOOPGAP_INSTRUMENT only; the macro is a plain
 * call otherwise). The mark above says which WAIT a slow pass ended in, not
 * which caller spent the time -- a 48 ms pass that ends in a 2 ms clear is
 * still reported as "blit". Each housekeeping sub-task is timed here and the
 * worst one per second is printed beside the gap. begin() closes any site
 * still open.
 *
 * ⚠️ 10 Hz-BLOCK SITES ONLY. The first version also wrapped the four
 * per-pass calls (rgb repeat, rtc_fast, second-edge, pump) and the RGB
 * flush gate; measured on hardware that dropped the scan rate from ~270 Hz
 * to ~175 Hz with the console attached -- every pass over 4 ms, and the
 * user felt dropped keystrokes. An instrument that costs 2 ms a pass is
 * the fault it is looking for. Sites inside the 100 ms tick run ten times a
 * second and are free. */
enum loop_site {
    LOOP_SITE_NONE = 0,
    LOOP_SITE_LEDS, LOOP_SITE_PAIR, LOOP_SITE_CONSUMER, LOOP_SITE_PARAM,
    LOOP_SITE_RTCTASK, LOOP_SITE_ANIM, LOOP_SITE_CONN, LOOP_SITE_STATUS,
    LOOP_SITE_TEXT, LOOP_SITE_LOCKS, LOOP_SITE_CLOCKF, LOOP_SITE_CLOCK,
    LOOP_SITE_BATT, LOOP_SITE_EECFG, LOOP_SITE_HEALTH, LOOP_SITE_RGBFLUSH,
    LOOP_SITE_COUNT
};
/* hid_protocol.c: raw HID (USB or the 2.4G dongle) carried an accepted media
 * command less than `ms` ago (the lock-LED channel's lease, R3-2). */
bool hid_rawhid_media_within(uint32_t now, uint32_t ms);
/* hid_protocol.c: raw HID set the clock (an accepted set) at most `s` seconds
 * ago, in rtc_get_seconds() (slept time included): the lock-LED clock's owner
 * rule. */
bool hid_rawhid_clock_within(uint32_t now_s, uint32_t s);

#ifdef LOOPGAP_INSTRUMENT
void loop_site_begin(uint8_t site);
void loop_site_end(void);
#    define LOOP_SITE(site, call) do { WDT_SCOPE(WDT_SITE_LOOP_BASE + (site)); loop_site_begin(site); call; loop_site_end(); } while (0)
#else
#    define LOOP_SITE(site, call) WDT_CALL(WDT_SITE_LOOP_BASE + (site), call)
#endif
