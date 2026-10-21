// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Moved verbatim from ak820pro.c in the phase-3 module split. */
#include "param_overlay.h"
#include "ak820pro.h"
#include "graphics/display.h"
#ifdef RGB_MATRIX_ENABLE
#    include "stock_fx.h"   /* stock_mode_name() */
#endif

/* ---- Hold-to-repeat for the adjust keys ----------------------------------
 *
 * QMK fires process_record once on press and once on release; there is no
 * firmware-side auto-repeat. This tracks the held key and re-invokes the same
 * step function from housekeeping_task_kb(), which runs every main-loop
 * iteration (~360-400 Hz here), so the timing is fine-grained.
 *
 * SAFETY: the held key is confirmed against the MATRIX each tick, not against
 * a release event. A release can resolve to a different keycode if the layer
 * changes mid-hold (Fn released first), which would otherwise strand the
 * repeat running forever. matrix_is_on() cannot lie about a physical key. */
static uint16_t rep_kc = KC_NO;
/* Set on every press of an adjust key, whether or not the value can still
 * move. At a range extreme nothing changes, so the polled param_status_task()
 * had nothing to report and the key looked dead -- the one case where feedback
 * matters MOST, because "already at the limit" and "this key does nothing" are
 * indistinguishable without it. */
static uint16_t param_force_kc = KC_NO;

static keypos_t rep_pos;
static uint32_t rep_since, rep_last;

static bool param_is_repeatable(uint16_t kc) {
    switch (kc) {
        case RM_HUEU: case RM_HUED:
        case RM_SATU: case RM_SATD:
        case RM_VALU: case RM_VALD:
        case RM_SPDU: case RM_SPDD:
        /* LCD backlight, same treatment: 24 levels is too many to tap through. */
        case SCR_UP:  case SCR_DN:
            return true;
        default:
            return false;
    }
}

static void param_repeat_step(uint16_t kc) {
    switch (kc) {
        case RM_HUEU: rgb_matrix_increase_hue();   break;
        case RM_HUED: rgb_matrix_decrease_hue();   break;
        case RM_SATU: rgb_matrix_increase_sat();   break;
        case RM_SATD: rgb_matrix_decrease_sat();   break;
        case RM_VALU: rgb_matrix_increase_val();   break;
        case RM_VALD: rgb_matrix_decrease_val();   break;
        case RM_SPDU: rgb_matrix_increase_speed(); break;
        case RM_SPDD: rgb_matrix_decrease_speed(); break;
        /* Unlike the rgb_matrix_* calls, which param_status_task() notices by
         * polling, the backlight has no polled state -- push the readout here
         * or the band would freeze on the first value of a sweep. */
        case SCR_UP:  display_brightness_up();   param_show_lcd_brightness(); break;
        case SCR_DN:  display_brightness_down(); param_show_lcd_brightness(); break;
        default: break;
    }
}

void param_show_lcd_brightness(void) {
#ifdef PARAM_OVERLAY
    /* Percentage of the LEVEL INDEX, not of the duty cycle. The levels are
     * perceptually spaced, so duty runs 2/4/6/8/10/12/15/17/19/21/23/25/29/33/
     * 38/42/48/54/60/69/77/88/100% and reads as erratic; the index divides
     * evenly and answers the question actually being asked -- how far up the
     * range am I. Buffer is oversized because the compiler cannot prove %u is
     * short (-Werror=format-truncation); the slot truncates to the band width. */
    char    buf[24];
    uint8_t lvl = display_get_brightness();
    uint8_t mx  = display_get_brightness_max();
    snprintf(buf, sizeof(buf), "LCD    %3u%%",
             (unsigned)(mx ? (lvl * 100u + mx / 2u) / mx : 0u));
    display_set_param_status(buf);
#endif
}

/* Arm/disarm the repeat. The caller returns through to QMK, which performs
 * the FIRST step itself -- this only handles the ones after the threshold. */
void param_repeat_process_record(uint16_t keycode, keyrecord_t *record) {
    if (param_is_repeatable(keycode)) {
        if (record->event.pressed) {
            rep_kc     = keycode;
            param_force_kc = keycode;    // show the readout even if pinned at an end
            rep_pos    = record->event.key;
            rep_since = timer_read32();
            rep_last  = timer_read32();
        } else if (keycode == rep_kc) {
            rep_kc = KC_NO;
        }
    } else if (record->event.pressed) {
        rep_kc = KC_NO;      // any other key cancels a hold in progress
    }
}

void param_repeat_task(void) {
    if (rep_kc == KC_NO) return;
    if (!matrix_is_on(rep_pos.row, rep_pos.col)) {   // physically released
        rep_kc = KC_NO;
        return;
    }
    uint32_t held = timer_elapsed32(rep_since);
    if (held < PARAM_REPEAT_DELAY_MS) return;
    /* Slow at first so a short hold nudges precisely, then fast so a full
     * traverse of 128 values does not take eight seconds. */
    uint16_t interval = (held > (PARAM_REPEAT_DELAY_MS + PARAM_REPEAT_FAST_AFTER_MS))
                            ? PARAM_REPEAT_FAST_MS
                            : PARAM_REPEAT_INTERVAL_MS;
    if (timer_elapsed32(rep_last) < interval) return;
    rep_last = timer_read32();
    param_repeat_step(rep_kc);
}

#ifdef PARAM_OVERLAY
/* Short effect names for the 12-character band.
 *
 * NOT rgb_matrix_get_mode_name(): that is gated behind RGB_MATRIX_MODE_NAME_ENABLE,
 * costs flash for all ~40 effect names, and returns the raw enum spelling --
 * "RAINBOW_MOVING_CHEVRON" is 22 characters against a 12-character band. Only 10
 * animations are enabled here (keyboard.json), so a hand-written table is both
 * smaller and far more readable. Each case is #ifdef'd on the effect's own enable
 * so this still builds if the animation list changes; anything unlisted falls
 * through to the mode number. */
static const char *rgb_mode_short(uint8_t mode) {
    switch (mode) {
        case RGB_MATRIX_NONE:                    return "Off";
#ifdef ENABLE_RGB_MATRIX_SOLID_COLOR
        case RGB_MATRIX_SOLID_COLOR:             return "Solid";
#endif
#ifdef ENABLE_RGB_MATRIX_ALPHAS_MODS
        case RGB_MATRIX_ALPHAS_MODS:             return "Alphas/Mods";
#endif
#ifdef ENABLE_RGB_MATRIX_BREATHING
        case RGB_MATRIX_BREATHING:               return "Breathing";
#endif
#if defined(RGB_MATRIX_CUSTOM_KB)
        case RGB_MATRIX_CUSTOM_RAINFALL:         return "Rainfall";
        case RGB_MATRIX_CUSTOM_DRIFT:            return "Drift";
#endif
#ifdef ENABLE_RGB_MATRIX_CYCLE_ALL
        case RGB_MATRIX_CYCLE_ALL:               return "Cycle All";
#endif
#ifdef ENABLE_RGB_MATRIX_CYCLE_LEFT_RIGHT
        case RGB_MATRIX_CYCLE_LEFT_RIGHT:        return "Cycle L-R";
#endif
#ifdef ENABLE_RGB_MATRIX_CYCLE_UP_DOWN
        case RGB_MATRIX_CYCLE_UP_DOWN:           return "Cycle U-D";
#endif
#ifdef ENABLE_RGB_MATRIX_CYCLE_PINWHEEL
        case RGB_MATRIX_CYCLE_PINWHEEL:          return "Pinwheel";
#endif
#ifdef ENABLE_RGB_MATRIX_RAINBOW_MOVING_CHEVRON
        case RGB_MATRIX_RAINBOW_MOVING_CHEVRON:  return "Chevron";
#endif
#ifdef ENABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS
        case RGB_MATRIX_JELLYBEAN_RAINDROPS:     return "Jellybean";
#endif
#ifdef ENABLE_RGB_MATRIX_TYPING_HEATMAP
        case RGB_MATRIX_TYPING_HEATMAP:          return "Heatmap";
#endif
        default:                                 return stock_mode_name(mode);   /* NULL if not stock */
    }
}

/* Poll the adjustable state and surface any change in the info band.
 *
 * POLLED, not hooked into process_record_kb: that catches the Fn hotkeys, the
 * RGBM_* custom keycodes, anything VIA changes AND the magic-key toggles, none
 * of which share a code path. A handful of byte comparisons on the 10 Hz tick.
 *
 * Percentages rather than raw 0-255 because the band is 12 characters and "53%"
 * is legible where "136" needs you to know the scale. Hue is the exception --
 * it is circular, so degrees are the meaningful unit. */

/* ALPHAS_MODS has no animation: it reuses rgb_matrix_config.speed as the hue
 * OFFSET of the second colour (alpha_mods_anim.h does `hsv.h += speed`). So the
 * speed keys are the second-colour dial, and reporting them as "Speed 50%"
 * describes the wrong quantity entirely. Report degrees off the base hue. */
static void fmt_speed(char *buf, size_t n, uint8_t sp) {
#ifdef ENABLE_RGB_MATRIX_ALPHAS_MODS
    if (rgb_matrix_get_mode() == RGB_MATRIX_ALPHAS_MODS) {
        snprintf(buf, n, "2nd  %+4d", (int)((sp * 360) / 256));
        return;
    }
#endif
    snprintf(buf, n, "Speed  %3u%%", (unsigned)((sp * 100u + 127u) / 255u));
}

void param_status_task(void) {
    static uint8_t last_mode = 0, last_h = 0, last_s = 0, last_v = 0, last_sp = 0;
    static bool    last_on = false, last_nkro = false;
    static bool    primed    = false;

    uint8_t mode = rgb_matrix_get_mode();
    uint8_t h    = rgb_matrix_get_hue();
    uint8_t sa   = rgb_matrix_get_sat();
    uint8_t v    = rgb_matrix_get_val();
    uint8_t sp   = rgb_matrix_get_speed();
    bool    on   = rgb_matrix_is_enabled();
    /* NKRO has NO other feedback anywhere on the board, and it is toggled by a
     * magic key (both shifts + N) that is easy to hit by accident. Finding out
     * what state it was in previously meant attaching a console -- see the Caps
     * Lock investigation in CLAUDE.md. */
    bool    nkro = keymap_config.nkro;

    /* First pass only records the baseline -- otherwise the band would flash a
     * readout at every boot for a change that never happened. */
    if (!primed) {
        primed = true;
        last_mode = mode; last_h = h; last_s = sa; last_v = v; last_sp = sp;
        last_on = on; last_nkro = nkro;
        return;
    }

    char buf[24];
    if (nkro != last_nkro) {
        snprintf(buf, sizeof(buf), "NKRO %s", nkro ? "On" : "Off");
    } else if (on != last_on) {
        /* Checked before the colour parameters: toggling RGB off leaves hue and
         * val untouched, so nothing else would report it -- and it explains why
         * the brightness keys appear to do nothing while it is off. */
        snprintf(buf, sizeof(buf), "RGB %s", on ? "On" : "Off");
    } else if (mode != last_mode) {
        const char *name = rgb_mode_short(mode);
        if (name) snprintf(buf, sizeof(buf), "%s", name);
        else      snprintf(buf, sizeof(buf), "Mode %u", (unsigned)mode);
    } else if (v != last_v) {
        snprintf(buf, sizeof(buf), "Bright %3u%%", (unsigned)((v * 100u + 127u) / 255u));
    } else if (h != last_h) {
        snprintf(buf, sizeof(buf), "Hue    %3u", (unsigned)((h * 360u) / 256u));
    } else if (sa != last_s) {
        snprintf(buf, sizeof(buf), "Sat    %3u%%", (unsigned)((sa * 100u + 127u) / 255u));
    } else if (sp != last_sp) {
        fmt_speed(buf, sizeof(buf), sp);
    } else if (param_force_kc != KC_NO) {
        /* Nothing moved, but an adjust key was pressed -- almost always because
         * the value is already at an end stop. Report the current value so the
         * key visibly does something and the limit is legible. */
        switch (param_force_kc) {
            case RM_HUEU: case RM_HUED:
                snprintf(buf, sizeof(buf), "Hue    %3u", (unsigned)((h * 360u) / 256u)); break;
            case RM_SATU: case RM_SATD:
                snprintf(buf, sizeof(buf), "Sat    %3u%%", (unsigned)((sa * 100u + 127u) / 255u)); break;
            case RM_VALU: case RM_VALD:
                snprintf(buf, sizeof(buf), "Bright %3u%%", (unsigned)((v * 100u + 127u) / 255u)); break;
            case RM_SPDU: case RM_SPDD:
                fmt_speed(buf, sizeof(buf), sp); break;
            default:
                param_force_kc = KC_NO;
                return;
        }
        param_force_kc = KC_NO;
    } else {
        return;
    }
    param_force_kc = KC_NO;

    last_mode = mode; last_h = h; last_s = sa; last_v = v; last_sp = sp;
    last_on = on; last_nkro = nkro;
    display_set_param_status(buf);
}
#endif // PARAM_OVERLAY
