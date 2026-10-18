// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The power caps: the idle ladder and its modes, and the low-battery lights
 * cut. See power.h; the design is plans/PHASE2-BUILD-PLAN.md, section 4.
 *
 * Why: nothing ever turned anything off. The RGB matrix and the LCD backlight
 * ran at the user's settings for as long as the board had power, touched or
 * not, and the MCU never sleeps (config.h, CORTEX_ENABLE_WFI_IDLE). Measured
 * at JD's settings (R1, 10-15): the RGB is 189 of the board's 306 mW, and the
 * keyboard sits idle ~88% of the time.
 *
 * Every stage is a CAP below the user's settings, never a change to them: the
 * dim through a ceiling on the lighting's own value (rgb_matrix_hsv_to_rgb,
 * below), the off stages through the driver's power scale
 * (sn32f2xx_set_power_scale), the screen through a backlight ceiling
 * (display_set_backlight_cap). So nothing reaches the EEPROM, VIA's view of
 * the lighting is unchanged, and the parameter overlay -- which polls the
 * rgb_matrix config -- does not fire. It is also why the low-battery cut lives
 * here: as a cap, the user's own RGB toggle cannot turn the lights back on
 * underneath it, which rgb_matrix_disable_noeeprom() allowed.
 *
 * Battery only (battery_on_battery): with the cable in, every mode is Full
 * Power. The wake is the keypress itself (or the knob), which also types as
 * normal; the lights follow within one 10 Hz tick.
 *
 * Not here yet: light sleep, the radio, deep sleep. Key scanning lives in the
 * row ISR, so sleeping the MCU needs a wake path of its own. */
#include "quantum.h"
#include "power.h"
#include "battery.h"
#include "kb_eeconfig.h"
#include "graphics/display.h"
#include "drivers/led/sn32f2xx.h"

#define NEVER UINT32_MAX

/* Each mode's idle times, ms: the RGB to the dim cap, the RGB off with the
 * screen capped at SCREEN_DIM_LEVEL, the screen's backlight off. */
typedef struct {
    uint32_t dim_ms, lights_off_ms, screen_off_ms;
} ladder_t;

static const ladder_t ladders[POWER_MODE_COUNT] = {
    [POWER_MODE_FULL]   = {NEVER, NEVER, NEVER},
    [POWER_MODE_NORMAL] = {60000u, 300000u, 900000u},    /* 1, 5, 15 min */
    [POWER_MODE_SAVER]  = {30000u, 120000u, 300000u},    /* 30 s, 2, 5 min */
    [POWER_MODE_MINIMAL] = {0u, 0u, 0u},                 /* dark at once, even typing */
};

/* Minimal mode's dark screen lights, at the user's level, for this long after
 * a message appears in the band, and while Fn+D's pages are open: otherwise a
 * key like Fn+B would seem to do nothing. */
#define MESSAGE_WAKE_MS 4000u

/* The dim: the lighting's value capped here, so 100% becomes 25% and a colour
 * keeps its hue (the driver's per-channel scale turned a dim purple red). */
#define DIM_VAL            64u
#define SCREEN_DIM_LEVEL    4u   /* of the 0..23 backlight levels; a ceiling */
#define BACKLIGHT_NO_CAP 0xFFu
/* Below this level on battery, the lighting follows Power Saver's stages
 * whatever the mode (BATTERY-GAUGE-PLAN.md, "Low battery"). Lighting only:
 * the radio is never the gauge's to touch. Its notice is given once a
 * discharge and re-arms on a real charging session, as the battery warning
 * does (codex). */
#define LOW_CAP_PM        150u
/* A mode left unset (a flash erased the EEPROM) is announced this long after
 * boot, when the panel is up, or once the band is free (a boot alert holds it
 * 60 s, and behind it the announcement would expire unseen). */
#define ANNOUNCE_AFTER_MS 3000u

static power_stage_t stage      = POWER_STAGE_ACTIVE;
static power_mode_t  mode       = POWER_MODE_NORMAL;
static bool          lights_cut = false;
static bool          low_noted  = false;   /* the low cap's notice given (or owed) */
static bool          low_owed   = false;   /* ... and waiting for a lit screen */
static uint16_t      low_sessions = 0;     /* battery_session_count() when it was noted */
static bool          announce   = false;
static uint8_t       val_cap    = 255;
#ifdef AK820_DEV_HOOKS
static uint8_t forced = 0xFF;   /* HC_DEVHOOK op 14: a stage held, 0xFF none */
#endif

/* The overlay holds 12 characters (display.c, CONN_STATUS_MAX). */
static const char *const mode_names[POWER_MODE_COUNT] = {
    [POWER_MODE_FULL]   = "Full power",
    [POWER_MODE_NORMAL] = "Normal power",
    [POWER_MODE_SAVER]  = "Power saver",
    [POWER_MODE_MINIMAL] = "Minimal mode",
};

static uint8_t applied_scale = 255;
static bool    shown_lit     = false;   /* Minimal's backlight lifted for a message */

static void apply(void) {
    uint8_t scale = 255, cap = BACKLIGHT_NO_CAP, vcap = 255;
    switch (stage) {
        case POWER_STAGE_ACTIVE:
            break;
        case POWER_STAGE_DIM:
            vcap = DIM_VAL;
            break;
        case POWER_STAGE_LIGHTS_OFF:
            scale = 0;
            cap   = SCREEN_DIM_LEVEL;
            break;
        case POWER_STAGE_SCREEN_OFF:
            scale = 0;
            cap   = shown_lit ? BACKLIGHT_NO_CAP : 0;
            break;
    }
    if (lights_cut) scale = 0;
    bool repaint = (vcap != val_cap) || (scale != applied_scale);
    val_cap       = vcap;
    applied_scale = scale;
    sn32f2xx_set_power_scale(scale);
    display_set_backlight_cap(cap);
    /* Every LED through the new caps at once: the driver's scale applies only
     * as an LED is set, and the value cap only as a colour is converted, and
     * an effect that rewrites a few LEDs a frame (Jellybean Raindrops) would
     * leave the rest as they were (codex). Re-running the effect's init pass
     * makes it paint them all (rgb_matrix_reinit_effect, a local core hook:
     * rgb_matrix_mode_noeeprom on the same mode does not set init). No
     * config change, so no EEPROM write and no parameter overlay; an effect's
     * own state restarts (Typing Heatmap's history), at each stage change. */
    /* Even while disabled: a quick disable, cap change and enable could
     * otherwise find the effect's trackers unchanged and skip the init (codex,
     * verify). Harmless while disabled: it renders nothing. */
    if (repaint) rgb_matrix_reinit_effect();
}

/* The dim stage's ceiling on every colour the effects convert, QMK's weak hook
 * (quantum/rgb_matrix/rgb_matrix.c). Every effect this board enables goes
 * through it, directly or through effect_runner_i; DRIFT's cached palette
 * keys on power_val_cap() so it rebuilds when the cap moves. */
rgb_t rgb_matrix_hsv_to_rgb(hsv_t hsv) {
    if (hsv.v > val_cap) hsv.v = val_cap;
    return hsv_to_rgb(hsv);
}

uint8_t power_val_cap(void) {
    return val_cap;
}

void power_set_lights_cut(bool cut) {
    if (cut == lights_cut) return;
    lights_cut = cut;
    apply();
}

bool power_rgb_dark(void) {
    return applied_scale == 0;
}

bool power_lights_cut(void) {
    return lights_cut;
}

void power_init(void) {
    uint8_t m;
    if (kb_eeconfig_get_power_mode(&m) && m < POWER_MODE_COUNT) {
        mode = (power_mode_t)m;
    } else {
        mode     = POWER_MODE_NORMAL;
        announce = true;   /* "says so": a flash reset it */
    }
}

/* The stage the idle time asks for under ladder `l`. */
static power_stage_t stage_for(const ladder_t *l, uint32_t idle) {
    if (idle >= l->screen_off_ms) return POWER_STAGE_SCREEN_OFF;
    if (idle >= l->lights_off_ms) return POWER_STAGE_LIGHTS_OFF;
    if (idle >= l->dim_ms) return POWER_STAGE_DIM;
    return POWER_STAGE_ACTIVE;
}

void power_task(void) {
    if (announce && timer_read32() >= ANNOUNCE_AFTER_MS && !display_band_busy()) {
        announce = false;
        kb_eeconfig_set_power_mode((uint8_t)mode);
        display_set_param_status(mode_names[mode]);
    }
    power_stage_t want  = POWER_STAGE_ACTIVE;
    bool          low   = false;
    uint16_t      level = battery_level_permille();
    if (battery_on_battery()) {
        uint32_t idle = last_input_activity_elapsed();
        low           = level <= LOW_CAP_PM;   /* LEVEL_UNKNOWN (0xFFFF) is not low */
        want          = stage_for(&ladders[mode], idle);
        if (low) {
            power_stage_t saver = stage_for(&ladders[POWER_MODE_SAVER], idle);
            if (saver > want) want = saver;
        }
    }
    /* The low cap's notice: once a discharge, and only where it can be read --
     * with the screen dark it waits for it to light, and it lapses if the cap
     * does (USB). A real charge re-arms it. Never in Minimal, which has
     * nothing left to cap; one owed from another mode waits there, and is
     * given after the next mode's own overlay. */
    if (low_noted && battery_session_count() != low_sessions) low_noted = false;
    if (low && !low_noted && mode != POWER_MODE_MINIMAL) {
        low_noted    = true;
        low_owed     = true;
        low_sessions = battery_session_count();
    }
    if (!low) low_owed = false;
#ifdef AK820_DEV_HOOKS
    if (forced != 0xFF) want = (power_stage_t)forced;
#endif
    bool lit = mode == POWER_MODE_MINIMAL &&
               (display_message_recent(MESSAGE_WAKE_MS) || display_debug_active());
    if (want != stage || lit != shown_lit) {
        stage     = want;
        shown_lit = lit;
        apply();
    }
    /* Only into a free band: over the mode Fn+B just picked it would replace
     * that before it could be read, and behind an alert it would expire
     * unseen and be lost. */
    if (low_owed && mode != POWER_MODE_MINIMAL && display_backlight_effective() > 0 &&
        !display_band_busy()) {
        low_owed = false;
        display_set_param_status("Saving power");
    }
}

power_stage_t power_stage(void) {
    return stage;
}

power_mode_t power_mode(void) {
    return mode;
}

void power_cycle_mode(void) {
    mode     = (power_mode_t)((mode + 1u) % POWER_MODE_COUNT);
    announce = false;
    kb_eeconfig_set_power_mode((uint8_t)mode);
    display_set_param_status(mode_names[mode]);
}

#ifdef AK820_DEV_HOOKS
void power_set_mode(power_mode_t m) {
    if (m >= POWER_MODE_COUNT) return;
    mode     = m;
    announce = false;
    kb_eeconfig_set_power_mode((uint8_t)mode);
}

void power_force_stage(uint8_t s) {
    forced = (s <= POWER_STAGE_SCREEN_OFF) ? s : 0xFF;
}

uint8_t power_forced_stage(void) {
    return forced;
}
#endif
