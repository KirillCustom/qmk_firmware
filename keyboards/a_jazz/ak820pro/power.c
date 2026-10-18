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
};

/* The dim: the lighting's value capped here, so 100% becomes 25% and a colour
 * keeps its hue (the driver's per-channel scale turned a dim purple red). */
#define DIM_VAL            64u
#define SCREEN_DIM_LEVEL    4u   /* of the 0..23 backlight levels; a ceiling */
#define BACKLIGHT_NO_CAP 0xFFu
/* Below this level on battery, the lighting follows Power Saver's stages
 * whatever the mode (BATTERY-GAUGE-PLAN.md, "Low battery"). Lighting only:
 * the radio is never the gauge's to touch. */
#define LOW_CAP_PM        150u
/* A mode left unset (a flash erased the EEPROM) is announced this long after
 * boot, when the panel is up. */
#define ANNOUNCE_AFTER_MS 3000u

static power_stage_t stage      = POWER_STAGE_ACTIVE;
static power_mode_t  mode       = POWER_MODE_NORMAL;
static bool          lights_cut = false;
static bool          low_cap    = false;
static bool          announce   = false;
static uint8_t       val_cap    = 255;
#ifdef AK820_DEV_HOOKS
static uint8_t forced = 0xFF;   /* HC_DEVHOOK op 14: a stage held, 0xFF none */
#endif

static const char *const mode_names[POWER_MODE_COUNT] = {
    [POWER_MODE_FULL]   = "Power: Full",
    [POWER_MODE_NORMAL] = "Power: Normal",
    [POWER_MODE_SAVER]  = "Power: Saver",
};

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
            cap   = 0;
            break;
    }
    if (lights_cut) scale = 0;
    val_cap = vcap;
    sn32f2xx_set_power_scale(scale);
    display_set_backlight_cap(cap);
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
    if (announce && timer_read32() >= ANNOUNCE_AFTER_MS) {
        announce = false;
        kb_eeconfig_set_power_mode((uint8_t)mode);
        display_set_param_status(mode_names[mode]);
    }
    power_stage_t want = POWER_STAGE_ACTIVE;
    bool          low  = false;
    if (battery_on_battery()) {
        uint32_t idle  = last_input_activity_elapsed();
        uint16_t level = battery_level_permille();
        low            = level <= LOW_CAP_PM;   /* LEVEL_UNKNOWN (0xFFFF) is not low */
        want           = stage_for(&ladders[mode], idle);
        if (low) {
            power_stage_t saver = stage_for(&ladders[POWER_MODE_SAVER], idle);
            if (saver > want) want = saver;
        }
    }
    if (low && !low_cap) display_set_param_status("Low: saving power");
    low_cap = low;
#ifdef AK820_DEV_HOOKS
    if (forced != 0xFF) want = (power_stage_t)forced;
#endif
    if (want != stage) {
        stage = want;
        apply();
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
