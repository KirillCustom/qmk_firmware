// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The power caps: the idle ladder, and the low-battery lights cut. See power.h.
 *
 * Why: nothing ever turned anything off. The RGB matrix and the LCD backlight
 * ran at the user's settings for as long as the board had power, touched or
 * not, and the MCU never sleeps (config.h, CORTEX_ENABLE_WFI_IDLE).
 *
 * Every stage is a CAP below the user's settings, never a change to them: the
 * RGB through the driver's power scale (sn32f2xx_set_power_scale), the screen
 * through a backlight ceiling (display_set_backlight_cap). So nothing reaches
 * the EEPROM, VIA's view of the lighting is unchanged, and the parameter
 * overlay -- which polls the rgb_matrix config -- does not fire. It is also why
 * the low-battery cut lives here: as a cap, the user's own RGB toggle cannot
 * turn the lights back on underneath it, which rgb_matrix_disable_noeeprom()
 * allowed.
 *
 * ⚠️ The idle ladder is COMPILED OUT unless POWER_LADDER_ENABLE is defined.
 * It changes the load as the board sits idle, which would corrupt the battery
 * calibration run (plans/BATTERY-GAUGE-PLAN.md), and which of its stages pays
 * for itself is unmeasured. The lights cut is always built.
 *
 * Ladder, when enabled: battery only (battery_on_battery), so with the cable in
 * the board stays exactly as configured. The wake is the keypress itself, which
 * also types as normal; the lights follow within one 10 Hz tick.
 *
 * Deep sleep -- stopping the row ISR and sleeping the MCU -- is the next stage
 * and not here yet: key scanning lives in that ISR, so it needs a wake path of
 * its own. */
#include "quantum.h"
#include "power.h"
#include "battery.h"
#include "graphics/display.h"
#include "drivers/led/sn32f2xx.h"

#ifndef POWER_DIM_MS
#    define POWER_DIM_MS 60000u          /* 1 min: RGB to a quarter */
#endif
#ifndef POWER_LIGHTS_OFF_MS
#    define POWER_LIGHTS_OFF_MS 300000u  /* 5 min: RGB off, screen dimmed */
#endif
#ifndef POWER_SCREEN_OFF_MS
#    define POWER_SCREEN_OFF_MS 900000u  /* 15 min: screen backlight off */
#endif
#define DIM_SCALE          64u   /* of 255 */
#define SCREEN_DIM_LEVEL    4u   /* of the 0..23 backlight levels; a ceiling */
#define BACKLIGHT_NO_CAP 0xFFu

static power_stage_t stage      = POWER_STAGE_ACTIVE;
static bool          lights_cut = false;

static void apply(void) {
    uint8_t scale = 255, cap = BACKLIGHT_NO_CAP;
    switch (stage) {
        case POWER_STAGE_ACTIVE:
            break;
        case POWER_STAGE_DIM:
            scale = DIM_SCALE;
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
    sn32f2xx_set_power_scale(scale);
    display_set_backlight_cap(cap);
}

void power_set_lights_cut(bool cut) {
    if (cut == lights_cut) return;
    lights_cut = cut;
    apply();
}

bool power_lights_cut(void) {
    return lights_cut;
}

void power_task(void) {
#ifdef POWER_LADDER_ENABLE
    power_stage_t want = POWER_STAGE_ACTIVE;
    if (battery_on_battery()) {
        uint32_t idle = last_input_activity_elapsed();
        if (idle >= POWER_SCREEN_OFF_MS) {
            want = POWER_STAGE_SCREEN_OFF;
        } else if (idle >= POWER_LIGHTS_OFF_MS) {
            want = POWER_STAGE_LIGHTS_OFF;
        } else if (idle >= POWER_DIM_MS) {
            want = POWER_STAGE_DIM;
        }
    }
    if (want != stage) {
        stage = want;
        apply();
    }
#endif
}

power_stage_t power_stage(void) {
    return stage;
}
