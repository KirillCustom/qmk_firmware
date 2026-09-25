// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The idle ladder. See power.h.
 *
 * Why: nothing ever turned anything off. The RGB matrix and the LCD backlight
 * ran at the user's settings for as long as the board had power, touched or
 * not, and the MCU never sleeps (config.h, CORTEX_ENABLE_WFI_IDLE). The lights
 * are most of the draw, so they go first.
 *
 * Every stage is a CAP below the user's settings, never a change to them: the
 * RGB through the driver's power scale (sn32f2xx_set_power_scale), the screen
 * through a backlight ceiling (display_set_backlight_cap). So nothing reaches
 * the EEPROM, VIA's view of the lighting is unchanged, and the parameter
 * overlay -- which polls the rgb_matrix config -- does not fire.
 *
 * Battery only, judged by VDD (battery_on_battery): with the cable in, the
 * board draws from USB and stays exactly as configured. The wake is the
 * keypress itself, which also types as normal; the lights follow within one
 * 10 Hz tick.
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

static power_stage_t stage = POWER_STAGE_ACTIVE;

static void apply(power_stage_t s) {
    switch (s) {
        case POWER_STAGE_ACTIVE:
            sn32f2xx_set_power_scale(255);
            display_set_backlight_cap(BACKLIGHT_NO_CAP);
            break;
        case POWER_STAGE_DIM:
            sn32f2xx_set_power_scale(DIM_SCALE);
            display_set_backlight_cap(BACKLIGHT_NO_CAP);
            break;
        case POWER_STAGE_LIGHTS_OFF:
            sn32f2xx_set_power_scale(0);
            display_set_backlight_cap(SCREEN_DIM_LEVEL);
            break;
        case POWER_STAGE_SCREEN_OFF:
            sn32f2xx_set_power_scale(0);
            display_set_backlight_cap(0);
            break;
    }
    stage = s;
}

void power_task(void) {
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
    if (want != stage) apply(want);
}

power_stage_t power_stage(void) {
    return stage;
}
