// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Power caps below the user's settings. The idle ladder (compiled in only with
 * POWER_LADDER_ENABLE): on battery, step down through progressively lower power
 * states as the board sits unused, and restore everything on the next keypress
 * or knob turn. The lights cut: the low-battery protection's RGB-off, held
 * here so it composes with the ladder. See power.c. */

typedef enum {
    POWER_STAGE_ACTIVE = 0,
    POWER_STAGE_DIM,          /* RGB at a quarter */
    POWER_STAGE_LIGHTS_OFF,   /* RGB dark, screen dimmed */
    POWER_STAGE_SCREEN_OFF,   /* RGB dark, screen backlight off */
} power_stage_t;

/* 10 Hz housekeeping. A no-op unless POWER_LADDER_ENABLE. */
void power_task(void);

power_stage_t power_stage(void);

/* RGB off as a cap, whatever the ladder or the user's toggle says. */
void power_set_lights_cut(bool cut);
bool power_lights_cut(void);
