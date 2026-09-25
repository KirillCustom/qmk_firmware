// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>

/* The idle ladder: on battery, step down through progressively lower power
 * states as the board sits unused, and restore everything on the next keypress
 * or knob turn. On USB power nothing changes. See power.c. */

typedef enum {
    POWER_STAGE_ACTIVE = 0,
    POWER_STAGE_DIM,          /* RGB at a quarter */
    POWER_STAGE_LIGHTS_OFF,   /* RGB dark, screen dimmed */
    POWER_STAGE_SCREEN_OFF,   /* RGB dark, screen backlight off */
} power_stage_t;

/* 10 Hz housekeeping. */
void power_task(void);

power_stage_t power_stage(void);
