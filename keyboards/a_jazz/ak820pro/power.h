// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Power caps below the user's settings. The idle ladder: on battery, step
 * down through progressively lower power states as the board sits unused, at
 * the times the power mode sets, and restore everything on the next keypress
 * or knob turn. The lights cut: the low-battery protection's RGB-off, held
 * here so it composes with the ladder. See power.c. */

typedef enum {
    POWER_STAGE_ACTIVE = 0,
    POWER_STAGE_DIM,          /* RGB at a quarter of its value */
    POWER_STAGE_LIGHTS_OFF,   /* RGB dark, screen dimmed */
    POWER_STAGE_SCREEN_OFF,   /* RGB dark, screen backlight off */
} power_stage_t;

/* The modes on Fn+B (PWR_MODE), persisted; Normal is the default. The macro
 * tells scripts/battery_sim which power.c it is building. */
#define POWER_HAS_MODES 1
typedef enum {
    POWER_MODE_FULL = 0,      /* the ladder never runs */
    POWER_MODE_NORMAL,        /* dim 1 min, RGB off 5, screen off 15 */
    POWER_MODE_SAVER,         /* dim 30 s, RGB off 2 min, screen off 5 */
    POWER_MODE_COUNT
} power_mode_t;

/* Post-init, after kb_eeconfig_init: the persisted mode. */
void power_init(void);

/* 10 Hz housekeeping. */
void power_task(void);

power_stage_t power_stage(void);
power_mode_t  power_mode(void);
/* Fn+B: the next mode, persisted, and shown as an overlay. */
void power_cycle_mode(void);

/* The dim stage's ceiling on the lighting's value: 255, or the cap. */
uint8_t power_val_cap(void);

/* RGB off as a cap, whatever the ladder or the user's toggle says. */
void power_set_lights_cut(bool cut);
bool power_lights_cut(void);

#ifdef AK820_DEV_HOOKS
/* The bench's (HC_DEVHOOK op 14): a mode without the key or the overlay, and
 * a stage held whatever the idle time or the supply (0xFF releases it). */
void    power_set_mode(power_mode_t m);
void    power_force_stage(uint8_t s);
uint8_t power_forced_stage(void);
#endif
