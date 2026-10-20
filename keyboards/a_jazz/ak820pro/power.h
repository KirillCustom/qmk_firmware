// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Power caps below the user's settings. The idle ladder: on battery, step
 * down through progressively lower power states as the board sits unused, at
 * the times the power mode sets, and restore the mode's typing state on the
 * next keypress or knob turn. The lights cut: the low-battery protection's
 * RGB-off, held here so it composes with the ladder. See power.c. */

typedef enum {
    POWER_STAGE_ACTIVE = 0,   /* typing: the mode's ceiling (Power saver's 128) */
    POWER_STAGE_DIM,          /* RGB's value capped at 64, a quarter of full */
    POWER_STAGE_LIGHTS_OFF,   /* RGB dark, screen dimmed: only when the bench holds it */
    POWER_STAGE_SCREEN_OFF,   /* RGB dark, screen backlight off */
} power_stage_t;

/* The modes on Fn+B (PWR_MODE), persisted; Normal is the default. The macros
 * tell scripts/battery_sim which power.c it is building: POWER_HAS_BULB, the
 * names below and power_lights_mode() (plans/POWER-MODE-ICONS-PLAN.md). */
#define POWER_HAS_MODES 1
#define POWER_HAS_BULB  1
typedef enum {
    POWER_MODE_FULL = 0,      /* "Always on": the ladder never runs */
    POWER_MODE_NORMAL,        /* "Balanced": dim 10 min, dark and asleep 20 */
    POWER_MODE_SAVER,         /* "Power saver": value capped at 128 while typing, dim
                               * 1 min, dark and asleep 3 */
    POWER_MODE_MINIMAL,       /* "Lights out": RGB and backlight off on battery, typing
                               * or not; the backlight lights a moment for a message;
                               * asleep after 1 min */
    POWER_MODE_COUNT
} power_mode_t;

/* Post-init, after kb_eeconfig_init: the persisted mode. */
void power_init(void);

/* 10 Hz housekeeping. */
void power_task(void);

power_stage_t power_stage(void);
power_mode_t  power_mode(void);
/* The mode whose stages the lights follow: the mode, or Saver's while the
 * low cap holds Always on or Balanced to them. The battery row's bulb. */
power_mode_t  power_lights_mode(void);
/* Fn+B: the next mode, persisted, and shown as an overlay. */
void power_cycle_mode(void);

/* The deep sleep is wanted: on battery, not Full, idle past the mode's sleep
 * time (Saver's when the battery is low). sleep.c decides when it can. */
bool power_sleep_wanted(void);
/* A wake after a long sleep (sleep.c): say the time left in hours of use, if
 * it matters (under 10 h), once the band is free. */
void power_wake_notice(void);

/* The ceiling on the lighting's value now applied: 255, Power saver's 128
 * while typing (any mode's when the battery is low), or the dim's 64. */
uint8_t power_val_cap(void);

/* RGB off as a cap, whatever the ladder or the user's toggle says. */
void power_set_lights_cut(bool cut);
bool power_lights_cut(void);
/* The RGB is dark: a dark stage or the lights cut (the driver's scale 0). */
bool power_rgb_dark(void);

#ifdef AK820_DEV_HOOKS
/* The bench's (HC_DEVHOOK op 14): a mode without the key or the overlay, and
 * a stage held whatever the idle time or the supply (0xFF releases it). */
void    power_set_mode(power_mode_t m);
void    power_force_stage(uint8_t s);
uint8_t power_forced_stage(void);
/* A short sleep time for the bench (not persisted; 0 clears it; at least
 * 30 s), so the soak runs through the ladder's own path. */
void    power_set_bench_sleep_ms(uint32_t ms);
#endif
