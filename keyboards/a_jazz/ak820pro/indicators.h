// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Indicator LEDs (Caps / Win Lock / Charging) + the 20 kHz CT16B3 software
 * PWM tick they and the LCD backlight run on, and the lock-state accessors
 * the LCD indicator band reads. QMK hooks led_update_kb / layer_state_set_kb
 * live in indicators.c. */

/* Start the CT16B3 (GPTD4) 20 kHz PWM tick. Call once from post-init. */
void pwm_tick_init(void);

/* 10 Hz housekeeping: refresh charging + Win-lock state from the pins/flags. */
void update_leds(void);

/* Lock states for the LCD indicator band. */
bool lock_state_fn(void);
bool lock_state_caps(void);
bool lock_state_gui(void);
bool lock_state_scroll(void);   /* settled: see scr_shown in indicators.c */
/* This board's KC_SCRL was pressed: the host's answer may show at once. */
void lock_note_scroll_key(void);
bool charge_is_charging(void);
/* True when there is no battery fitted: on USB power, module reporting 0, and
 * the charger has never held a charge long enough to prove a pack exists. See
 * the reasoning block in indicators.c -- it latches once a pack is confirmed. */
bool battery_is_absent(void);

uint8_t indicator_get_brightness(void);
void    indicator_set_brightness(uint8_t level);
