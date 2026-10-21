// Copyright 2026 KirillCustom
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

/* The stock firmware's 19 lighting modes, ported from the V1.13 image
 * (docs/STOCK-EFFECTS-RE.md in the AjazzTimeSync project). Each mode is a
 * thin RGB_MATRIX_EFFECT in rgb_matrix_kb.inc that calls stock_fx_run(). */

#include <stdint.h>
#include <stdbool.h>
#include "rgb_matrix.h"

typedef enum {
    SFX_STATIC,
    SFX_SINGLE_ON,
    SFX_SINGLE_OFF,
    SFX_GLITTER,
    SFX_FALLING,
    SFX_COLOURFUL,
    SFX_BREATH,
    SFX_SPECTRUM,
    SFX_OUTWARD,
    SFX_SCROLL_DOWN,
    SFX_SCROLL_UP,
    SFX_ROLL_RIGHT,
    SFX_ROLL_LEFT,
    SFX_ROTATING,
    SFX_EXPLODE,
    SFX_LAUNCH,
    SFX_RIPPLES,
    SFX_FLOW_FWD,
    SFX_FLOW_BACK,
    SFX_PULSATING,
    SFX_TILT_RIGHT,
    SFX_TILT_LEFT,
    SFX_SHUTTLE,
    SFX_COUNT
} sfx_id_t;

/* The effect body: init on params->init, advance by wall time, draw the chunk. */
bool stock_fx_run(sfx_id_t id, effect_params_t *params);

/* Every matrix key event, from process_record_kb. Ignored unless a stock
 * reactive mode is the running effect. */
void stock_fx_key(uint8_t row, uint8_t col, bool pressed);

/* "Rainbow" = the stock colour index 8: per-mode rainbow variants and a random
 * colour per lit key. Persisted in the kb datablock. */
bool stock_fx_get_rainbow(void);
void stock_fx_set_rainbow(bool on);

/* The stock mode list (stock order, 1-based like the stock protocol, 0 = Off). */
#define STOCK_MODE_COUNT 20
uint8_t     stock_mode_of(uint8_t rgb_mode);          // rgb_matrix mode -> stock number, 0xFF if not stock
uint8_t     stock_mode_rgb(uint8_t stock);            // stock number -> rgb_matrix mode (first direction)
void        stock_mode_step(bool reverse);            // Fn+\: next stock mode, skipping Off
bool        stock_mode_has_direction(uint8_t rgb_mode);
uint8_t     stock_mode_flip_direction(uint8_t rgb_mode); // the other direction's rgb_matrix mode
const char *stock_mode_name(uint8_t rgb_mode);        // short Latin name, NULL if not stock
