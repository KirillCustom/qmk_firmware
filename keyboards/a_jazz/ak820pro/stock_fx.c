// Copyright 2026 KirillCustom
// SPDX-License-Identifier: GPL-2.0-or-later

/* The stock lighting engine, ported from the V1.13 image.
 *
 * The stock frame is COLOUR x MASK: every LED has a colour and a 0-255 mask,
 * and nearly every mode animates the mask, not the colour. A mode advances in
 * whole STEPS, one every speed_mul[speed] * period / 400 ticks of 0.5 ms; a
 * second, independent clock FADES every non-zero mask by 1 each 1.5 ms when the
 * mode wants a trail (255 to dark in 382 ms at any speed). Geometry is the
 * electrical matrix (6 rows x 15 columns, identical to QMK's), never x/y.
 *
 * Deliberate departures from the stock code, all fixes of stock bugs:
 * Ripples ORs a new ring into its columns (stock assigned, wiping rings already
 * running there), Tilt left cycles 6 rows like Tilt right (stock: 7, with a
 * dark row), and the random palette index never reads past the table.
 *
 * Cost per frame: one pass over 81 masks for the fade, the steps due (a few
 * byte-array passes each), and the chunk's 3 scale8s per LED. One divide per
 * frame, in the step interval; none per LED (Cortex-M0 has no divider). */

#include "stock_fx.h"
#include "stock_fx_tables.h"
#include "kb_eeconfig.h"
#include "power.h"   /* power_val_cap(): the power ladder's ceiling on the value knob */
#include "loop_acct.h"   /* ACCT_RGB: what the stock modes cost, on health's accounting page */
#include "timer.h"
#include "lib/lib8tion/lib8tion.h"
#include <string.h>

#define LEDS   RGB_MATRIX_LED_COUNT
#define NCOLS  17   /* stock column arrays; 15 and 16 have no LEDs */

static uint8_t cur = 0xFF;           /* running sfx_id_t */
static bool    cur_rainbow;          /* the rainbow flag the running mode was set up with */
static bool    rainbow;              /* the setting */
static bool    rainbow_loaded;

static uint8_t mask[LEDS];
static uint8_t aux[LEDS];            /* Single On/Off fade phase, or the LED's group in wave modes */
static rgb_t   pix[LEDS];            /* per-LED colour when per_led */
static bool    per_led;              /* pix[] holds the colours; else the live HSV colour */
static bool    fade_on;

static uint8_t bits[4][NCOLS];       /* row bitmasks per column: L R / A B C D / drops / pulse / tilt */
static uint8_t ph[31];               /* group phase: wave (mod 96) or rainbow (mod 192) */
static uint8_t ph2[NCOLS];           /* rainbow Shuttle's backward phases */
static uint8_t ngroups;
static uint8_t pos;                  /* ring / walk position / breath position */
static uint8_t preset;               /* Breath's rainbow preset */
static bool    up;                   /* Pulsating's source direction */
static bool    rb_wave;              /* the mode runs its rainbow (hue) variant */

static uint16_t period;              /* stock period, in units of the stock formula */
static uint32_t last_ms;
static uint16_t step_acc, fade_acc;  /* in 0.5 ms ticks */

/* ---------------------------------------------------------------- helpers */

static inline uint8_t led_at(uint8_t r, uint8_t c) {
    return (r < MATRIX_ROWS && c < MATRIX_COLS) ? g_led_config.matrix_co[r][c] : NO_LED;
}
static inline uint8_t led_cell(uint8_t cell) {
    return cell == 0xFF ? NO_LED : led_at(cell >> 4, cell & 15);
}
static inline uint8_t wrap192(uint16_t h) { return h >= 192 ? h - 192 : h; }
static inline rgb_t wheel(uint8_t h) {   /* stock channel phases: R = W[h+64], G = W[h+128], B = W[h] */
    return (rgb_t){sfx_wheel[wrap192(h + 64)], sfx_wheel[wrap192(h + 128)], sfx_wheel[h]};
}
static inline rgb_t from3(const uint8_t *p) { return (rgb_t){p[0], p[1], p[2]}; }
static inline rgb_t random_colour(void) {
    return from3(sfx_palette[((uint16_t)random8() * 106) >> 8]);
}

/* Light one cell: full mask, and a fresh random colour in rainbow. */
static void light_led(uint8_t i) {
    if (i == NO_LED) return;
    mask[i] = 255;
    if (cur_rainbow) pix[i] = random_colour();
}
static void light_bits(uint8_t nsets, uint8_t shift) {
    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
        uint8_t m = 0;
        for (uint8_t s = 0; s < nsets; s++) m |= bits[s][c];
        m >>= shift;
        for (uint8_t r = 0; r < MATRIX_ROWS; r++, m >>= 1)
            if (m & 1) light_led(led_at(r, c));
    }
}

static void fill_pix_random(void) {
    for (uint8_t i = 0; i < LEDS; i++) pix[i] = random_colour();
}

/* Group of each LED for the wave modes, from a cell table. */
static void groups_from_cells(const uint8_t *cells, uint8_t n, uint8_t width) {
    memset(aux, 0xFF, sizeof(aux));
    for (uint8_t g = 0; g < n; g++)
        for (uint8_t k = 0; k < width; k++) {
            uint8_t i = led_cell(cells[g * width + k]);
            if (i != NO_LED) aux[i] = g;
        }
    ngroups = n;
}
static void groups_by_row(void) {
    memset(aux, 0xFF, sizeof(aux));
    for (uint8_t r = 0; r < MATRIX_ROWS; r++)
        for (uint8_t c = 0; c < MATRIX_COLS; c++) {
            uint8_t i = led_at(r, c);
            if (i != NO_LED) aux[i] = r;
        }
    ngroups = MATRIX_ROWS;
}
static void groups_by_col(void) {
    memset(aux, 0xFF, sizeof(aux));
    for (uint8_t r = 0; r < MATRIX_ROWS; r++)
        for (uint8_t c = 0; c < MATRIX_COLS; c++) {
            uint8_t i = led_at(r, c);
            if (i != NO_LED) aux[i] = c;
        }
    ngroups = MATRIX_COLS;
}

/* Wave modes: mono = a 96-step brightness wave per group, rainbow = a 192-step
 * hue per group (mask stays full). */
static void wave_paint(void) {
    for (uint8_t i = 0; i < LEDS; i++) {
        uint8_t g = aux[i];
        if (g == 0xFF) { mask[i] = 0; continue; }
        if (rb_wave) { pix[i] = wheel(ph[g]); mask[i] = 255; }
        else         mask[i] = sfx_wave[ph[g]];
    }
}
static void wave_init(const uint8_t *p0, const uint8_t *hue0) {
    for (uint8_t g = 0; g < ngroups; g++) ph[g] = rb_wave ? hue0[g] : p0[g];
    per_led = rb_wave;
    wave_paint();
}
static void wave_step(bool dir) {
    uint8_t len = rb_wave ? 192 : 96;
    for (uint8_t g = 0; g < ngroups; g++)
        ph[g] = dir ? (ph[g] + 1 == len ? 0 : ph[g] + 1) : (ph[g] ? ph[g] - 1 : len - 1);
    wave_paint();
}

/* Explode / Launch: L moves left, R moves right, one column a step. */
static void shift_lr(void) {
    for (uint8_t c = 1; c < NCOLS; c++) {
        bits[0][c - 1]         = bits[0][c];
        bits[1][NCOLS - c]     = bits[1][NCOLS - 1 - c];
    }
    bits[0][NCOLS - 1] = 0;
    bits[1][0]         = 0;
}

/* ---------------------------------------------------------------- modes */

static void mode_init(uint8_t id) {
    memset(mask, 0, sizeof(mask));
    memset(bits, 0, sizeof(bits));
    per_led = false;
    fade_on = false;
    rb_wave = false;
    pos     = 0;

    switch (id) {
        case SFX_STATIC:        /* 9 rings open from H, then hold */
            period  = 300;
            per_led = cur_rainbow;
            break;
        case SFX_SINGLE_ON:
            period  = 160;
            per_led = cur_rainbow;
            memset(aux, 0, sizeof(aux));
            break;
        case SFX_SINGLE_OFF:
            period  = 160;
            per_led = cur_rainbow;
            if (per_led) fill_pix_random();
            memset(mask, 255, sizeof(mask));
            memset(aux, 63, sizeof(aux));
            break;
        case SFX_GLITTER:       /* every key its own phase of a 256-step sawtooth */
            period  = 300;
            per_led = cur_rainbow;
            if (per_led) fill_pix_random();
            for (uint8_t r = 0; r < MATRIX_ROWS; r++)
                for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                    uint8_t i = led_at(r, c);
                    if (i != NO_LED) mask[i] = sfx_glitter_phase[r][c];
                }
            break;
        case SFX_FALLING:
            period  = 700;
            per_led = cur_rainbow;
            fade_on = true;
            memcpy(bits[0], sfx_fall_start, sizeof(sfx_fall_start));
            light_bits(1, 0);
            break;
        case SFX_COLOURFUL:     /* static rainbow, 60 degrees between neighbours, drifting */
            period  = 100;
            per_led = true;
            for (uint8_t r = 0; r < MATRIX_ROWS; r++)
                for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                    uint8_t i = led_at(r, c);
                    if (i == NO_LED) continue;
                    uint8_t k = (uint8_t)(c + 3 * r);
                    while (k >= 6) k -= 6;
                    aux[i]  = k * 32;
                    pix[i]  = wheel(aux[i]);
                    mask[i] = 255;
                }
            break;
        case SFX_BREATH:
            period  = 240;
            per_led = cur_rainbow;
            preset  = 0;
            if (per_led) for (uint8_t i = 0; i < LEDS; i++) pix[i] = from3(sfx_preset[0]);
            break;
        case SFX_SPECTRUM:      /* whole board one colour round the wheel */
            period  = 200;
            per_led = true;
            ph[0]   = 0;
            for (uint8_t i = 0; i < LEDS; i++) { pix[i] = wheel(0); mask[i] = 255; }
            break;
        case SFX_OUTWARD:
            rb_wave = cur_rainbow;
            period  = rb_wave ? 80 : 100;
            groups_from_cells(&sfx_rings[0][0], 9, 12);
            wave_init(sfx_outward_p0, sfx_outward_hue);
            break;
        case SFX_SCROLL_DOWN:
        case SFX_SCROLL_UP:
            rb_wave = cur_rainbow;
            period  = 100;
            groups_by_row();
            wave_init(sfx_scroll_p0, sfx_scroll_hue);
            break;
        case SFX_ROLL_RIGHT:
        case SFX_ROLL_LEFT:
            rb_wave = cur_rainbow;
            period  = 80;
            groups_by_col();
            wave_init(sfx_roll_p0, sfx_roll_hue);
            break;
        case SFX_ROTATING:
            rb_wave = cur_rainbow;
            period  = 80;
            groups_from_cells(&sfx_rot_groups[0][0], 31, 3);
            wave_init(sfx_rot_p0, sfx_rot_hue);
            break;
        case SFX_EXPLODE:
            period  = 150;
            per_led = cur_rainbow;
            fade_on = true;
            break;
        case SFX_LAUNCH:
            period  = 400;
            per_led = cur_rainbow;
            break;
        case SFX_RIPPLES:
            period  = 240;
            per_led = cur_rainbow;
            fade_on = true;
            break;
        case SFX_FLOW_FWD:
        case SFX_FLOW_BACK:
            period  = 200;
            per_led = cur_rainbow;
            fade_on = true;
            break;
        case SFX_PULSATING:
            period  = 400;
            per_led = cur_rainbow;
            fade_on = true;
            up      = true;
            memcpy(bits[0], sfx_pulse_start, sizeof(sfx_pulse_start));
            light_bits(1, 1);
            break;
        case SFX_TILT_RIGHT:
        case SFX_TILT_LEFT:
            period  = 600;
            per_led = cur_rainbow;
            fade_on = true;
            for (uint8_t c = 0; c < NCOLS; c++) bits[0][c] = 1u << (c % 6);
            light_bits(1, 0);
            break;
        case SFX_SHUTTLE:
            rb_wave = cur_rainbow;
            if (rb_wave) {      /* the rainbow Shuttle is a different effect: rows scroll hue both ways */
                period  = 80;
                per_led = true;
                for (uint8_t c = 0; c < MATRIX_COLS; c++) ph[c] = ph2[c] = sfx_roll_hue[c];
                memset(mask, 255, sizeof(mask));
            } else {
                period  = 350;
                fade_on = true;
            }
            break;
    }
}

static void mode_step(uint8_t id) {
    switch (id) {
        case SFX_STATIC:
            if (pos < 9) {
                for (uint8_t k = 0; k < 12; k++) {
                    uint8_t i = led_cell(sfx_rings[pos][k]);
                    if (i == NO_LED) continue;
                    mask[i] = 255;
                    if (per_led) pix[i] = from3(sfx_ring_rgb[pos]);
                }
                pos++;
            }
            break;
        case SFX_SINGLE_ON:     /* 63 = held; after release 62 steps down the ease curve */
            for (uint8_t i = 0; i < LEDS; i++)
                if (aux[i] && aux[i] != 63) mask[i] = sfx_ease[--aux[i]];
            break;
        case SFX_SINGLE_OFF:    /* 0 = held dark; after release 62 steps back up */
            for (uint8_t i = 0; i < LEDS; i++)
                if (aux[i] && aux[i] != 63) mask[i] = sfx_ease[++aux[i]];
            break;
        case SFX_GLITTER:
            for (uint8_t i = 0; i < LEDS; i++) {
                if (mask[i]) mask[i]--;
                else         light_led(i);
            }
            break;
        case SFX_FALLING:       /* a drop per column, a row a step; bit 6 is the dark pause row */
            for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                bits[0][c] <<= 1;
                if (bits[0][c] & 0x80) bits[0][c] = 1;
            }
            light_bits(1, 0);
            break;
        case SFX_COLOURFUL:
            for (uint8_t i = 0; i < LEDS; i++) {
                if (aux[i] == 0xFF) continue;
                aux[i] = aux[i] ? aux[i] - 1 : 191;
                pix[i] = wheel(aux[i]);
            }
            break;
        case SFX_BREATH: {
            uint8_t lvl = sfx_ease[pos];
            if (++pos >= 128) {
                pos = 0;
                if (per_led) {
                    preset = (preset + 1) & 7;
                    for (uint8_t i = 0; i < LEDS; i++) pix[i] = from3(sfx_preset[preset]);
                }
            }
            memset(mask, lvl, sizeof(mask));
            break;
        }
        case SFX_SPECTRUM: {
            ph[0]   = wrap192(ph[0] + 1);
            rgb_t c = wheel(ph[0]);
            for (uint8_t i = 0; i < LEDS; i++) pix[i] = c;
            break;
        }
        case SFX_OUTWARD:
        case SFX_SCROLL_DOWN:
        case SFX_ROLL_RIGHT:
        case SFX_ROTATING:
            wave_step(false);
            break;
        case SFX_SCROLL_UP:
        case SFX_ROLL_LEFT:
            wave_step(true);
            break;
        case SFX_EXPLODE:
            shift_lr();
            light_bits(2, 0);
            break;
        case SFX_LAUNCH:        /* no trail: the mask is cleared every step */
            memset(mask, 0, sizeof(mask));
            shift_lr();
            light_bits(2, 0);
            break;
        case SFX_RIPPLES: {     /* four quadrants of a Manhattan ring, radius +1 a step */
            uint8_t *A = bits[0], *B = bits[1], *C = bits[2], *D = bits[3];
            A[0] >>= 1;
            D[0] <<= 1;
            for (uint8_t c = 1; c < NCOLS; c++) {
                A[c - 1] |= A[c]; A[c] >>= 1;
                D[c - 1] |= D[c]; D[c] <<= 1;
            }
            B[NCOLS - 1] >>= 1;
            C[NCOLS - 1] <<= 1;
            for (int8_t c = NCOLS - 2; c >= 0; c--) {
                B[c + 1] |= B[c]; B[c] >>= 1;
                C[c + 1] |= C[c]; C[c] <<= 1;
            }
            for (uint8_t c = 0; c < NCOLS; c++) { C[c] &= 0x3F; D[c] &= 0x3F; }
            light_bits(4, 0);
            break;
        }
        case SFX_FLOW_FWD:
        case SFX_FLOW_BACK:
            pos = (id == SFX_FLOW_FWD) ? (pos + 1 == 82 ? 0 : pos + 1) : (pos ? pos - 1 : 81);
            light_led(led_cell(sfx_key_order[pos]));
            break;
        case SFX_PULSATING: {   /* a zigzag born in column 9, spreading both ways */
            uint8_t *P = bits[0];
            for (uint8_t c = 0; c < 9; c++) P[c] = P[c + 1];
            for (uint8_t c = NCOLS - 1; c > 9; c--) P[c] = P[c - 1];
            if (up) { P[9] <<= 1; if (P[9] & 0x80) up = false; }
            else    { P[9] >>= 1; if (P[9] & 0x01) up = true; }
            light_bits(1, 1);
            break;
        }
        case SFX_TILT_RIGHT:    /* each column's dot climbs a row: the '\' stripes slide right */
            for (uint8_t c = 0; c < NCOLS; c++) { bits[0][c] >>= 1; if (!bits[0][c]) bits[0][c] = 0x20; }
            light_bits(1, 0);
            break;
        case SFX_TILT_LEFT:
            for (uint8_t c = 0; c < NCOLS; c++) { bits[0][c] <<= 1; if (bits[0][c] & 0x40) bits[0][c] = 1; }
            light_bits(1, 0);
            break;
        case SFX_SHUTTLE:
            if (rb_wave) {
                for (uint8_t c = 0; c < MATRIX_COLS; c++) {
                    ph[c]  = wrap192(ph[c] + 1);
                    ph2[c] = ph2[c] ? ph2[c] - 1 : 191;
                    for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
                        uint8_t i = led_at(r, c);
                        if (i != NO_LED) pix[i] = wheel((r & 1) ? ph[c] : ph2[c]);
                    }
                }
            } else {            /* even rows ride right, odd rows ride left, with a trail */
                if (++pos == NCOLS) pos = 0;
                for (uint8_t r = 0; r < MATRIX_ROWS; r++)
                    light_led(led_at(r, (r & 1) ? NCOLS - 1 - pos : pos));
            }
            break;
    }
}

/* ---------------------------------------------------------------- time */

/* Stock speed is 6 levels; QMK's 0-255 knob interpolates between them, so the
 * stock levels sit at 0, 51, 102, 153, 204 and 255. */
static uint16_t step_interval_ticks(void) {
    uint8_t s   = rgb_matrix_config.speed;
    uint8_t seg = s / 51;
    if (seg > 4) seg = 4;
    uint8_t  f   = s - seg * 51;
    uint16_t mul = sfx_speed_mul[seg] - (uint16_t)(sfx_speed_mul[seg] - sfx_speed_mul[seg + 1]) * f / 51;
    uint16_t t   = (uint32_t)mul * period / 400;
    return t ? t : 1;
}

static void advance(uint8_t id) {
    uint32_t now = timer_read32();
    uint32_t dt  = now - last_ms;
    last_ms      = now;
    if (dt > 250) dt = 250;              /* after a stall, resume rather than fast-forward */
    uint16_t ticks = (uint16_t)dt * 2;

    if (fade_on) {
        fade_acc += ticks;
        uint8_t dec = fade_acc / 3;      /* 1 per 1.5 ms */
        fade_acc -= dec * 3;
        if (dec)
            for (uint8_t i = 0; i < LEDS; i++) mask[i] = qsub8(mask[i], dec);
    }

    uint16_t interval = step_interval_ticks();
    step_acc += ticks;
    for (uint8_t n = 0; step_acc >= interval; n++) {
        if (n == 16) { step_acc = 0; break; }   /* never more than 16 steps a frame */
        step_acc -= interval;
        mode_step(id);
    }
}

/* ---------------------------------------------------------------- API */

bool stock_fx_get_rainbow(void) {
    if (!rainbow_loaded) {
        rainbow        = kb_eeconfig_get_stockfx_flags() & 1;
        rainbow_loaded = true;
    }
    return rainbow;
}

void stock_fx_set_rainbow(bool on) {
    rainbow        = on;
    rainbow_loaded = true;
    kb_eeconfig_set_stockfx_flags(on ? 1 : 0);
}

static bool run_frame(sfx_id_t id, effect_params_t *params) {
    if (params->iter == 0) {             /* the simulation moves once a frame, not once a chunk */
        bool rb = stock_fx_get_rainbow();
        if (params->init || cur != id || cur_rainbow != rb) {
            cur         = id;
            cur_rainbow = rb;
            mode_init(id);
            last_ms  = timer_read32();
            step_acc = 0;
            fade_acc = 0;
        } else {
            advance(id);
        }
    }

    hsv_t hsv = rgb_matrix_config.hsv;
    uint8_t v = hsv.v;
    if (v > power_val_cap()) v = power_val_cap();
    hsv.v      = 255;
    rgb_t base = hsv_to_rgb(hsv);        /* uncapped: v scales after, as DRIFT does */

    RGB_MATRIX_USE_LIMITS(led_min, led_max);
    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();
        rgb_t   c = per_led ? pix[i] : base;
        uint8_t m = scale8(mask[i], v);
        /* The stock white balance: red at 2/3 on every key (the composer
         * FUN_0000cc74, its usual branch). This board's red LEDs are the
         * strong ones; without it white reads pink-violet. */
        rgb_matrix_set_color(i, scale8(scale8(c.r, m), 170), scale8(c.g, m), scale8(c.b, m));
    }
    return rgb_matrix_check_finished_leds(led_max);
}

static void key_event(uint8_t row, uint8_t col, bool pressed) {
    if (!rgb_matrix_is_enabled() || stock_mode_of(rgb_matrix_get_mode()) == 0xFF) return;
    if (row >= MATRIX_ROWS || col >= MATRIX_COLS) return;
    uint8_t i = led_at(row, col);
    uint8_t m = 1u << row;

    switch (cur) {
        case SFX_SINGLE_ON:
            if (i == NO_LED) return;
            if (pressed) { aux[i] = 63; if (per_led) pix[i] = random_colour(); }
            else         aux[i] = 62;
            mask[i] = sfx_ease[aux[i]];
            break;
        case SFX_SINGLE_OFF:
            if (i == NO_LED) return;
            if (pressed) aux[i] = 0;
            else { aux[i] = 1; if (per_led) pix[i] = random_colour(); }
            mask[i] = sfx_ease[aux[i]];
            break;
        case SFX_EXPLODE:       /* a ray each way along the key's row; every press widens all rays by one */
            if (!pressed) return;
            bits[0][col] |= m;
            bits[1][col] |= m;
            for (uint8_t c = 1; c < NCOLS; c++) {
                bits[0][c - 1]     |= bits[0][c];
                bits[1][NCOLS - c] |= bits[1][NCOLS - 1 - c];
            }
            light_bits(2, 0);
            break;
        case SFX_LAUNCH:        /* the key's whole column, both ways */
            if (!pressed) return;
            bits[0][col] = bits[1][col] = 0x3F;
            light_bits(2, 0);
            break;
        case SFX_RIPPLES:
            if (!pressed) return;
            for (uint8_t s = 0; s < 4; s++) bits[s][col] |= m;
            light_bits(4, 0);
            break;
        default:
            break;
    }
}

bool stock_fx_run(sfx_id_t id, effect_params_t *params) {
    bool more;
    ACCT(ACCT_RGB, more = run_frame(id, params));
    return more;
}

void stock_fx_key(uint8_t row, uint8_t col, bool pressed) {
    ACCT(ACCT_RGB, key_event(row, col, pressed));
}

/* ---------------------------------------------------------------- mode list */

/* Stock order (gohv protocol.rs LightingMode, 0x00-0x13). 0 = Off is not an
 * effect; the directional modes list their first direction here. */
static const uint8_t stock_rgb[STOCK_MODE_COUNT] = {
    RGB_MATRIX_NONE,
    RGB_MATRIX_CUSTOM_STOCK_STATIC,
    RGB_MATRIX_CUSTOM_STOCK_SINGLE_ON,
    RGB_MATRIX_CUSTOM_STOCK_SINGLE_OFF,
    RGB_MATRIX_CUSTOM_STOCK_GLITTERING,
    RGB_MATRIX_CUSTOM_STOCK_FALLING,
    RGB_MATRIX_CUSTOM_STOCK_COLOURFUL,
    RGB_MATRIX_CUSTOM_STOCK_BREATH,
    RGB_MATRIX_CUSTOM_STOCK_SPECTRUM,
    RGB_MATRIX_CUSTOM_STOCK_OUTWARD,
    RGB_MATRIX_CUSTOM_STOCK_SCROLL_DOWN,
    RGB_MATRIX_CUSTOM_STOCK_ROLL_RIGHT,
    RGB_MATRIX_CUSTOM_STOCK_ROTATING,
    RGB_MATRIX_CUSTOM_STOCK_EXPLODE,
    RGB_MATRIX_CUSTOM_STOCK_LAUNCH,
    RGB_MATRIX_CUSTOM_STOCK_RIPPLES,
    RGB_MATRIX_CUSTOM_STOCK_FLOW_RIGHT,
    RGB_MATRIX_CUSTOM_STOCK_PULSATING,
    RGB_MATRIX_CUSTOM_STOCK_TILT_RIGHT,
    RGB_MATRIX_CUSTOM_STOCK_SHUTTLE,
};

/* The stock effects are contiguous in the enum, in sfx_id_t order. */
#define SFX_FIRST RGB_MATRIX_CUSTOM_STOCK_STATIC
_Static_assert(RGB_MATRIX_CUSTOM_STOCK_SHUTTLE - SFX_FIRST == SFX_SHUTTLE, "stock effects must stay contiguous and in sfx_id_t order");
/* via.json's Effect dropdown names these numbers outright: change both together. */
_Static_assert(SFX_FIRST == 4 && RGB_MATRIX_SOLID_COLOR == 1 && RGB_MATRIX_ALPHAS_MODS == 2 &&
               RGB_MATRIX_TYPING_HEATMAP == 3 && RGB_MATRIX_CUSTOM_RAINFALL == 27 && RGB_MATRIX_CUSTOM_DRIFT == 28,
               "effect numbers moved: update via.json's Effect options");

static const uint8_t sfx_stock[SFX_COUNT] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11, 12, 13, 14, 15, 16, 16, 17, 18, 18, 19,
};

static const char *const sfx_name[SFX_COUNT] = {
    "Static", "Single On", "Single Off", "Glittering", "Falling", "Colourful", "Breath",
    "Spectrum", "Outward", "Scroll Down", "Scroll Up", "Roll Right", "Roll Left", "Rotating",
    "Explode", "Launch", "Ripples", "Flow Right", "Flow Left", "Pulsating", "Tilt Right",
    "Tilt Left", "Shuttle",
};

static inline bool is_stock(uint8_t rgb_mode) {
    return rgb_mode >= SFX_FIRST && rgb_mode < SFX_FIRST + SFX_COUNT;
}

uint8_t stock_mode_of(uint8_t rgb_mode) {
    return is_stock(rgb_mode) ? sfx_stock[rgb_mode - SFX_FIRST] : 0xFF;
}

uint8_t stock_mode_rgb(uint8_t stock) {
    return stock < STOCK_MODE_COUNT ? stock_rgb[stock] : RGB_MATRIX_NONE;
}

void stock_mode_step(bool reverse) {
    uint8_t s = stock_mode_of(rgb_matrix_get_mode());
    if (s == 0xFF) s = reverse ? 1 : STOCK_MODE_COUNT - 1;   /* off the list: enter at the near end */
    if (reverse) s = (s <= 1) ? STOCK_MODE_COUNT - 1 : s - 1;
    else         s = (s >= STOCK_MODE_COUNT - 1) ? 1 : s + 1;
    rgb_matrix_mode(stock_rgb[s]);
}

bool stock_mode_has_direction(uint8_t rgb_mode) {
    return stock_mode_flip_direction(rgb_mode) != rgb_mode;
}

uint8_t stock_mode_flip_direction(uint8_t rgb_mode) {
    if (!is_stock(rgb_mode)) return rgb_mode;
    switch (rgb_mode - SFX_FIRST) {
        case SFX_SCROLL_DOWN: return rgb_mode + 1;
        case SFX_SCROLL_UP:   return rgb_mode - 1;
        case SFX_ROLL_RIGHT:  return rgb_mode + 1;
        case SFX_ROLL_LEFT:   return rgb_mode - 1;
        case SFX_FLOW_FWD:    return rgb_mode + 1;
        case SFX_FLOW_BACK:   return rgb_mode - 1;
        case SFX_TILT_RIGHT:  return rgb_mode + 1;
        case SFX_TILT_LEFT:   return rgb_mode - 1;
        default:              return rgb_mode;
    }
}

const char *stock_mode_name(uint8_t rgb_mode) {
    return is_stock(rgb_mode) ? sfx_name[rgb_mode - SFX_FIRST] : NULL;
}
