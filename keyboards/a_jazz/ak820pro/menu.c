// Copyright 2026 KirillCustom
// SPDX-License-Identifier: GPL-2.0-or-later
/* The settings menu -- see menu.h.
 *
 * Every row is recomposed on each change and on the 10 Hz tick; the page
 * (display_menu_row) repaints only the cells that differ, one glyph a pass,
 * so a composed-but-unchanged menu costs a few memcmps and no LCD work.
 * Changes apply at once and persist through the usual debounced paths: QMK's
 * delayed RGB EEPROM write, kb_eeconfig's settled flush. */
#include "quantum.h"
#include "menu.h"
#include "ak820pro.h"
#include "kb_eeconfig.h"
#include "power.h"
#include "stock_fx.h"
#include "graphics/display.h"
#include "graphics/lcd_bus.h"   /* anim_toggle(), anim_active() */

#ifndef AK820_BUILD_TOKEN
#    define AK820_BUILD_TOKEN 0u   /* build.sh supplies one */
#endif

#define MENU_IDLE_MS 20000u
#define KNOB_ROW     0      /* the knob's press, matrix [0,14] in keyboard.json */
#define KNOB_COL     14
#define COLS         DISPLAY_MENU_COLS
#define HUE_STEP     8
#define LEVEL_STEP   16

enum {
    I_MODE, I_HUE, I_SAT, I_VAL, I_SPEED, I_DIR, I_RAINBOW, I_LIGHTS,   /* page 1: rows 1-8 */
    I_LCD_LEVEL, I_LCD_ON, I_CLOCK, I_POWER, I_ANIM, I_BUILD,           /* page 2: rows 1-6 */
    I_COUNT
};
#define PAGE2_FIRST I_LCD_LEVEL
_Static_assert(PAGE2_FIRST <= DISPLAY_MENU_ROWS - 1 && I_COUNT - PAGE2_FIRST <= DISPLAY_MENU_ROWS - 2, "each page must fit under its title");

/* VALUE: a press starts an edit, the knob changes it. TOGGLE and ACTION act
 * on the press. INFO only shows. */
enum { K_VALUE, K_TOGGLE, K_ACTION, K_INFO };
static const uint8_t item_kind[I_COUNT] = {
    K_VALUE, K_VALUE, K_VALUE, K_VALUE, K_VALUE, K_TOGGLE, K_TOGGLE, K_TOGGLE,
    K_VALUE, K_TOGGLE, K_VALUE, K_VALUE, K_ACTION, K_INFO,
};
static const char *const item_label[I_COUNT] = {
    "Mode", "Colour", "Saturation", "Brightness", "Speed", "Direction", "Rainbow", "Lights",
    "LCD level", "Screen", "Clock", "Power", "Animation", "Build",
};

/* The stock names without the direction (stock_mode_name() carries it). */
static const char *const stock_base_name[STOCK_MODE_COUNT] = {
    "Off", "Static", "Single On", "Single Off", "Glittering", "Falling", "Colourful", "Breath",
    "Spectrum", "Outward", "Scrolling", "Rolling", "Rotating", "Explode", "Launch", "Ripples",
    "Flowing", "Pulsating", "Tilt", "Shuttle",
};

/* After the stock list: the extra effects this build keeps. */
static const uint8_t extra_mode[] = {
#ifdef ENABLE_RGB_MATRIX_ALPHAS_MODS
    RGB_MATRIX_ALPHAS_MODS,
#endif
#ifdef ENABLE_RGB_MATRIX_TYPING_HEATMAP
    RGB_MATRIX_TYPING_HEATMAP,
#endif
    RGB_MATRIX_CUSTOM_RAINFALL,
    RGB_MATRIX_CUSTOM_DRIFT,
};
static const char *const extra_name[] = {
#ifdef ENABLE_RGB_MATRIX_ALPHAS_MODS
    "Alphas/Mods",
#endif
#ifdef ENABLE_RGB_MATRIX_TYPING_HEATMAP
    "Heatmap",
#endif
    "Rainfall",
    "Drift",
};
#define N_STOCK (STOCK_MODE_COUNT - 1)   /* without Off: that is the Lights toggle */
#define N_MODES (N_STOCK + ARRAY_SIZE(extra_mode))

static const char *const clock_name[DISPLAY_CLOCK_MODE_COUNT] = {"24h", "12h", "Off", "Date"};
static const char *const power_name[POWER_MODE_COUNT]        = {"Always on", "Balanced", "Saver", "Lights out"};

static bool     menu_on  = false;
static bool     editing  = false;
static uint8_t  sel      = 0;
static uint32_t menu_at  = 0;

/* ---------------------------------------------------------------- the modes */

static uint8_t mode_at(uint8_t i) {
    return i < N_STOCK ? stock_mode_rgb(i + 1) : extra_mode[i - N_STOCK];
}

static uint8_t mode_index(uint8_t m) {
    uint8_t s = stock_mode_of(m);
    if (s != 0xFF && s > 0) return s - 1;
    for (uint8_t i = 0; i < ARRAY_SIZE(extra_mode); i++)
        if (extra_mode[i] == m) return N_STOCK + i;
    return 0xFF;
}

static const char *mode_label(uint8_t m) {
    uint8_t i = mode_index(m);
    if (i == 0xFF) return "Other";
    return i < N_STOCK ? stock_base_name[i + 1] : extra_name[i - N_STOCK];
}

/* ---------------------------------------------------------------- text */

static char *put(char *p, const char *s) {
    while (*s) *p++ = *s++;
    *p = '\0';
    return p;
}

static char *put_u(char *p, uint16_t v) {
    char    t[6];
    uint8_t n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = t[--n];
    *p = '\0';
    return p;
}

static char *put_pct(char *p, uint8_t v) {
    return put(put_u(p, (uint16_t)(((uint16_t)v * 100u + 127u) / 255u)), "%");
}

static const char *on_off(bool on) { return on ? "On" : "Off"; }

static void value_text(uint8_t item, char *out) {
    uint8_t m = rgb_matrix_get_mode();
    out[0]    = '\0';
    switch (item) {
        case I_MODE:   put(out, mode_label(m)); break;
        case I_HUE:    put_u(out, (uint16_t)(((uint32_t)rgb_matrix_get_hue() * 360u) >> 8)); break;
        case I_SAT:    put_pct(out, rgb_matrix_get_sat()); break;
        case I_VAL:    put_pct(out, rgb_matrix_get_val()); break;
        case I_SPEED:  put_pct(out, rgb_matrix_get_speed()); break;
        case I_DIR:
            if (!stock_mode_has_direction(m)) { put(out, "-"); break; }
            {
                bool first = stock_mode_rgb(stock_mode_of(m)) == m;
                if (stock_mode_of(m) == 10) put(out, first ? "Down" : "Up");      /* Scrolling */
                else                        put(out, first ? "Right" : "Left");   /* Rolling, Flowing, Tilt */
            }
            break;
        case I_RAINBOW: put(out, on_off(stock_fx_get_rainbow())); break;
        case I_LIGHTS:  put(out, on_off(rgb_matrix_is_enabled())); break;
        case I_LCD_LEVEL: {   /* the level as a share of the top one: 0-100% */
            uint8_t mx = display_get_brightness_max();
            put(put_u(out, mx ? (uint16_t)((display_get_brightness() * 100u + mx / 2u) / mx) : 0u), "%");
            break;
        }
        case I_LCD_ON:  put(out, on_off(display_get_power())); break;
        case I_CLOCK: {
            uint8_t c = display_get_clock_mode();
            put(out, c < DISPLAY_CLOCK_MODE_COUNT ? clock_name[c] : "?");
            break;
        }
        case I_POWER: {
            uint8_t pm = power_mode();
            put(out, pm < POWER_MODE_COUNT ? power_name[pm] : "?");
            break;
        }
        case I_ANIM:    put(out, anim_active() ? "Stop" : "Play"); break;
        case I_BUILD: {
            static const char hex[] = "0123456789abcdef";
            uint32_t t = (uint32_t)AK820_BUILD_TOKEN;
            for (int8_t k = 7; k >= 0; k--) { out[k] = hex[t & 15]; t >>= 4; }
            out[8] = '\0';
            break;
        }
    }
}

/* "> Label      value", or "> Label    <value>" while editing. */
static void row_item(uint8_t row, uint8_t item) {
    char    val[16], line[COLS + 1];
    uint8_t n = 0;
    value_text(item, val);
    bool    ed = editing && item == sel;

    memset(line, ' ', COLS);
    line[COLS] = '\0';
    line[0]    = (item == sel) ? '>' : ' ';
    for (const char *l = item_label[item]; *l && n < 10; l++, n++) line[2 + n] = *l;

    uint8_t vmax = (uint8_t)(COLS - (2 + n + 1));   /* what is left right of the label and a space */
    uint8_t vn   = (uint8_t)strlen(val) + (ed ? 2 : 0);
    if (vn > vmax) vn = vmax;
    uint8_t c = COLS - vn;
    if (ed) line[c++] = '<';
    for (const char *v = val; *v && c < COLS - (ed ? 1 : 0); v++) line[c++] = *v;
    if (ed) line[COLS - 1] = '>';
    display_menu_row(row, line);
}

static void render(void) {
    if (sel < PAGE2_FIRST) {
        display_menu_row(0, "LIGHTING          1/2");
        for (uint8_t i = 0; i < PAGE2_FIRST; i++) row_item(1 + i, i);
    } else {
        display_menu_row(0, "SCREEN / SYSTEM   2/2");
        uint8_t r = 1;
        for (uint8_t i = PAGE2_FIRST; i < I_COUNT; i++) row_item(r++, i);
        while (r < DISPLAY_MENU_ROWS - 1) display_menu_row(r++, "");
        display_menu_row(DISPLAY_MENU_ROWS - 1, "Knob/Enter  Esc: exit");
    }
}

/* ---------------------------------------------------------------- actions */

static void menu_close(void) {
    menu_on = false;
    editing = false;
    display_menu_close();
}

static uint8_t step_level(uint8_t v, int8_t d, uint8_t max) {
    int16_t x = (int16_t)v + d * LEVEL_STEP;
    if (x < 0) x = 0;
    if (x > max) x = max;
    return (uint8_t)x;
}

static void change(uint8_t item, int8_t d) {
    uint8_t m = rgb_matrix_get_mode();
    uint8_t h = rgb_matrix_get_hue(), s = rgb_matrix_get_sat(), v = rgb_matrix_get_val();
    switch (item) {
        case I_MODE: {
            uint8_t i = mode_index(m);
            i         = (i == 0xFF) ? 0 : (uint8_t)((i + N_MODES + d) % N_MODES);
            rgb_matrix_mode(mode_at(i));
            break;
        }
        case I_HUE:     rgb_matrix_sethsv((uint8_t)(h + d * HUE_STEP), s, v); break;
        case I_SAT:     rgb_matrix_sethsv(h, step_level(s, d, 255), v); break;
        case I_VAL:     rgb_matrix_sethsv(h, s, step_level(v, d, RGB_MATRIX_MAXIMUM_BRIGHTNESS)); break;
        case I_SPEED:   rgb_matrix_set_speed(step_level(rgb_matrix_get_speed(), d, 255)); break;
        case I_DIR:     if (stock_mode_has_direction(m)) rgb_matrix_mode(stock_mode_flip_direction(m)); break;
        case I_RAINBOW: stock_fx_set_rainbow(!stock_fx_get_rainbow()); break;
        case I_LIGHTS:  rgb_matrix_toggle(); break;
        case I_LCD_LEVEL:
            if (d > 0) display_brightness_up();
            else       display_brightness_down();
            break;
        case I_LCD_ON:
            display_toggle_power();
            if (!display_get_power()) menu_close();   /* nothing left to show it on */
            break;
        case I_CLOCK: {
            uint8_t c = (uint8_t)((display_get_clock_mode() + DISPLAY_CLOCK_MODE_COUNT + d) % DISPLAY_CLOCK_MODE_COUNT);
            display_set_clock_mode(c);
            kb_eeconfig_set_clock_mode(c);   /* as Fn+C */
            break;
        }
        case I_POWER:   power_cycle_mode(); break;   /* one way round, as Fn+B */
        case I_ANIM:    menu_close(); anim_toggle(); break;   /* the animation takes the panel */
        default:        break;
    }
}

static void press_item(void) {
    switch (item_kind[sel]) {
        case K_VALUE:  editing = !editing; break;
        case K_TOGGLE:
        case K_ACTION: change(sel, 1); break;
        default:       break;
    }
}

static void move(int8_t d) {
    editing = false;
    sel     = (uint8_t)((sel + I_COUNT + d) % I_COUNT);
}

static void menu_open(void) {
    if (!display_menu_open()) return;   /* the animation owns the panel, or it is off */
    menu_on = true;
    editing = false;
    sel     = 0;
    menu_at = timer_read32();
    render();
}

/* ---------------------------------------------------------------- API */

bool menu_active(void) { return menu_on; }

bool menu_process_record(uint16_t keycode, keyrecord_t *record) {
    bool enc = IS_ENCODEREVENT(record->event);
    if (!menu_on) {
        if (enc || keycode != MENU_OPEN || !record->event.pressed) return false;
        menu_open();
        return true;
    }
    menu_at = timer_read32();
    if (enc) {   /* press and release of every click; one step per click */
        if (record->event.pressed) {
            int8_t d = record->event.type == ENCODER_CW_EVENT ? 1 : -1;
            if (editing) change(sel, d);
            else         move(d);
        }
    } else {
        /* Releases pass: a key held from before the menu must not stick, and
         * the release of a press the menu took is a no-op. Layer keys pass. */
        if (!record->event.pressed) return false;
        if (IS_QK_MOMENTARY(keycode) || IS_QK_LAYER_TAP(keycode) || IS_QK_ONE_SHOT_LAYER(keycode)) return false;
        if ((record->event.key.row == KNOB_ROW && record->event.key.col == KNOB_COL) || keycode == KC_ENT ||
            keycode == KC_SPC) {
            press_item();
        } else {
            switch (keycode) {
                case KC_UP:   if (editing) change(sel, 1);  else move(-1); break;
                case KC_DOWN: if (editing) change(sel, -1); else move(1);  break;
                case KC_LEFT: if (item_kind[sel] == K_VALUE || sel == I_DIR) change(sel, -1); break;
                case KC_RGHT: if (item_kind[sel] == K_VALUE || sel == I_DIR) change(sel, 1);  break;
                case KC_ESC:  if (editing) editing = false; else menu_close(); break;
                default:      break;   /* swallowed: nothing reaches the host */
            }
        }
    }
    if (menu_on) render();
    return true;
}

void menu_task(void) {
    if (!menu_on) return;
    if (!display_menu_active()) { menu_on = false; editing = false; return; }   /* closed under us */
    if (timer_elapsed32(menu_at) >= MENU_IDLE_MS) { menu_close(); return; }
    render();   /* values changed elsewhere (the power ladder, the animation) */
}
