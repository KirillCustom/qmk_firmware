// Copyright 2026 Fernando Birra <fernando.birra@gmail.com>
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#include "ak820pro.h"

/* Layer indices come from ak820pro.h (shared with board code -- the Fn-layer
 * mask in indicators.c is derived from them). */
#define KC_TASK LGUI(KC_TAB)        // Task viewer
#define KC_FLXP LGUI(KC_E)          // Windows file explorer
#define KC_MCTL KC_MISSION_CONTROL  // Mission Control
#define KC_LPAD KC_LAUNCHPAD        // Launchpad

/* STOCK-LIKE LAYOUT (kirillcustom, 2026-10-21), replacing JD's personal one: every
 * key does what its keycap says. Caps Lock is Caps Lock; the key right of F12 is
 * Delete; the knob press is Mute; the right column is Home / PgUp / PgDn; Fn is
 * only the key right of the right Alt. Fn layer = the stock shortcuts:
 *   Mac: F-row is media by default (F8 play/pause), Fn+F-row is F1-F12; Windows: the reverse
 *   Esc bootloader · Del ANIM_TOG · 6/7 saturation · -/= speed
 *   Home LCD on/off · PgUp/PgDn LCD brightness · Q/W/E BT slots · R 2.4G
 *   \ next effect · D debug page · X RGB on/off · C clock format · B power mode
 *   Win lock (Fn+Win) · Up/Down RGB brightness · Left/Right hue
 * VIA's stored keymap overrides this array: after flashing, reset the dynamic
 * keymap (VIA id_dynamic_keymap_reset) to load it. */
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [WINBASE] = LAYOUT_82_ansi(
        KC_ESC,     KC_F1,      KC_F2,      KC_F3,      KC_F4,      KC_F5,      KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     KC_F11,     KC_F12,     KC_DEL,     KC_MUTE,
        KC_GRV,     KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       KC_MINS,    KC_EQL,     KC_BSPC,    KC_HOME,
        KC_TAB,     KC_Q,       KC_W,       KC_E,       KC_R,       KC_T,       KC_Y,       KC_U,       KC_I,       KC_O,       KC_P,       KC_LBRC,    KC_RBRC,    KC_BSLS,    KC_PGUP,
        KC_CAPS,    KC_A,       KC_S,       KC_D,       KC_F,       KC_G,       KC_H,       KC_J,       KC_K,       KC_L,       KC_SCLN,    KC_QUOT,    KC_ENT,     KC_PGDN,
        KC_LSFT,    KC_Z,       KC_X,       KC_C,       KC_V,       KC_B,       KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,    KC_RSFT,    KC_UP,
        KC_LCTL,    KC_LGUI,    KC_LALT,    KC_SPC,     KC_RALT,    MO(WINFN),  KC_RCTL,    KC_LEFT,    KC_DOWN,    KC_RGHT
    ),
    [WINFN] = LAYOUT_82_ansi(
        QK_BOOT,    KC_BRID,    KC_BRIU,    LGUI(KC_TAB),LGUI(KC_E), _______,    _______,    KC_MPRV,    KC_MPLY,    KC_MNXT,    KC_MUTE,    KC_VOLD,    KC_VOLU,    ANIM_TOG,   MENU_OPEN,
        _______,    _______,    _______,    _______,    _______,    _______,    RM_SATD,    RM_SATU,    _______,    _______,    _______,    RM_SPDD,    RM_SPDU,    _______,    SCR_TOG,
        _______,    BT1,        BT2,        BT3,        BT24G,      _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    RM_NEXT,    SCR_UP,
        _______,    _______,    _______,    DBG_PAGE,   _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    SCR_DN,
        _______,    _______,    RM_TOGG,    CLK_MODE,   _______,    PWR_MODE,   _______,    _______,    _______,    _______,    _______,    _______,    RM_VALU,
        _______,    GU_TOGG,    _______,    _______,    _______,    _______,    _______,    RM_HUED,    RM_VALD,    RM_HUEU
    ),
    [MACBASE] = LAYOUT_82_ansi(
        KC_ESC,     KC_BRID,    KC_BRIU,    KC_MCTL,    KC_LPAD,    KC_F5,      KC_F6,      KC_MPRV,    KC_MPLY,    KC_MNXT,    KC_MUTE,    KC_VOLD,    KC_VOLU,    KC_DEL,     KC_MUTE,
        KC_GRV,     KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       KC_MINS,    KC_EQL,     KC_BSPC,    KC_HOME,
        KC_TAB,     KC_Q,       KC_W,       KC_E,       KC_R,       KC_T,       KC_Y,       KC_U,       KC_I,       KC_O,       KC_P,       KC_LBRC,    KC_RBRC,    KC_BSLS,    KC_PGUP,
        KC_CAPS,    KC_A,       KC_S,       KC_D,       KC_F,       KC_G,       KC_H,       KC_J,       KC_K,       KC_L,       KC_SCLN,    KC_QUOT,    KC_ENT,     KC_PGDN,
        KC_LSFT,    KC_Z,       KC_X,       KC_C,       KC_V,       KC_B,       KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,    KC_RSFT,    KC_UP,
        KC_LCTL,    KC_LALT,    KC_LGUI,    KC_SPC,     KC_RGUI,    MO(MACFN),  KC_RALT,    KC_LEFT,    KC_DOWN,    KC_RGHT
    ),
    [MACFN] = LAYOUT_82_ansi(
        QK_BOOT,    KC_F1,      KC_F2,      KC_F3,      KC_F4,      KC_F5,      KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     KC_F11,     KC_F12,     ANIM_TOG,   MENU_OPEN,
        _______,    _______,    _______,    _______,    _______,    _______,    RM_SATD,    RM_SATU,    _______,    _______,    _______,    RM_SPDD,    RM_SPDU,    _______,    SCR_TOG,
        _______,    BT1,        BT2,        BT3,        BT24G,      _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    RM_NEXT,    SCR_UP,
        _______,    _______,    _______,    DBG_PAGE,   _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    SCR_DN,
        _______,    _______,    RM_TOGG,    CLK_MODE,   _______,    PWR_MODE,   _______,    _______,    _______,    _______,    _______,    _______,    RM_VALU,
        _______,    _______,    _______,    _______,    _______,    _______,    _______,    RM_HUED,    RM_VALD,    RM_HUEU
    )
};


bool dip_switch_update_user(uint8_t index, bool active) {
    // Mac/Windows layer switch. The wireless slider (index 1/2) and dashboard
    // icons are handled at keyboard level in ak820pro.c (dip_switch_update_kb).
    if (index == 0) {
        if (active) {
            set_single_persistent_default_layer(WINBASE);
        } else {
            set_single_persistent_default_layer(MACBASE);
            keymap_config.no_gui = false;
        }
    }
    return true;
}


bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    // Only Mac-specific media keys live here; SCR_TOG and the BT* keycodes are
    // handled at keyboard level in ak820pro.c (process_record_kb).
    switch (keycode) {
        case KC_MISSION_CONTROL:
            if (record->event.pressed) {
                host_consumer_send(0x29F);
            } else {
                host_consumer_send(0);
            }
            return false;  // Skip all further processing of this key
        case KC_LAUNCHPAD:
            if (record->event.pressed) {
                host_consumer_send(0x2A0);
            } else {
                host_consumer_send(0);
            }
            return false;  // Skip all further processing of this key
        default:
            return true;  // Process all other keycodes normally
    }
}


#if defined(ENCODER_MAP_ENABLE)
/* The knob does quarter-step volume on the Mac layers and whole steps on the
 * Windows ones, because Shift+Alt+Volume is an APPLE convention -- macOS reads it
 * as "fine adjustment", Windows ignores the modifiers and its step size is fixed
 * regardless. The mode slider already selects the base layer, so the knob follows
 * the platform with no extra configuration.
 *
 * Modified consumer keycodes on an encoder NEED a non-zero ENCODER_MAP_KEY_DELAY
 * (see config.h). At the default of 0 the press and release are adjacent
 * instructions and the host can miss the modifier state entirely -- measured as
 * ~1/3 of clicks doing full steps, and occasionally Alt-only, which opens the
 * macOS Sound settings dialog. */
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [WINBASE] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [WINFN]   = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [MACBASE] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [MACFN]   = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU) }
};
#endif
