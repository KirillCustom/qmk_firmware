// Copyright 2026 KirillCustom
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

/* The settings menu on the LCD, opened with Fn + a press of the knob
 * (MENU_OPEN, in both Fn layers). Two pages: Lighting (the stock modes and
 * their colour, saturation, brightness, speed, direction, rainbow, on/off)
 * and Screen / system (LCD brightness and power, clock format, power mode,
 * the animation, the firmware build).
 *
 * The knob or Up/Down move; a knob press or Enter edits the item (a toggle
 * flips at once), the knob then changes it and a press or Esc ends the edit;
 * Left/Right change the item directly; Esc leaves. It closes by itself after
 * MENU_IDLE_MS. While it is up every key press goes to it and none to the
 * host; releases and layer keys pass, so nothing sticks.
 *
 * Key handling follows quill4gen7's menu (qmk_firmware ak820pro-notify,
 * f8950d9ae1). The page is the Fn+D page's machinery (display_menu_*). */

#include <stdint.h>
#include <stdbool.h>
#include "action.h"

/* Key and knob events, early in process_record_kb. True when consumed. */
bool menu_process_record(uint16_t keycode, keyrecord_t *record);

/* 10 Hz: the idle timeout, and noticing the page was closed under it. */
void menu_task(void);

bool menu_active(void);
