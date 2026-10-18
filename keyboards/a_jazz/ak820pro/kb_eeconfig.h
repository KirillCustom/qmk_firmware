// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Persisted keyboard config (the EEPROM kb datablock). One module owns the
 * struct so its layout has exactly one home; everything else goes through
 * accessors. The block is 5 bytes (bt_profile, rtc_period u16,
 * lcd_brightness+1, clock_mode+1 -- the last appended 2026-09-01, growing
 * the block from 4; the battery level and the power mode followed) and its
 * layout is ASSIGN-ONLY: existing on-device bytes
 * must keep their meaning, and 0 means "unset" for every field added after
 * first ship (a fresh block is zeros).
 *
 * Writes are coalesced and deferred -- see kb_eeconfig.c. */

/* Read the block from EEPROM. Call once from post-init, before any consumer
 * (rtc_init reads the period, display_init the brightness). */
void kb_eeconfig_init(void);

/* 10 Hz housekeeping: programs the flash once dirty values settle. */
void kb_eeconfig_task(void);

/* Last BT slot selected (CH582_PROFILE_BT_1..3), validated; falls back to
 * slot 1 on a fresh/garbage block. */
uint8_t kb_eeconfig_get_bt_profile(void);
void    kb_eeconfig_set_bt_profile(uint8_t p);

/* Converged RTC divider period; 0 = never persisted. Sanity-range checking
 * is the consumer's job (rtc_init clamps to the plausible ILRC window). */
uint16_t kb_eeconfig_get_rtc_period(void);
void     kb_eeconfig_set_rtc_period(uint16_t period);

/* Stored LCD backlight level. Returns false when unset (fresh block, or a
 * poisoned test value) -- the caller keeps its compile-time default. */
bool kb_eeconfig_get_lcd_brightness(uint8_t *level);
void kb_eeconfig_set_lcd_brightness(uint8_t level);

/* Clock band format (enum display_clock_mode: 0 = 24h, 1 = 12h, 2 = off).
 * Returns false when unset (fresh block) or garbage -- the caller keeps its
 * compile-time default (24h). */
bool kb_eeconfig_get_clock_mode(uint8_t *mode);
void kb_eeconfig_set_clock_mode(uint8_t mode);

/* The battery level (battery.c), in per mille, kept to 0.5 %. It
 * exists for one case: a reboot at the top clamp, where `5C` reads 100 and the
 * voltage cannot say how far into the clamp the pack is -- and every cable <->
 * BT slider flip is a reboot. Returns false when unset. */
bool kb_eeconfig_get_batt_level(uint16_t *permille);
void kb_eeconfig_set_batt_level(uint16_t permille);

/* The power mode (power_mode_t). Returns false when unset (fresh block, so
 * after every flash) or garbage -- power.c then takes Normal and says so. */
bool kb_eeconfig_get_power_mode(uint8_t *mode);
void kb_eeconfig_set_power_mode(uint8_t mode);

#ifdef WDT_TEST_HOOKS
/* Force a real, immediate flash program (for the reset-during-write test). */
void kb_eeconfig_test_write(void);
#endif
