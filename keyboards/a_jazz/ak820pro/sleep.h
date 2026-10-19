// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Sleep experiments (plans/SLEEP-PLAN.md), dev builds only: off unless
 * HC_DEVHOOK op 14 turns them on, so the board otherwise runs as without them.
 *
 * Light sleep's sizing build, a bitmask, each bit honoured only while the RGB
 * is dark (power_rgb_dark()) and dropped the moment it lights: */
#define SLEEP_LS_SCAN  0x01u   /* the row ISR scan-only, one key row per ~0.86 ms */
#define SLEEP_LS_YIELD 0x02u   /* sleep the main loop `yield_ms` (1-4) at each pass's end */
#define SLEEP_LS_WFI   0x04u   /* WFI in the idle thread, PMU_CTRL 0 (USB stays up) */
#define SLEEP_LS_TICK  0x08u   /* stop the 20 kHz tick: the screen dark, no indicator lit */
#define SLEEP_LS_PMU   0x10u   /* PMU Sleep (CTRL 4) around the WFI, for `pmu_s` seconds:
                                * the USB transceiver turns off, so it ends by itself */
#define SLEEP_LS_ALL   0x1Fu

/* The mask ends by itself after `ttl_s` (0: 240 s), PMU Sleep after `pmu_s`
 * (0: 60 s): a probe that cuts USB must end with nothing sent to end it. */
void    sleep_light_set(uint8_t mask, uint8_t yield_ms, uint8_t pmu_s, uint8_t ttl_s);
uint8_t sleep_light_yield(void);
uint8_t sleep_light_mask(void);
uint8_t sleep_light_applied(void);   /* the bits in force now */
extern volatile uint32_t ak820_wfi_count;   /* WFIs the idle thread executed */

/* The deep-sleep experiment: arm it to sleep `delay_s` from now (or at the
 * first pass after that where it is safe), waking on a key or after `cap_s`
 * seconds asleep. `no_feed`: leave the watchdog unfed while asleep (does it
 * count in Deep-sleep?). */
void sleep_deep_arm(uint8_t delay_s, uint16_t cap_s, bool no_feed);
void sleep_deep_cancel(void);   /* disarm, if armed and not yet asleep */
/* The last run's record, 24 bytes, little endian: u8 state (0 idle, 1 armed,
 * 2 done), u8 reason (0 none, 1 key, 2 cap, 3 strays, 4 blocked), u8 blocked
 * (the preconditions that failed, bits: 1 key down, 2 USB, 4 radio TX,
 * 8 display, 16 flash, 32 RTC, 64 a column off P0/P2, 128 no watchdog
 * counting), u8 slept (0: a key was already down, or no IHRC); u16 seconds asleep, u16 stray wakes;
 * u32 P0 and u32 P2 edge bits that woke it; u16 PLL lock loops; u16
 * key_presses gained across the wake (counted 1 s after); u8 the lock loops'
 * ceiling hit (1); u8 the USB transceiver was off after the wake (1); u8 the
 * host configured USB again within 20 s of the wake (1). */
void sleep_deep_record(uint8_t *out24);

/* Each main-loop pass, at its very end (after the watchdog kick). */
void sleep_task(void);
