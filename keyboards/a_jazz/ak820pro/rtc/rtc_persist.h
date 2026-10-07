// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* When the RTC divider's period is SAVED (Phase 1b, C2;
 * plans/BATTERY-GAUGE-REFINE-PLAN.md).
 *
 * Correcting and persisting used to be one step: each correction path (the
 * USB-SOF window, the PCF trim) wrote the EEPROM whenever its value moved >= 64
 * ticks from the stored one. Every internal-flash write blanks the LEDs for a
 * few ms, and on battery the PCF path did it 6-8 times a day (C1, the soaks of
 * 2026-10-05/07). The correction paths now only PROPOSE: they still correct
 * the live period exactly as before, and each accepted value becomes the
 * candidate here. This scheduler owns every write:
 *   - first save: nothing stored (0, or outside 28000-40000) and uptime >= 10
 *     min -- whether or not the candidate is still changing, which is the
 *     starvation trap rtc.c's SOF comment documents (a converged loop that
 *     never stored anything);
 *   - later saves: |candidate - stored| >= 64 AND >= 6 h since this boot's
 *     last save (or since boot): <= 4 a day from this field, one budget for
 *     both paths.
 * The persisted value only seeds the NEXT boot; the live period is corrected
 * regardless, so holding a save for hours costs nothing but a slightly worse
 * seed. */

#define RTC_PERSIST_FIRST_MS  600000u                 /* 10 min of uptime */
#define RTC_PERSIST_MIN_DELTA 64u                     /* ticks, ~0.2% */
#define RTC_PERSIST_GAP_MS    (6u * 3600u * 1000u)    /* 6 h: <= 4 saves a day */
#define RTC_PERSIST_CHECK_MS  60000u                  /* the scheduler runs once a minute */
#define RTC_PERIOD_SANE_MIN   28000u
#define RTC_PERIOD_SANE_MAX   40000u

/* Pure, for the host tests: save `cand` now? `stored` is the EEPROM's value;
 * `last_save_ms` is the uptime of this boot's last save, 0 for none (the gap
 * then counts from boot). A cand outside the sane window (0: none) never is. */
bool rtc_persist_due(uint32_t now_ms, uint16_t stored, uint16_t cand, uint32_t last_save_ms);

/* A correction path accepted `period` (FLASH_RTC_PATH_SOF or _PCF). Counted as
 * a proposal (HC_FLASHW); kept as the candidate if sane. Never writes. */
void rtc_persist_propose(uint16_t period, uint8_t path);

/* From rtc_task; runs its check once a minute and makes the one write. */
void rtc_persist_task(void);
