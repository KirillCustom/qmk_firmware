// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Hardware watchdog (SN32 WDT via the ChibiOS WDG driver).
 *
 * Call order: watchdog_boot_check() once, early in post-init (reads and
 * clears the reset-cause flag, maintains the consecutive-reset counter);
 * watchdog_start() at the END of post-init; watchdog_kick() at the END of
 * housekeeping_task_kb(), after housekeeping_task_user(). */
void watchdog_boot_check(void);
void watchdog_start(void);
void watchdog_kick(void);
void watchdog_stop(void);

/* Consecutive WDT resets leading into this boot (0 on a normal boot). */
uint8_t watchdog_reset_count(void);
/* Raw RSTST snapshot from boot: bit0 SW, bit1 WDT, bit2 LVD (brownout),
 * bit3 EXT, bit4 POR. Names the cause of slider power-switchover resets. */
uint8_t watchdog_boot_rstst(void);
/* This boot immediately followed a WDT reset. */
bool watchdog_fired_last_boot(void);
/* Too many consecutive WDT resets: the watchdog stays unarmed this boot so
 * a deterministic boot-time failure cannot become an endless reset loop. */
bool watchdog_degraded(void);
/* Armed and counting: what a deep sleep's recovery relies on. */
bool watchdog_running(void);
