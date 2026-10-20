// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"   /* st_lld_get_counter(), inline */

/* Per-task duration accounting for the main loop (Phase 1b, D1;
 * plans/BATTERY-GAUGE-REFINE-PLAN.md).
 *
 * The stall marks (LOOP_MARK_*) say which wait a slow pass ENDED in, not
 * where its time went: a 48 ms pass ending in a 2 ms clear reports as
 * "blit". This times every board task in the loop, in MUTUALLY EXCLUSIVE
 * scopes, and attributes each slow pass (>= 10 ms, health.c's count_ge_10ms
 * passes exactly) to all of them at once: a stall made of 8 ms of display
 * plus 3 ms of battery shows as both. Whatever no scope covers -- QMK's
 * keyboard_task, the RGB effect, USB, raw HID -- is UNACCOUNTED, the pass
 * length less the scopes.
 *
 * Timestamps are the 16-bit system tick at CH_CFG_ST_FREQUENCY (5.33 us),
 * read INLINE with st_lld_get_counter() -- the counter chVTGetSystemTimeX()
 * returns through an out-of-line stGetCounter() call. Not timer_read32(),
 * which takes a lock. An earlier per-pass site profiler cost ~2 ms a pass,
 * which is the fault it was looking for (ak820pro.h, LOOP_SITE); this one is
 * gated on a runtime flag so its cost can be measured on and off (gate 3).
 *
 * ⚠️ Gate 3 (2026-10-05) measured the first version at 0.092 ms a pass,
 * 2.5% of the main loop's passes (the gate is 1%). It made three calls per
 * scope, and every pass summed, added in 64 bits and cleared all twelve
 * scopes. Main-loop cycles are dear here: the row ISR takes ~73% of the CPU,
 * so each one costs ~3.7x in wall time. So now: no calls, one 32-bit add per
 * scope into a running total, one 48-byte copy per pass, and the per-scope
 * work only on the rare slow pass and on a fold into the 64-bit totals every
 * ACCT_FOLD_PASSES passes.
 *
 * Main-loop only. Counters are since boot or since health_reset(). */

enum acct_scope {
    ACCT_CH582 = 0,   /* bluetooth_task: the CH582F RX drain, polls, TX pump */
    ACCT_BATT,        /* battery_task + power_task */
    ACCT_DISP_HK,     /* display_housekeeping_task: the panel's 10 Hz work */
    ACCT_PUMP,        /* display_blit_pump: one glyph a pass, the Fn+D clears */
    ACCT_RTC,         /* rtc_task: the minute check, PCF I2C, the trim */
    ACCT_RTC_FAST,    /* rtc_fast_task: <= one queued PCF transaction a pass */
    ACCT_LEDS,        /* update_leds: the indicators */
    ACCT_HEALTH,      /* health_task */
    ACCT_EECFG,       /* kb_eeconfig_task: the deferred kb-config flush */
    ACCT_EDGE,        /* display_second_edge_task: the clock digits on the tick */
    ACCT_ANIM,        /* anim_task */
    ACCT_HK_OTHER,    /* the rest of the 10 Hz block: param status, the hold
                       * tasks, the consumer sequencer. The per-pass param
                       * repeat and user hook are NOT timed (trimmed after
                       * gate 3): they fall in UNACCOUNTED. */
    ACCT_N,
    ACCT_UNACCOUNTED = ACCT_N,   /* reported, never a scope */
    ACCT_REPORTED
};

extern bool     loop_acct_on;
/* Running per-scope ticks since boot, wrapping at 2^32 (read only as
 * differences). The scope index is a constant at every ACCT(), so the add is a
 * load, an add and a store at a fixed address. */
extern uint32_t loop_acct_run[ACCT_N];

#define ACCT_NOW() ((uint16_t)st_lld_get_counter())
/* Dev builds: each scope is also published, while it runs, as the overrun
 * monitor's observed context (overrun_diag.h's od_ctx; ACCT_UNACCOUNTED
 * between scopes). Two stores a scope; the scopes do not nest. */
#ifdef AK820_DEV_HOOKS
extern volatile uint8_t od_ctx;
#    define ACCT_CTX(scope) (od_ctx = (uint8_t)(scope))
#else
#    define ACCT_CTX(scope) ((void)0)
#endif
#define ACCT(scope, call)                                                          \
    do {                                                                           \
        ACCT_CTX(scope);                                                           \
        if (loop_acct_on) {                                                        \
            uint16_t acct_t0_ = ACCT_NOW();                                        \
            call;                                                                  \
            loop_acct_run[(scope)] += (uint16_t)(ACCT_NOW() - acct_t0_);           \
        } else {                                                                   \
            call;                                                                  \
        }                                                                          \
        ACCT_CTX(ACCT_UNACCOUNTED);                                                \
    } while (0)

/* health_loop_tick(): one pass ended, `gap_ms` long (its own measurement). A
 * pass of ALIAS_MS (340 ms) or more is counted as slow and ringed with 0xFF
 * scope bytes, but kept out of the scope totals: the 16-bit tick may have
 * wrapped inside it. */
void loop_acct_pass_end(uint32_t gap_ms);
/* health_loop_tick() before its settle: drop whatever scope time accumulated,
 * and do not count the pass that follows (it began before the drop). */
void loop_acct_discard(void);
void loop_acct_reset(void);
void loop_acct_set_enabled(bool on);

/* Raw HID HC_ACCT pages (hid_protocol.c), 28 bytes each, little-endian.
 * Scope order is enum acct_scope, with UNACCOUNTED last (ACCT_REPORTED = 13).
 *   page 0: u8 enabled, u8 ACCT_REPORTED, u8 ring length (16), u8 next ring
 *           slot; u32 passes, u32 slow passes, u32 CH_CFG_ST_FREQUENCY,
 *           u32 uptime_ms, u32 ring entries ever written, u32 long passes
 *           (>= 340 ms: counted slow, not attributed). The pass after a
 *           reset or an on/off is not counted at all.
 *   page 1: u32 slow-pass ticks, scopes 0-6     page 2: scopes 7-12
 *   page 3: u32 all-pass MILLISECONDS, scopes 0-6   page 4: scopes 7-12
 *   page 5: u16 times each scope was the largest in a slow pass, 0-12
 *   pages 16-31: ring slot 0-15: u32 uptime_ms at the pass's end (0: empty),
 *           u8 pass ms, u8 ms per scope 0-12 (saturating at 255)
 * Slow-pass totals are whole system ticks (the host converts with the
 * frequency); all-pass totals are ms, since they run for hours. */
bool loop_acct_fill(uint8_t page, uint8_t *out28);
