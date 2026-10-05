// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Per-task duration accounting -- see loop_acct.h. */
#include "quantum.h"
#include "loop_acct.h"

#define SLOW_MS  10u
#define RING_LEN 16u
#define ST_HZ    ((uint32_t)CH_CFG_ST_FREQUENCY)
/* ms <-> ticks in 32 bits: ticks = ms * (ST_HZ / 500) / 2, exact at 187,500 Hz
 * (375 / 2), and a 60 s pass is 22.5 M -- no overflow, no 64-bit divide on
 * the per-pass path. */
_Static_assert(CH_CFG_ST_FREQUENCY % 500 == 0, "the ms <-> tick arithmetic below wants ST_HZ / 500 whole");
#define ST_PER_2MS (ST_HZ / 500u)

bool loop_acct_on = true;   /* default on in the diagnostic build; gate 3 turns it off for the A/B */

static uint32_t pass_ticks[ACCT_N];          /* this pass, per scope */
static uint32_t slow_ticks[ACCT_REPORTED];   /* summed over slow passes */
static uint64_t all_ticks[ACCT_REPORTED];    /* summed over every pass: u64, since
                                             * a u32 of ticks overflows in ~6 h */
static uint16_t top_count[ACCT_REPORTED];
static uint32_t passes, slow_passes, ring_written;

typedef struct {
    uint32_t uptime_ms;
    uint8_t  pass_ms;
    uint8_t  scope_ms[ACCT_REPORTED];
} ring_entry_t;
static ring_entry_t ring[RING_LEN];
static uint8_t      ring_next;

void loop_acct_add(uint8_t scope, sysinterval_t ticks) {
    if (scope < ACCT_N) pass_ticks[scope] += ticks;
}

static uint8_t ticks_to_ms8(uint32_t t) {
    if (t > 60000u * ST_PER_2MS) return 255u;   /* past a minute: saturated anyway */
    uint32_t ms = (t * 2u + ST_PER_2MS / 2u) / ST_PER_2MS;
    return (uint8_t)(ms > 255u ? 255u : ms);
}

void loop_acct_pass_end(uint32_t gap_ms) {
    if (!loop_acct_on) {
        memset(pass_ticks, 0, sizeof pass_ticks);
        return;
    }
    uint32_t accounted = 0;
    for (uint8_t s = 0; s < ACCT_N; s++) accounted += pass_ticks[s];
    /* The pass length is health.c's millisecond gap, so a slow pass here is
     * exactly one of its count_ge_10ms; the unaccounted remainder inherits its
     * +-1 ms. */
    uint32_t pass_t = (gap_ms > 60000u ? 60000u : gap_ms) * ST_PER_2MS / 2u;
    uint32_t unacc  = pass_t > accounted ? pass_t - accounted : 0u;

    passes++;
    for (uint8_t s = 0; s < ACCT_N; s++) all_ticks[s] += pass_ticks[s];
    all_ticks[ACCT_UNACCOUNTED] += unacc;

    if (gap_ms >= SLOW_MS) {
        slow_passes++;
        uint8_t  top = ACCT_UNACCOUNTED;
        uint32_t top_t = unacc;
        for (uint8_t s = 0; s < ACCT_N; s++) {
            slow_ticks[s] += pass_ticks[s];
            if (pass_ticks[s] > top_t) { top_t = pass_ticks[s]; top = s; }
        }
        slow_ticks[ACCT_UNACCOUNTED] += unacc;
        if (top_count[top] < UINT16_MAX) top_count[top]++;

        ring_entry_t *e = &ring[ring_next];
        e->uptime_ms = timer_read32();
        if (e->uptime_ms == 0) e->uptime_ms = 1;
        e->pass_ms = (uint8_t)(gap_ms > 255u ? 255u : gap_ms);
        for (uint8_t s = 0; s < ACCT_N; s++) e->scope_ms[s] = ticks_to_ms8(pass_ticks[s]);
        e->scope_ms[ACCT_UNACCOUNTED] = ticks_to_ms8(unacc);
        ring_next = (uint8_t)((ring_next + 1u) % RING_LEN);
        ring_written++;
    }
    memset(pass_ticks, 0, sizeof pass_ticks);
}

void loop_acct_reset(void) {
    memset(pass_ticks, 0, sizeof pass_ticks);
    memset(slow_ticks, 0, sizeof slow_ticks);
    memset(all_ticks, 0, sizeof all_ticks);
    memset(top_count, 0, sizeof top_count);
    memset(ring, 0, sizeof ring);
    passes = slow_passes = ring_written = 0;
    ring_next = 0;
}

void loop_acct_set_enabled(bool on) {
    loop_acct_on = on;
    memset(pass_ticks, 0, sizeof pass_ticks);
}

bool loop_acct_fill(uint8_t page, uint8_t *out28) {
    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    memset(out28, 0, 28);
    if (page == 0) {
        *p++ = loop_acct_on ? 1u : 0u;
        *p++ = ACCT_REPORTED;
        *p++ = RING_LEN;
        *p++ = ring_next;
        PUT32(passes); PUT32(slow_passes); PUT32(ST_HZ); PUT32(timer_read32()); PUT32(ring_written);
    } else if (page == 1 || page == 2) {
        uint8_t first = (page == 1) ? 0u : 7u;
        for (uint8_t s = first; s < first + 7u && s < ACCT_REPORTED; s++) PUT32(slow_ticks[s]);
    } else if (page == 3 || page == 4) {
        uint8_t first = (page == 3) ? 0u : 7u;
        for (uint8_t s = first; s < first + 7u && s < ACCT_REPORTED; s++)
            PUT32((uint32_t)(all_ticks[s] * 1000u / ST_HZ));   /* ms */
    } else if (page == 5) {
        for (uint8_t s = 0; s < ACCT_REPORTED; s++) PUT16(top_count[s]);
    } else if (page >= 16 && page < 16u + RING_LEN) {
        const ring_entry_t *e = &ring[page - 16u];
        PUT32(e->uptime_ms);
        *p++ = e->pass_ms;
        for (uint8_t s = 0; s < ACCT_REPORTED; s++) *p++ = e->scope_ms[s];
    } else {
        return false;
    }
#undef PUT16
#undef PUT32
    return true;
}
