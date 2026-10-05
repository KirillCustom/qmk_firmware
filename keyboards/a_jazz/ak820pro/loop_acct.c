// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Per-task duration accounting -- see loop_acct.h. */
#include "quantum.h"
#include "loop_acct.h"

#define SLOW_MS  10u
#define RING_LEN 16u
/* The 16-bit tick wraps every 349.5 ms, so a scope that long would read as
 * its remainder and its time would land in "unaccounted" (codex, gate 2). A
 * scope can only be that long inside a pass at least that long, so such a
 * pass is counted, ringed with 0xFF scope bytes, and kept out of the scope
 * totals and the largest-scope counts. */
#define ALIAS_MS 340u
#define ST_HZ    ((uint32_t)CH_CFG_ST_FREQUENCY)
/* ms <-> ticks in 32 bits: ticks = ms * (ST_HZ / 500) / 2, exact at 187,500 Hz
 * (375 / 2), and a 60 s pass is 22.5 M -- no overflow, no 64-bit divide on
 * the per-pass path. */
_Static_assert(CH_CFG_ST_FREQUENCY % 500 == 0, "the ms <-> tick arithmetic below wants ST_HZ / 500 whole");
#define ST_PER_2MS (ST_HZ / 500u)

bool loop_acct_on = true;   /* default on in the diagnostic build; gate 3 turns it off for the A/B */

/* Every pass: the scopes add into loop_acct_run (ACCT, inline), and the pass
 * end copies it to pass_base -- that is all. A slow pass's own ticks are
 * run - pass_base. The 64-bit all-pass totals are folded in every
 * ACCT_FOLD_PASSES passes from run - fold_base, so the 32-bit differences
 * cannot wrap: 4096 passes of < 340 ms each is < 261 M ticks a scope. */
#define ACCT_FOLD_PASSES 4096u
uint32_t        loop_acct_run[ACCT_N];
static uint32_t pass_base[ACCT_N];           /* loop_acct_run at the last pass end */
static uint32_t fold_base[ACCT_N];           /* ... at the last fold */
static uint32_t fold_gap_ms;                 /* counted passes' ms since the last fold */
static uint16_t fold_n;
static uint32_t slow_ticks[ACCT_REPORTED];   /* summed over slow passes */
static uint64_t all_ticks[ACCT_REPORTED];    /* summed over every counted pass: u64,
                                             * since a u32 of ticks overflows in ~6 h */
static uint16_t top_count[ACCT_REPORTED];
static uint32_t passes, slow_passes, ring_written, long_passes;
/* The pass in progress when the accounting was reset, enabled or disabled
 * began before it: its gap is health.c's, but its scopes are not all here. */
static bool     skip_next = true;   /* and at boot, until the first closed pass */

typedef struct {
    uint32_t uptime_ms;
    uint8_t  pass_ms;
    uint8_t  scope_ms[ACCT_REPORTED];
} ring_entry_t;
static ring_entry_t ring[RING_LEN];
static uint8_t      ring_next;

static uint8_t ticks_to_ms8(uint32_t t) {
    if (t > 60000u * ST_PER_2MS) return 255u;   /* past a minute: saturated anyway */
    uint32_t ms = (t * 2u + ST_PER_2MS / 2u) / ST_PER_2MS;
    return (uint8_t)(ms > 255u ? 255u : ms);
}

/* Drop the scope time since the last pass end from every total: a pass that is
 * not counted (skipped) or not attributable (wrapped), or the partial pass at
 * an on/off or a discard. */
static void drop_partial(void) {
    for (uint8_t s = 0; s < ACCT_N; s++) {
        fold_base[s] += loop_acct_run[s] - pass_base[s];
        pass_base[s] = loop_acct_run[s];
    }
}

/* The all-pass totals: the scopes' ticks since the last fold, and the counted
 * passes' length less them as UNACCOUNTED (clipped at 0 over the fold, where
 * the old per-pass version clipped each pass: the same to within each gap's
 * +-1 ms). */
static void fold(void) {
    uint32_t accounted = 0;
    for (uint8_t s = 0; s < ACCT_N; s++) {
        uint32_t d = loop_acct_run[s] - fold_base[s];
        all_ticks[s] += d;
        accounted += d;
        fold_base[s] = loop_acct_run[s];
    }
    uint32_t pass_t = fold_gap_ms * ST_PER_2MS / 2u;   /* < 4096 x 340 ms: < 261 M */
    all_ticks[ACCT_UNACCOUNTED] += pass_t > accounted ? pass_t - accounted : 0u;
    fold_gap_ms = 0;
    fold_n      = 0;
}

static void ring_put(uint8_t pass_ms, const uint32_t *ticks, uint32_t unacc) {
    ring_entry_t *e = &ring[ring_next];
    e->uptime_ms = timer_read32();
    if (e->uptime_ms == 0) e->uptime_ms = 1;
    e->pass_ms = pass_ms;
    if (ticks) {
        for (uint8_t s = 0; s < ACCT_N; s++) e->scope_ms[s] = ticks_to_ms8(ticks[s]);
        e->scope_ms[ACCT_UNACCOUNTED] = ticks_to_ms8(unacc);
    } else {
        memset(e->scope_ms, 0xFF, sizeof e->scope_ms);
    }
    ring_next = (uint8_t)((ring_next + 1u) % RING_LEN);
    ring_written++;
}

/* A slow pass (>= SLOW_MS, < ALIAS_MS): its own ticks, attributed. */
static void slow_pass(uint32_t gap_ms) {
    uint32_t t[ACCT_N], accounted = 0;
    for (uint8_t s = 0; s < ACCT_N; s++) {
        t[s] = loop_acct_run[s] - pass_base[s];
        accounted += t[s];
    }
    /* The pass length is health.c's millisecond gap, so a slow pass here is
     * exactly one of its count_ge_10ms; the unaccounted remainder inherits its
     * +-1 ms. */
    uint32_t pass_t = gap_ms * ST_PER_2MS / 2u;
    uint32_t unacc  = pass_t > accounted ? pass_t - accounted : 0u;
    slow_passes++;
    uint8_t  top   = ACCT_UNACCOUNTED;
    uint32_t top_t = unacc;
    for (uint8_t s = 0; s < ACCT_N; s++) {
        slow_ticks[s] += t[s];
        if (t[s] > top_t) { top_t = t[s]; top = s; }
    }
    slow_ticks[ACCT_UNACCOUNTED] += unacc;
    if (top_count[top] < UINT16_MAX) top_count[top]++;
    ring_put((uint8_t)(gap_ms > 255u ? 255u : gap_ms), t, unacc);
}

void loop_acct_discard(void) {
    drop_partial();
    skip_next = true;
}

void loop_acct_pass_end(uint32_t gap_ms) {
    if (!loop_acct_on) return;   /* nothing accumulates while off; on resyncs */
    if (skip_next) {
        drop_partial();
        skip_next = false;
        return;
    }
    passes++;
    if (gap_ms >= ALIAS_MS) {   /* scope times may have wrapped: attribute nothing */
        drop_partial();
        long_passes++;
        slow_passes++;
        ring_put(255u, NULL, 0);
        return;
    }
    if (gap_ms >= SLOW_MS) slow_pass(gap_ms);
    /* The common case is only this: the copy, two adds and a compare. */
    memcpy(pass_base, loop_acct_run, sizeof pass_base);
    fold_gap_ms += gap_ms;
    if (++fold_n >= ACCT_FOLD_PASSES) fold();
}

void loop_acct_reset(void) {
    memset(loop_acct_run, 0, sizeof loop_acct_run);
    memset(pass_base, 0, sizeof pass_base);
    memset(fold_base, 0, sizeof fold_base);
    memset(slow_ticks, 0, sizeof slow_ticks);
    memset(all_ticks, 0, sizeof all_ticks);
    memset(top_count, 0, sizeof top_count);
    memset(ring, 0, sizeof ring);
    passes = slow_passes = ring_written = long_passes = 0;
    fold_gap_ms = 0;
    fold_n      = 0;
    ring_next   = 0;
    skip_next   = true;
}

void loop_acct_set_enabled(bool on) {
    drop_partial();   /* the pass in progress is not counted either way */
    loop_acct_on = on;
    skip_next    = true;
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
        PUT32(long_passes);
    } else if (page == 1 || page == 2) {
        uint8_t first = (page == 1) ? 0u : 7u;
        for (uint8_t s = first; s < first + 7u && s < ACCT_REPORTED; s++) PUT32(slow_ticks[s]);
    } else if (page == 3 || page == 4) {
        /* The folded totals plus what is pending since the last fold, read
         * without folding (this runs mid-pass, from raw HID). */
        uint64_t all[ACCT_REPORTED];
        uint32_t accounted = 0;
        for (uint8_t s = 0; s < ACCT_N; s++) {
            uint32_t d = loop_acct_run[s] - fold_base[s];
            all[s] = all_ticks[s] + d;
            accounted += d;
        }
        uint32_t pass_t = fold_gap_ms * ST_PER_2MS / 2u;
        all[ACCT_UNACCOUNTED] = all_ticks[ACCT_UNACCOUNTED] + (pass_t > accounted ? pass_t - accounted : 0u);
        uint8_t first = (page == 3) ? 0u : 7u;
        for (uint8_t s = first; s < first + 7u && s < ACCT_REPORTED; s++)
            PUT32((uint32_t)(all[s] * 1000u / ST_HZ));   /* ms */
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
