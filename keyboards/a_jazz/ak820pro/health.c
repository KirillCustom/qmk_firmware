// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Unified health counters -- see health.h for the design rules. */
#include "quantum.h"
#include "health.h"
#include "ak820pro.h"   /* LOOP_MARK_*, loop_stall_mark */
#include "watchdog.h"
#include "watchdog_record.h"
#include "graphics/lcd_bus.h"
#include "bluetooth/ch582f_ajazz.h"
#include "loop_acct.h"     /* D1: the per-task attribution of each pass */
#include "flash_stats.h"   /* C1: which writer each flash session belongs to */

/* Worst main-loop gap since boot, max-hold. Measured unconditionally: one
 * timer read and a compare per pass. The first 10 s are ignored -- boot
 * blocks deliberately (lcd_init alone spends 240 ms in wait_ms), and a
 * max-hold seeded by boot would never report the steady state again. */
#define HEALTH_SETTLE_MS 10000u

static uint32_t loop_gap_max = 0;
static uint32_t rx_malformed = 0;

/* Phase-1 stall measurement. Counters, not a histogram: the question is "how
 * often does a keystroke-eating stall happen, and what was it", which buckets
 * answer worse than thresholds do. All of this is a compare and an increment
 * on a timer read that already happens -- no new timer read per pass. */
static uint32_t count_ge_10ms = 0, count_ge_25ms = 0, passes = 0;
static uint32_t flash_writes = 0;
/* Per-mark maxima are u16: a gap needing 65 s would be a hang, not a stall,
 * and the narrower fields buy room on a page that was exactly full. */
static uint16_t flash_gap_max = 0, blit_gap_max = 0, i2c_gap_max = 0;
/* The gate's discriminator. A >=25 ms stall attributed to FLASH is a
 * wear-levelling consolidation: understood, bounded, and expected whenever a
 * test writes enough entries. Anything else at that length is a stall we
 * cannot explain, and that is what must never regress. */
static uint16_t count_ge_25ms_nonflash = 0;
static uint16_t key_presses = 0;

/* ---- Per-row input sampling (LOOP-BUDGET phase 1b) --------------------------
 *
 * Until 2026-09-03 the SN32 driver published after scanning ONE row, so
 * scan_rate (which counts matrix_scan() calls) was NOT a full-matrix refresh
 * rate, and an aliasing beat between the consume rate and the PWM row cycle
 * left a key unsampled for up to 169 ms. These counters found that; the driver
 * now scans every row every cycle (firmware e6cce28858) and they stay to keep
 * it measurable. Expected today: every row sampled ~215 times a second, i.e.
 * ~4 rows per main-loop consume, worst gap ~5 ms outside flash programming.
 *
 * row_samples  -- fairness: are all rows sampled equally often? u16: wraps
 *                 every ~5 min at 215/s; row_samples_total (page 4) is u32.
 * row_gap_max  -- worst observed interval between samples of the SAME row,
 *                 kept in system ticks and converted to ms at read time
 * raw_edges    -- raw matrix transitions seen at consume time, i.e. BEFORE
 *                 debounce. Compare against key_presses (which is AFTER
 *                 debounce, in process_record_kb): a press and its release are
 *                 two raw edges, so raw_edges ~= 2 x key_presses when nothing
 *                 is being eaten in between. */
static uint16_t row_samples[MATRIX_ROWS];
static uint32_t row_samples_total = 0;
static uint16_t row_gap_max_ticks = 0;
static uint8_t  row_gap_max_row = 0;
static uint32_t raw_edges = 0, consumes = 0, cooked_changes = 0;
static volatile uint8_t last_row_scanned = 0;

/* ---- Row-ISR cost ------------------------------------------------------------
 *
 * rgb_callback() re-arms the PWM counter at its END, so its period is (ISR
 * duration + ~53 us), and the ISR's own cost -- not the timer -- sets the row
 * rate. The timer arithmetic predicts 18,750 ISR/s; the per-row counters above
 * measured ~3,870. That is the difference between "1 ms" and "4.6 ms" per key,
 * and it is also the ceiling on the main loop, so it is measured here rather
 * than argued: entries, summed and min/max duration, in system ticks
 * (CH_CFG_ST_FREQUENCY = 187,500 Hz -> 5.33 us resolution). Two tick reads and
 * a few adds per ISR. */
static uint32_t  isr_entries = 0, isr_ticks_sum = 0;
static uint16_t  isr_ticks_min = 0xFFFFu, isr_ticks_max = 0;
static systime_t isr_t_enter;

void sn32f2xx_isr_enter_hook(void) { isr_t_enter = chVTGetSystemTimeX(); }

void sn32f2xx_isr_exit_hook(void) {
    sysinterval_t d = chTimeDiffX(isr_t_enter, chVTGetSystemTimeX());
    isr_entries++;
    isr_ticks_sum += d;
    if (d > isr_ticks_max) isr_ticks_max = (uint16_t)d;
    if (d < isr_ticks_min) isr_ticks_min = (uint16_t)d;
}

void input_note_row_scan(uint8_t row) {
    /* ISR context. Timing lives here because the event happens here: the ISR
     * scans ~4 rows per consume, so a consume-side timestamp would refresh one
     * row's clock and report nonsense (it showed 159-308 ms while sampling had
     * actually improved). chVTGetSystemTimeX() is the ISR-safe (X-suffixed)
     * ChibiOS time source; timer_read32() takes a lock and must not be called
     * from here.
     *
     * systime_t is 16 bits at 187.5 kHz on this chip: chTimeDiffX() handles the
     * wrap, and the longest representable gap is ~349 ms. A real gap beyond
     * that would alias -- but the only thing that pauses row scanning is flash
     * programming (tens of ms), so anything longer is a dead ISR, which the
     * watchdog reports separately. */
    if (row >= MATRIX_ROWS) return;
    static systime_t row_last[MATRIX_ROWS];
    static uint8_t   row_primed = 0;   /* bit per row: row_last[] is valid */
    systime_t now = chVTGetSystemTimeX();

    last_row_scanned = row;
    row_samples[row]++;
    row_samples_total++;
    if (row_primed & (1u << row)) {
        sysinterval_t d = chTimeDiffX(row_last[row], now);
        if (d > row_gap_max_ticks) {
            row_gap_max_ticks = (uint16_t)d;
            row_gap_max_row   = row;
        }
    } else {
        row_primed |= (uint8_t)(1u << row);
    }
    row_last[row] = now;
}

void input_note_consume(const matrix_row_t *raw, const matrix_row_t *fresh, uint8_t rows) {
    /* Main-loop context: safe to read the timer here. */
    consumes++;
    /* Per-row gap timing moved into the ISR -- see input_note_row_scan(). */

    /* Raw edges: population count of the XOR, per row, before the copy. */
    for (uint8_t i = 0; i < rows; i++) {
        matrix_row_t diff = raw[i] ^ fresh[i];
        if (!diff) continue;
        cooked_changes++;              /* rows differing at consume time */
        while (diff) { raw_edges += (diff & 1u); diff >>= 1; }
    }
}
static uint8_t  loop_gap_max_mark = LOOP_MARK_NONE;
static uint8_t  last_mark = LOOP_MARK_NONE;

void health_loop_tick(void) {
    static uint32_t last = 0;
    uint32_t now = timer_read32();
    watchdog_record_uptime(now); /* reuse this timer read; no per-scope timers */

    /* Sole owner of loop_stall_mark: read once, clear once, publish via
     * health_last_mark(). This runs BEFORE loop_gap_task() in the same pass,
     * so a second reader of loop_stall_mark would always see NONE. */
    last_mark = loop_stall_mark;
    loop_stall_mark = LOOP_MARK_NONE;
    flash_stats_pass();

    if (now < HEALTH_SETTLE_MS) {
        last = now;
        loop_acct_discard();   /* boot's scope time is not the first counted pass's */
        return;
    }
    uint32_t gap = now - last;
    last = now;
    passes++;
    if (gap >= 10) count_ge_10ms++;
    if (gap >= 25) count_ge_25ms++;
    if (gap > loop_gap_max) { loop_gap_max = gap; loop_gap_max_mark = last_mark; }
    uint16_t g16 = (gap > 0xFFFFu) ? 0xFFFFu : (uint16_t)gap;
    switch (last_mark) {
        case LOOP_MARK_FLASH: if (g16 > flash_gap_max) flash_gap_max = g16; break;
        case LOOP_MARK_BLIT:  if (g16 > blit_gap_max)  blit_gap_max  = g16; break;
        case LOOP_MARK_I2C:   if (g16 > i2c_gap_max)   i2c_gap_max   = g16; break;
        default: break;
    }
    if (gap >= 25 && last_mark != LOOP_MARK_FLASH) count_ge_25ms_nonflash++;
    loop_acct_pass_end(gap);   /* the same pass and gap, so its slow passes are count_ge_10ms's */
}

uint8_t health_last_mark(void) { return last_mark; }

void health_note_flash_write(void) { flash_writes++; }
void health_note_key_press(void)   { key_presses++; }
uint16_t health_key_presses(void)  { return key_presses; }

void health_reset(void) {
    /* Watchdog counters are boot facts and are NOT cleared -- see health.h. */
    chSysLock();
    loop_gap_max = 0; loop_gap_max_mark = LOOP_MARK_NONE;
    count_ge_10ms = 0; count_ge_25ms = 0; passes = 0;
    flash_writes = 0; flash_gap_max = 0; blit_gap_max = 0; i2c_gap_max = 0;
    count_ge_25ms_nonflash = 0; key_presses = 0; rx_malformed = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) row_samples[i] = 0;
    row_samples_total = 0;
    row_gap_max_ticks = 0; row_gap_max_row = 0;
    raw_edges = 0; consumes = 0; cooked_changes = 0;
    isr_entries = 0; isr_ticks_sum = 0; isr_ticks_min = 0xFFFFu; isr_ticks_max = 0;
    chSysUnlock();
    loop_acct_reset();     /* main-loop state, like everything they count */
    flash_stats_reset();
}

void health_fill2(uint8_t *out28) {
    uint32_t c10, c25, ps, fw;
    uint16_t fg, bg, ig, nf, kp;
    uint8_t  mk;
    chSysLock();
    c10 = count_ge_10ms; c25 = count_ge_25ms; ps = passes; fw = flash_writes;
    fg  = flash_gap_max; bg  = blit_gap_max;  ig = i2c_gap_max;
    nf  = count_ge_25ms_nonflash; kp = key_presses; mk = loop_gap_max_mark;
    chSysUnlock();

    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    PUT32(c10); PUT32(c25); PUT32(ps); PUT32(fw);          /* 16 */
    PUT16(fg);  PUT16(bg);  PUT16(ig);                      /* 22 */
    PUT16(nf);  PUT16(kp);                                  /* 26 */
#undef PUT16
#undef PUT32
    *p++ = mk;
    *p++ = 0;   /* reserved */
}

void health_note_rx_malformed(void) {
    rx_malformed++;
}

void health_fill(uint8_t *out28) {
    /* Snapshot everything inside one short critical section so the reply is
     * one instant, not a mix. Today every source is main-loop-owned and the
     * mask is technically free insurance -- but rx_malformed is documented
     * as a future parser hook and scan rate lives in platform code, so the
     * atomicity is part of the contract, not an optimisation. */
    uint32_t sent, to, drop, blit, gap, malf;
    chSysLock();
    ch582_tx_stats(&sent, &to, &drop);
    blit = lcd_blit_timeouts();
    gap  = loop_gap_max;
    malf = rx_malformed;
#ifdef DEBUG_MATRIX_SCAN_RATE
    uint16_t sr = (uint16_t)get_matrix_scan_rate();
#else
    uint16_t sr = 0;
#endif
    chSysUnlock();

    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
    PUT32(blit);
    PUT32(sent);
    PUT32(to);
    PUT32(drop);
    PUT32(malf);
    PUT32(gap);
#undef PUT32
    *p++ = sr & 0xFF;
    *p++ = (sr >> 8) & 0xFF;
    *p++ = watchdog_reset_count();
    *p++ = (watchdog_fired_last_boot() ? 1 : 0) | (watchdog_degraded() ? 2 : 0);
}

void health_task(void) {
#ifdef CONSOLE_ENABLE
    /* On-meaningful-change only. Scan rate is quantised into 25 Hz bands and
     * the loop gap is already a max-hold, so a healthy board prints this a
     * handful of times after boot and then goes quiet -- console traffic
     * itself perturbs the host (2026-08-30 incident). */
    static uint32_t s_blit = 0, s_to = 0, s_drop = 0, s_malf = 0, s_gap = 0;
    static uint16_t s_band = 0;
    static uint8_t  s_wdt = 0;
    static bool     primed = false;

    uint32_t sent, to, drop;
    ch582_tx_stats(&sent, &to, &drop);
    uint32_t blit = lcd_blit_timeouts();
    uint8_t  wdt  = watchdog_reset_count();
#ifdef DEBUG_MATRIX_SCAN_RATE
    uint16_t band = (uint16_t)(get_matrix_scan_rate() / 25);
#else
    uint16_t band = 0;
#endif

    /* `sent` is printed as context but deliberately does NOT trigger a line
     * -- it rises on every BT keystroke and would make the output continuous. */
    bool changed = blit != s_blit || to != s_to || drop != s_drop ||
                   rx_malformed != s_malf || loop_gap_max != s_gap ||
                   band != s_band || wdt != s_wdt;
    if (!primed) {
        /* Skip the boot transient; report only changes after settle. */
        if (timer_read32() < HEALTH_SETTLE_MS) return;
        primed = true;
        changed = true; /* one baseline line so the log shows the healthy state */
    }
    if (!changed) return;

    s_blit = blit; s_to = to; s_drop = drop;
    s_malf = rx_malformed; s_gap = loop_gap_max; s_band = band; s_wdt = wdt;

    printf("[health] blit_to=%lu tx=%lu to=%lu drop=%lu malf=%lu gap=%lums scan~%u wdt=%u%s%s\n",
           blit, sent, to, drop, rx_malformed, loop_gap_max,
           (unsigned)(band * 25), watchdog_reset_count(),
           watchdog_fired_last_boot() ? " FIRED" : "",
           watchdog_degraded() ? " DEGRADED" : "");
#endif
}

void health_fill3(uint8_t *out28) {
    uint16_t rs[MATRIX_ROWS], gm;
    uint8_t  gr;
    uint32_t re, cs, cc;
    chSysLock();
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) rs[i] = row_samples[i];
    gm = row_gap_max_ticks; gr = row_gap_max_row;
    re = raw_edges;   cs = consumes; cc = cooked_changes;
    chSysUnlock();
    /* Ticks -> ms outside the lock (a 64-bit divide on an M0). Rounds up, so a
     * one-tick gap reads as 1 ms rather than 0. */
    uint32_t gm_ms = TIME_I2MS(gm);
    if (gm_ms > 0xFFFFu) gm_ms = 0xFFFFu;

    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    for (uint8_t i = 0; i < 6; i++) PUT16(i < MATRIX_ROWS ? rs[i] : 0);  /* 12 */
    PUT16(gm_ms);                                                        /* 14 */
    PUT32(re); PUT32(cs); PUT32(cc);                                     /* 26 */
#undef PUT16
#undef PUT32
    *p++ = gr;
    *p++ = MATRIX_ROWS;
}

void health_isr_stats(uint32_t *entries, uint32_t *ticks_sum, uint16_t *min_ticks, uint16_t *max_ticks) {
    chSysLock();
    *entries = isr_entries; *ticks_sum = isr_ticks_sum;
    *min_ticks = isr_ticks_min; *max_ticks = isr_ticks_max;
    chSysUnlock();
}

uint32_t health_isr_tick_hz(void) { return (uint32_t)CH_CFG_ST_FREQUENCY; }

uint16_t health_row_gap_max_ms(uint8_t *row) {
    uint16_t ticks; uint8_t r;
    chSysLock();
    ticks = row_gap_max_ticks; r = row_gap_max_row;
    chSysUnlock();
    if (row) *row = r;
    uint32_t ms = TIME_I2MS(ticks);
    return (uint16_t)(ms > 0xFFFFu ? 0xFFFFu : ms);
}

uint32_t health_loop_gap_max_ms(void)        { return loop_gap_max; }
uint8_t  health_loop_gap_max_mark(void)      { return loop_gap_max_mark; }
uint32_t health_count_ge_10ms(void) {
    return count_ge_10ms;
}

uint16_t health_count_ge_25ms_nonflash(void) { return count_ge_25ms_nonflash; }

void health_fill4(uint8_t *out28) {
    uint32_t ie, its, rst, up;
    uint16_t imin, imax;
    chSysLock();
    ie = isr_entries; its = isr_ticks_sum; imin = isr_ticks_min; imax = isr_ticks_max;
    rst = row_samples_total;
    chSysUnlock();
    up = timer_read32();   /* main-loop context; a timebase for rate arithmetic */

    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    PUT32(ie); PUT32(its);                        /* 8 */
    PUT16(imin); PUT16(imax);                     /* 12 */
    PUT32((uint32_t)CH_CFG_ST_FREQUENCY);         /* 16 */
    PUT32(up);                                    /* 20 */
    PUT32(rst);                                   /* 24 */
    PUT32(0);                                     /* 28: reserved */
#undef PUT16
#undef PUT32
}

/* ---- page 6: crash-hunt vitals (plans/CRASH-HUNT-PLAN.md, B2-B4) ---------- */

#ifndef AK820_BUILD_TOKEN
#define AK820_BUILD_TOKEN 0u   /* build.sh supplies one; 0 = an unarchived build */
#endif

/* crt0 paints both stacks with CRT0_STACKS_FILL_PATTERN at boot. */
#define STACK_PAINT 0x55555555u
extern uint32_t __main_stack_base__, __main_stack_end__;
extern uint32_t __process_stack_base__, __process_stack_end__;

/* A paint-based watermark: bytes never written since boot, counted up from
 * the region's base. It OVERSTATES headroom when a frame reserves space
 * without writing it, or stores the paint value itself; it is a floor on how
 * deep the stack went, not a measurement of minimum SP. The bottom word is the
 * canary: changed means the region was exhausted, and reads 0. Bounded,
 * volatile reads; nothing per main-loop pass. (crash-hunt review, finding 10) */
static uint16_t stack_free(const uint32_t *base, const uint32_t *end) {
    const volatile uint32_t *p = (const volatile uint32_t *)base;
    const volatile uint32_t *e = (const volatile uint32_t *)end;
    if (p >= e || *p != STACK_PAINT) return 0;
    while (p < e && *p == STACK_PAINT) p++;
    return (uint16_t)((uintptr_t)p - (uintptr_t)base);
}

void health_fill6(uint8_t *out28) {
    lcd_blit_stats_t b;
    lcd_blit_stats(&b);
    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    PUT32(timer_read32());                                              /* 4 */
    PUT32((uint32_t)AK820_BUILD_TOKEN);                                 /* 8 */
    PUT16(stack_free(&__main_stack_base__, &__main_stack_end__));       /* 10 */
    PUT16(stack_free(&__process_stack_base__, &__process_stack_end__)); /* 12 */
    for (unsigned i = 0; i < BLIT_FAULT_KINDS; i++) PUT16(b.kinds[i]);  /* 20 */
    PUT16(b.busy_waits); PUT16(b.retry_successes);                      /* 24 */
    PUT32(b.issued);                                                    /* 28 */
#undef PUT16
#undef PUT32
}
