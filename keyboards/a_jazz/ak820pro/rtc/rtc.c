#include "watchdog_record.h"
// rtc.c
// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#include "rtc.h"
#include "quantum.h"
#include "kb_eeconfig.h"   /* persisted divider period (phase 4) */
#include "flash_stats.h"   /* C1: which path proposes each save */
#include "rtc_persist.h"   /* C2: proposals here, every save there */
#include "hal.h"
#include "../graphics/lcd_bus.h"
#include "../graphics/display.h"   /* display_splash_done() */
#include "../ak820pro.h"
#include "../sleep.h"              /* the deep sleep's pass budget */
#include "../overrun_diag.h"       /* dev: each transaction's kind and spans */

#include <time.h>


#ifndef RTC_SCL_PIN
#    define RTC_SCL_PIN A14
#endif

#ifndef RTC_SDA_PIN
#    define RTC_SDA_PIN A15
#endif


#define PCF8563_ADDR            0x51
#define PCF8563_REG_SECONDS     0x02
#define PCF8563_VL_FLAG         0x80
/* 20 ms normally; 5 ms for the deep sleep's transactions (the pre-sleep read
 * and the wake's recovery: plans/DEEP-SLEEP-BUILD-PLAN.md, section 4, "The
 * budget"). The software I2C applies it as a deadline from the transaction's
 * start; with rtc.c's 15-NOP bit delay a whole read is well under 1 ms, so 5
 * bounds only a stuck bus. */
static bool pcf_short_timeout = false;
#define PCF8563_I2C_TIMEOUT     (pcf_short_timeout ? TIME_MS2I(5) : TIME_MS2I(20))


#ifndef PCF8563_I2C_DELAY_NOPS
#    define PCF8563_I2C_DELAY_NOPS 15
#endif


static void rtc_i2c_delay(void)
{

    for (volatile uint32_t i = 0;
         i < PCF8563_I2C_DELAY_NOPS;
         i++) {
        __asm__ volatile("nop");
    }

}


static const I2CConfig i2ccfg = {
    .addr10 = false,
    .scl    = RTC_SCL_PIN,
    .sda    = RTC_SDA_PIN,
    .delay  = rtc_i2c_delay,
};



static inline uint8_t bcd2dec(uint8_t v)
{
    return (uint8_t)(((v >> 4) * 10) + (v & 0x0F));
}


static inline uint8_t dec2bcd(uint8_t v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}


/*
 * ============================================================================
 * PCF8563 reference RTC
 * ============================================================================
 */



/* ---- Bus guard: never bit-bang I2C while a flash->LCD DMA is in flight ----
 *
 * lcd_bus.c states the hazard outright: "The bit-banged RTC I2C (SCL=A14,
 * SDA=A15) shares port A with the flash SPI1 pins (SCK=A12, CS=A13); its
 * open-drain pin-mode toggling glitches A12/A13 mid-DMA and corrupts the flash
 * read. Callers must suspend RTC polling while this is true."
 *
 * That contract was enforced ONLY for the animation player, via anim_active().
 * The dashboard blits constantly -- clock, text, battery, playback -- and had no
 * guard at all, so every PCF transaction was free to land in the middle of one.
 * A glitched SPI1 starves the DMA, the completion IRQ never arrives, and
 * blit_done is stuck false forever: the hang.
 *
 * Draining first is sufficient rather than merely narrowing: both the I2C and
 * every blit are started from the main loop, so once the in-flight blit is done
 * no new one can begin before this transaction finishes. Same argument as
 * backing_store_pre_write_hook().
 *
 * The counter is the evidence. It records how often a transaction WOULD have
 * landed on a live blit, so it measures the hazard rate even now that it is
 * prevented -- if it climbs while timeouts stay at zero, this was the cause. */
static uint16_t i2c_blit_overlap = 0;

uint16_t rtc_i2c_overlaps(void) { return i2c_blit_overlap; }

/* ---- Phase 0 instrumentation (clock-sync plan) ----------------------------
 * Everything here is OBSERVATION: reads of SECCNT / FRMNO and a few adds. No
 * behaviour changes in this phase. Layouts are served by rtc_status_fill(). */
static volatile uint32_t rtc_seconds_count = 0;   /* moved up: needed by stamps */

static uint16_t stale_count    = 0;   /* rtc_now() gave up (SECIF pending, ISR starved) */
static uint8_t  pcf_avail_fail = 0;   /* consecutive failed PCF reads (reset by any success) */
static uint16_t i2c_fail_count = 0;   /* PCF transactions that returned != MSG_OK */
static uint32_t i2c_max_cycles = 0;   /* longest PCF transaction, ILRC cycles */
static uint32_t last_sync_secs = 0;   /* rtc_seconds_count at the last host set */
static bool     host_synced    = false;
static uint16_t host_sync_gen  = 0;   /* bumped by every host set (a deep sleep's recovery yields to a newer one) */
static uint16_t clock_gen      = 0;   /* bumped by every change to the calendar the display shows */
static int16_t  last_host_offset_ms = 0;

/* ---- Phase 1: reload ownership (PLAN.md 3.0, R1-R3) -----------------------
 * P_nom is the NOMINAL period (register value) owned here; RTCD1.period is
 * whatever is live in the register (the ACTIVE period). In steady state
 * nobody writes the register. A trim, or the restore after a phase set,
 * sets reload_pending and the tick ISR performs the single write at the
 * match, where the counter is ~0 anyway.
 *
 * On R3 (latency compensation): a write whose value is the new STEADY value
 * cannot be compensated -- the register must end up holding P_nom, so the
 * match-to-match interval containing the write is long by exactly the ISR
 * service latency L, measured 0-7 cycles (T0.2, <= 200 us) idle and under
 * BT typing. That is far inside the host uncertainty (~3 ms), so the final
 * restore is written UNcompensated. Compensation is reserved for transient
 * values (Phase 2 slew segments), where it is exact. */
static uint32_t          P_nom = SN32_RTC_PERIOD_DEFAULT;
static volatile bool     reload_pending = false;
static volatile uint16_t reload_writes  = 0;   /* every ISR reload write */

/* ---- Phase 2: slew (PLAN.md 3.3) ------------------------------------------
 * A phase correction of D cycles is applied as N shortened/lengthened
 * intervals: register P_nom - d for N-1 of them, P_nom - d - r for the last
 * (r = remainder), then P_nom. Two or three ISR writes total; the transient
 * ones are latency-compensated (R3, exact), the final restore is not (costs
 * <= L, measured 0-7 cycles). The estimator window is invalidated by every
 * write (R2). SLEW_STEP: 2 % of a period, i.e. 20 ms per second. */
enum { SLEW_IDLE = 0, SLEW_START = 1, SLEW_RUN = 2 };
static volatile uint8_t  slew_state = SLEW_IDLE;
static volatile int32_t  slew_d = 0, slew_r = 0;
static volatile uint16_t slew_left = 0;
static volatile uint16_t slew_count = 0;      /* slews started, for the console/health */
#define SLEW_MAX_MS 500

/* ---- Phase 2: FRMNO frequency estimator (PLAN.md 3.4) ---------------------
 * Per accepted RTC second the ISR adds the host-ms delta (FRMNO) and the
 * exact interval length in ILRC cycles. Every WIN ticks housekeeping turns
 * the ratio into P_target. 64-bit sums (128 x 40000 x 1000 overflows 32). */
#define LATE_CYCLES     170    /* ~5 ms: T0.2 measured max 7 */
#define WIN_INITIAL     32
/* Was 128. The ILRC wanders a few hundred ppm on 5-minute scales (measured
 * 2026-09-03 against a reference proven clean: the nominal period walked
 * 33225 -> 33198 -> 33260 -> 33248 within an hour), and a 128-s window with
 * half-steps lagged it by 100-160 ms per 5 min. A 32-s window costs ~2 ticks
 * (~60 ppm) of estimate noise per window -- the window's frame count is exact
 * to +-1 frame at each end -- against the 5-10 ticks the target moves between
 * windows, so tracking wins. Both windows being 32 leaves the lock streak
 * with nothing to switch; it is kept as the console's lock indicator. */
#define WIN_LOCKED      32
static volatile uint64_t win_ms = 0, win_cycles = 0;
static volatile uint16_t win_n = 0;
static volatile bool     win_valid = true;
static volatile uint16_t window_rejects = 0;
static volatile uint32_t interval_cycles = 0;  /* exact length of the interval that just started */
static volatile uint8_t  sof_ok_run = 0, sof_bad_run = 0;   /* consecutive accepted / rejected samples */
static uint16_t win_target = WIN_INITIAL;
static uint8_t  win_locked_streak = 0;
static int16_t  sof_bias_ppm = 0;              /* from the host (0x03 [13..14]); 0 until told */

/* ---- Phase 2: reference-source state machine (PLAN.md 3.4) ---------------- */
enum { REF_NONE = 0, REF_PCF_LEGACY = 1, REF_SOF = 2 };
static uint8_t  ref_state = REF_PCF_LEGACY;
static uint16_t ref_transitions = 0;

/* Deferred PCF write (R4/R5): the HID handler queues, rtc_fast_task() writes
 * -- one transaction per main-loop pass, only when no LCD blit is in flight,
 * and never via the blit-draining bus guard. */
static bool       pcf_pending = false;
static rtc_time_t pcf_pending_time;
static uint16_t   deferred_passes = 0;    /* passes skipped because a blit was busy */
static bool       pcf_backoff = false;    /* 3 consecutive failures: one attempt/min */
static bool       pcf_maybe_stopped = false; /* recovery pending: loud health flag (Phase 3) */
static uint32_t   pcf_backoff_until = 0;  /* rtc_seconds_count */

/* Tick-ISR service latency: SECCNT at callback entry (cycles since the match
 * restarted the counter). Sets LATE_CYCLES for later phases (T0.2). */
static volatile uint16_t lat_min = 0xFFFF, lat_max = 0, lat_n = 0;
static volatile uint32_t lat_sum = 0;

/* USB SOF frame-number observation (T0.3). The ISR reads the volatile mirror
 * ONCE and never touches SN_USB when it is false. */
static volatile uint8_t  usb_active_mirror = 0;
static volatile uint8_t  fn_valid = 0;
static volatile uint16_t fn_last  = 0;
static volatile uint32_t sof_frames_total = 0;    /* continuous while continuity holds */
static volatile uint8_t  sof_epoch = 0;           /* bumps on every continuity break */
static volatile uint16_t d_zero = 0, d_reject = 0, d_ok = 0;
#define D_RING_N 14
static volatile uint16_t d_ring[D_RING_N];
static volatile uint8_t  d_ring_i = 0;

/* Elapsed ILRC cycles between two (seconds_count, SECCNT) stamps; only valid
 * for spans well under a second (an I2C transaction). */
static inline void cyc_stamp(uint32_t *sc, uint32_t *cnt) {
    *sc  = rtc_seconds_count;
    *cnt = SN_RTC->SECCNT;
}
static inline uint32_t cyc_elapsed(uint32_t sc0, uint32_t c0, uint32_t sc1, uint32_t c1) {
    int32_t d = (int32_t)c1 - (int32_t)c0 + (int32_t)(sc1 - sc0) * (int32_t)(rtc_lld_get_period(&RTCD1) + 1);
    return d < 0 ? 0 : (uint32_t)d;
}
/* PCF transactions since boot, and the count at the last pass's end
 * (rtc_pass_mark): a deep sleep's recovery runs its gauge step only in a pass
 * that made none (codex, deep-sleep build review 2). */
static uint32_t pcf_xfers = 0, pcf_xfers_mark = 0;

static void i2c_note(msg_t r, uint32_t sc0, uint32_t c0) {
    pcf_xfers++;
    uint32_t sc1, c1; cyc_stamp(&sc1, &c1);
    uint32_t e = cyc_elapsed(sc0, c0, sc1, c1);
    if (e > i2c_max_cycles) i2c_max_cycles = e;
    if (r != MSG_OK && i2c_fail_count < 0xFFFF) i2c_fail_count++;
}

/* Every PCF transaction, one I2C call: its accounting, and after a failure
 * the driver's restart. The fallback driver reports every error (a NACK, a
 * lost arbitration, the deadline) as MSG_TIMEOUT, which leaves I2CD1 in
 * I2C_LOCKED, and nothing else ever restarted it: the next call ran on a
 * locked driver, unnoticed only because the HAL's assertions are compiled
 * out. The restart repairs the driver's state; it sends no clocks to free a
 * stuck bus. The caller's failure, retries and back-off stand. */
static msg_t pcf_xfer(uint8_t kind, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx)
{
    uint32_t sc0, c0; cyc_stamp(&sc0, &c0);
    OD_PCF_BEGIN(kind);
    msg_t r = i2cMasterTransmitTimeout(&I2CD1, PCF8563_ADDR, tx, ntx, rx, nrx, PCF8563_I2C_TIMEOUT);
    OD_PCF_END(kind, r == MSG_OK);
    i2c_note(r, sc0, c0);
    if (r != MSG_OK) {
        i2cStop(&I2CD1);
        i2cStart(&I2CD1, &i2ccfg);
    }
    return r;
}

static void rtc_bus_guard(void) {
    /* Outermost mark wins -- see lcd_bus.c; this calls lcd_blit_wait() too. */
    if (loop_stall_mark == LOOP_MARK_NONE) loop_stall_mark = LOOP_MARK_I2C;
    if (lcd_blit_busy() && i2c_blit_overlap < 0xFFFFu) {
        i2c_blit_overlap++;
    }
    lcd_blit_wait();
}

/* The full calendar with NO bus guard: the caller already knows no
 * flash->LCD DMA is in flight (rtc_fast_task's slot, or the deep sleep's
 * admission, whose display bus is idle). */
static bool pcf_read_raw(rtc_time_t *out)
{
    WDT_SCOPE(WDT_SITE_I2C_READ);
    uint8_t reg = PCF8563_REG_SECONDS;
    uint8_t buf[7];


    msg_t r = pcf_xfer(OD_PCF_R7, &reg, 1, buf, sizeof(buf));
    if (r != MSG_OK) {
        if (pcf_avail_fail < 255) pcf_avail_fail++;
        return false;
    }


    if (buf[0] & PCF8563_VL_FLAG) {
        if (pcf_avail_fail < 255) pcf_avail_fail++;
        return false;
    }
    pcf_avail_fail = 0;


    out->seconds = bcd2dec(buf[0] & 0x7F);
    out->minutes = bcd2dec(buf[1] & 0x7F);
    out->hours   = bcd2dec(buf[2] & 0x3F);
    out->day     = bcd2dec(buf[3] & 0x3F);
    out->weekday = buf[4] & 0x07;
    out->month   = bcd2dec(buf[5] & 0x1F);
    out->year    = 2000U + bcd2dec(buf[6]);


    if (out->seconds > 59 ||
        out->minutes > 59 ||
        out->hours > 23 ||
        out->day < 1 ||
        out->day > 31 ||
        out->month < 1 ||
        out->month > 12) {
        return false;
    }


    return true;
}

/* Legacy path (the seed, the 60 s discipline): guarded, synchronous. */
static bool pcf_read(rtc_time_t *out)
{
    rtc_bus_guard();
    return pcf_read_raw(out);
}

/* The 8-byte time write with NO bus guard: the caller must already know no
 * flash->LCD DMA is in flight (rtc_fast_task checks lcd_blit_busy() itself,
 * and blits only start from the same main-loop context, after it). */
static bool pcf_write_raw(const rtc_time_t *t)
{
    WDT_SCOPE(WDT_SITE_I2C_WRITE);
    uint8_t buf[8] = {
        PCF8563_REG_SECONDS,
        dec2bcd(t->seconds),
        dec2bcd(t->minutes),
        dec2bcd(t->hours),
        dec2bcd(t->day),
        t->weekday & 0x07,
        dec2bcd(t->month),
        dec2bcd((uint8_t)(t->year % 100)),
    };


    return pcf_xfer(OD_PCF_W8, buf, sizeof(buf), NULL, 0) == MSG_OK;
}

/* Legacy path (rtc_set_time, the trim): guarded, synchronous. */
static bool pcf_write(const rtc_time_t *t)
{
    rtc_bus_guard();
    return pcf_write_raw(t);
}

/* Single-register PCF access, bus-guarded like the rest. Phase 0 uses these
 * for the STOP-bit and 1-byte-read timing tests; Phase 3's deferred state
 * machine will call the same primitives without the guard (and lift the
 * ifdef -- until then the daily build has no caller, and -Werror objects). */
#ifdef WDT_TEST_HOOKS
static bool pcf_reg_read(uint8_t reg, uint8_t *v) {
    WDT_SCOPE(WDT_SITE_I2C_READ);
    rtc_bus_guard();
    return pcf_xfer(OD_PCF_R1, &reg, 1, v, 1) == MSG_OK;
}
static bool pcf_reg_write(uint8_t reg, uint8_t v) {
    WDT_SCOPE(WDT_SITE_I2C_WRITE);
    rtc_bus_guard();
    uint8_t buf[2] = { reg, v };
    return pcf_xfer(OD_PCF_W1, buf, 2, NULL, 0) == MSG_OK;
}
#endif /* WDT_TEST_HOOKS */



/*
 * ============================================================================
 * ChibiOS RTC conversion helpers
 * ============================================================================
 */


static void rtc_to_chibiostime(const rtc_time_t *src,
                               RTCDateTime *dst)
{
    dst->year = src->year - RTC_BASE_YEAR;
    dst->month = src->month;
    dst->day = src->day;
    dst->dayofweek = src->weekday;
    dst->dstflag = 0;


    dst->millisecond =
        ((uint32_t)src->hours * 3600UL +
         (uint32_t)src->minutes * 60UL +
         src->seconds) * 1000UL;
}



static void chibiostime_to_rtc(const RTCDateTime *src,
                               rtc_time_t *dst)
{
    dst->year = src->year + RTC_BASE_YEAR;
    dst->month = src->month;
    dst->day = src->day;
    dst->weekday = src->dayofweek;


    dst->hours =
        (uint8_t)(src->millisecond / 3600000UL);

    dst->minutes =
        (uint8_t)((src->millisecond / 60000UL) % 60);

    dst->seconds =
        (uint8_t)((src->millisecond / 1000UL) % 60);
}


/*
 * ============================================================================
 * Synchronization
 * ============================================================================
 */


static bool rtc_valid;


#ifdef RTC_AUTO_CALIBRATION

#ifndef RTC_CHECK_INTERVAL_S
#    define RTC_CHECK_INTERVAL_S 3600
#endif


#ifndef RTC_DRIFT_THRESHOLD_S
#    define RTC_DRIFT_THRESHOLD_S 2
#endif

/* Minimum rate-measurement window, in reference seconds. The PCF reads to 1 s,
 * so this sets the best resolution a single trim can have (~0.3% at 300 s). */
#ifndef RTC_CAL_MIN_WINDOW_S
#    define RTC_CAL_MIN_WINDOW_S 300
#endif

/* Minimum tick-vs-real discrepancy worth acting on. Must be > 1 so reference
 * quantisation alone cannot trigger a trim. */
#ifndef RTC_CAL_MIN_DIFF_S
#    define RTC_CAL_MIN_DIFF_S 2
#endif

static volatile uint32_t rtc_check_seconds;

/* Rate-trim state. The SN32 RTC is clocked from the ILRC (see
 * SN32_RTC_CLK_SOURCE in hal_rtc_lld.h), an untrimmed internal RC oscillator,
 * against a hardcoded SN32_RTC_PERIOD_DEFAULT of 32000 that assumes exactly
 * 32 kHz. Measured on this unit: ~34.3 kHz, i.e. the clock gains ~4 s per
 * minute. Phase snapping alone therefore leaves a sawtooth as large as the
 * check interval's worth of drift, which is what made the clock look "absolutely
 * horrid" -- it was, but only between snaps.
 *
 * These open a rate-measurement window so the divider itself can be corrected.
 * Both quantities are snap-immune -- a free-running tick count and the PCF's own
 * absolute time -- so a phase correction mid-window does not invalidate it. */
static uint32_t rtc_cal_ticks0;     /* rtc_seconds_count when the window opened */
static int32_t  rtc_cal_ref0;       /* PCF epoch seconds when the window opened */
static bool     rtc_cal_valid;      /* the two above are meaningful */

#endif



// Free-running count of RTC second interrupts (~seconds since rtc_init). A cheap
// once-per-second edge source (no localtime()) for pacing the display refresh.
// (rtc_seconds_count itself is declared above, with the Phase 0 instruments.)

/* A deep sleep stops the CPU while the RTC counts on: its second interrupts
 * are cleared, not served, so the sleeper hands the count back here, to both
 * clocks the handler advances (ours and the LLD's calendar time, which
 * rtcGetTime() reads; codex, the sleep plan's review), with interrupts still
 * locked so no served second lands between. Only while nothing is in flight
 * that a skipped handler would break (a slew or a reload): rtc_sleep_ok(). */
void rtc_account_sleepI(uint32_t seconds) {
    if (seconds == 0) return;
    rtc_seconds_count += seconds;
    rtc_lld_add_secondsI(&RTCD1, seconds);
#ifdef RTC_AUTO_CALIBRATION
    rtc_check_seconds = MIN(rtc_check_seconds + seconds, RTC_CHECK_INTERVAL_S);
#endif
}

static void rtc_second_cb(RTCDriver *rtcp, rtcevent_t event)
{
    (void)rtcp;
    (void)event;

    /* T0.2: service latency -- cycles since the match restarted the counter.
     * Read FIRST, before anything else in the handler adds to it. */
    uint32_t L = SN_RTC->SECCNT;
    uint32_t ended = interval_cycles;      /* exact length of the interval that just ended */

    rtc_seconds_count++;

    /* ---- Reload scheduling: at most ONE register write per tick, at the
     * match where the counter has just restarted. Priority: a running slew,
     * then a pending steady reload. Every write invalidates the estimator
     * window (R2) and records the exact length of the interval it starts. */
    bool     wrote = false;
    uint32_t next  = rtc_lld_get_period(&RTCD1) + 1;     /* default: unchanged register */
    if (slew_state == SLEW_START) {
        /* transient value, latency-compensated (R3): interval == v+1 exactly */
        int32_t v = (int32_t)P_nom - slew_d - (slew_left == 1 ? slew_r : 0);
        int32_t w = v - (int32_t)L;
        if (v >= 14000 && v <= 80000 && w >= 14000 && L < (uint32_t)v / 2) {
            rtc_lld_set_period(&RTCD1, (uint32_t)w);
            next = (uint32_t)v + 1;
            slew_state = SLEW_RUN;
        } else {                                          /* out of range: abandon, restore */
            rtc_lld_set_period(&RTCD1, P_nom);
            next = L + P_nom + 1;
            slew_state = SLEW_IDLE;
        }
        wrote = true;
    } else if (slew_state == SLEW_RUN) {
        slew_left--;
        if (slew_left == 1 && slew_r != 0) {
            int32_t v = (int32_t)P_nom - slew_d - slew_r;
            int32_t w = v - (int32_t)L;
            if (v >= 14000 && v <= 80000 && w >= 14000 && L < (uint32_t)v / 2) {
                rtc_lld_set_period(&RTCD1, (uint32_t)w);
                next = (uint32_t)v + 1;
            } else {
                rtc_lld_set_period(&RTCD1, P_nom);
                next = L + P_nom + 1;
                slew_state = SLEW_IDLE;
            }
            wrote = true;
        } else if (slew_left == 0) {
            rtc_lld_set_period(&RTCD1, P_nom);            /* final restore: uncompensated */
            next = L + P_nom + 1;
            slew_state = SLEW_IDLE;
            wrote = true;
        }
    } else if (reload_pending) {
        reload_pending = false;
        if (P_nom >= 14000 && P_nom <= 80000) {
            rtc_lld_set_period(&RTCD1, P_nom);
            next = L + P_nom + 1;
            wrote = true;
        }
    }
    if (wrote) { reload_writes++; win_valid = false; }
    interval_cycles = next;

#ifdef RTC_AUTO_CALIBRATION
    rtc_check_seconds = MIN(rtc_check_seconds + 1, RTC_CHECK_INTERVAL_S);
#endif

    if (L < lat_min) lat_min = (uint16_t)L;
    if (L > lat_max) lat_max = (uint16_t)L;
    if (lat_n < 0xFFFF) { lat_n++; lat_sum += L; }

    /* ---- USB SOF frame-number delta per RTC second: the frequency reference.
     * Mirror read once; SN_USB untouched when the bus is down. */
    uint8_t ua = usb_active_mirror;
    if (!ua) {
        if (fn_valid) sof_epoch++;
        fn_valid = 0;
        win_valid = false;
        sof_ok_run = 0;
        if (sof_bad_run < 255) sof_bad_run++;
    } else {
        uint16_t fn = (uint16_t)(SN_USB->FRMNO & 0x7FFu);
        uint16_t d  = fn_valid ? (uint16_t)((fn - fn_last) & 0x7FFu) : 0;
        fn_last  = fn;
        if (!fn_valid) { fn_valid = 1; win_valid = false; }
        else {
            if (d) sof_frames_total += d;
            bool ok = (d >= 900 && d <= 1100) && (L < LATE_CYCLES);
            if (d == 0)                   { d_zero++;   sof_epoch++; }
            else if (d < 900 || d > 1100) { d_reject++; sof_epoch++; }
            else                          { d_ok++; }
            d_ring[d_ring_i] = d;
            d_ring_i = (uint8_t)((d_ring_i + 1) % D_RING_N);

            if (ok) {
                sof_bad_run = 0;
                if (sof_ok_run < 255) sof_ok_run++;
                /* The interval that just ended must be fully known and not
                 * contain a write: `ended` is exact, and a write this tick
                 * only affects the NEXT interval. A write LAST tick set
                 * win_valid false, so this sample opens a fresh window. */
                if (win_valid && ended) {
                    win_ms     += d;
                    win_cycles += ended;
                    win_n++;
                } else {
                    win_valid = true; win_ms = 0; win_cycles = 0; win_n = 0;
                    if (ended) { win_ms = d; win_cycles = ended; win_n = 1; }
                }
            } else {
                sof_ok_run = 0;
                if (sof_bad_run < 255) sof_bad_run++;
                if (win_valid && win_n) window_rejects++;
                win_valid = false;
            }
        }
    }
}

/* ---- Phase 2: window evaluation + reference-source state machine ----------
 * Thread context, 10 Hz (from rtc_task). Reads the ISR's accumulators under
 * a short lock, proposes P_nom (applied by the ISR at the next match). */
static void rtc_ref_task(void)
{
    uint8_t ua = usb_active_mirror;
    uint8_t okrun, badrun;
    chSysLock();
    okrun = sof_ok_run; badrun = sof_bad_run;
    chSysUnlock();

    /* Transitions. Exactly one estimator owns P_nom at a time. */
    uint8_t prev = ref_state;
    if (ref_state != REF_SOF) {
        if (ua && okrun >= 3) {
            ref_state = REF_SOF;
#ifdef RTC_AUTO_CALIBRATION
            rtc_cal_valid = false;            /* PCF span reset */
#endif
            win_target = WIN_INITIAL; win_locked_streak = 0;
        } else if (ref_state == REF_PCF_LEGACY && pcf_avail_fail >= 3) {
            ref_state = REF_NONE;
        } else if (ref_state == REF_NONE && pcf_avail_fail == 0) {
            ref_state = REF_PCF_LEGACY;       /* a successful read cleared the counter */
        }
    } else {
        if (!ua || badrun >= 3) {
            ref_state = REF_PCF_LEGACY;       /* its own counter moves it on to NONE if the PCF is dead */
            chSysLock(); win_valid = false; chSysUnlock();
#ifdef RTC_AUTO_CALIBRATION
            rtc_cal_valid = false;
#endif
        }
    }
    if (ref_state != prev) {
        ref_transitions++;
#ifdef CONSOLE_ENABLE
        printf("[rtc] ref %u -> %u\n", prev, ref_state);
#endif
    }

    if (ref_state != REF_SOF) return;

    /* Window evaluation. */
    uint64_t ms, cyc; uint16_t n; bool valid;
    chSysLock();
    valid = win_valid; n = win_n; ms = win_ms; cyc = win_cycles;
    if (valid && n >= win_target) { win_valid = false; }   /* consume: the ISR opens a fresh one */
    chSysUnlock();
    if (!valid || n < win_target || ms == 0) return;

    uint64_t f_est  = (cyc * 1000u) / ms;                             /* Hz, assuming 1.000 ms frames */
    int64_t  f_true = (int64_t)f_est + ((int64_t)f_est * sof_bias_ppm) / 1000000;
    int32_t  P_target = (int32_t)f_true - 1;
    if (P_target < 28000 || P_target > 40000) {
#ifdef CONSOLE_ENABLE
        printf("[rtc] window rejected: f_est=%lu\n", (unsigned long)f_est);
#endif
        return;
    }
    int32_t delta = P_target - (int32_t)P_nom;
    if (delta > -16 && delta < 16) { if (win_locked_streak < 255) win_locked_streak++; }
    else win_locked_streak = 0;
    if (win_locked_streak >= 2) win_target = WIN_LOCKED;

    /* Step size. Half-step damping exists because a single window's target
     * carries ~2 ticks of frame-quantisation noise and applying all of it
     * would ring. But when two consecutive windows agree on the DIRECTION the
     * target is moving in, that is drift, not noise -- the ILRC wandering --
     * and half-stepping just lags it. So: half on the first window that
     * disagrees with the last (noise, or the onset of a move), full once two
     * in a row agree. Noise alternates sign and gets halved; drift persists
     * and gets followed within one window. */
    static int8_t last_sign = 0;
    if (delta != 0) {
        int8_t   sign = (delta > 0) ? 1 : -1;
        uint32_t np   = (uint32_t)((int32_t)P_nom + delta / 2);
        if (delta == 1 || delta == -1) np = (uint32_t)P_target;      /* integer half-step would stall */
        else if (sign == last_sign)    np = (uint32_t)P_target;      /* two windows agree: follow it */
        last_sign = sign;
#ifdef CONSOLE_ENABLE
        printf("[rtc] sof window n=%u ms=%lu -> f=%lu target=%ld P %lu -> %lu\n",
               n, (unsigned long)ms, (unsigned long)f_est, (long)P_target,
               (unsigned long)P_nom, (unsigned long)np);
#endif
        P_nom = np;
        reload_pending = true;
    } else {
        last_sign = 0;
    }
    /* Propose every accepted window's value -- INCLUDING a window whose delta
     * is 0: a cleanly converged loop otherwise never stored anything (seen
     * 2026-09-01: 33212 held for two windows, EEPROM still 0, next reboot
     * re-seeded at 33600). Whether and when it is SAVED is rtc_persist.c's
     * (C2): first at 10 min if nothing is stored, then <= 4 a day. */
    if (P_nom >= 28000 && P_nom <= 40000) rtc_persist_propose((uint16_t)P_nom, FLASH_RTC_PATH_SOF);
}

/* Coherent sub-second read (PLAN.md 3.1). Lock-free: a tick landing inside
 * the snapshot shows as c1 != c2; a match whose ISR has not yet run shows as
 * SECIF pending. Either way retry; after 8 tries report failure and NO time. */
bool rtc_now(rtc_stamp_t *s)
{
    if (!rtc_valid) return false;
    for (int tries = 0; tries < 8; tries++) {
        uint32_t c1  = rtc_seconds_count;
        uint32_t cnt = SN_RTC->SECCNT;
        uint32_t pa  = rtc_lld_get_period(&RTCD1);
        bool pending = (SN_RTC->RIS & mskRTC_SECIF) != 0;
        RTCDateTime dt;
        rtcGetTime(&RTCD1, &dt);
        uint32_t c2  = rtc_seconds_count;
        if (c1 == c2 && !pending) {
            chibiostime_to_rtc(&dt, &s->t);
            s->cnt = cnt;
            s->period_active = pa;
            s->seconds_count = c1;
            return true;
        }
    }
    if (stale_count < 0xFFFF) stale_count++;
    return false;
}

/* Status blocks for raw HID -- layouts mirrored in hostagent/rtc_phase0.py. */
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

void rtc_status_fill_page4(uint8_t *out);
static bool acq_state_done(void);
void rtc_status_fill(uint8_t page, uint8_t *out)
{
    switch (page) {
    case 1: {   /* 21 bytes: the RTC_GET_TIME[11..31] tail */
        rtc_stamp_t s;
        bool ok = rtc_now(&s);
        out[0] = 2;                                          /* RTC_PROTO_VERSION */
        put32(&out[1], ok ? s.cnt : 0);                      /* [12..15] */
        put16(&out[5], (uint16_t)rtc_lld_get_period(&RTCD1));/* [16..17] active (live register) */
        put16(&out[7], (uint16_t)P_nom);                     /* [18..19] nominal */
        out[9]  = (host_synced ? 0x01 : 0) | (slew_state != SLEW_IDLE ? 0x02 : 0) |
                  (acq_state_done() ? 0x04 : 0) | ((pcf_backoff || pcf_maybe_stopped) ? 0x08 : 0) |
                  (stale_count ? 0x10 : 0);   /* [20] flags */
        put16(&out[10], (uint16_t)last_host_offset_ms);      /* [21..22] */
        put16(&out[12], (uint16_t)sof_bias_ppm);             /* [23..24] in use */
        out[14] = ref_state;                                 /* [25] */
        uint32_t age = host_synced ? (rtc_seconds_count - last_sync_secs) / 60u : 255u;
        out[15] = (uint8_t)(age > 254 ? 255 : age);          /* [26] sync_age_min */
        out[16] = sof_epoch;                                 /* [27] */
        put32(&out[17], sof_frames_total);                   /* [28..31] */
        break;
    }
    case 2: {   /* 28 bytes of counters */
        put16(&out[0], stale_count);
        put16(&out[2], i2c_fail_count);
        put16(&out[4], deferred_passes);
        put32(&out[6], i2c_max_cycles);
        put16(&out[10], window_rejects);
        put16(&out[12], ref_transitions);
        put16(&out[14], lat_min == 0xFFFF ? 0 : lat_min);
        put16(&out[16], lat_max);
        put16(&out[18], lat_n ? (uint16_t)(lat_sum / lat_n) : 0);
        put16(&out[20], lat_n);
        put16(&out[22], d_zero);
        put16(&out[24], d_reject);
        out[26] = (uint8_t)sizeof(time_t);
        out[27] = usb_active_mirror | (fn_valid ? 2 : 0);
        break;
    }
    case 3: {   /* 28 bytes: the delta ring, oldest first */
        for (int i = 0; i < D_RING_N; i++) {
            put16(&out[2 * i], d_ring[(d_ring_i + i) % D_RING_N]);
        }
        break;
    }
    case 4:
        memset(out, 0, 28);
        rtc_status_fill_page4(out);
        break;
    default:
        memset(out, 0, 28);
        break;
    }
}

#ifdef WDT_TEST_HOOKS
/* Phase 0 hardware-fact tests. arg = request bytes from [4], reply written
 * from [3]. All of these mutate the clock or the PCF on purpose. */
void rtc_test_op(uint8_t op, const uint8_t *arg, uint8_t *reply)
{
    memset(reply, 0, 29);
    reply[0] = op;
    switch (op) {
    case 1:     /* T0.1: does writing SECCNTV (same value) reset SECCNT?
                 * -> [1..4] cnt before, [5..8] right after, [9..12] ~100 us later, [13..16] period */
    case 2: {   /* same but write period+1 then restore (two writes; second reset expected too) */
        uint32_t p = rtc_lld_get_period(&RTCD1);
        chSysLock();
        uint32_t a = SN_RTC->SECCNT;
        rtc_lld_set_period(&RTCD1, op == 2 ? p + 1 : p);
        uint32_t b = SN_RTC->SECCNT;
        chSysUnlock();
        wait_us(100);
        uint32_t c = SN_RTC->SECCNT;
        if (op == 2) rtc_lld_set_period(&RTCD1, p);
        put32(&reply[1], a); put32(&reply[5], b); put32(&reply[9], c); put32(&reply[13], p);
        break;
    }
    case 3:     /* reset the latency / delta statistics */
        chSysLock();
        lat_min = 0xFFFF; lat_max = 0; lat_n = 0; lat_sum = 0;
        d_zero = d_reject = d_ok = 0;
        chSysUnlock();
        break;
    case 4: {   /* T0.4: time one 1-byte PCF read -> [1] byte, [2..5] cycles, [6] ok */
        uint32_t before = i2c_max_cycles; i2c_max_cycles = 0;
        uint8_t v = 0;
        bool ok = pcf_reg_read(PCF8563_REG_SECONDS, &v);
        reply[1] = v; put32(&reply[2], i2c_max_cycles); reply[6] = ok;
        if (before > i2c_max_cycles) i2c_max_cycles = before;
        break;
    }
    case 5: {   /* T0.4: time one 8-byte PCF write (rewrites the CURRENT time) -> [2..5] cycles, [6] ok */
        rtc_time_t t;
        if (!rtc_get_time(&t)) { reply[6] = 0xFF; break; }
        uint32_t before = i2c_max_cycles; i2c_max_cycles = 0;
        bool ok = pcf_write(&t);
        put32(&reply[2], i2c_max_cycles); reply[6] = ok;
        if (before > i2c_max_cycles) i2c_max_cycles = before;
        break;
    }
    case 6: {   /* T0.5 step 1: RMW STOP=1, then write the 7 time bytes in arg[0..6]
                 * (yy mm dd wd hh mi ss) -> [1] ctl before, [2] ctl written, [3] ok */
        uint8_t ctl = 0;
        if (!pcf_reg_read(0x00, &ctl)) { reply[3] = 0; break; }
        reply[1] = ctl;
        uint8_t nctl = (uint8_t)(ctl | 0x20);
        if (!pcf_reg_write(0x00, nctl)) { reply[3] = 0; break; }
        reply[2] = nctl;
        rtc_time_t t = { .year = (uint16_t)(2000 + arg[0]), .month = arg[1], .day = arg[2],
                         .weekday = arg[3], .hours = arg[4], .minutes = arg[5], .seconds = arg[6] };
        reply[3] = pcf_write(&t) ? 1 : 0;
        break;
    }
    case 7: {   /* T0.5 step 2: release STOP now -> [1..4] sec_count before, [5..8] cnt before,
                 * [9..12] sec_count after, [13..16] cnt after, [17] ctl written, [18] ok */
        uint8_t ctl = 0;
        if (!pcf_reg_read(0x00, &ctl)) { reply[18] = 0; break; }
        uint8_t nctl = (uint8_t)(ctl & ~0x20);
        uint32_t sc0, c0, sc1, c1;
        cyc_stamp(&sc0, &c0);
        bool ok = pcf_reg_write(0x00, nctl);
        cyc_stamp(&sc1, &c1);
        put32(&reply[1], sc0); put32(&reply[5], c0); put32(&reply[9], sc1); put32(&reply[13], c1);
        reply[17] = nctl; reply[18] = ok;
        break;
    }
    case 8: {   /* T0.5 step 3 (polled): read the PCF seconds byte with a board stamp
                 * -> [1] byte, [2..5] sec_count, [6..9] cnt (stamp taken AFTER the read) */
        uint8_t v = 0;
        bool ok = pcf_reg_read(PCF8563_REG_SECONDS, &v);
        uint32_t sc, c; cyc_stamp(&sc, &c);
        reply[1] = v; put32(&reply[2], sc); put32(&reply[6], c); reply[10] = ok;
        break;
    }
    case 9: {   /* read Control_status_1 -> [1] value, [2] ok */
        uint8_t v = 0;
        reply[2] = pcf_reg_read(0x00, &v); reply[1] = v;
        break;
    }
    case 10: {  /* T0.1 fallback: RTCEN 1->0->1 -> [1..4] cnt before, [5..8] after */
        chSysLock();
        uint32_t a = SN_RTC->SECCNT;
        SN_RTC->CTRL &= ~1u;
        SN_RTC->CTRL |= 1u;
        uint32_t b = SN_RTC->SECCNT;
        chSysUnlock();
        put32(&reply[1], a); put32(&reply[5], b);
        break;
    }
    default:
        reply[0] = 0xFF;
        break;
    }
}
#endif

/* The NOMINAL divider period (R1). The live register may briefly differ
 * (a shortened first period after a phase set) until the next tick. */
uint32_t rtc_get_period(void) {
    return P_nom;
}

uint32_t rtc_get_seconds(void) {
    return rtc_seconds_count;
}


static void rtc_seed_from_pcf(void)
{
    rtc_time_t pcf;
    RTCDateTime dt;

    if (!pcf_read(&pcf)) {
        return;
    }

    rtc_to_chibiostime(&pcf, &dt);
    rtcSetTime(&RTCD1,&dt);

    rtc_valid = true;
}

/*
 * ============================================================================
 * Reference check
 * ============================================================================
 */


#ifdef RTC_AUTO_CALIBRATION


static void rtc_clock_discipline(void)
{
    rtc_time_t reference;
    RTCDateTime current;
    RTCDateTime target;

    /*
     * PCF8563 is only accessed during a reference check.
     */
    if (!pcf_read(&reference)) {
        return;
    }

    rtcGetTime(&RTCD1, &current);
    rtc_to_chibiostime(&reference, &target);

    struct tm ref_tm;
    struct tm cur_tm;

    rtcConvertDateTimeToStructTm(&target, &ref_tm, NULL);
    rtcConvertDateTimeToStructTm(&current, &cur_tm, NULL);

    time_t  ref_epoch = mktime(&ref_tm);
    int32_t error     = (int32_t)(ref_epoch - mktime(&cur_tm));

    /*
     * Rate trim. Snapping the phase alone leaves the divider wrong, so the clock
     * immediately re-drifts at the same rate; correcting SECCNTV is what actually
     * makes it keep time.
     *
     * The measurement window is built from two snap-IMMUNE quantities:
     * rtc_seconds_count (a free-running count of SN32 second IRQs, untouched by
     * rtcSetTime) and the PCF's own absolute time (never written here). A phase
     * snap therefore does NOT invalidate the window.
     *
     * An earlier version got exactly that wrong -- it restarted the span on every
     * snap, and since the snap fires whenever drift exceeds the threshold, the
     * trim never survived to collect a second sample and so never ran once.
     *
     * Self-refining: as the period converges, the window must grow longer before
     * ticks and real seconds differ by a whole second, so each estimate is finer
     * than the last with no hand-tuned constant.
     */
    uint32_t ticks_now = rtc_seconds_count;

    if (!rtc_cal_valid) {
        rtc_cal_ticks0 = ticks_now;
        rtc_cal_ref0   = (int32_t)ref_epoch;
        rtc_cal_valid  = true;
    } else {
        int32_t ticks = (int32_t)(ticks_now - rtc_cal_ticks0);
        int32_t real  = (int32_t)ref_epoch - rtc_cal_ref0;

        int32_t diff = ticks - real;

        /* Two guards, both learned the hard way. The reference has 1 s
         * resolution, so a short window ALWAYS shows |diff| == 1 whether or not
         * the clock is wrong. An earlier version trimmed on any nonzero diff and
         * restarted the window each time, so the window never grew past 60 ticks,
         * resolution stayed at 1.7%, and it limit-cycled between the two adjacent
         * quantised answers (33103 <-> 33664) indefinitely.
         *
         * Requiring a minimum window makes quantisation a small fraction of the
         * measurement; requiring |diff| >= 2 means noise alone cannot trigger a
         * trim. Together the window grows on its own as the period converges. */
        if ((ticks > 0) && (real >= RTC_CAL_MIN_WINDOW_S) &&
            ((diff >= RTC_CAL_MIN_DIFF_S) || (diff <= -RTC_CAL_MIN_DIFF_S))) {
            uint32_t period = P_nom;   /* nominal, not the live register (R1) */

            if (period > 0) {
                uint32_t np = (uint32_t)(((uint64_t)period * (uint64_t)ticks) /
                                         (uint64_t)real);

                /* Damp to half the computed step. The estimate carries up to a
                 * quantum of error, and applying it in full is what lets an
                 * overshoot become a standing oscillation; half-stepping turns
                 * that into convergence at the cost of one extra window. */
                np = (uint32_t)((int32_t)period + (((int32_t)np - (int32_t)period) / 2));

                /* Clamp hard: one bad PCF read must not strand the divider
                 * somewhere the clock cannot recover from. */
                if (np < (SN32_RTC_PERIOD_DEFAULT / 2U)) np = SN32_RTC_PERIOD_DEFAULT / 2U;
                if (np > (SN32_RTC_PERIOD_DEFAULT * 2U)) np = SN32_RTC_PERIOD_DEFAULT * 2U;

                if (np != period) {
                    /* Phase 1 re-route (R1/R2): propose, do not write. The
                     * tick ISR applies it at the match. The old direct
                     * rtc_lld_set_period() here reset SECCNT mid-second and
                     * threw away the elapsed fraction (mean 0.5 s) on every
                     * trim. Estimator arithmetic above is unchanged. */
                    P_nom = np;
                    reload_pending = true;
                    /* Propose ANY accepted, sane trim. NOT a wait-for-convergence
                     * rule: near lock the measurement windows grow very long
                     * and step sizes are not guaranteed monotonic (the
                     * recorded 695->46 trace happened to shrink), so a "two
                     * small steps" trigger could wait forever. The SAVE is
                     * rtc_persist.c's (C2). A 64-tick threshold alone did NOT
                     * keep temperature wander from causing steady rewrites, as
                     * this comment used to claim: on battery this path saved
                     * 6-8 times a day (C1, 2026-10-05/07), each blanking the
                     * LEDs. The scheduler's 6 h budget is what bounds it. */
                    if (np >= 28000 && np <= 40000) rtc_persist_propose((uint16_t)np, FLASH_RTC_PATH_PCF);
                    printf("[rtc] trim %lu -> %lu (%ld ticks / %ld s)\n",
                           (unsigned long)period, (unsigned long)np,
                           (long)ticks, (long)real);
                }
            }
            /* Period changed: ticks either side are different units. New window. */
            rtc_cal_ticks0 = ticks_now;
            rtc_cal_ref0   = (int32_t)ref_epoch;
        }
    }

    /*
     * Phase snap. Independent of the trim, and deliberately does NOT restart the
     * calibration window -- see above.
     */
#ifdef RTC_LOG_EVERY_PASS
    /* The snap log below fires only when |error| > threshold, so errors of 0, 1
     * and 2 s -- exactly the regime worth watching -- cannot be seen at all.
     * Behind a flag because at RTC_CHECK_INTERVAL_S 60 this is ~1440 lines/day. */
    printf("[rtc] error %ld s\n", (long)error);
#endif
    if ((error > RTC_DRIFT_THRESHOLD_S) || (error < -RTC_DRIFT_THRESHOLD_S)) {
        rtcSetTime(&RTCD1, &target);
        clock_gen++;
        printf("[rtc] corrected drift of %ld seconds\n", (long)error);
    }
}


#endif

/*
 * ============================================================================
 * Public API
 * ============================================================================
 */


void rtc_init(void)
{
    i2cStart(&I2CD1, &i2ccfg);

    /* Divider seed, best first: the PERSISTED converged period (phase 4 --
     * survives power cycles, correct on ANY unit, makes the compile-time seed
     * irrelevant once the trim has run once), else RTC_PERIOD_INITIAL (a
     * hand-measured, per-unit value -- see the CLAUDE.md warning never to
     * ship it to another board), else the LLD's nominal 32000 and a ~40 min
     * climb. The sanity window brackets any plausible ILRC (spec'd loosely
     * around 32 kHz): a stored value outside it is garbage, not data. */
    uint16_t stored = kb_eeconfig_get_rtc_period();
    if (stored >= 28000 && stored <= 40000) {
        P_nom = stored;
    }
#ifdef RTC_PERIOD_INITIAL
    else {
        P_nom = RTC_PERIOD_INITIAL;
    }
#else
    else {
        P_nom = SN32_RTC_PERIOD_DEFAULT;
    }
#endif
    rtc_lld_set_period(&RTCD1, P_nom);   /* init: the one direct steady write */

    rtcSetCallback(&RTCD1, rtc_second_cb);
    rtc_seed_from_pcf();
}



bool rtc_get_time(rtc_time_t *out)
{
    RTCDateTime dt;

    if (!rtc_valid) {
        return false;
    }

    rtcGetTime(&RTCD1, &dt);
    chibiostime_to_rtc(&dt, out);

    return true;
}



bool rtc_set_time(const rtc_time_t *t)
{
    RTCDateTime dt;

    /* Synchronous I2C: never inside a deep sleep's recovery, whose passes are
     * budgeted (plans/DEEP-SLEEP-BUILD-PLAN.md, section 4) -- the whole of it,
     * not only the clock's part (codex, deep-sleep build review 1). The host
     * retries. */
    if (rtc_wake_busy() || sleep_recovering()) return false;
    host_sync_gen++;
    clock_gen++;
    rtc_to_chibiostime(t, &dt);

    bool ok = pcf_write(t);

    rtcSetTime(&RTCD1, &dt);
    rtc_valid = true;
    host_synced    = true;
    last_sync_secs = rtc_seconds_count;

    return ok;
}

/* ---- Phase 1: phase-correct set (PLAN.md 3.2) -------------------------------
 * "At the instant this packet was received, true time was t + ms." The
 * software seconds are set and the prescaler is restarted with a FIRST
 * period sized so the next SECIF lands exactly on the following boundary;
 * the tick ISR then restores P_nom (reload_pending). Whole-second phase
 * error after this: HID delivery (host-compensated) + <= 1 cycle. */
uint8_t rtc_set_time_ms(const rtc_time_t *t, uint16_t ms, uint8_t flags, int16_t bias_ppm, int16_t *offset_before)
{
    if (ms > 999) return RTC_SET_REJECT;
    if (bias_ppm != 0x7FFF && bias_ppm >= -600 && bias_ppm <= 600) sof_bias_ppm = bias_ppm;
    if (!(flags & RTC_SETF_INTERNAL)) host_sync_gen++;   /* a host's: a deep sleep's recovery yields to it */

    rtc_stamp_t now;
    bool have_now = rtc_now(&now);
    if (!have_now && rtc_valid) return RTC_SET_RETRY;   /* SECIF pending / ISR starved: caller retries */

    /* Target epoch (the LLD's convention: mktime on the wall fields, no TZ). */
    RTCDateTime dt_t;
    struct tm   tm_t;
    rtc_to_chibiostime(t, &dt_t);
    rtcConvertDateTimeToStructTm(&dt_t, &tm_t, NULL);
    time_t e_t = mktime(&tm_t);
    if (e_t == (time_t)-1) return RTC_SET_REJECT;

    /* Offset before correction: (t + ms) - board_now, ms, clamped to s16. */
    int32_t off = 0;
    if (have_now) {
        RTCDateTime dt_n; struct tm tm_n;
        rtc_to_chibiostime(&now.t, &dt_n);
        rtcConvertDateTimeToStructTm(&dt_n, &tm_n, NULL);
        time_t e_n = mktime(&tm_n);
        /* Board fraction = 1 - remaining/(nominal period): correct in steady
         * state AND inside a still-running shortened first period (a set
         * arriving within a second of the previous one). */
        int64_t frac_ms = 1000 - (int64_t)(now.period_active + 1 - now.cnt) * 1000 / (int64_t)(P_nom + 1);
        int64_t o = ((int64_t)e_t - (int64_t)e_n) * 1000 + (int64_t)ms - frac_ms;
        off = (o > 32767) ? 32767 : (o < -32768) ? -32768 : (int32_t)o;
    }

    /* Step or slew (PLAN.md 3.2/3.3): a small correction on an already
     * synced clock is slewed over N seconds at 20 ms/s so the display never
     * jumps; the first sync after boot, a large offset, or force_step steps. */
    if (have_now && host_synced && !(flags & RTC_SETF_FORCE_STEP) &&
        off >= -SLEW_MAX_MS && off <= SLEW_MAX_MS) {
        int64_t D    = (int64_t)off * (int64_t)(P_nom + 1) / 1000;      /* cycles, signed */
        int64_t absD = D < 0 ? -D : D;
        int32_t step = (int32_t)((uint64_t)(P_nom + 1) * 20u / 1000u);   /* 2 % of a period */
        uint16_t N   = (uint16_t)((absD + step - 1) / step);
        if (N < 1) N = 1;
        int32_t d = (int32_t)(D / N);                                   /* toward zero */
        int32_t r = (int32_t)(D - (int64_t)d * N);
        chSysLock();
        slew_d = d; slew_r = r; slew_left = N; slew_state = SLEW_START;
        chSysUnlock();
        slew_count++;
        last_host_offset_ms = (int16_t)off;
        last_sync_secs = rtc_seconds_count;
        if (offset_before) *offset_before = (int16_t)off;
        if (!(flags & RTC_SETF_SKIP_PCF)) {
            pcf_pending_time = *t;          /* whole seconds; phase-correct PCF is Phase 3 */
            pcf_pending      = true;
        }
#ifdef CONSOLE_ENABLE
        printf("[rtc] slew %ld ms: N=%u d=%ld r=%ld\n", (long)off, N, (long)d, (long)r);
#endif
        return RTC_SET_SLEWING;
    }

    /* First period: R ms to the next boundary. Too close -> label the one after. */
    uint32_t R     = 1000u - ms;
    time_t   sec   = e_t;
    uint64_t first = ((uint64_t)(P_nom + 1) * R) / 1000u;
    if (R < RTC_MIN_FIRST_MS) {
        sec   += 1;
        first += P_nom + 1;
    }
    if (first < 2) return RTC_SET_REJECT;
    uint32_t reg = (uint32_t)(first - 1);
    if (reg < 500 || reg > 0xFFFFFu) return RTC_SET_REJECT;   /* first-period range, not the steady one */

    struct tm tm_s;
    if (localtime_r(&sec, &tm_s) == NULL) return RTC_SET_REJECT;
    RTCDateTime dt_s;
    rtcConvertStructTmToDateTime(&tm_s, 0, &dt_s);

    chSysLock();
    slew_state = SLEW_IDLE;             /* a step supersedes any running slew */
    rtcSetTime(&RTCD1, &dt_s);          /* software seconds */
    clock_gen++;
    reload_pending = true;              /* ISR restores P_nom at the next match */
    rtc_lld_set_period(&RTCD1, reg);    /* resets SECCNT: next SECIF in R ms */
    interval_cycles = 0;                /* the interval now running is not a known length */
    win_valid = false;
    chSysUnlock();

    rtc_valid      = true;
    host_synced    = true;
    last_sync_secs = rtc_seconds_count;
    last_host_offset_ms = (int16_t)off;
    if (offset_before) *offset_before = (int16_t)off;

    if (!(flags & RTC_SETF_SKIP_PCF)) {
        rtc_time_t pt;
        chibiostime_to_rtc(&dt_s, &pt);
        pcf_pending_time = pt;         /* R5: the fast task performs the I2C */
        pcf_pending      = true;
    }
    return RTC_SET_STEPPED;
}

/* ---- Phase 3: PCF phase-correct write (PLAN.md 3.5) and boot acquisition
 * (PLAN.md 3.6). Both run from rtc_fast_task(): one I2C transaction per
 * main-loop pass at most, only when no LCD blit is in flight, never via the
 * blit-draining bus guard (R4). --------------------------------------------- */

/* Board time in milliseconds since boot, monotonic across ticks and phase
 * sets (seconds_count * 1000 + fraction of the current interval). */
static bool board_ms_now(int64_t *ms)
{
    rtc_stamp_t s;
    if (!rtc_now(&s)) return false;
    int64_t frac = 1000 - (int64_t)(s.period_active + 1 - s.cnt) * 1000 / (int64_t)(P_nom + 1);
    *ms = (int64_t)s.seconds_count * 1000 + frac;
    return true;
}

/* Un-guarded single-register access for the fast task (the caller has
 * already checked lcd_blit_busy()). */
static bool pcf_reg_read_raw(uint8_t reg, uint8_t *v) {
    msg_t r = pcf_xfer(OD_PCF_R1, &reg, 1, v, 1);
    if (r != MSG_OK) { if (pcf_avail_fail < 255) pcf_avail_fail++; return false; }
    pcf_avail_fail = 0;
    return true;
}
static bool pcf_reg_write_raw(uint8_t reg, uint8_t v) {
    uint8_t buf[2] = { reg, v };
    return pcf_xfer(OD_PCF_W1, buf, 2, NULL, 0) == MSG_OK;
}

#define PCF_CTL_STOP        0x20
#define PCF_D_FIRST_MS      490     /* clone's release-to-first-increment (T0.5: 489-494 ms) -- initial value */
static int32_t pcf_d_first_ms = PCF_D_FIRST_MS;   /* adapted from the bracket measurement below */
static int64_t ps_boundary_ms = 0;                /* the true boundary we aimed the increment at */
static int16_t pcf_boundary_err_ms = 0;           /* measured PCF increment - intended boundary */
static int64_t ps_brk_prev_t = 0;                 /* bracket: previous read stamp */
static int16_t ps_brk_prev_sec = -1;
#define PCF_WRITE_MS        1       /* one 1-byte write on this bus (T0.4) */
#define PCF_LATE_MS         10      /* missed the release window: restart */
#define PCF_MAX_ATTEMPTS    5       /* per transaction before recovery / back-off */

enum { PS_IDLE = 0, PS_STOP_READ, PS_STOP_WRITE, PS_TIME_WRITE, PS_RELEASE_READ,
       PS_RELEASE_WRITE, PS_VERIFY, PS_RECOVER, PS_BRACKET, PS_RECOVER_VERIFY };
static uint8_t  ps_state = PS_IDLE;
static uint8_t  ps_ctl = 0;
static bool     stop_asserted = false;
static uint8_t  ps_attempts = 0;
static time_t   ps_S = 0;                 /* the second being written */
static int64_t  ps_trel_ms = 0;           /* release instant, board ms */
static int8_t   pcf_release_err_ms = 0;   /* achieved release - T_rel, last run */
static uint16_t pcf_runs_ok = 0, pcf_restarts = 0;

/* Choose S (the next whole second per the board clock) and T_rel. */
static bool ps_plan(void)
{
    rtc_stamp_t s; int64_t now_ms;
    if (!rtc_now(&s) || !board_ms_now(&now_ms)) return false;
    RTCDateTime dt; struct tm tm;
    rtc_to_chibiostime(&s.t, &dt);
    rtcConvertDateTimeToStructTm(&dt, &tm, NULL);
    time_t cur = mktime(&tm);
    if (cur == (time_t)-1) return false;
    int64_t frac = 1000 - (int64_t)(s.period_active + 1 - s.cnt) * 1000 / (int64_t)(P_nom + 1);
    int64_t next_boundary = now_ms + (1000 - frac);       /* boundary of cur+1 */
    /* The PCF's FIRST increment after release moves the register from S to
     * S+1, and we time that increment onto boundary(cur+1), where true time
     * becomes cur+1. So the register must hold S = cur (not cur+1 -- that
     * off-by-one put the PCF a second ahead on the first Phase 3 build). */
    ps_S = cur;
    ps_boundary_ms = next_boundary;
    ps_trel_ms = next_boundary - pcf_d_first_ms;
    if (ps_trel_ms - now_ms < 5 + PCF_WRITE_MS + 3) {    /* too close: the one after */
        ps_S += 1; ps_trel_ms += 1000; ps_boundary_ms += 1000;
    }
    return true;
}

static void ps_fail(void)
{
    if (++ps_attempts < PCF_MAX_ATTEMPTS) return;
    ps_attempts = 0;
    if (stop_asserted) { ps_state = PS_RECOVER; pcf_maybe_stopped = true; }
    else { ps_state = PS_IDLE; pcf_pending = false; pcf_backoff = true; pcf_backoff_until = rtc_seconds_count + 60; }
}

/* One state step; returns after at most one I2C transaction. */
static void pcf_machine_step(void)
{
    switch (ps_state) {
    case PS_IDLE:
        if (!pcf_pending) return;
        if (pcf_backoff) {
            if ((int32_t)(rtc_seconds_count - pcf_backoff_until) < 0) return;
            pcf_backoff = false;
        }
        ps_state = PS_STOP_READ; ps_attempts = 0;
        /* fall through */
    case PS_STOP_READ:
        if (pcf_reg_read_raw(0x00, &ps_ctl)) { ps_state = PS_STOP_WRITE; ps_attempts = 0; }
        else ps_fail();
        return;
    case PS_STOP_WRITE:
        if (pcf_reg_write_raw(0x00, (uint8_t)(ps_ctl | PCF_CTL_STOP))) {
            stop_asserted = true; ps_attempts = 0;
            if (ps_plan()) ps_state = PS_TIME_WRITE;   /* else: stay, re-plan next pass */
        } else ps_fail();
        return;
    case PS_TIME_WRITE: {
        struct tm tm_s; rtc_time_t t;
        if (localtime_r(&ps_S, &tm_s) == NULL) { ps_fail(); return; }
        RTCDateTime dt; rtcConvertStructTmToDateTime(&tm_s, 0, &dt);
        chibiostime_to_rtc(&dt, &t);
        if (pcf_write_raw(&t)) { ps_state = PS_RELEASE_READ; ps_attempts = 0; }
        else ps_fail();
        return;
    }
    case PS_RELEASE_READ: {
        int64_t now_ms;
        if (!board_ms_now(&now_ms)) return;
        if (now_ms < ps_trel_ms - PCF_WRITE_MS - 3) return;          /* not yet */
        if (now_ms > ps_trel_ms + PCF_LATE_MS) {                      /* missed: restart from STOP (still asserted) */
            pcf_restarts++; ps_state = PS_STOP_WRITE; ps_attempts = 0; return;
        }
        if (pcf_reg_read_raw(0x00, &ps_ctl)) { ps_state = PS_RELEASE_WRITE; ps_attempts = 0; }
        else ps_fail();
        return;
    }
    case PS_RELEASE_WRITE: {
        int64_t now_ms;
        if (!board_ms_now(&now_ms)) return;
        if (now_ms < ps_trel_ms - PCF_WRITE_MS) return;              /* wait for the exact pass */
        if (now_ms > ps_trel_ms + PCF_LATE_MS) {                      /* a busy pass delayed us */
            pcf_restarts++; ps_state = PS_STOP_WRITE; ps_attempts = 0; return;
        }
        if (pcf_reg_write_raw(0x00, (uint8_t)(ps_ctl & ~PCF_CTL_STOP))) {
            int64_t after; if (!board_ms_now(&after)) after = now_ms + PCF_WRITE_MS;
            int64_t err = after - ps_trel_ms;
            pcf_release_err_ms = (int8_t)(err > 127 ? 127 : err < -128 ? -128 : err);
            stop_asserted = false; ps_state = PS_VERIFY; ps_attempts = 0;
        } else ps_fail();
        return;
    }
    case PS_VERIFY: {
        uint8_t v;
        if (pcf_reg_read_raw(0x00, &v)) {
            if (v & PCF_CTL_STOP) { stop_asserted = true; ps_state = PS_RECOVER; pcf_maybe_stopped = true; return; }
            pcf_runs_ok++; pcf_pending = false; pcf_maybe_stopped = false;
            ps_state = PS_BRACKET; ps_attempts = 0; ps_brk_prev_sec = -1;
        } else ps_fail();
        return;
    }
    case PS_BRACKET: {
        /* Verify the RESULT, not the intent: bracket the PCF's first increment
         * with 1-byte reads (one per pass) from 60 ms before the aimed boundary
         * to 400 ms after; the increment's midpoint minus the aimed boundary is
         * the real phase error, and half of it feeds back into the release lead
         * so the next run lands closer. Never more than ~120 reads, only on a
         * sync, only with the bus idle. */
        int64_t now_ms;
        if (!board_ms_now(&now_ms)) return;
        if (now_ms < ps_boundary_ms - 60) return;
        if (now_ms > ps_boundary_ms + 400) {                  /* no increment seen: give up quietly */
            ps_state = PS_IDLE; ps_brk_prev_sec = -1;
#ifdef CONSOLE_ENABLE
            printf("[rtc] pcf set S: release err %d ms, increment NOT seen\n", pcf_release_err_ms);
#endif
            return;
        }
        uint8_t v;
        if (!pcf_reg_read_raw(PCF8563_REG_SECONDS, &v)) return;
        int64_t t_after; if (!board_ms_now(&t_after)) return;
        int16_t sec = (int16_t)bcd2dec(v & 0x7F);
        if (ps_brk_prev_sec >= 0 && sec != ps_brk_prev_sec) {
            int64_t edge = (ps_brk_prev_t + t_after) / 2;
            int64_t err  = edge - ps_boundary_ms;              /* + = PCF late */
            pcf_boundary_err_ms = (int16_t)(err > 32767 ? 32767 : err < -32768 ? -32768 : err);
            /* Adapt the lead: PCF late -> release earlier next time. */
            int32_t nd = pcf_d_first_ms + (int32_t)(err / 2);
            if (nd < 300) nd = 300;
            if (nd > 800) nd = 800;
            pcf_d_first_ms = nd;
            ps_state = PS_IDLE; ps_brk_prev_sec = -1;
#ifdef CONSOLE_ENABLE
            printf("[rtc] pcf set S ok: release err %d ms, boundary err %d ms, D_first -> %ld\n",
                   pcf_release_err_ms, pcf_boundary_err_ms, (long)pcf_d_first_ms);
#endif
            return;
        }
        ps_brk_prev_sec = sec; ps_brk_prev_t = (now_ms + t_after) / 2;
        return;
    }
    case PS_RECOVER: {
        /* STOP must never be left asserted: clear it with the preserved
         * control bits, indefinitely, at the back-off cadence. The write and
         * its check are a pass each, as every other step (codex, deep-sleep
         * build review 1: a recovery's passes take one transaction). */
        if ((int32_t)(rtc_seconds_count - pcf_backoff_until) < 0) return;
        pcf_backoff_until = rtc_seconds_count + 5;
        if (pcf_reg_write_raw(0x00, (uint8_t)(ps_ctl & ~PCF_CTL_STOP))) ps_state = PS_RECOVER_VERIFY;
        return;
    }
    case PS_RECOVER_VERIFY: {
        uint8_t v;
        if (pcf_reg_read_raw(0x00, &v) && !(v & PCF_CTL_STOP)) {
            stop_asserted = false; pcf_maybe_stopped = false;
            ps_state = PS_IDLE; pcf_pending = false;
            pcf_backoff = true; pcf_backoff_until = rtc_seconds_count + 60;
        } else {
            ps_state = PS_RECOVER;   /* again at the back-off cadence */
        }
        return;
    }
    }
}

/* Boot acquisition: bracket the PCF's seconds increment with 1-byte reads
 * every ACQ_EVERY passes (~20 ms), then take the full calendar and step the
 * board to the PCF's phase. Cancelled by a host sync. Re-armed by a deep
 * sleep's recovery.
 *
 * ONE I2C transaction per pass (plans/DEEP-SLEEP-BUILD-PLAN.md, step 4; codex,
 * the deep-sleep verifies): the bracket's read, the full calendar and the
 * confirming read each get a pass of their own. Split, the full read can come
 * a second after the bracket: it must show the second the bracket saw, within
 * ACQ_FULL_MAX_MS of the edge, or the bracket is dropped and reacquired (a
 * full read a second late would otherwise pass the confirmation, and the
 * elapsed time set the clock nearly a second wrong). The 1.5 s timeout runs
 * from the first start, across reacquisitions, and with the cancellation is
 * checked every pass (acq_watch), even while the bus defers the work. */
#define ACQ_EVERY       8
#define ACQ_TIMEOUT_MS  1500
#define ACQ_FULL_MAX_MS 900
/* 0-3 as before (the status page reports the state); FULL and CONFIRM are
 * the split's transient steps. */
enum { ACQ_WAIT_SPLASH = 0, ACQ_COARSE, ACQ_DONE, ACQ_ABORTED, ACQ_FULL, ACQ_CONFIRM };
static uint8_t  acq_state = ACQ_WAIT_SPLASH;
static uint8_t  acq_pass = 0, acq_fails = 0;
static int64_t  acq_t0 = 0, acq_prev_t = 0, acq_edge = 0;
static int16_t  acq_prev_sec = -1, acq_new_sec = -1;
static uint16_t acq_unc_ms = 0;
static int16_t  acq_step_ms = 0;
static rtc_time_t acq_full;
#ifdef AK820_DEV_HOOKS
static bool acq_dev_delay = false;   /* HC_DEVHOOK op 14 sub 6: the next full read held 1 s, once */
void rtc_dev_delay_full_once(void) { acq_dev_delay = true; }
#endif

static bool acq_state_done(void) { return acq_state == ACQ_DONE; }
static bool acq_active(void) {
    return acq_state == ACQ_COARSE || acq_state == ACQ_FULL || acq_state == ACQ_CONFIRM;
}

static void acq_reacquire(void) {
    acq_state = ACQ_COARSE; acq_prev_sec = -1; acq_pass = 0;
}

/* Every pass, before the bus admission: the timeout and the cancellation. */
static void acq_watch(void)
{
    if (!acq_active()) return;
    if (host_synced) { acq_state = ACQ_ABORTED; return; }   /* the host outranks the PCF */
    int64_t now; if (board_ms_now(&now) && now - acq_t0 > ACQ_TIMEOUT_MS) acq_state = ACQ_ABORTED;
}

/* One step; true if it used the bus. */
static bool acquisition_step(void)
{
    if (acq_state == ACQ_DONE || acq_state == ACQ_ABORTED) return false;
    if (host_synced) { acq_state = ACQ_ABORTED; return false; }
    if (acq_state == ACQ_WAIT_SPLASH) {
        if (!display_splash_done() || !rtc_valid) return false;
        int64_t ms; if (!board_ms_now(&ms)) return false;
        acq_t0 = ms; acq_state = ACQ_COARSE; acq_pass = 0; acq_prev_sec = -1;
    }
    int64_t t_before; if (!board_ms_now(&t_before)) return false;
    if (t_before - acq_t0 > ACQ_TIMEOUT_MS) { acq_state = ACQ_ABORTED; return false; }
    if (acq_state == ACQ_FULL) {
#ifdef AK820_DEV_HOOKS
        if (acq_dev_delay) {
            if (t_before - acq_edge < 1000) return false;
            acq_dev_delay = false;   /* now a second late: it must reacquire */
        }
#endif
        if (t_before - acq_edge > ACQ_FULL_MAX_MS) { acq_reacquire(); return false; }   /* the bracket expired */
        if (!pcf_read_raw(&acq_full)) { if (++acq_fails >= 3) acq_state = ACQ_ABORTED; return true; }
        if (acq_full.seconds != acq_new_sec) { acq_reacquire(); return true; }        /* a second late */
        acq_state = ACQ_CONFIRM;
        return true;
    }
    if (acq_state == ACQ_CONFIRM) {
        uint8_t v2;
        if (!pcf_reg_read_raw(PCF8563_REG_SECONDS, &v2)) { if (++acq_fails >= 3) acq_state = ACQ_ABORTED; return true; }
        if (bcd2dec(v2 & 0x7F) != acq_full.seconds) { acq_reacquire(); return true; }   /* raced a rollover */
        int64_t now_ms; if (!board_ms_now(&now_ms)) return true;
        int64_t elapsed = now_ms - acq_edge;                                            /* ms since the PCF boundary */
        if (elapsed < 0) elapsed = 0;
        if (elapsed > 990) { acq_reacquire(); return true; }   /* never clamp a late step onto the wrong second */
        int16_t off = 0;
        uint8_t st = rtc_set_time_ms(&acq_full, (uint16_t)elapsed,
                                     RTC_SETF_FORCE_STEP | RTC_SETF_SKIP_PCF | RTC_SETF_INTERNAL, 0x7FFF, &off);
        host_synced = false;                /* this was the PCF, not the host */
        acq_step_ms = off;
        acq_state = (st == RTC_SET_STEPPED) ? ACQ_DONE : ACQ_ABORTED;
#ifdef CONSOLE_ENABLE
        printf("[rtc] acquisition: step %d ms, +/-%u ms\n", off, acq_unc_ms);
#endif
        return true;
    }
    if (++acq_pass < ACQ_EVERY) return false;
    acq_pass = 0;
    uint8_t v;
    if (!pcf_reg_read_raw(PCF8563_REG_SECONDS, &v)) { if (++acq_fails >= 3) acq_state = ACQ_ABORTED; return true; }
    int64_t t_after; if (!board_ms_now(&t_after)) return true;
    int16_t sec = (int16_t)bcd2dec(v & 0x7F);
    if (acq_prev_sec >= 0 && sec != acq_prev_sec) {
        /* Edge between the previous read and this one: midpoint, half-width. */
        acq_edge    = (acq_prev_t + t_after) / 2;
        acq_unc_ms  = (uint16_t)((t_after - acq_prev_t) / 2);
        acq_new_sec = sec;
        acq_state   = ACQ_FULL;
        return true;
    }
    acq_prev_sec = sec; acq_prev_t = (t_before + t_after) / 2;
    return true;
}

/* --- Deep sleep's recovery (plans/DEEP-SLEEP-BUILD-PLAN.md, section 4) -------
 * The RTC counts on asleep, but its second runs ~13% slow there (3139 s
 * counted for 3600, 10-19), so the slept time comes from the PCF: a read just
 * before the lock and one after the wake, whole seconds. The order:
 *  - under the lock (rtc_wake_beginI): the SOF reference broken completely,
 *    the trim window restarted (the slow asleep seconds are not the awake
 *    ILRC's), the 60 s discipline suspended, the host-sync generation noted;
 *  - once sleep.c's input gate lets go (rtc_wake_allow): the coarse read. The
 *    slept seconds, checked against the counted; rtc_seconds_count takes the
 *    difference; the calendar is set to the PCF's whole seconds, unless a host
 *    sync came since the wake (the newer time wins); the old host sync's flag
 *    cleared, once;
 *  - then the boot acquisition, every field reset, for the phase.
 * Every PCF transaction meanwhile -- these, the acquisition's, the write
 * machine's -- goes through one arbiter: at most one a pass, from the fast
 * task's slot, with the display bus idle, under 10 ms into the pass, with the
 * 5 ms timeout; a pass that used it defers its display work. */
enum { WK_IDLE = 0, WK_COARSE, WK_ACQ };
#define RTC_ADMIT_US 10000u
static uint8_t  wk_state = WK_IDLE;
static bool     wk_allowed, wk_pre_ok, wk_from_pcf, wk_coarse_done, wk_gauge_taken;
static uint32_t wk_counted, wk_slept_s;
static time_t   wk_pre_epoch;
static uint16_t wk_gen;

static bool rtc_time_epoch(const rtc_time_t *t, time_t *e)
{
    RTCDateTime dt; struct tm tm;
    rtc_to_chibiostime(t, &dt);
    rtcConvertDateTimeToStructTm(&dt, &tm, NULL);
    *e = mktime(&tm);
    return *e != (time_t)-1;
}

static bool bus_admit(void)
{
    return display_bus_idle() && sleep_pass_us() < RTC_ADMIT_US;
}

#ifdef AK820_DEV_HOOKS
static bool burst_active(void);
#endif

bool rtc_sleep_ok(void)
{
#ifdef AK820_DEV_HOOKS
    if (burst_active()) return false;   /* a bench burst of PCF reads holds the board awake */
#endif
    /* Nothing in flight that a skipped second handler would break (a slew, a
     * reload), and the PCF quiet: no write pending or running (its verify and
     * STOP recovery included), no acquisition, no recovery unfinished. An
     * acquisition still waiting for a first valid read (a dead PCF) does not
     * hold the board awake. */
    bool acq_busy = acq_active() || (acq_state == ACQ_WAIT_SPLASH && rtc_valid);
    return slew_state == SLEW_IDLE && !reload_pending && ps_state == PS_IDLE && !pcf_pending && !acq_busy &&
           wk_state == WK_IDLE;
}

bool rtc_sleep_prepare(void)
{
    if (!bus_admit()) return false;
    rtc_time_t t;
    pcf_short_timeout = true;
    wk_pre_ok = pcf_read_raw(&t) && rtc_time_epoch(&t, &wk_pre_epoch);
    pcf_short_timeout = false;
    return true;   /* read or not: a failed read leaves the counted seconds to stand */
}

void rtc_wake_beginI(uint32_t counted_s)
{
    win_valid = false;   /* the estimator's window spans time it did not see */
    fn_valid  = 0;       /* the SOF count's continuity (USB was down) */
    sof_epoch++;         /* ... broken outright, and the reference re-qualifies */
    sof_ok_run = 0;
#ifdef RTC_AUTO_CALIBRATION
    rtc_cal_valid = false;
#endif
    wk_state       = WK_COARSE;
    wk_allowed     = false;
    wk_coarse_done = false;
    wk_gauge_taken = false;
    wk_from_pcf    = false;
    wk_counted     = counted_s;
    wk_slept_s     = counted_s;
    wk_gen         = host_sync_gen;
}

void rtc_wake_allow(void)
{
    wk_allowed = true;
}

void rtc_wake_gauge_taken(void)
{
    wk_gauge_taken = true;
}

uint32_t rtc_seconds_since_boot(void)
{
    return rtc_seconds_count;
}

bool rtc_pass_used_bus(void)
{
    return pcf_xfers != pcf_xfers_mark;
}

void rtc_pass_mark(void)
{
    pcf_xfers_mark = pcf_xfers;
}

bool rtc_wake_busy(void)
{
    return wk_state != WK_IDLE;
}

/* sleep.c's bound on a whole recovery: whatever is left ends now. A coarse
 * read that never ran leaves the counted seconds to stand; the wake's
 * acquisition is dropped; the discipline resumes with a fresh window. */
void rtc_wake_abandon(void)
{
    if (wk_state == WK_IDLE) return;
    if (wk_state == WK_COARSE) {
        wk_from_pcf    = false;
        wk_slept_s     = wk_counted;
        wk_coarse_done = true;
    } else if (acq_state != ACQ_DONE) {
        acq_state = ACQ_ABORTED;
    }
    wk_state = WK_IDLE;
}

bool rtc_wake_slept(uint32_t *slept_s, bool *from_pcf)
{
    if (wk_state == WK_COARSE) return false;   /* the coarse read still to come */
    *slept_s  = wk_slept_s;
    *from_pcf = wk_from_pcf;
    return true;
}

uint16_t rtc_clock_generation(void)
{
    return clock_gen;
}

int16_t rtc_wake_acq_step_ms(void)
{
    return acq_step_ms;
}

uint8_t rtc_wake_acq_result(void)
{
    return acq_state;
}

static void acq_rearm(void)
{
    acq_pass = 0; acq_fails = 0;
    acq_t0 = 0; acq_prev_t = 0; acq_edge = 0;
    acq_prev_sec = -1; acq_new_sec = -1;
    acq_unc_ms = 0; acq_step_ms = 0;
    acq_state = rtc_valid ? ACQ_WAIT_SPLASH : ACQ_ABORTED;
}

/* The coarse step: one transaction. */
static void wake_coarse(void)
{
    rtc_time_t t;
    time_t     after = 0;
    bool       ok    = wk_pre_ok && pcf_read_raw(&t) && rtc_time_epoch(&t, &after);
    int64_t    slept = ok ? (int64_t)(after - wk_pre_epoch) : -1;
    /* Checked against the counted seconds: asleep the ILRC runs ~13-16% slow,
     * so the PCF's figure is a little more; one far off is a bad read. Never
     * negative: a short sleep's window reaches below zero, and -1 s as u32
     * would debit the pack ~49.7 days (codex, deep-sleep build review 4). */
    if (ok && (slept < 0 || slept < (int64_t)wk_counted / 2 - 2 || slept > (int64_t)wk_counted * 2 + 2)) ok = false;
    wk_from_pcf = ok;
    wk_slept_s  = ok ? (uint32_t)slept : wk_counted;
    if (ok) {
        int32_t diff = (int32_t)(wk_slept_s - wk_counted);
        chSysLock();
        rtc_seconds_count += (uint32_t)diff;   /* stays seconds since boot */
        chSysUnlock();
        if (host_sync_gen == wk_gen) {         /* no host sync since the wake: the PCF's seconds */
            RTCDateTime dt;
            rtc_to_chibiostime(&t, &dt);
            rtcSetTime(&RTCD1, &dt);
            clock_gen++;
        }
    }
    if (host_sync_gen == wk_gen) host_synced = false;   /* the old sync no longer holds: cleared once */
    wk_coarse_done = true;
    acq_rearm();
    wk_state = WK_ACQ;
}

/* One recovery step under the arbiter; true if it used the bus. */
static bool wake_step(void)
{
    if (wk_state == WK_COARSE) {
        if (!wk_allowed) return false;
        wake_coarse();
        return true;
    }
    /* WK_ACQ: the acquisition until it is done or aborted; then the clock's
     * part ends. It starts only once the gauge has taken the slept seconds,
     * so the pass after the coarse read is free of I2C for it (codex,
     * deep-sleep build review 2). */
    if (acq_state == ACQ_DONE || acq_state == ACQ_ABORTED) {
        wk_state = WK_IDLE;
        return false;
    }
    if (!wk_gauge_taken) return false;
    return acquisition_step();
}

#ifdef AK820_DEV_HOOKS
/* A burst of 7-byte PCF reads on demand (HC_DEVHOOK op 23): the overrun
 * A/B's load on the I2C (plans/OVERRUN-CAPS-PLAN.md, step 1, item 3). Into a
 * scratch buffer, so it sets no clock and moves no state machine, but a good
 * read resets pcf_avail_fail like any other. One read a pass at most, and
 * only in a pass with no PCF transaction yet, with the deep sleep's recovery
 * admission and its short deadline; never while a sync, an acquisition or a
 * recovery owns the PCF (rtc_fast_task returns before it), and the reference
 * check and the deep sleep wait for it. The gaps are random, so it cannot
 * lock to the echo's period or the loop's. */
static uint16_t      bu_left, bu_done, bu_failed, bu_skipped;
static uint8_t       bu_lo = 40, bu_hi = 80;
static uint32_t      bu_first_ms, bu_last_ms, bu_rng = 0x2545F491u;
/* The gaps run on the system tick, checked every pass (timer_read32 is a
 * 64-bit software divide here, ~250 us under the row ISR); `due` latches,
 * so a wait that outlasts the 16-bit tick's ~349 ms wrap stays due. */
static systime_t     bu_t0;
static sysinterval_t bu_wait;
static bool          bu_due;

static uint32_t bu_gap(void)
{
    bu_rng ^= bu_rng << 13;
    bu_rng ^= bu_rng >> 17;
    bu_rng ^= bu_rng << 5;
    return bu_lo + (bu_rng % (uint32_t)(bu_hi - bu_lo + 1u));
}

static bool burst_active(void)
{
    return bu_left != 0;
}

bool rtc_burst_start(uint16_t n, uint8_t lo_ms, uint8_t hi_ms)
{
    if (n == 0 || n > 2000u || lo_ms < 20u || hi_ms < lo_ms) return false;
    bu_lo = lo_ms; bu_hi = hi_ms;
    bu_done = bu_failed = bu_skipped = 0;
    bu_first_ms = bu_last_ms = 0;
    bu_rng ^= timer_read32() | 1u;
    bu_t0   = chVTGetSystemTimeX();
    bu_wait = TIME_MS2I(bu_gap());
    bu_due  = false;
    bu_left = n;
    return true;
}

void rtc_burst_stop(void)
{
    bu_left = 0;
}

/* True if it read this pass. */
static bool burst_step(void)
{
    if (!bu_left) return false;
    if (!bu_due) {
        if (chTimeDiffX(bu_t0, chVTGetSystemTimeX()) < bu_wait) return false;
        bu_due = true;
    }
    if (rtc_pass_used_bus() || !bus_admit()) {
        if (bu_skipped < 0xFFFFu) bu_skipped++;
        return false;
    }
    rtc_time_t scratch;
    pcf_short_timeout = true;
    bool ok = pcf_read_raw(&scratch);
    pcf_short_timeout = false;
    display_defer_pass();
    uint32_t now = timer_read32();
    if (!bu_done && !bu_failed) bu_first_ms = now;
    bu_last_ms = now;
    if (ok) bu_done++;
    else if (bu_failed < 0xFFFFu) bu_failed++;
    bu_left--;
    bu_t0   = chVTGetSystemTimeX();
    bu_wait = TIME_MS2I(bu_gap());
    bu_due  = false;
    return true;
}

/* Op 23's reply: [0..1] reads left, [2..3] done, [4..5] failed, [6..9] ms
 * from the first read to the last, [10] active, [11] 0, [12..13] due passes
 * skipped (the bus used or not admitted), [14] the gap's low, [15] high. */
void rtc_burst_fill(uint8_t *out)
{
    memset(out, 0, 28);
    put16(&out[0], bu_left);
    put16(&out[2], bu_done);
    put16(&out[4], bu_failed);
    put32(&out[6], bu_last_ms - bu_first_ms);
    out[10] = burst_active() ? 1 : 0;
    put16(&out[12], bu_skipped);
    out[14] = bu_lo;
    out[15] = bu_hi;
}
#endif /* AK820_DEV_HOOKS */

/* Per main-loop pass, BEFORE display_blit_pump() (R4). At most one PCF
 * transaction, only when no blit is in flight; never the bus guard. */
void rtc_fast_task(void)
{
    acq_watch();   /* the acquisition's timeout and cancellation, every pass, even deferred */
    if (lcd_blit_busy()) {
        if ((pcf_pending || ps_state != PS_IDLE) && deferred_passes < 0xFFFF) deferred_passes++;
        return;
    }
    /* A deep sleep's recovery: the arbiter's stricter rules for every PCF
     * transaction until the WHOLE recovery ends (sleep_recovering()), not
     * only the clock's part (see "Deep sleep's recovery" above; codex,
     * deep-sleep build review 1): after the acquisition, a host sync's write
     * machine runs under them too. The clock's part ends when the acquisition
     * does, whether or not the bus is free this pass. */
    if (wk_state == WK_ACQ && (acq_state == ACQ_DONE || acq_state == ACQ_ABORTED)) wk_state = WK_IDLE;
    if (wk_state != WK_IDLE || sleep_recovering()) {
        if (!bus_admit()) return;
        uint32_t n0 = pcf_xfers;
        pcf_short_timeout = true;
        if (wk_state != WK_IDLE) wake_step();
        else pcf_machine_step();
        pcf_short_timeout = false;
        if (pcf_xfers != n0) display_defer_pass();
        return;
    }
    if (acq_state == ACQ_WAIT_SPLASH || acq_active()) { acquisition_step(); return; }
#ifdef AK820_DEV_HOOKS
    if (!pcf_pending && ps_state == PS_IDLE && burst_step()) return;
#endif
    pcf_machine_step();
}

void rtc_status_fill_page4(uint8_t *out)
{
    out[0] = ps_state; out[1] = stop_asserted; out[2] = (uint8_t)pcf_release_err_ms;
    put16(&out[3], pcf_runs_ok); put16(&out[5], pcf_restarts);
    out[7] = pcf_maybe_stopped; out[8] = acq_state; put16(&out[9], acq_unc_ms);
    put16(&out[11], (uint16_t)acq_step_ms); out[13] = ps_attempts; out[14] = pcf_pending;
    out[15] = ref_state; put16(&out[16], slew_count); put16(&out[18], reload_writes);
    put16(&out[20], (uint16_t)pcf_boundary_err_ms); put16(&out[22], (uint16_t)pcf_d_first_ms);
}



void rtc_task(void)
{
    /* USB-active mirror for the tick ISR (PLAN.md 3.4): I-class read under a
     * short lock, published as a plain volatile byte. */
    chSysLock();
    uint8_t ua = (usbGetDriverStateI(&USBD1) == USB_ACTIVE) ? 1 : 0;
    chSysUnlock();
    usb_active_mirror = ua;

    rtc_ref_task();      /* Phase 2: SOF window evaluation + reference-source transitions */
    rtc_persist_task();  /* C2: the one place the period is saved, <= once a minute */

#ifdef RTC_AUTO_CALIBRATION

    /* Suspended through a deep sleep's recovery: its coarse read and the
     * acquisition own the PCF, its passes are budgeted, and the trim window
     * restarts after it. And never while a write to the PCF is pending or
     * running: until it lands the PCF holds the time before a host's sync,
     * and a snap to it would undo the sync (codex, deep-sleep build review
     * 1). The check stays due and runs once the write is done. */
    if (wk_state != WK_IDLE || sleep_recovering()) return;
    if (pcf_pending || ps_state != PS_IDLE) return;
#ifdef AK820_DEV_HOOKS
    if (burst_active()) return;   /* the check stays due; one PCF transaction a pass */
#endif
    if (rtc_check_seconds < RTC_CHECK_INTERVAL_S) {
        return;
    }
    if (ref_state == REF_SOF) {
        /* SOF owns P_nom and the host owns phase: the PCF is write-only here
         * (PLAN.md 3.4/3.6). The check counter still runs so the legacy
         * discipline resumes promptly if the cable goes away. */
        chSysLock();
        rtc_check_seconds = 0;
        chSysUnlock();
        return;
    }

    /* The 1 Hz RTC callback RMWs rtc_check_seconds in ISR context; a bare
     * main-loop store here can lose against it (load/store/store-old+1) and
     * re-trigger a check ~1 s later. Benign -- the trim measures from
     * snap-immune sources, not this counter -- but masking one store is
     * cheaper than reasoning about it again. (Audit C-1.) */
    chSysLock();
    rtc_check_seconds = 0;
    chSysUnlock();
    rtc_clock_discipline();
#endif
}