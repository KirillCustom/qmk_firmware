// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Wall date/time in plain decimal (human-natural: month 1-12, year e.g. 2026). */
typedef struct {
    uint8_t  seconds;  // 0-59
    uint8_t  minutes;  // 0-59
    uint8_t  hours;    // 0-23
    uint8_t  day;      // 1-31
    uint8_t  weekday;  // 0-6
    uint8_t  month;    // 1-12
    uint16_t year;     // e.g. 2026
} rtc_time_t;

/* The clock behind this API is two physical devices that the module hides: an
 * external battery-backed PCF8563 (accurate reference) and the SN32's internal
 * RTC (the live 1 Hz clock the display reads), seeded from and -- with
 * RTC_AUTO_CALIBRATION -- continuously disciplined to the PCF8563. Consumers just
 * ask for the time. */

/* Bring up the clock: start I2C, enable the SN32 1 Hz counter, and take a first
 * seed from the PCF8563. Call once from keyboard_post_init_kb (thread context).
 * If the PCF isn't readable/set yet, rtc_task() keeps retrying. */
void rtc_init(void);

/* Read the current wall time. Returns false until the clock has been seeded from
 * a valid PCF8563 (caller should show an "unset" placeholder in that case). */
bool rtc_get_time(rtc_time_t *out);

/* Set the time: persist to the PCF8563 AND set the running SN32 clock (so the
 * display updates immediately). Returns false if the PCF write failed (i.e. the
 * time was applied but not persisted). Blocking I2C -- call from thread context. */
bool rtc_set_time(const rtc_time_t *t);

/* Free-running count of RTC second interrupts (~seconds since rtc_init) -- cheap,
 * no calendar conversion. For once-per-second edge detection (e.g. pacing the
 * display refresh). NOT wall time; use rtc_get_time() for that. */
uint32_t rtc_get_seconds(void);
/* The seconds a deep sleep's wake has credited to rtc_get_seconds() for what
 * the ILRC missed asleep (wake_coarse), summed since boot, wrapping: a stamp
 * taken from the count can take a later credit back out
 * (rtc/rtc_payload.h's rtc_stamp_age_s). */
uint32_t rtc_seconds_credit(void);
/* The lock-LED clock's rate trim (rtc_set_time_ms with RTC_SETF_FRAME): the
 * trims made since boot and the period the last one set (0: none yet). */
void rtc_frame_trim_stats(uint16_t *trims, uint16_t *period);
/* Live SN32 divider period (trimmed register value). */
uint32_t rtc_get_period(void);

/* Per-housekeeping tick: retries the initial seed until it takes and runs the
 * auto-calibration pass when one is due. Cheap no-op otherwise. Call every
 * housekeeping pass from thread context. */
void rtc_task(void);
/* Deep sleep (plans/DEEP-SLEEP-BUILD-PLAN.md, sections 1 and 4):
 *  - rtc_sleep_ok(): nothing in flight a skipped second handler would break,
 *    the PCF quiet (no write pending or running, no acquisition, no recovery);
 *  - rtc_sleep_prepare(): the PCF's "before" read, just before the lock, by
 *    the arbiter (the display bus idle, under 10 ms into the pass). False:
 *    not admitted this pass, so the sleep waits;
 *  - rtc_account_sleepI(): the counted seconds, under the lock (I-class);
 *  - rtc_wake_beginI(): a real sleep's wake, under the lock: the SOF reference
 *    broken, the trim window restarted, the discipline suspended;
 *  - rtc_wake_allow(): sleep.c's input gate let go; the coarse read may run;
 *  - rtc_wake_slept(): the slept seconds once the coarse read is done (from the
 *    PCF, or the counted ones if it failed); rtc_wake_busy(): until the
 *    acquisition has ended;
 *  - rtc_wake_gauge_taken(): the gauge has the slept seconds; the acquisition
 *    starts only after it, so the gauge's pass is free of I2C;
 *  - rtc_pass_used_bus(): a PCF transaction since the last rtc_pass_mark(),
 *    which sleep_task() calls at each pass's end;
 *  - rtc_clock_generation(): bumped by every change to the calendar shown, so
 *    the display can tell a paint made after the correction. */
bool     rtc_sleep_ok(void);
bool     rtc_sleep_prepare(void);
void     rtc_account_sleepI(uint32_t seconds);
void     rtc_wake_beginI(uint32_t counted_s);
void     rtc_wake_allow(void);
bool     rtc_wake_busy(void);
void     rtc_wake_abandon(void);   /* sleep.c's bound: what is left ends now */
bool     rtc_wake_slept(uint32_t *slept_s, bool *from_pcf);
void     rtc_wake_gauge_taken(void);
bool     rtc_pass_used_bus(void);
/* The RTC's seconds since boot, slept ones included: one volatile read, so
 * cheap and ISR-safe (QMK's timer_read32 is a 64-bit software divide here). */
uint32_t rtc_seconds_since_boot(void);
void     rtc_pass_mark(void);
int16_t  rtc_wake_acq_step_ms(void);
uint8_t  rtc_wake_acq_result(void);   /* the acquisition's state: 2 done, 3 aborted */
uint16_t rtc_clock_generation(void);
#ifdef AK820_DEV_HOOKS
/* The acquisition's split, tested (HC_DEVHOOK op 14 sub 6): its next full read
 * held 1 s past the bracket's edge, once. */
void rtc_dev_delay_full_once(void);
/* The overrun A/B's burst of PCF reads (HC_DEVHOOK op 23): n (1..2000) reads,
 * random gaps of lo_ms..hi_ms (lo >= 20); or, with mask_us (1..3000), as many
 * spans of interrupts masked that long instead, the receiver's depth probe.
 * False if refused. */
bool rtc_burst_start(uint16_t n, uint8_t lo_ms, uint8_t hi_ms, uint16_t mask_us);
void rtc_burst_stop(void);
void rtc_stuck_test(void);   /* op 23 sub 5: one ordinary read into its real deadline */
void rtc_burst_fill(uint8_t *out28);
#endif

/* How many PCF8563 transactions would have collided with a live flash->LCD DMA.
 * See the bus-guard note in rtc.c: the bit-banged I2C shares port A with the
 * flash SPI1 pins, so an overlap glitches the transfer. Nonzero means the
 * hazard is real on this unit. */
uint16_t rtc_i2c_overlaps(void);

/* ---- Sub-second clock (clock-sync plan, Phase 0) ------------------------
 * A coherent snapshot of the live clock: whole seconds plus the SN32 RTC
 * prescaler count (0..period, ~30 us per cycle on this unit's ILRC). Returns
 * false -- with NO timestamp -- if a consistent snapshot could not be taken
 * (a second event pending while its ISR is starved); callers postpone. */
typedef struct {
    rtc_time_t t;              /* whole seconds, as rtc_get_time() */
    uint32_t   cnt;            /* SECCNT at the snapshot */
    uint32_t   period_active;  /* SECCNTV in force at the snapshot (period = +1 cycles) */
    uint32_t   seconds_count;  /* rtc_get_seconds() at the snapshot */
} rtc_stamp_t;
bool rtc_now(rtc_stamp_t *s);

/* Raw-HID status blocks (layouts in PLAN.md section 3.7 / hid_protocol.c).
 * page 1: the 21-byte tail shared with RTC_GET_TIME[11..31]
 * page 2: 28 bytes of counters (stale reads, I2C failures, ISR latency ...)
 * page 3: the last 14 FRMNO deltas, u16 each (Phase 0 observation ring) */
void rtc_status_fill(uint8_t page, uint8_t *out);

/* ---- Phase 1: phase-correct set + deferred PCF write ---------------------- */
#define RTC_MIN_FIRST_MS   20      /* closer than this to a boundary: label the next one */
#define RTC_SETF_FORCE_STEP 0x01   /* (Phase 2: bypass the slew decision) */
#define RTC_SETF_SKIP_PCF   0x02   /* do not queue the PCF write */
#define RTC_SETF_FRAME      0x40   /* a lock-LED clock frame (type 4): closes the rate trim's window; never from a host's flags (rtc_ms_payload_valid) */
#define RTC_SETF_INTERNAL   0x80   /* the board's own (the acquisition): not a host sync */
enum { RTC_SET_STEPPED = 0, RTC_SET_SLEWING = 1, RTC_SET_RETRY = 0xFE, RTC_SET_REJECT = 0xFF };

/* "At the instant this call is made, true time is t + ms." Sets the software
 * seconds and re-phases the prescaler so the next tick lands on the boundary;
 * queues the PCF write for rtc_fast_task(). Returns an RTC_SET_* status and,
 * when measurable, the offset (t+ms) - board_now in ms via *offset_before.
 * The caller validates the calendar; ms > 999 is rejected here too. */
uint8_t rtc_set_time_ms(const rtc_time_t *t, uint16_t ms, uint8_t flags, int16_t bias_ppm, int16_t *offset_before);

/* Per main-loop pass, BEFORE display_blit_pump(): performs at most one
 * queued PCF transaction, only when no LCD blit is in flight. Never blocks
 * on the LCD. */
void rtc_fast_task(void);

#ifdef WDT_TEST_HOOKS
/* Phase 0 hardware-fact tests (instrumented builds only). These DELIBERATELY
 * mutate the RTC phase / PCF registers; resync the clock afterwards.
 * op codes and reply layouts: see hid_protocol.c HC_RTCTEST. */
void rtc_test_op(uint8_t op, const uint8_t *arg, uint8_t *reply);
#endif
