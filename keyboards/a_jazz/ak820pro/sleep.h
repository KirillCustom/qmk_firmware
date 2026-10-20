// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <ch.h>

/* The deep sleep: the idle ladder's last stage, in every build
 * (plans/DEEP-SLEEP-BUILD-PLAN.md). power.c says when it is wanted
 * (power_sleep_wanted); sleep.c sleeps at the first pass where its
 * preconditions hold, wakes on a key, the knob, a charger or a daily backstop,
 * and owns the wake's recovery until every step has ended.
 *
 * Light sleep's sizing bits and the bench's controls (HC_DEVHOOK op 14) are
 * dev builds only. */

/* Each main-loop pass, at its very end (after the watchdog kick): the
 * recovery's steps, the sleep itself, and the pass budget's mark. */
void sleep_task(void);

/* The pass budget (plan section 4, "The budget"): system ticks since the
 * pass began. Marked at the end of each pass, so the next pass's count
 * includes its keyboard and raw-HID work, which run before housekeeping.
 * Compare with TIME_US2I() of a constant, which folds at compile time:
 * converting the count instead (TIME_I2US) is a 64-bit software divide on
 * this M0, ~250 us under the row ISR, and rtc_fast_task asks every pass
 * (Gremlin's 10-20 ge10 count, +39 against +11 per 1000 requests). */
sysinterval_t sleep_pass_ticks(void);

/* A wake's recovery is under way: no sleep, and deferred flash writes wait. */
bool sleep_recovering(void);

/* The sleep's figures for HC_POWER page 1 (hid_protocol.c), 29 bytes:
 * u8 0x81, u8 the last wake's source (SLEEP_WAKE_*), u8 validity bits
 * (SLEEP_V_*), u8 the recovery's open steps (SLEEP_RC_* still pending);
 * u32 sleeps since boot (also the last sleep's sequence number), u32 slept
 * seconds since boot, u32 the last sleep's PCF seconds, u32 its counted
 * seconds; u16 the acquisition's step (ms, signed), u16 PLL lock loops, u16
 * strays; u16 the blockers seen while a sleep was last wanted; then u8 the
 * last recovery's steps that ended on their own timeout or the 10 s bound
 * rather than by doing their job (SLEEP_RC_*: the input not counted in 50
 * ms, no battery report in 2 s, under 1.5 s of supply samples by 3 s, the
 * repaint not done in 500 ms, or whatever the bound ended). */
#define SLEEP_PAGE1_SIZE 29u
void sleep_page1_fill(uint8_t *out);

enum { SLEEP_WAKE_NONE = 0, SLEEP_WAKE_KEY, SLEEP_WAKE_BACKSTOP, SLEEP_WAKE_STRAYS, SLEEP_WAKE_BLOCKED,
       SLEEP_WAKE_KNOB, SLEEP_WAKE_CHARGE };
#define SLEEP_V_PCF       0x01u   /* the slept seconds are the PCF's (else the counted ones) */
#define SLEEP_V_ACQ_DONE  0x02u   /* the acquisition set the phase (else it aborted or was skipped) */
#define SLEEP_V_USB_SOF   0x04u   /* a host's frames came after the wake */
#define SLEEP_V_REATTACH  0x08u   /* the USB re-attach ran */
#define SLEEP_V_USB_20S   0x10u   /* a host's frames within 20 s of the wake */
#define SLEEP_V_PLL_CEIL  0x20u   /* the PLL's lock loops hit their ceiling (a reset follows) */
#define SLEEP_V_SLEPT     0x40u   /* the last attempt slept (else a key was already down) */
#define SLEEP_V_BOUND     0x80u   /* the recovery hit its 10 s bound: what was left was ended */

/* The steps still open in a recovery (the page's fourth byte). */
#define SLEEP_RC_INPUT    0x01u
#define SLEEP_RC_CLOCK    0x02u
#define SLEEP_RC_GAUGE    0x04u
#define SLEEP_RC_RADIO    0x08u
#define SLEEP_RC_SUPPLY   0x10u
#define SLEEP_RC_USB      0x20u
#define SLEEP_RC_DISPLAY  0x40u

/* --- Dev builds: light sleep's sizing build (plans/SLEEP-PLAN.md) ----------
 * A bitmask, each bit honoured only while the RGB is dark (power_rgb_dark())
 * and dropped the moment it lights: */
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

/* --- Dev builds: the bench's deep sleep (HC_DEVHOOK op 14 sub 4) -----------
 * Arm a sleep `delay_s` from now (or at the first pass after that where it is
 * safe), regardless of the ladder, waking on the usual sources or after `cap_s`
 * counted seconds (0: the 24 h backstop). `no_feed`: leave the watchdog unfed
 * asleep. `preset_s`: the counted seconds start there (the 32-bit widths'
 * test; the clocks take only what was counted past it). */
void sleep_deep_arm(uint8_t delay_s, uint32_t cap_s, bool no_feed, uint32_t preset_s);
void sleep_deep_cancel(void);   /* disarm, if armed and not yet asleep */
/* The last run's record, version 2, 27 bytes, little endian: u8 2 (the
 * version); u8 state (0 idle, 1 armed, 2 done); u8 reason (SLEEP_WAKE_*);
 * u16 blocked (the preconditions that failed, DSB_* in sleep.c); u8 flags
 * (SLEEP_V_*); u32 counted seconds; u16 strays; u16 the P0 edge bits, u16 the
 * P2 edge bits, u32 the P1 edge bits, that woke it; u16 PLL lock loops; u16
 * key_presses gained across the wake (counted 1 s after); u16 the sleep's
 * sequence number (low 16 bits); u8 the recovery's timed-out steps (as page
 * 1's last byte). */
#define SLEEP_RECORD_SIZE 27u
void sleep_deep_record(uint8_t *out);
/* Each teardown's wake, the last eight (HC_DEVHOOK op 24): the page keeps only
 * the last sleep's. Slot 0..7, 28 bytes, little endian: u16 the entry's
 * sequence number (entry n is in slot n % 8), u8 reason (SLEEP_WAKE_*), u8
 * flags (SLEEP_V_*), u32 counted seconds, u32 the P0, P1 and P2 bits that
 * woke it, u32 the uptime ms after the wake, u16 strays; [26..27] entries
 * written. Slot 8 is the last stray's diagnostic instead (sleep.c,
 * stray_diag_t): u16 the entry's sequence number, u16 strays within the first
 * one's counted second, u32 that second, u32 the whole RIS of P0, P1 and P2,
 * u32 NVIC->ISPR[0], u32 SCB->ICSR. False for a slot past 8. */
bool sleep_wake_ring_fill(uint8_t slot, uint8_t *out28);
