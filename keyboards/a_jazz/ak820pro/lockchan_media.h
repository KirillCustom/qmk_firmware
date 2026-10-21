// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lockchan.h"

/* What a committed lock-LED frame means (plans/LOCK-LED-CHANNEL-PLAN.md,
 * R2-9, version 0's types 1 and 2, frozen; type 3 added 10-20, "Play times"):
 *
 *   type 1, now playing: [icon][n0][line 0: n0 bytes][line 1: the rest]
 *     2 <= len <= 42; n0 <= 19, n0 <= len - 2, line 1 = len - 2 - n0 <= 21;
 *     icon exactly 0-3 (any other bit rejects); every text byte printable
 *     ASCII 0x20-0x7E. Playback off.
 *   type 2, clear: exactly zero bytes. Playback off.
 *   type 3, transport: exactly [icon][state][pos_hi][pos_lo][dur_hi][dur_lo],
 *     icon 0-3, state 0 or 1 (playing): the shown slot's icon and the
 *     playback readout, so a play/pause or a seek needs no title. The
 *     position is the host's at its pick, before the preamble; the board adds
 *     its anchor-to-commit time (lockchan_media_position). A new type, not a
 *     bit in type 1 (codex, the play-times review, P2 1): a board before it
 *     rejects the frame whole and keeps its text.
 *   type 4, clock (10-20, "The wireless clock"): exactly raw HID's
 *     RTC_SET_TIME_MS body, yy mm dd wd hh mi ss ms_lo ms_hi flags bias_lo
 *     bias_hi, local wall time, meaning "at the frame's anchor, true time was
 *     this". rtc/rtc_payload.h's rules, plus flags 0 and bias 0x7FFF (no
 *     wireless sample feeds a learner). Not a display frame: the board's
 *     clock (ch582f_ajazz.c), never the text slot.
 *
 * Anything else is rejected whole. Pure logic, as lockchan.c is: the board
 * applies a parsed frame (ch582f_ajazz.c), the host tests parse it. */

#define LOCKCHAN_TYPE_NOW_PLAYING 1
#define LOCKCHAN_TYPE_CLEAR       2
#define LOCKCHAN_TYPE_TRANSPORT   3
#define LOCKCHAN_TYPE_CLOCK       4
#define LOCKCHAN_CLOCK_LEN        12
#define LOCKCHAN_MEDIA_MAX_L0     19
#define LOCKCHAN_MEDIA_MAX_L1     21
#define LOCKCHAN_TRANSPORT_LEN    6

typedef struct {
    uint8_t        type;          /* LOCKCHAN_TYPE_* */
    uint8_t        icon;          /* types 1 and 3 */
    uint8_t        n0, n1;        /* type 1: the lines' lengths, either may be 0 */
    const uint8_t *l0, *l1;       /* into the frame's payload */
    uint8_t        state;         /* type 3: 0 not playing, 1 playing */
    uint16_t       pos, dur;      /* type 3: seconds; dur 0 is unknown (a stream) */
} lockchan_media_t;

/* True, with *m filled, when the frame is a valid type 1, 2 or 3. On false,
 * *m is untouched and nothing may be applied. */
bool lockchan_media_parse(const lockchan_frame_t *f, lockchan_media_t *m);

/* The position to show at the commit: the frame's, plus the time from its
 * anchor to its commit, rounded to the second; never past a known duration,
 * never past 0xFFFF. */
uint16_t lockchan_media_position(const lockchan_media_t *m, uint32_t anchor_ms, uint32_t commit_ms);

/* A type 4's time: rtc_time_t's fields, year less 2000, and the ms. */
typedef struct {
    uint8_t  yy, mo, dd, wd, hh, mi, ss;
    uint16_t ms;
} lockchan_clock_t;

/* True, with *c filled, when the frame is a valid type 4. */
bool lockchan_clock_parse(const lockchan_frame_t *f, lockchan_clock_t *c);

/* *c plus `elapsed_ms` (the anchor to the set), with the second, minute,
 * hour, day, month and year carried and the weekday moved by the days
 * carried (the host's numbering kept). False when the result leaves raw HID's
 * range (2026-2098), or elapsed_ms is over LOCKCHAN_CLOCK_MAX_ELAPSED_MS: no
 * anchor that old is trusted. */
#define LOCKCHAN_CLOCK_MAX_ELAPSED_MS 60000u
bool lockchan_clock_advance(const lockchan_clock_t *c, uint32_t elapsed_ms, lockchan_clock_t *out);

/* A committed type 4 on its way to the board's clock (ch582f_ajazz.c holds
 * one; codex, the wireless clock's review, must-fix 3, and its
 * implementation review, 7: pure, so the host test runs it). Each attempt
 * asks the board, through the hooks, whether another owner has the clock,
 * then sets it with the time advanced from the frame's own anchor. A RETRY
 * keeps the frame's copy and tries again each poll until
 * LOCKCHAN_CLOCK_RETRY_MS past its commit, a fixed deadline; a drop (a link
 * change, a wake, a raw-HID set) or a newer type 4 ends it. */
#define LOCKCHAN_CLOCK_RETRY_MS  2000u
#define LOCKCHAN_SET_RETRY       0xFE   /* rtc.h's RTC_SET_RETRY */
#define LOCKCHAN_SET_REJECT      0xFF   /* rtc.h's RTC_SET_REJECT */
typedef struct {
    /* Another owner has the clock now: usb_mode, unlinked, or raw HID set it
     * within its window. */
    bool (*owned_elsewhere)(void);
    /* rtc_set_time_ms with flags 0 and the bias unknown: an RTC_SET_*
     * status, and the offset before (target minus board, ms). */
    uint8_t (*set)(const lockchan_clock_t *at, int16_t *off);
} lockchan_clock_hooks_t;

typedef struct {
    lockchan_clock_t t;
    uint32_t         anchor_ms, commit_ms;
    bool             pending;
    uint8_t          status;    /* the last applied's RTC_SET_*; 0xFF none yet */
    int16_t          off;       /* the last applied's offset */
    uint16_t         applied, refused, rejected, retried, dropped;
} lockchan_clock_state_t;

#define LOCKCHAN_CLOCK_STATE_INIT {.status = 0xFF}
void lockchan_clock_commit(lockchan_clock_state_t *s, const lockchan_frame_t *f, uint32_t now,
                           const lockchan_clock_hooks_t *h);
void lockchan_clock_poll(lockchan_clock_state_t *s, uint32_t now, const lockchan_clock_hooks_t *h);
void lockchan_clock_drop(lockchan_clock_state_t *s);

/* The play icon, as the frame and display.h's DISPLAY_ICON_PLAY number it. */
#define LOCKCHAN_ICON_PLAY 1

/* What a type 1 does to the playback readout, from the slot it replaces
 * (plans/BACKLOG.md, "The clock flashes"). Turning the readout off at every
 * type 1 showed the clock for the ~3 s until its type 3:
 *   KEEP   the slot shows these lines with the play icon, and so does the
 *          frame: the host's 60 s refresh, or its resend at a wake. The
 *          readout stands, its own expiry unrenewed, so it still ends no
 *          later than the title it was set with would have.
 *   GUESS  other lines replace a shown title, with the play icon: a new
 *          track, counted from the frame's anchor until its type 3, which
 *          the host sends after every type 1. A track resumed part way
 *          reads low until then.
 *   OFF    anything else: no title shown (expired, or a wake), a pause or a
 *          stop, or the same title resumed (the position is the type 3's).
 * `shown` is whether the slot's title is live (its lease not spent); l0/n0,
 * l1/n1 and `icon` are what it holds. Pure, for the host test. */
enum { LOCKCHAN_READOUT_OFF = 0, LOCKCHAN_READOUT_KEEP, LOCKCHAN_READOUT_GUESS };
uint8_t lockchan_media_readout(const lockchan_media_t *m, bool shown, const char *l0, uint8_t n0,
                               const char *l1, uint8_t n1, uint8_t icon);
