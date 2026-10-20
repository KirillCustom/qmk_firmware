// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lockchan.h"

/* What a committed lock-LED frame means (plans/LOCK-LED-CHANNEL-PLAN.md,
 * R2-9, version 0's types; the playback bytes added 10-20, "Play times"):
 *
 *   type 1, now playing: [icon][n0][line 0: n0 bytes][line 1: the rest]
 *     2 <= len <= 42; n0 <= 19, n0 <= len - 2, line 1 = len - 2 - n0 <= 21;
 *     icon 0-3; every text byte printable ASCII 0x20-0x7E.
 *     With icon's bit 7 set, 5 more bytes end the frame:
 *       [state][pos_hi][pos_lo][dur_hi][dur_lo], state 0 or 1 (playing),
 *     so line 1 is what lies between line 0 and them (7 <= len <= 47). The
 *     position is the host's at the frame's start; the board adds its own
 *     anchor-to-commit time (lockchan_media_position).
 *     Any other icon bit rejects.
 *   type 2, clear: exactly zero bytes.
 *
 * Anything else is rejected whole. Pure logic, as lockchan.c is: the board
 * applies a parsed frame (ch582f_ajazz.c), the host tests parse it. */

#define LOCKCHAN_TYPE_NOW_PLAYING 1
#define LOCKCHAN_TYPE_CLEAR       2
#define LOCKCHAN_MEDIA_MAX_L0     19
#define LOCKCHAN_MEDIA_MAX_L1     21
#define LOCKCHAN_MEDIA_PLAYBACK   0x80   /* icon's bit 7: the playback bytes follow */
#define LOCKCHAN_MEDIA_PB_LEN     5

typedef struct {
    uint8_t        type;          /* LOCKCHAN_TYPE_* */
    uint8_t        icon;          /* type 1 */
    uint8_t        n0, n1;        /* type 1: the lines' lengths, either may be 0 */
    const uint8_t *l0, *l1;       /* into the frame's payload */
    bool           playback;      /* type 1 with bit 7: the next three hold */
    uint8_t        state;         /* 0 not playing, 1 playing */
    uint16_t       pos, dur;      /* seconds; dur 0 is unknown (a stream) */
} lockchan_media_t;

/* True, with *m filled, when the frame is a valid type 1 or 2. On false, *m
 * is untouched and nothing may be applied. */
bool lockchan_media_parse(const lockchan_frame_t *f, lockchan_media_t *m);

/* The position to show at the commit: the frame's, plus the time from its
 * anchor to its commit, rounded to the second; never past a known duration,
 * never past 0xFFFF. */
uint16_t lockchan_media_position(const lockchan_media_t *m, uint32_t anchor_ms, uint32_t commit_ms);
