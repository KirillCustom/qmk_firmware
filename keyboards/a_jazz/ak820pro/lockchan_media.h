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
 *
 * Anything else is rejected whole. Pure logic, as lockchan.c is: the board
 * applies a parsed frame (ch582f_ajazz.c), the host tests parse it. */

#define LOCKCHAN_TYPE_NOW_PLAYING 1
#define LOCKCHAN_TYPE_CLEAR       2
#define LOCKCHAN_TYPE_TRANSPORT   3
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
