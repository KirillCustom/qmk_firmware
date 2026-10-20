// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lockchan.h"

/* What a committed lock-LED frame means (plans/LOCK-LED-CHANNEL-PLAN.md,
 * R2-9, version 0's types, frozen for the first release):
 *
 *   type 1, now playing: [icon][n0][line 0: n0 bytes][line 1: the rest]
 *     2 <= len <= 42; n0 <= 19, n0 <= len - 2, line 1 = len - 2 - n0 <= 21;
 *     icon exactly 0-3 (any other bit, the reserved playback bit 7 included,
 *     rejects); every text byte printable ASCII 0x20-0x7E.
 *   type 2, clear: exactly zero bytes.
 *
 * Anything else is rejected whole. Pure logic, as lockchan.c is: the board
 * applies a parsed frame (ch582f_ajazz.c), the host tests parse it. */

#define LOCKCHAN_TYPE_NOW_PLAYING 1
#define LOCKCHAN_TYPE_CLEAR       2
#define LOCKCHAN_MEDIA_MAX_L0     19
#define LOCKCHAN_MEDIA_MAX_L1     21

typedef struct {
    uint8_t        type;          /* LOCKCHAN_TYPE_* */
    uint8_t        icon;          /* type 1 */
    uint8_t        n0, n1;        /* type 1: the lines' lengths, either may be 0 */
    const uint8_t *l0, *l1;       /* into the frame's payload */
} lockchan_media_t;

/* True, with *m filled, when the frame is a valid type 1 or 2. On false, *m
 * is untouched and nothing may be applied. */
bool lockchan_media_parse(const lockchan_frame_t *f, lockchan_media_t *m);
