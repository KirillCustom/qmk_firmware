// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* The lock-LED channel's receiver (plans/LOCK-LED-CHANNEL-PLAN.md, "The wire
 * format"): frames a host writes through the keyboard-LED output report, which
 * the CH582F forwards as `5A` frames, decoded one LED byte at a time. A sender
 * first makes sure the board has seen its baseline (a change and back, then
 * the quiet): the first byte after a reset only sets the baseline.
 *
 * Pure logic: no HAL, no QMK, no globals. scripts/lockchan_test.py compiles this
 * file on the host and checks it against the reference encoder
 * (scripts/lockchan.py), including deleted, merged, inserted and corrupted
 * reports. The firmware holds one instance and feeds it from the 5A parser,
 * main-loop context only. Work per report is a few dozen operations, plus a
 * CRC-16 over at most 52 bytes when a payload completes. */

#define LOCKCHAN_MAX_PAYLOAD 48
#define LOCKCHAN_GAP_MS      150u     /* between transitions inside a frame */
#define LOCKCHAN_QUIET_MS    100u     /* before a preamble's first transition */
#define LOCKCHAN_LIFETIME_MS 12000u   /* a whole frame; P0's longest is ~8.7 s */

/* Profiles: which LED bits carry data. Caps (0x02) never does. */
enum {
    LOCKCHAN_P0 = 0,   /* Num + Scroll, base 3 (two lights; Windows' toggles) */
    LOCKCHAN_P1 = 1,   /* Num + bits 3-4, base 7 (the default; Scroll untouched) */
    LOCKCHAN_P2 = 2,   /* Num + Scroll + bits 3-4, base 15 */
    LOCKCHAN_NPROFILES
};

typedef enum {
    LOCKCHAN_ABORT_GAP,         /* no transition for LOCKCHAN_GAP_MS */
    LOCKCHAN_ABORT_LIFETIME,
    LOCKCHAN_ABORT_CAPS,        /* the Caps bit moved mid-frame */
    LOCKCHAN_ABORT_HEADER_CRC,
    LOCKCHAN_ABORT_HEADER,      /* version, profile or length not allowed */
    LOCKCHAN_ABORT_RANGE,       /* a block decoded past its bit count */
    LOCKCHAN_ABORT_CRC,         /* the frame CRC-16 */
    LOCKCHAN_ABORT_TRAILER,
    LOCKCHAN_ABORT_RESET,       /* the link went down, or a reset */
    LOCKCHAN_NABORTS
} lockchan_abort_t;

typedef struct {
    uint8_t  profile, type, seq, len;
    uint8_t  payload[LOCKCHAN_MAX_PAYLOAD];
    uint32_t anchor_ms;   /* when the preamble's last transition arrived */
    uint32_t commit_ms;   /* when the trailer's last one did */
} lockchan_frame_t;

typedef struct {
    /* Receiver state. */
    uint8_t  st;
    bool     have_prev;
    uint8_t  prev;              /* the last LED byte fed */
    uint32_t last_change_ms;    /* the last transition of any data-capable bit */
    uint32_t frame_start_ms;
    uint32_t anchor_ms;
    uint8_t  pre_mask, pre_n;
    uint8_t  sec_profile;       /* the profile of the section being decoded */
    uint8_t  pay_profile;
    uint8_t  baseline;          /* the LED byte before the preamble */
    uint8_t  trl_mid;           /* the trailer's expected first state */
    /* The current section's block decoder. */
    uint32_t acc;
    uint8_t  acc_digits;
    uint8_t  blk_bits, blk_digits;   /* the block being decoded */
    uint16_t sec_bits, got_bits;
    uint8_t  hdr[4];
    uint8_t  buf[LOCKCHAN_MAX_PAYLOAD + 2];
    /* Results. */
    lockchan_frame_t last;
    uint32_t frames;            /* committed */
    uint32_t preambles;         /* syncs reached */
    uint16_t aborts[LOCKCHAN_NABORTS];
} lockchan_t;

void lockchan_init(lockchan_t *lc);
/* Every LED byte the link delivers, raw (before any guard), with its time.
 * Returns true when it completed a frame: lockchan_last() holds it. */
bool lockchan_feed(lockchan_t *lc, uint8_t leds, uint32_t now_ms);
/* Each main-loop pass: times out a stalled frame; resets while unlinked. */
void lockchan_poll(lockchan_t *lc, uint32_t now_ms, bool linked);
/* The link went down or changed (a 5B down state, a slot select, pairing):
 * drop any frame in progress and the LED baseline AT ONCE, before the next byte
 * of the same UART drain, which may already hold a reconnect and the rest of
 * the old frame (codex, flash 2h). */
void lockchan_link_lost(lockchan_t *lc);
/* Mid-frame: the band's settled Scroll Lock waits while this is true. */
bool lockchan_busy(const lockchan_t *lc);
/* Zero the counters and the last frame; a frame in progress carries on. */
void lockchan_clear_stats(lockchan_t *lc);
