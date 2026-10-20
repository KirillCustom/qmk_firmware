// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later

/* Raw HID over the 2.4G dongle (plans/RAWHID-24G-PLAN.md, stage 2 revision
 * 2): the module-side collector, the request envelope, the freshness rule and
 * the allow-list, as pure functions of bytes, times and flags. No hardware
 * calls, so scripts/raw81_test.py compiles this file on the host and drives
 * every split, deletion and truncation through it; ch582f_ajazz.c feeds it
 * from the UART and acts on its verdicts.
 *
 * On the wire, from the module: 81 <32 bytes> <sum = (0x81 + the 32) & 0xFF>.
 * The 32 bytes are the host's request:
 *   [0] 0xC7  [1] seq  [2..3] CRC-16/CCITT-FALSE, big-endian, over [0..1]
 *   and [4..31]  [4..31] the inner command (28 bytes)
 * To the module, two frames per response: the reply (91 <32> <sum>), then
 * the receipt (91 <32> <sum>):
 *   [0] 0xC7  [1] seq  [2] status  [3] the last accepted seq
 *   [4..5] CRC-16 of the reply, big-endian (status 0 only)
 *   [6] 1 if [3] is meaningful (a seq has been accepted since the reset)
 *   [7..31] 0 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RAW81_FRAME_LEN 34u   /* 81, 32 bytes, sum */
#define RAW81_PAYLOAD   32u
#define RAW81_INNER     28u
#define RAW81_MAGIC     0xC7u

typedef enum { RAW81_IDLE = 0, RAW81_COLLECT = 1, RAW81_QUARANTINE = 2 } raw81_state_t;

/* What the caller does with the byte it just fed. */
typedef enum {
    RAW81_TO_WINDOW = 0,   /* not ours: the 3-byte window takes it */
    RAW81_TAKEN     = 1,   /* consumed (collecting or quarantined) */
    RAW81_STARTED   = 2,   /* consumed as an 81 header: clear the window */
    RAW81_FRAME     = 3,   /* consumed, and 34 bytes are in: call raw81_commit() */
} raw81_feed_t;

/* raw81_commit()'s verdict. */
typedef enum {
    RAW81_V_OK        = 0,   /* a fresh, allowed request: inner[] holds it */
    RAW81_V_STALE     = 1,   /* well formed, seq not fresh: receipt status 3 */
    RAW81_V_FORBIDDEN = 2,   /* well formed, not on the allow-list: status 2 */
    RAW81_V_BADENV    = 3,   /* outer sum good, magic or CRC bad: quarantined; status 1 */
    RAW81_V_BADSUM    = 4,   /* quarantined, no answer */
    RAW81_V_TAINT     = 5,   /* a UART error under the frame: quarantined, no answer */
    RAW81_V_LIFETIME  = 6,   /* older than its lifetime at commit: quarantined, no answer */
} raw81_verdict_t;

/* Receipt status bytes. */
enum { RAW81_ST_OK = 0, RAW81_ST_BADCRC = 1, RAW81_ST_FORBIDDEN = 2, RAW81_ST_STALE = 3 };

typedef struct {
    raw81_state_t state;
    bool          boundary;   /* the next byte may start an 81 */
    bool          tainted;    /* a UART error since the line was last silent */
    uint8_t       buf[RAW81_FRAME_LEN];
    uint8_t       len;
    uint32_t      start_ms;   /* when the collection began */
    uint16_t      last_tick;  /* the last byte, in the caller's 16-bit ticks */
    bool          have_last;  /* freshness: a seq has been accepted since the reset */
    uint8_t       last_seq;
    uint8_t       seq;        /* the seq of the frame just committed (verdicts OK..BADENV) */
    uint8_t       inner[RAW81_INNER];
    /* counters */
    uint32_t ok, stale, forbidden, badenv, badsum, taint, lifetime, qbytes;
} raw81_rx_t;

uint16_t raw81_crc16(const uint8_t *p, uint8_t n, uint16_t crc);

void raw81_init(raw81_rx_t *rx);
/* A resync or a wake: nothing half-collected survives, freshness starts over. */
void raw81_reset(raw81_rx_t *rx);
/* The link moved: freshness starts over; a collection in progress is
 * quarantined, so its tail cannot reach the window. */
void raw81_link_lost(raw81_rx_t *rx);

/* At a drain's start: latch a UART error (any state) and abort a collection
 * that saw one or outlived lifetime_ms. */
void raw81_poll(raw81_rx_t *rx, bool uart_err, uint32_t now_ms, uint32_t lifetime_ms);
/* One byte from the module, before the window sees it. */
raw81_feed_t raw81_byte(raw81_rx_t *rx, uint8_t c, uint16_t now_tick, uint32_t now_ms);
/* After the window processed a byte raw81_byte() returned TO_WINDOW for:
 * a boundary only right after a frame or ACK it completed. */
void raw81_window_done(raw81_rx_t *rx, bool matched);
/* The input queue is empty: true when that ended a quarantine (the caller
 * clears the window), after silence_ticks of no bytes. */
bool raw81_empty(raw81_rx_t *rx, uint16_t now_tick, uint16_t silence_ticks);
/* 34 bytes are in: check the sum, the taint, the lifetime, the envelope, the
 * freshness and the allow-list. */
raw81_verdict_t raw81_commit(raw81_rx_t *rx, bool uart_err, uint32_t now_ms, uint32_t lifetime_ms);

/* The positive allow-list over a 28-byte inner command (exact bytes). */
bool raw81_allowed(const uint8_t *inner);
/* Build the receipt payload (32 bytes) for a verdict; reply is the 32-byte
 * reply for status 0, NULL otherwise. */
void raw81_receipt(uint8_t out[RAW81_PAYLOAD], uint8_t seq, uint8_t status, bool have_last,
                   uint8_t last_seq, const uint8_t *reply);
/* Wrap a 32-byte payload as a 91 frame (34 bytes). */
void raw81_frame91(uint8_t out[RAW81_FRAME_LEN], const uint8_t payload[RAW81_PAYLOAD]);
