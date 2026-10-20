// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later

/* Raw HID over the 2.4G dongle (plans/RAWHID-24G-PLAN.md, stage 2 revision
 * 3): the module-side collector, the request envelope, sessions and
 * freshness, the allow-list and the receipt, as pure functions of bytes,
 * times and flags. No hardware calls, so scripts/raw81_test.py compiles this
 * file on the host and drives every split, deletion and truncation through
 * it; ch582f_ajazz.c feeds it from the UART and acts on its verdicts.
 *
 * On the wire, from the module: 81 <32 bytes> <sum = (0x81 + the 32) & 0xFF>.
 * The 32 bytes are the host's request, envelope v2:
 *   [0] 0xC6  [1] session (nonzero)  [2..3] seq, big-endian
 *   [4..5] CRC-16/CCITT-FALSE, big-endian, over [0..3] and [6..31]
 *   [6] mask  [7..31] the inner command's bytes 1..25, XOR mask
 * The inner's byte 0 is always 0x07 and is not sent. The host picks the
 * mask so no byte of the frame is 0x5A/0x5B/0x5C/0x61 (an older board's
 * window cannot act on it; plans/RAWHID-24G-DAEMON-PLAN.md, R3-1).
 * To the module, per transaction: the reply (91 <32> <sum>, status 0 only),
 * then the receipt (91 <32> <sum>):
 *   [0] 0xC7  [1] session  [2..3] seq  [4] status  [5..6] the last admitted
 *   seq (0 if none)  [7..8] CRC-16 of the reply (status 0)
 *   [9..10] CRC-16 of [0..8]  [11..31] 0 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RAW81_FRAME_LEN 34u   /* 81, 32 bytes, sum */
#define RAW81_PAYLOAD   32u
#define RAW81_HEAD      6u    /* magic, session, seq x2, CRC x2 */
#define RAW81_INNER     26u
#define RAW81_MAGIC     0xC7u   /* the receipt's */
#define RAW81_MAGIC_REQ 0xC6u   /* a request's, envelope v2 (masked) */
/* A session changes only after this long with no admitted request: no packet
 * the radio delayed can outlive it (the dongle forwards within milliseconds
 * and stores nothing), and two owners cannot interleave. */
/* The session's idle is the caller's: an elapsed-time count that no clock
 * adjustment moves (ch582f_ajazz.c extends the 16-bit system tick; the
 * RTC's seconds count is NOT elapsed time: a phase set or a wake correction
 * moves it, codex's latency review). The plan's minimum is 1 s. */

typedef enum { RAW81_IDLE = 0, RAW81_COLLECT = 1, RAW81_QUARANTINE = 2 } raw81_state_t;

/* What the caller does with the byte it just fed. */
typedef enum {
    RAW81_TO_WINDOW = 0,   /* not ours: the 3-byte window takes it */
    RAW81_TAKEN     = 1,   /* consumed (collecting or quarantined) */
    RAW81_STARTED   = 2,   /* consumed as an 81 header: clear the window */
    RAW81_FRAME     = 3,   /* consumed, and 34 bytes are in: call raw81_commit() */
} raw81_feed_t;

/* raw81_commit()'s verdict. OK leaves freshness unchanged: the caller calls
 * raw81_admit() only if it actually dispatches the request. */
typedef enum {
    RAW81_V_OK        = 0,   /* fresh and allowed: inner[] holds it */
    RAW81_V_STALE     = 1,   /* well formed, seq not fresh: status 3 */
    RAW81_V_FORBIDDEN = 2,   /* well formed, not on the allow-list: status 2 */
    RAW81_V_BADENV    = 3,   /* outer sum good, magic or CRC bad: quarantined; status 1 */
    RAW81_V_BADSUM    = 4,   /* quarantined, no answer */
    RAW81_V_TAINT     = 5,   /* a UART error under the frame: quarantined, no answer */
    RAW81_V_LIFETIME  = 6,   /* older than its lifetime at commit: quarantined, no answer */
    RAW81_V_BUSY      = 7,   /* another session inside the idle time: status 4 */
} raw81_verdict_t;

/* Receipt status bytes. */
enum {
    RAW81_ST_OK = 0, RAW81_ST_BADCRC = 1, RAW81_ST_FORBIDDEN = 2, RAW81_ST_STALE = 3,
    RAW81_ST_BUSY = 4,
};

typedef struct {
    raw81_state_t state;
    bool          boundary;    /* the next byte may start an 81 */
    bool          tainted;     /* a UART error since the line was last silent */
    uint8_t       buf[RAW81_FRAME_LEN];
    uint8_t       len;
    uint16_t      start_tick;  /* when the collection began, in the caller's 16-bit ticks */
    uint16_t      last_tick;   /* the last byte, in the caller's 16-bit ticks */
    /* the session: none after a reset or a link change */
    bool          have_session;
    uint8_t       session;
    uint16_t      last_seq;    /* the last ADMITTED seq in the session */
    uint32_t      admitted_at; /* when it was admitted, in the caller's long count */
    /* the frame just committed (every verdict but BADSUM, TAINT, LIFETIME) */
    uint8_t       c_session;
    uint16_t      c_seq;
    uint8_t       inner[RAW81_INNER];
    /* counters */
    uint32_t ok, stale, forbidden, badenv, badsum, taint, lifetime, qbytes, busy;
} raw81_rx_t;

uint16_t raw81_crc16(const uint8_t *p, uint8_t n, uint16_t crc);

void raw81_init(raw81_rx_t *rx);
/* A resync or a wake: nothing half-collected survives; the session ends. */
void raw81_reset(raw81_rx_t *rx);
/* The link moved: the session ends; a collection in progress is
 * quarantined, so its tail cannot reach the window. */
void raw81_link_lost(raw81_rx_t *rx);

/* At a drain's start: latch a UART error (any state) and abort a collection
 * that saw one or outlived lifetime_ticks. Ticks, not ms: the caller's
 * millisecond clock is a software 64-bit divide on the M0, and this runs every
 * pass; the 16-bit tick's wrap (~0.35 s) is far past any lifetime here. */
void raw81_poll(raw81_rx_t *rx, bool uart_err, uint16_t now_tick, uint16_t lifetime_ticks);
/* One byte from the module, before the window sees it. */
raw81_feed_t raw81_byte(raw81_rx_t *rx, uint8_t c, uint16_t now_tick);
/* After the window processed a byte raw81_byte() returned TO_WINDOW for:
 * a boundary only right after a frame or ACK it completed. */
void raw81_window_done(raw81_rx_t *rx, bool matched);
/* The input queue is empty: true when that ended a quarantine (the caller
 * clears the window), after silence_ticks of no bytes. */
bool raw81_empty(raw81_rx_t *rx, uint16_t now_tick, uint16_t silence_ticks);
/* 34 bytes are in: check the taint, the lifetime, the sum, the envelope, the
 * session, the freshness and the allow-list. */
/* now_long and idle_long are the caller's elapsed-time count, used only for
 * the session's idle: another session waits until idle_long has passed since
 * the last admission. */
raw81_verdict_t raw81_commit(raw81_rx_t *rx, bool uart_err, uint16_t now_tick, uint16_t lifetime_ticks,
                             uint32_t now_long, uint32_t idle_long);
/* The caller dispatched a request: its identity (the one the caller reserved
 * at commit) becomes the session's last admitted seq, starting the session if
 * it was new. BUSY never refreshes the idle time; only this does. */
void raw81_admit(raw81_rx_t *rx, uint8_t session, uint16_t seq, uint32_t now_long);

/* The positive allow-list over a 26-byte inner command (exact bytes). */
bool raw81_allowed(const uint8_t *inner);
/* Build the receipt payload (32 bytes); reply is the 32-byte reply for
 * status 0, NULL otherwise. */
void raw81_receipt(uint8_t out[RAW81_PAYLOAD], uint8_t session, uint16_t seq, uint8_t status,
                   uint16_t last_seq, const uint8_t *reply);
/* Wrap a 32-byte payload as a 91 frame (34 bytes). */
void raw81_frame91(uint8_t out[RAW81_FRAME_LEN], const uint8_t payload[RAW81_PAYLOAD]);
