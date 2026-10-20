// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* The PCF8563's write bookkeeping (rtc.c's phase-correct writer), pure so the
 * host can test it (scripts/rtc_pcf_sync_test.c). Codex, the wireless clock's
 * design review, 10-20:
 *
 * - must-fix 4: a write planned for an older request must not consume a newer
 *   one. Each request bumps req_gen; a run takes run_gen at its start, and
 *   only a run of the latest generation clears `pending`.
 * - must-fix 5: a failed write must not leave the PCF authoritative. `stale`
 *   is set on every accepted host correction and cleared only by a write of
 *   the latest generation; while stale, nothing takes the PCF's calendar (the
 *   discipline's snap, the acquisition, a wake's coarse step). A run that gives
 *   up clears `pending` (a dead PCF must not hold the board awake: a sleep
 *   waits on `pending`) and retries after PCF_SYNC_RETRY_S.
 *
 * Seconds are rtc.c's rtc_seconds_count (monotonic since boot, wraps).
 */

#define PCF_SYNC_RETRY_S 60u

typedef struct {
    uint16_t req_gen;    /* bumped by every request */
    uint16_t run_gen;    /* the request generation the running write carries */
    bool     pending;    /* a write is wanted */
    bool     stale;      /* the PCF may not hold the latest accepted host time */
    uint32_t retry_at;   /* a stale PCF's next request, after a give-up */
} pcf_sync_t;

/* A write is wanted (a host's set, or the board's). */
static inline void pcf_sync_request(pcf_sync_t *s) {
    s->pending = true;
    s->req_gen++;
}

/* A host's correction was accepted: the PCF no longer holds the time. */
static inline void pcf_sync_host_accepted(pcf_sync_t *s) {
    s->stale = true;
}

/* The writer leaves idle: what it writes answers this generation. */
static inline void pcf_sync_run_start(pcf_sync_t *s) {
    s->run_gen = s->req_gen;
}

/* The write verified. Clears `pending` (and `stale`) only if no newer request
 * came meanwhile; returns whether another run is still wanted. */
static inline bool pcf_sync_run_ok(pcf_sync_t *s) {
    if (s->run_gen == s->req_gen) {
        s->pending = false;
        s->stale   = false;
    }
    return s->pending;
}

/* The writer gave up (or its STOP recovery ended): not pending, so the board
 * may sleep, but a stale PCF is asked for again later. */
static inline void pcf_sync_gave_up(pcf_sync_t *s, uint32_t now_s) {
    s->pending  = false;
    s->retry_at = now_s + PCF_SYNC_RETRY_S;
}

/* A synchronous write (the legacy set) landed: it holds the latest time. A
 * phase-correct run still pending writes from the board's own clock, which is
 * that time too. */
static inline void pcf_sync_written(pcf_sync_t *s) {
    s->stale = false;
}

/* Is a stale, idle PCF due a fresh request? */
static inline bool pcf_sync_retry_due(const pcf_sync_t *s, uint32_t now_s) {
    return s->stale && !s->pending && (int32_t)(now_s - s->retry_at) >= 0;
}
