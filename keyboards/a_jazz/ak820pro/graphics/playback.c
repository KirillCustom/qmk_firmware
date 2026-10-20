// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#include "graphics/playback.h"

/* From elapsed time, not a count of serviced ticks, so a stretch without
 * housekeeping (Fn+D, an animation) costs nothing. Unsigned wrap keeps
 * `now - from_ms` right across the timer's rollover. */
static uint16_t advance(uint16_t from, uint32_t from_ms, uint32_t now, uint16_t dur) {
    uint32_t pos = from + (now - from_ms) / 1000u;
    if (dur && pos > dur) pos = dur;
    return pos > 0xFFFFu ? 0xFFFFu : (uint16_t)pos;
}

static void guess_deadline(playback_t *p, uint32_t now) {
    if (p->guess != PLAYBACK_GUESS_NONE && now - p->guess_ms > PLAYBACK_KEY_GUESS_MS) p->guess = PLAYBACK_GUESS_NONE;
}

bool playback_set(playback_t *p, uint8_t state, uint16_t pos, uint16_t dur, uint32_t timeout_ms, uint32_t now) {
    bool want    = (state != 0);
    bool changed = (want != p->active);
    p->active    = want;
    p->ticking   = want;   /* the host only reports a position while PLAYING */
    p->guess     = PLAYBACK_GUESS_NONE;
    p->base      = pos;
    p->dur       = dur;
    p->base_ms   = now;
    p->stamp     = now;
    p->timeout   = timeout_ms;
    return changed;
}

bool playback_off(playback_t *p) {
    bool was  = p->active;
    p->active = false;
    p->guess  = PLAYBACK_GUESS_NONE;
    return was;
}

void playback_key(playback_t *p, uint32_t now) {
    if (!p->active) return;
    guess_deadline(p, now);
    uint16_t shown = playback_shown(p, now);
    p->guess       = (p->guess == PLAYBACK_GUESS_FROZEN) ? PLAYBACK_GUESS_RESUMED : PLAYBACK_GUESS_FROZEN;
    p->guess_pos   = shown;
    p->guess_ms    = now;
}

bool playback_service(playback_t *p, uint32_t now) {
    if (!p->active) return false;
    if (now - p->stamp > p->timeout) {
        playback_off(p);
        return true;
    }
    guess_deadline(p, now);
    return false;
}

uint16_t playback_shown(const playback_t *p, uint32_t now) {
    switch (p->guess) {
        case PLAYBACK_GUESS_FROZEN:  return p->guess_pos;
        case PLAYBACK_GUESS_RESUMED: return advance(p->guess_pos, p->guess_ms, now, p->dur);
        default:                     return p->ticking ? advance(p->base, p->base_ms, now, p->dur) : advance(p->base, now, now, p->dur);
    }
}

bool playback_ticking(const playback_t *p) {
    return p->ticking && p->guess != PLAYBACK_GUESS_FROZEN;
}
