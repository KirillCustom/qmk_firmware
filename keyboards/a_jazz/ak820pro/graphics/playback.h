// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

/* The playback readout's state, apart from its painting (display.c owns one
 * and draws it in place of the clock). Pure: every call takes `now`, so the
 * host test drives it (scripts/playback_test.c).
 *
 * Two things set what it shows, and neither overwrites the other:
 * - **the host's anchor** (base at base_ms, advancing while playing), set
 *   only by a host's word (raw HID or the lock-LED channel's type 3);
 * - **a media key's guess** (frozen, or resumed from where it froze), which
 *   lasts PLAYBACK_KEY_GUESS_MS from the last press and then gives the
 *   anchor back. A key never moves the anchor, so a guess the player ignored
 *   cannot leave the readout behind for good (codex, the play-times verify,
 *   P2 2). */

#include <stdbool.h>
#include <stdint.h>

/* A media key's guess, at most: raw HID's poll corrects it within seconds,
 * but the lock lights send nothing when the player ignored the key. */
#define PLAYBACK_KEY_GUESS_MS 15000u

enum { PLAYBACK_GUESS_NONE = 0, PLAYBACK_GUESS_FROZEN, PLAYBACK_GUESS_RESUMED };

typedef struct {
    bool     active;     /* the band is the readout's */
    bool     ticking;    /* the host's word: advancing */
    uint8_t  guess;      /* PLAYBACK_GUESS_* */
    uint16_t base;       /* the host's position at base_ms */
    uint16_t dur;        /* 0: unknown (a stream) */
    uint16_t guess_pos;  /* frozen there, or resumed from there at guess_ms */
    uint32_t base_ms;
    uint32_t stamp;      /* the last host word, for the expiry */
    uint32_t timeout;    /* the last setter's */
    uint32_t guess_ms;   /* the last press */
} playback_t;

/* A host's word; state 0 hands the band back. Ends any guess. True when the
 * band changes owner. */
bool playback_set(playback_t *p, uint8_t state, uint16_t pos, uint16_t dur, uint32_t timeout_ms, uint32_t now);
/* Off, as at a deep sleep's wake. True when it was on. */
bool playback_off(playback_t *p);
/* The media key: freeze what is shown, or resume from where it froze. A
 * guess past its deadline is dropped first, so this press acts on the
 * anchor. */
void playback_key(playback_t *p, uint32_t now);
/* The expiry and the guess's deadline. True when the readout expired. */
bool playback_service(playback_t *p, uint32_t now);
/* What the band shows at `now`: whole seconds, never past a known duration
 * or 0xFFFF. */
uint16_t playback_shown(const playback_t *p, uint32_t now);
/* Ticking as shown: the host's word, unless a key froze it. */
bool playback_ticking(const playback_t *p);
