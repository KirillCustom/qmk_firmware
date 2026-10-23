// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The lock-LED channel's receiver. The wire format is specified in
 * plans/LOCK-LED-CHANNEL-PLAN.md ("The wire format"); scripts/lockchan.py is
 * the reference encoder, and scripts/lockchan_test.py checks the two agree.
 * The idea, lock lights toggled as data bits, is quill4gen7's, proved in the
 * notify branch of github.com/quill4gen7/ak820-pro; this is a separate
 * implementation. */
#include "lockchan.h"
#include <string.h>

#define CAPS_BIT   0x02u
#define DATA_BITS  0xFDu   /* every bit any profile uses: all but Caps */
#define PRE_HIGH   0x18u   /* the P1/P2 preamble flips bits 3-4 together */
#define PRE_PAD    0xE0u   /* the P3 preamble flips the padding bits 5-7 together */
#define PRE_LOW    0x05u   /* the P0 preamble flips Num and Scroll together */

enum { ST_IDLE, ST_PRE, ST_HDR, ST_PAY, ST_TRL };

typedef struct {
    uint8_t mask;          /* the data bits, OR'd */
    uint8_t bits[5];       /* the data bits, digit-subset order */
    uint8_t nbits;         /* how many */
    uint8_t base;          /* 2^nbits - 1: a transition is a non-empty subset */
    uint8_t block_bits;    /* bits per full block, */
    uint8_t block_digits;  /* in this many digits */
    uint8_t trl_a, trl_b;  /* the trailer's middle step flips one of these */
} profile_t;

/* base^block_digits >= 2^block_bits, and fits a uint32_t (15^8 and 31^6 < 2^32). */
static const profile_t profiles[LOCKCHAN_NPROFILES] = {
    [LOCKCHAN_P0] = {0x05, {0x01, 0x04}, 2, 3, 19, 12, 0x01, 0x04},
    [LOCKCHAN_P1] = {0x19, {0x01, 0x08, 0x10}, 3, 7, 14, 5, 0x08, 0x10},
    [LOCKCHAN_P2] = {0x1D, {0x01, 0x04, 0x08, 0x10}, 4, 15, 31, 8, 0x08, 0x10},
    [LOCKCHAN_P3] = {0xF8, {0x08, 0x10, 0x20, 0x40, 0x80}, 5, 31, 29, 6, 0x08, 0x10},
};

/* Digits that carry r bits: the least n with base^n >= 2^r (r <= block_bits). */
static uint8_t digits_for(const profile_t *p, uint8_t r) {
    uint64_t need = (uint64_t)1 << r, pw = 1;
    uint8_t  n    = 0;
    while (pw < need) { pw *= p->base; n++; }
    return n;
}

static uint8_t crc8(const uint8_t *d, uint8_t n) {   /* poly 0x07, init 0 */
    unsigned c = 0;
    while (n--) {
        c ^= *d++;
        for (uint8_t i = 0; i < 8; i++) c = ((c & 0x80u) ? (c << 1) ^ 0x07u : c << 1) & 0xFFu;
    }
    return (uint8_t)c;
}

static uint16_t crc16(uint16_t init, const uint8_t *d, uint8_t n) {   /* CCITT-FALSE */
    unsigned c = init;
    while (n--) {
        c ^= (unsigned)*d++ << 8;
        for (uint8_t i = 0; i < 8; i++) c = ((c & 0x8000u) ? (c << 1) ^ 0x1021u : c << 1) & 0xFFFFu;
    }
    return (uint16_t)c;
}

static void start_section(lockchan_t *lc, uint8_t profile, uint16_t bits) {
    lc->sec_profile = profile;
    lc->sec_bits    = bits;
    lc->got_bits    = 0;
    lc->acc         = 0;
    lc->acc_digits  = 0;
}

static void abort_frame(lockchan_t *lc, lockchan_abort_t why) {
    if (lc->aborts[why] != UINT16_MAX) lc->aborts[why]++;
    lc->st = ST_IDLE;
}

/* A transition's subset of the section's data bits, as a digit 0..base-1. */
static uint8_t digit_of(const profile_t *p, uint8_t x) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < p->nbits; i++)
        if (x & p->bits[i]) s |= (uint8_t)(1u << i);
    return (uint8_t)(s - 1u);
}

/* Append the low `n` bits of v, most significant first. */
static void put_bits(lockchan_t *lc, uint8_t *dst, uint32_t v, uint8_t n) {
    while (n--) {
        uint16_t at = lc->got_bits++;
        if ((v >> n) & 1u) dst[at >> 3] |= (uint8_t)(0x80u >> (at & 7u));
    }
}

/* One digit into the current section; true once the section is complete. */
static bool section_digit(lockchan_t *lc, uint8_t *dst, uint8_t d) {
    const profile_t *p = &profiles[lc->sec_profile];
    if (lc->acc_digits == 0) {   /* a new block: full, or the section's last, shorter one */
        uint16_t left  = (uint16_t)(lc->sec_bits - lc->got_bits);
        lc->blk_bits   = left < p->block_bits ? (uint8_t)left : p->block_bits;
        lc->blk_digits = lc->blk_bits == p->block_bits ? p->block_digits : digits_for(p, lc->blk_bits);
    }
    lc->acc = lc->acc * p->base + d;
    if (++lc->acc_digits < lc->blk_digits) return false;
    if (lc->acc >> lc->blk_bits) { abort_frame(lc, LOCKCHAN_ABORT_RANGE); return false; }
    put_bits(lc, dst, lc->acc, lc->blk_bits);
    lc->acc        = 0;
    lc->acc_digits = 0;
    return lc->got_bits == lc->sec_bits;
}

static void end_header(lockchan_t *lc) {
    uint8_t ver = lc->hdr[0] >> 6, prof = (lc->hdr[0] >> 4) & 3u, len = lc->hdr[2];
    if (crc8(lc->hdr, 3) != lc->hdr[3]) { abort_frame(lc, LOCKCHAN_ABORT_HEADER_CRC); return; }
    /* A bits 3-4 preamble (header in P1) may announce P1 or P2; a padding-bits
     * one (header in P3), only P3; a Num + Scroll one (header in P0), only P0. */
    bool ok = ver == 0 && len <= LOCKCHAN_MAX_PAYLOAD &&
              (lc->sec_profile == LOCKCHAN_P1 ? (prof == LOCKCHAN_P1 || prof == LOCKCHAN_P2)
                                              : prof == lc->sec_profile);
    if (!ok) { abort_frame(lc, LOCKCHAN_ABORT_HEADER); return; }
    lc->pay_profile = prof;
    memset(lc->buf, 0, sizeof(lc->buf));
    start_section(lc, prof, (uint16_t)((len + 2u) * 8u));
    lc->st = ST_PAY;
}

static void end_payload(lockchan_t *lc, uint8_t cur) {
    uint8_t  len = lc->hdr[2];
    uint16_t c   = crc16(crc16(0xFFFFu, lc->hdr, 4), lc->buf, len);
    if (c != (uint16_t)((lc->buf[len] << 8) | lc->buf[len + 1])) { abort_frame(lc, LOCKCHAN_ABORT_CRC); return; }
    /* The trailer: two transitions back to the baseline, the first through
     * baseline ^ trl_a (or trl_b, when the payload already ended there). */
    const profile_t *p  = &profiles[lc->pay_profile];
    uint8_t          b  = lc->baseline & p->mask;
    lc->trl_mid         = (uint8_t)(b ^ ((cur & p->mask) == (b ^ p->trl_a) ? p->trl_b : p->trl_a));
    lc->pre_n           = 0;
    lc->st              = ST_TRL;
}

static bool commit(lockchan_t *lc, uint32_t now) {
    lockchan_frame_t *f = &lc->last;
    f->profile          = lc->pay_profile;
    f->type             = lc->hdr[0] & 0x0Fu;
    f->seq              = lc->hdr[1];
    f->len              = lc->hdr[2];
    memcpy(f->payload, lc->buf, f->len);
    f->anchor_ms = lc->anchor_ms;
    f->commit_ms = now;
    lc->frames++;
    lc->st = ST_IDLE;
    return true;
}

/* IDLE: is this transition a preamble's first? */
static void idle_transition(lockchan_t *lc, uint8_t prev, uint8_t x, uint32_t quiet, uint32_t now) {
    if (quiet < LOCKCHAN_QUIET_MS) return;
    if ((x == PRE_HIGH || x == PRE_PAD) && (prev & x) == 0) lc->pre_mask = x;
    else if (x == PRE_LOW) lc->pre_mask = PRE_LOW;
    else return;
    lc->st             = ST_PRE;
    lc->pre_n          = 1;
    lc->baseline       = prev;
    lc->frame_start_ms = now;
}

bool lockchan_feed(lockchan_t *lc, uint8_t leds, uint32_t now) {
    if (!lc->have_prev) {
        lc->have_prev      = true;
        lc->prev           = leds;
        lc->last_change_ms = now;
        return false;
    }
    uint8_t prev = lc->prev;
    lc->prev     = leds;
    if ((prev ^ leds) & CAPS_BIT) {   /* the user's Caps, or the Mac's report of it */
        if (lc->st == ST_PRE) lc->st = ST_IDLE;
        else if (lc->st != ST_IDLE) abort_frame(lc, LOCKCHAN_ABORT_CAPS);
    }
    if (lc->st == ST_IDLE || lc->st == ST_PRE) {
        uint8_t x = (prev ^ leds) & DATA_BITS;
        if (!x) return false;
        uint32_t quiet     = now - lc->last_change_ms;
        lc->last_change_ms = now;
        if (lc->st == ST_PRE) {
            if (quiet < LOCKCHAN_QUIET_MS && x == lc->pre_mask) {
                if (++lc->pre_n == 3) {
                    lc->preambles++;
                    lc->anchor_ms = now;
                    lc->st        = ST_HDR;
                    memset(lc->hdr, 0, sizeof(lc->hdr));
                    start_section(lc, lc->pre_mask == PRE_HIGH  ? LOCKCHAN_P1
                                      : lc->pre_mask == PRE_PAD ? LOCKCHAN_P3
                                                                : LOCKCHAN_P0, 32);
                }
                return false;
            }
            lc->st = ST_IDLE;   /* not a preamble after all: never synced, not an abort */
        }
        idle_transition(lc, prev, x, quiet, now);
        return false;
    }
    const profile_t *p  = &profiles[lc->sec_profile];
    uint8_t          xd = (prev ^ leds) & DATA_BITS;
    if (!xd) return false;   /* a repeat, or only Caps */
    /* Stalled: any data-capable bit ends it, not only this section's, or a
     * stuck Num + Scroll section would ignore the next frame's bits 3-4
     * preamble until poll() timed it out. */
    uint32_t gap = now - lc->last_change_ms;
    if (gap > LOCKCHAN_GAP_MS) {
        abort_frame(lc, LOCKCHAN_ABORT_GAP);
        lc->last_change_ms = now;
        idle_transition(lc, prev, xd, gap, now);
        return false;
    }
    uint8_t x = xd & p->mask;
    if (!x) return false;   /* bits this section does not use: Scroll under P1, Num and Scroll under P3 */
    lc->last_change_ms = now;
    if (now - lc->frame_start_ms > LOCKCHAN_LIFETIME_MS) { abort_frame(lc, LOCKCHAN_ABORT_LIFETIME); return false; }
    switch (lc->st) {
        case ST_HDR:
            if (section_digit(lc, lc->hdr, digit_of(p, x))) end_header(lc);
            return false;
        case ST_PAY:
            if (section_digit(lc, lc->buf, digit_of(p, x))) end_payload(lc, leds);
            return false;
        case ST_TRL:   /* pre_n counts the trailer's transitions here */
            if (lc->pre_n++ == 0) {
                if ((leds & p->mask) != lc->trl_mid) abort_frame(lc, LOCKCHAN_ABORT_TRAILER);
                return false;
            }
            if ((leds & p->mask) != (lc->baseline & p->mask)) { abort_frame(lc, LOCKCHAN_ABORT_TRAILER); return false; }
            return commit(lc, now);
    }
    return false;
}

void lockchan_link_lost(lockchan_t *lc) {
    if (lc->st != ST_IDLE) abort_frame(lc, LOCKCHAN_ABORT_RESET);
    lc->have_prev = false;   /* the link-down zeroes the LED state */
}

void lockchan_poll(lockchan_t *lc, uint32_t now, bool linked) {
    if (!linked) {
        lockchan_link_lost(lc);
        return;
    }
    if (lc->st == ST_IDLE) return;
    if (lc->st == ST_PRE) {   /* the next transition would start a new one */
        if (now - lc->last_change_ms >= LOCKCHAN_QUIET_MS) lc->st = ST_IDLE;
        return;
    }
    if (now - lc->last_change_ms > LOCKCHAN_GAP_MS) {
        abort_frame(lc, LOCKCHAN_ABORT_GAP);
    } else if (now - lc->frame_start_ms > LOCKCHAN_LIFETIME_MS) {
        abort_frame(lc, LOCKCHAN_ABORT_LIFETIME);
    }
}

bool lockchan_busy(const lockchan_t *lc) {
    return lc->st != ST_IDLE;
}

bool lockchan_holds_scroll(const lockchan_t *lc) {
    switch (lc->st) {
        case ST_IDLE: return false;
        case ST_PRE:
        case ST_HDR: return lc->pre_mask != PRE_PAD;   /* P0, or P1/P2 until the header says which */
        default: return lc->pay_profile == LOCKCHAN_P0 || lc->pay_profile == LOCKCHAN_P2;
    }
}

void lockchan_init(lockchan_t *lc) {
    memset(lc, 0, sizeof(*lc));
}

void lockchan_clear_stats(lockchan_t *lc) {
    memset(&lc->last, 0, sizeof(lc->last));
    lc->frames = lc->preambles = 0;
    memset(lc->aborts, 0, sizeof(lc->aborts));
}
