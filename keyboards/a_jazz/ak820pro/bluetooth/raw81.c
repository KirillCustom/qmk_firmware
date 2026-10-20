// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later

/* Raw HID over the 2.4G dongle: the pure part (raw81.h). Every rule here is
 * from plans/RAWHID-24G-PLAN.md, stage 2 revision 2, and the codex reviews
 * it cites; scripts/raw81_test.py exercises each one on the host. */
#include <string.h>
#include "raw81.h"

uint16_t raw81_crc16(const uint8_t *p, uint8_t n, uint16_t crc) {
    /* CRC-16/CCITT-FALSE: poly 0x1021, no reflection; init by the caller
     * (0xFFFF). Bitwise: 28-32 bytes a request, not worth a table. */
    for (uint8_t i = 0; i < n; i++) {
        crc ^= (uint16_t)(p[i] << 8);
        for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

void raw81_init(raw81_rx_t *rx) {
    memset(rx, 0, sizeof(*rx));
    rx->boundary = true;
}

void raw81_reset(raw81_rx_t *rx) {
    rx->state     = RAW81_IDLE;
    rx->boundary  = true;
    rx->tainted   = false;
    rx->have_last = false;
}

void raw81_link_lost(raw81_rx_t *rx) {
    rx->have_last = false;
    if (rx->state == RAW81_COLLECT) {
        rx->state    = RAW81_QUARANTINE;
        rx->boundary = false;
    }
}

static void abort_to(raw81_rx_t *rx, uint32_t *why) {
    (*why)++;
    rx->state    = RAW81_QUARANTINE;
    rx->boundary = false;
}

void raw81_poll(raw81_rx_t *rx, bool uart_err, uint32_t now_ms, uint32_t lifetime_ms) {
    if (uart_err) rx->tainted = true;   /* remembered until the line is silent */
    if (rx->state != RAW81_COLLECT) return;
    if (uart_err) abort_to(rx, &rx->taint);
    else if (now_ms - rx->start_ms >= lifetime_ms) abort_to(rx, &rx->lifetime);
}

raw81_feed_t raw81_byte(raw81_rx_t *rx, uint8_t c, uint16_t now_tick, uint32_t now_ms) {
    rx->last_tick = now_tick;
    if (rx->state == RAW81_QUARANTINE) {
        rx->qbytes++;
        return RAW81_TAKEN;
    }
    if (rx->state == RAW81_COLLECT) {
        rx->buf[rx->len++] = c;
        return rx->len == RAW81_FRAME_LEN ? RAW81_FRAME : RAW81_TAKEN;
    }
    if (rx->boundary && c == 0x81) {
        /* Decided before the byte is consumed: `5C 25 | 81` never gets here,
         * since the 5C left the boundary false. The window starts over (a
         * pending `61 0D` must not meet a `0A` after the frame). */
        rx->buf[0]   = c;
        rx->len      = 1;
        rx->start_ms = now_ms;
        if (rx->tainted) abort_to(rx, &rx->taint);   /* a burst already known corrupt */
        else rx->state = RAW81_COLLECT;
        return RAW81_STARTED;
    }
    return RAW81_TO_WINDOW;
}

void raw81_window_done(raw81_rx_t *rx, bool matched) {
    rx->boundary = matched;
}

bool raw81_empty(raw81_rx_t *rx, uint16_t now_tick, uint16_t silence_ticks) {
    /* A collection is judged only by its lifetime, never by silence. A stale
     * stamp (older than the tick's wrap) only under-measures the silence. */
    if (rx->state == RAW81_COLLECT) return false;
    if ((uint16_t)(now_tick - rx->last_tick) < silence_ticks) return false;
    bool ended    = rx->state == RAW81_QUARANTINE;
    rx->state     = RAW81_IDLE;
    rx->boundary  = true;
    rx->tainted   = false;
    return ended;
}

raw81_verdict_t raw81_commit(raw81_rx_t *rx, bool uart_err, uint32_t now_ms, uint32_t lifetime_ms) {
    if (uart_err) rx->tainted = true;
    if (rx->tainted) {
        abort_to(rx, &rx->taint);
        return RAW81_V_TAINT;
    }
    if (now_ms - rx->start_ms >= lifetime_ms) {
        abort_to(rx, &rx->lifetime);
        return RAW81_V_LIFETIME;
    }
    uint8_t sum = 0;
    for (uint8_t i = 0; i < RAW81_FRAME_LEN - 1u; i++) sum += rx->buf[i];
    if (sum != rx->buf[RAW81_FRAME_LEN - 1u]) {
        abort_to(rx, &rx->badsum);
        return RAW81_V_BADSUM;
    }
    const uint8_t *p = &rx->buf[1];
    rx->seq          = p[1];
    uint16_t crc     = raw81_crc16(p, 2, 0xFFFFu);
    crc              = raw81_crc16(&p[4], RAW81_INNER, crc);
    if (p[0] != RAW81_MAGIC || crc != (uint16_t)((p[2] << 8) | p[3])) {
        /* A good outer sum over a bad envelope is corruption the sum missed
         * (a deletion, a truncation): no trusted boundary, quarantine. */
        abort_to(rx, &rx->badenv);
        return RAW81_V_BADENV;
    }
    rx->state    = RAW81_IDLE;
    rx->boundary = true;
    if (rx->have_last) {
        uint8_t d = (uint8_t)(rx->seq - rx->last_seq);
        if (d == 0 || d > 127u) {
            rx->stale++;
            return RAW81_V_STALE;
        }
    }
    rx->have_last = true;
    rx->last_seq  = rx->seq;
    if (!raw81_allowed(&p[4])) {
        rx->forbidden++;
        return RAW81_V_FORBIDDEN;
    }
    memcpy(rx->inner, &p[4], RAW81_INNER);
    rx->ok++;
    return RAW81_V_OK;
}

bool raw81_allowed(const uint8_t *in) {
    if (in[0] != 0x07) return false;          /* id_custom_set_value only */
    if (in[1] == 0x12) {                       /* text: set, clear, set line, playback */
        return in[2] >= 0x01 && in[2] <= 0x04;
    }
    if (in[1] == 0x13) {                       /* health: the pure reads */
        switch (in[2]) {
            case 0x01: /* HC_GET */
            case 0x02: /* HC_CONN */
            case 0x04: /* HC_GET2 */
            case 0x06: /* HC_GET3 */
            case 0x07: /* HC_GET4 */
                return true;
            case 0x0E: /* HC_LINK: page 0 and the profile pages 0x21-0x2A */
                return in[3] == 0x00 || (in[3] >= 0x21 && in[3] <= 0x2A);
            default:
                return false;
        }
    }
    return false;
}

void raw81_receipt(uint8_t out[RAW81_PAYLOAD], uint8_t seq, uint8_t status, bool have_last,
                   uint8_t last_seq, const uint8_t *reply) {
    memset(out, 0, RAW81_PAYLOAD);
    out[0] = RAW81_MAGIC;
    out[1] = seq;
    out[2] = status;
    out[3] = have_last ? last_seq : 0;
    if (status == RAW81_ST_OK && reply) {
        uint16_t c = raw81_crc16(reply, RAW81_PAYLOAD, 0xFFFFu);
        out[4]     = (uint8_t)(c >> 8);
        out[5]     = (uint8_t)c;
    }
    out[6] = have_last ? 1 : 0;
}

void raw81_frame91(uint8_t out[RAW81_FRAME_LEN], const uint8_t payload[RAW81_PAYLOAD]) {
    uint8_t s = 0x91;
    out[0]    = 0x91;
    for (uint8_t i = 0; i < RAW81_PAYLOAD; i++) {
        out[1 + i] = payload[i];
        s += payload[i];
    }
    out[RAW81_FRAME_LEN - 1u] = s;
}
