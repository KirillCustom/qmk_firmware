// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later

/* Raw HID over the 2.4G dongle: the pure part (raw81.h). Every rule here is
 * from plans/RAWHID-24G-PLAN.md, stage 2 revision 2, and the codex reviews
 * it cites; scripts/raw81_test.py exercises each one on the host. */
#include <string.h>
#include "raw81.h"

/* CRC-16/CCITT-FALSE: poly 0x1021, no reflection; init by the caller
 * (0xFFFF). Table-driven (512 bytes of flash): a request and its response
 * take four CRCs over ~100 bytes, and bitwise that was a measurable share of
 * the pass on a CPU the row ISR leaves 27% of. */
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

uint16_t raw81_crc16(const uint8_t *p, uint8_t n, uint16_t crc) {
    for (uint8_t i = 0; i < n; i++) crc = (uint16_t)((crc << 8) ^ crc_table[((crc >> 8) ^ p[i]) & 0xFFu]);
    return crc;
}

void raw81_init(raw81_rx_t *rx) {
    memset(rx, 0, sizeof(*rx));
    rx->boundary = true;
}

void raw81_reset(raw81_rx_t *rx) {
    rx->state        = RAW81_IDLE;
    rx->boundary     = true;
    rx->tainted      = false;
    rx->have_session = false;
}

void raw81_link_lost(raw81_rx_t *rx) {
    rx->have_session = false;
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

void raw81_poll(raw81_rx_t *rx, bool uart_err, uint16_t now_tick, uint16_t lifetime_ticks) {
    if (uart_err) rx->tainted = true;   /* remembered until the line is silent */
    if (rx->state != RAW81_COLLECT) return;
    if (uart_err) abort_to(rx, &rx->taint);
    else if ((uint16_t)(now_tick - rx->start_tick) >= lifetime_ticks) abort_to(rx, &rx->lifetime);
}

raw81_feed_t raw81_byte(raw81_rx_t *rx, uint8_t c, uint16_t now_tick) {
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
        rx->start_tick = now_tick;
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

raw81_verdict_t raw81_commit(raw81_rx_t *rx, bool uart_err, uint16_t now_tick, uint16_t lifetime_ticks,
                             uint32_t now_s) {
    if (uart_err) rx->tainted = true;
    if (rx->tainted) {
        abort_to(rx, &rx->taint);
        return RAW81_V_TAINT;
    }
    if ((uint16_t)(now_tick - rx->start_tick) >= lifetime_ticks) {
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
    rx->c_session    = p[1];
    rx->c_seq        = (uint16_t)((p[2] << 8) | p[3]);
    uint16_t crc     = raw81_crc16(p, 4, 0xFFFFu);
    crc              = raw81_crc16(&p[RAW81_HEAD], RAW81_INNER, crc);
    if (p[0] != RAW81_MAGIC || p[1] == 0 || crc != (uint16_t)((p[4] << 8) | p[5])) {
        /* A good outer sum over a bad envelope is corruption the sum missed
         * (a deletion, a truncation): no trusted boundary, quarantine. */
        abort_to(rx, &rx->badenv);
        return RAW81_V_BADENV;
    }
    rx->state    = RAW81_IDLE;
    rx->boundary = true;
    if (rx->have_session && rx->c_session == rx->session) {
        /* Fresh: 1..32767 ahead of the last admitted seq, mod 2^16. */
        uint16_t d = (uint16_t)(rx->c_seq - rx->last_seq);
        if (d == 0 || d > 32767u) {
            rx->stale++;
            return RAW81_V_STALE;
        }
    } else if (rx->have_session && now_s - rx->admitted_s < RAW81_SESSION_IDLE_S) {
        /* Another session while this one is live: two owners, or a packet
         * from before a host restart. Neither may interleave. */
        rx->busy++;
        return RAW81_V_BUSY;
    }
    /* Otherwise a new session may start here (no session, or this one idle
     * for RAW81_SESSION_IDLE_S): admitting it is the caller's raw81_admit(). */
    if (!raw81_allowed(&p[RAW81_HEAD])) {
        rx->forbidden++;
        return RAW81_V_FORBIDDEN;
    }
    memcpy(rx->inner, &p[RAW81_HEAD], RAW81_INNER);
    rx->ok++;
    return RAW81_V_OK;
}

/* The identity is the reserved request's (copied at commit into the caller's
 * request slot), never c_session/c_seq: a later frame may overwrite those
 * before the deferred dispatch (codex, revision 3's verify, must-fix 2). */
void raw81_admit(raw81_rx_t *rx, uint8_t session, uint16_t seq, uint32_t now_s) {
    rx->have_session = true;
    rx->session      = session;
    rx->last_seq     = seq;
    rx->admitted_s   = now_s;
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

void raw81_receipt(uint8_t out[RAW81_PAYLOAD], uint8_t session, uint16_t seq, uint8_t status,
                   uint16_t last_seq, const uint8_t *reply) {
    memset(out, 0, RAW81_PAYLOAD);
    out[0] = RAW81_MAGIC;
    out[1] = session;
    out[2] = (uint8_t)(seq >> 8);
    out[3] = (uint8_t)seq;
    out[4] = status;
    out[5] = (uint8_t)(last_seq >> 8);
    out[6] = (uint8_t)last_seq;
    if (status == RAW81_ST_OK && reply) {
        uint16_t c = raw81_crc16(reply, RAW81_PAYLOAD, 0xFFFFu);
        out[7]     = (uint8_t)(c >> 8);
        out[8]     = (uint8_t)c;
    }
    /* The receipt's own CRC: the reply's CRC does not cover the identity and
     * status a host trusts (codex, revision 2's verify, must-fix 2). */
    uint16_t c = raw81_crc16(out, 9, 0xFFFFu);
    out[9]     = (uint8_t)(c >> 8);
    out[10]    = (uint8_t)c;
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
