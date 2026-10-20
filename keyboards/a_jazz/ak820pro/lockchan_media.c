// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#include "lockchan_media.h"

static bool printable(const uint8_t *s, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        if (s[i] < 0x20 || s[i] > 0x7E) return false;
    }
    return true;
}

bool lockchan_media_parse(const lockchan_frame_t *f, lockchan_media_t *m) {
    if (f->type == LOCKCHAN_TYPE_CLEAR) {
        if (f->len != 0) return false;
        *m = (lockchan_media_t){.type = LOCKCHAN_TYPE_CLEAR};
        return true;
    }
    if (f->type == LOCKCHAN_TYPE_TRANSPORT) {
        const uint8_t *p = f->payload;
        if (f->len != LOCKCHAN_TRANSPORT_LEN || p[0] > 3 || p[1] > 1) return false;
        *m = (lockchan_media_t){
            .type  = LOCKCHAN_TYPE_TRANSPORT,
            .icon  = p[0],
            .state = p[1],
            .pos   = (uint16_t)((p[2] << 8) | p[3]),
            .dur   = (uint16_t)((p[4] << 8) | p[5]),
        };
        return true;
    }
    if (f->type != LOCKCHAN_TYPE_NOW_PLAYING) return false;
    if (f->len < 2 || f->len > 2 + LOCKCHAN_MEDIA_MAX_L0 + LOCKCHAN_MEDIA_MAX_L1) return false;
    uint8_t icon = f->payload[0], n0 = f->payload[1];
    if (icon > 3) return false;                         /* any bit beyond 0-3 */
    if (n0 > LOCKCHAN_MEDIA_MAX_L0 || n0 > f->len - 2) return false;
    uint8_t n1 = (uint8_t)(f->len - 2 - n0);
    if (n1 > LOCKCHAN_MEDIA_MAX_L1) return false;
    if (!printable(&f->payload[2], (uint8_t)(n0 + n1))) return false;
    *m = (lockchan_media_t){
        .type = LOCKCHAN_TYPE_NOW_PLAYING,
        .icon = icon,
        .n0   = n0,
        .n1   = n1,
        .l0   = &f->payload[2],
        .l1   = &f->payload[2 + n0],
    };
    return true;
}

uint16_t lockchan_media_position(const lockchan_media_t *m, uint32_t anchor_ms, uint32_t commit_ms) {
    uint32_t pos = m->pos + (uint32_t)(commit_ms - anchor_ms + 500u) / 1000u;   /* wraps with the timer */
    if (m->dur && pos > m->dur) pos = m->dur;
    return pos > 0xFFFFu ? 0xFFFFu : (uint16_t)pos;
}
