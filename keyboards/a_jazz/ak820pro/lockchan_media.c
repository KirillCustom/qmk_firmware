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
    if (f->type != LOCKCHAN_TYPE_NOW_PLAYING) return false;
    if (f->len < 2 || f->len > 2 + LOCKCHAN_MEDIA_MAX_L0 + LOCKCHAN_MEDIA_MAX_L1) return false;
    uint8_t icon = f->payload[0], n0 = f->payload[1];
    if (icon > 3) return false;                         /* bit 7 (playback) included */
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
