// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>
#include "lockchan_media.h"
#include "rtc/rtc_payload.h"

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

static bool same_line(const uint8_t *a, uint8_t na, const char *b, uint8_t nb) {
    return na == nb && (na == 0 || memcmp(a, b, na) == 0);
}

uint8_t lockchan_media_readout(const lockchan_media_t *m, bool shown, const char *l0, uint8_t n0,
                               const char *l1, uint8_t n1, uint8_t icon) {
    if (m->type != LOCKCHAN_TYPE_NOW_PLAYING || !shown || m->icon != LOCKCHAN_ICON_PLAY) return LOCKCHAN_READOUT_OFF;
    if (same_line(m->l0, m->n0, l0, n0) && same_line(m->l1, m->n1, l1, n1))
        return icon == LOCKCHAN_ICON_PLAY ? LOCKCHAN_READOUT_KEEP : LOCKCHAN_READOUT_OFF;
    return LOCKCHAN_READOUT_GUESS;
}

bool lockchan_clock_parse(const lockchan_frame_t *f, lockchan_clock_t *c) {
    const uint8_t *p = f->payload;
    if (f->type != LOCKCHAN_TYPE_CLOCK || f->len != LOCKCHAN_CLOCK_LEN) return false;
    if (!rtc_ms_payload_valid(p)) return false;
    if (p[9] != 0 || p[10] != 0xFF || p[11] != 0x7F) return false;   /* flags 0, bias 0x7FFF */
    *c = (lockchan_clock_t){
        .yy = p[0], .mo = p[1], .dd = p[2], .wd = p[3], .hh = p[4], .mi = p[5], .ss = p[6],
        .ms = (uint16_t)(p[7] | (p[8] << 8)),
    };
    return true;
}

/* Civil dates as days since 1970-01-01 and back (H. Hinnant's
 * days_from_civil and civil_from_days), timezone-free: the board's clock is
 * local wall time, and the host's conversion has already been made. */
static int32_t days_from_civil(int32_t y, uint8_t m, uint8_t d) {
    y -= m <= 2;
    int32_t era = y / 400;
    int32_t yoe = y - era * 400;
    int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;   /* days since 1970-01-01 */
}

static void civil_from_days(int32_t z, int32_t *y, uint8_t *m, uint8_t *d) {
    z += 719468;
    int32_t  era = z / 146097;
    int32_t  doe = z - era * 146097;
    int32_t  yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int32_t  doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int32_t  mp  = (5 * doy + 2) / 153;
    *d = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
    *m = (uint8_t)(mp < 10 ? mp + 3 : mp - 9);
    *y = yoe + era * 400 + (*m <= 2);
}

bool lockchan_clock_advance(const lockchan_clock_t *c, uint32_t elapsed_ms, lockchan_clock_t *out) {
    if (elapsed_ms > LOCKCHAN_CLOCK_MAX_ELAPSED_MS) return false;
    int32_t days = days_from_civil(2000 + c->yy, c->mo, c->dd);
    /* ms in the day: < 86,400,000 + 60,000, inside int32. */
    int32_t tod  = ((int32_t)c->hh * 3600 + c->mi * 60 + c->ss) * 1000 + c->ms + (int32_t)elapsed_ms;
    int32_t carry = tod / 86400000;
    tod -= carry * 86400000;
    days += carry;
    int32_t y; uint8_t m, d;
    civil_from_days(days, &y, &m, &d);
    if (y < 2026 || y > 2098) return false;
    *out = (lockchan_clock_t){
        .yy = (uint8_t)(y - 2000), .mo = m, .dd = d,
        .wd = (uint8_t)((c->wd + carry) % 7),
        .hh = (uint8_t)(tod / 3600000), .mi = (uint8_t)(tod / 60000 % 60), .ss = (uint8_t)(tod / 1000 % 60),
        .ms = (uint16_t)(tod % 1000),
    };
    return true;
}
