// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* RTC_SET_TIME_MS's 12-byte body, validated BEFORE anything is written:
 *   yy mm dd wd hh mi ss ms_lo ms_hi flags bias_lo bias_hi
 * calendar incl. day-of-month and leap year, year 2026..2098 (the PCF stores
 * year%100 and the +1 s branch must stay representable), ms <= 999, reserved
 * flag bits zero, sof_bias in +-600 ppm or the 0x7FFF "unknown" sentinel.
 * Shared by raw HID (hid_protocol.c) and the lock-LED clock frame
 * (lockchan_media.c, type 4), so both take the same rules (codex, the
 * wireless clock's review). Pure, for the host tests. */

static inline uint8_t rtc_days_in_month(uint16_t year, uint8_t mo) {
    static const uint8_t dim[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    if (mo == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) return 29;
    return dim[mo - 1];
}

/* The age, in seconds, of a stamp taken at rtc_get_seconds() == at_s with
 * rtc_seconds_credit() == credit_at, now. A deep sleep's wake credits the
 * seconds count with what the slow ILRC missed (wake_coarse), and can do so
 * AFTER a stamp made during the wake's recovery: that credit belongs to the
 * sleep before the stamp, so it is taken back out (codex, the wireless
 * clock's implementation review, 1). A credit for a sleep after the stamp
 * comes out too, which only lengthens the age's window by the ILRC's ~13%
 * of that sleep: conservative. Unsigned throughout: both counts wrap. */
static inline uint32_t rtc_stamp_age_s(uint32_t at_s, uint32_t credit_at, uint32_t now_s, uint32_t credit_now) {
    int32_t age = (int32_t)((now_s - at_s) - (credit_now - credit_at));
    return age < 0 ? 0u : (uint32_t)age;
}

/* The lock-LED clock's rate trim (type 4). Off USB nothing else trims the
 * divider in practice: the legacy PCF check looks once an hour, and every
 * accepted host set restarts its window, so 10-minute frames starve it
 * (the Mac's cut test, 10-21: ~2.5 ms/s, a 1.6 s sawtooth). A frame's offset
 * over the time since the last accepted host set, with nothing between that
 * moved the clock or its period, is the rate, to a few ms in 600 s.
 *
 * `p` is the period register (a second is p + 1 cycles), `off_ms` the frame's
 * target minus board (negative: the board ahead, its seconds too short),
 * `interval_s` the board's seconds since the set. Returns the new period, or
 * `p` when there is nothing to do: an interval outside RTC_FT_MIN_S ..
 * RTC_FT_MAX_S, a rate under RTC_FT_MIN_PPM (the host's latency jitter) or
 * over RTC_FT_MAX_PPM (a host's step, not drift). Half the step, unless the
 * last trim's sign agrees: noise alternates and is halved, drift persists and
 * is followed (the SOF loop's rule). `*last_sign` carries that; a rate under
 * the floor clears it. Clamped to the sane window, 28000..40000. Pure, for
 * the host tests. */
#define RTC_FT_MIN_S    300u
#define RTC_FT_MAX_S    3600u
#define RTC_FT_MIN_PPM  50
#define RTC_FT_MAX_PPM  10000
static inline uint32_t rtc_frame_trim(uint32_t p, int32_t off_ms, uint32_t interval_s, int8_t *last_sign) {
    if (interval_s < RTC_FT_MIN_S || interval_s > RTC_FT_MAX_S) return p;
    int64_t ppm = -(int64_t)off_ms * 1000 / (int64_t)interval_s;   /* off / (interval * 1000) * 1e6 */
    if (ppm > RTC_FT_MAX_PPM || ppm < -RTC_FT_MAX_PPM) return p;
    if (ppm < RTC_FT_MIN_PPM && ppm > -RTC_FT_MIN_PPM) { *last_sign = 0; return p; }
    int64_t delta = ((int64_t)p + 1) * ppm / 1000000;
    if (delta == 0) return p;
    int8_t sign = delta > 0 ? 1 : -1;
    if (sign != *last_sign && (delta / 2) != 0) delta /= 2;
    *last_sign = sign;
    int64_t np = (int64_t)p + delta;
    if (np < 28000) np = 28000;
    if (np > 40000) np = 40000;
    return (uint32_t)np;
}

static inline bool rtc_ms_payload_valid(const uint8_t *p) {
    uint8_t yy = p[0], mo = p[1], dd = p[2];
    if (yy < 26 || yy > 98) return false;
    if (mo < 1 || mo > 12) return false;
    if (dd < 1 || dd > rtc_days_in_month((uint16_t)(2000 + yy), mo)) return false;
    if (p[3] > 6 || p[4] > 23 || p[5] > 59 || p[6] > 59) return false;
    uint16_t ms = (uint16_t)(p[7] | (p[8] << 8));
    if (ms > 999) return false;
    if (p[9] & 0xFC) return false;
    int16_t bias = (int16_t)(p[10] | (p[11] << 8));
    if (bias != 0x7FFF && (bias < -600 || bias > 600)) return false;
    return true;
}
