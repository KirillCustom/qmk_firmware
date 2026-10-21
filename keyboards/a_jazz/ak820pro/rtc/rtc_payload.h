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
