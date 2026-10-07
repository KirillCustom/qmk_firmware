// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The RTC period's persistence scheduler -- see rtc_persist.h. */
#include "rtc_persist.h"
#include "quantum.h"
#include "kb_eeconfig.h"
#include "flash_stats.h"

static uint16_t cand;            /* the latest accepted period, 0 for none this boot */
static uint8_t  cand_path;
static uint32_t last_save_ms;    /* this boot's last save, 0 for none */
static uint32_t last_check_ms;

bool rtc_persist_due(uint32_t now_ms, uint16_t stored, uint16_t c, uint32_t last_ms) {
    if (c < RTC_PERIOD_SANE_MIN || c > RTC_PERIOD_SANE_MAX) return false;
    if (now_ms < RTC_PERSIST_FIRST_MS) return false;
    if (stored < RTC_PERIOD_SANE_MIN || stored > RTC_PERIOD_SANE_MAX) return true;   /* nothing stored */
    uint16_t d = (c > stored) ? (uint16_t)(c - stored) : (uint16_t)(stored - c);
    if (d < RTC_PERSIST_MIN_DELTA) return false;
    return now_ms - last_ms >= RTC_PERSIST_GAP_MS;
}

void rtc_persist_propose(uint16_t period, uint8_t path) {
    flash_stats_rtc_proposal(period, path, false);
    if (period >= RTC_PERIOD_SANE_MIN && period <= RTC_PERIOD_SANE_MAX) {
        cand      = period;
        cand_path = path;
    }
}

void rtc_persist_task(void) {
    uint32_t now = timer_read32();
    if (now - last_check_ms < RTC_PERSIST_CHECK_MS) return;
    last_check_ms = now;
    if (!rtc_persist_due(now, kb_eeconfig_get_rtc_period(), cand, last_save_ms)) return;
    kb_eeconfig_set_rtc_period(cand);   /* coalesced and deferred by kb_eeconfig */
    flash_stats_rtc_store(cand, cand_path);
    last_save_ms = now;
}
