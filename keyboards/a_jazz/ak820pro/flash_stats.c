// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Internal-flash write attribution -- see flash_stats.h. */
#include "quantum.h"
#include "eeprom.h"
#include "nvm_eeprom_eeconfig_internal.h"   /* EECONFIG_* offsets in the emulated EEPROM */
#include "flash_stats.h"
#include "wear_leveling_efl_config.h"   /* WEAR_LEVELING_BACKING_SIZE */

/* The writer the next session belongs to, set by the EEPROM write that is
 * about to open it, and consumed (back to OTHER) when it does. */
static uint8_t  pending_writer = FW_OTHER, pending_mask = 0;
static uint8_t  session_writer = FW_OTHER;

static uint32_t sessions[FW_COUNT], programs[FW_COUNT];
static uint16_t erases[FW_COUNT];
static uint16_t kb_field_sessions[KBF_COUNT], kb_mixed, kb_unknown;
static uint32_t last_erase_ms;

static uint16_t rtc_last_proposed, rtc_last_stored;
static uint8_t  rtc_last_proposed_path, rtc_last_stored_path;
static uint32_t rtc_proposals[2], rtc_stores[2];

#define KB_START ((uint32_t)(uintptr_t)EECONFIG_KB_DATABLOCK)
#define RGB_START ((uint32_t)(uintptr_t)EECONFIG_RGB_MATRIX)
#define RGB_LEN  8u                                        /* EECONFIG_RGB_MATRIX is a uint64_t */
#define VIA_START ((uint32_t)(EECONFIG_SIZE))              /* VIA's magic, config, keymap, macros */

/* kb_eeconfig.c's kb_config_t, byte by byte: bt, rtc lo, rtc hi, lcd, clock,
 * battery, power mode, stock lighting flags. A layout change there must change this table. */
static const uint8_t kb_byte_field[] = {KBF_BT, KBF_RTC, KBF_RTC, KBF_LCD, KBF_CLOCK, KBF_BATT, KBF_PWR, KBF_FX};
_Static_assert(sizeof(kb_byte_field) == EECONFIG_KB_DATA_SIZE, "one field per kb datablock byte");

/* Called by drivers/eeprom/eeprom_wear_leveling.c's eeprom_write_block() just
 * before wear_leveling_write(), which skips the write when nothing changes. */
void eeprom_wear_leveling_write_hook(uint32_t addr, const void *buf, size_t len) {
    const uint8_t *b = (const uint8_t *)buf;
    uint8_t        w = FW_OTHER, mask = 0;
    if (addr >= KB_START && addr < KB_START + EECONFIG_KB_DATA_SIZE) {
        w = FW_KB;
        for (size_t i = 0; i < len && addr + i < KB_START + EECONFIG_KB_DATA_SIZE; i++) {
            uint8_t cur = eeprom_read_byte((const uint8_t *)(uintptr_t)(addr + i));   /* the cache */
            if (cur != b[i]) mask |= kb_byte_field[addr + i - KB_START];
        }
    } else if (addr >= RGB_START && addr < RGB_START + RGB_LEN) {
        w = FW_RGB;
    } else if (addr >= VIA_START) {
        w = FW_VIA;
    }
    pending_writer = w;
    pending_mask   = mask;
}

void flash_stats_session(void) {
    session_writer = pending_writer;
    sessions[session_writer]++;
    if (session_writer == FW_KB) {
        uint8_t m = pending_mask;
        if (m == 0) {
            if (kb_unknown < UINT16_MAX) kb_unknown++;
        } else if (m & (m - 1u)) {
            if (kb_mixed < UINT16_MAX) kb_mixed++;
        } else {
            for (uint8_t i = 0; i < KBF_COUNT; i++)
                if ((m >> i) & 1u && kb_field_sessions[i] < UINT16_MAX) kb_field_sessions[i]++;
        }
    }
    pending_writer = FW_OTHER;
    pending_mask   = 0;
}

void flash_stats_pass(void) {
    pending_writer = FW_OTHER;
    pending_mask   = 0;
}

void flash_stats_op(bool erase) {
    if (erase) {
        if (erases[session_writer] < UINT16_MAX) erases[session_writer]++;
        last_erase_ms = timer_read32();
        if (last_erase_ms == 0) last_erase_ms = 1;
    } else {
        programs[session_writer]++;
    }
}

void flash_stats_rtc_proposal(uint16_t period, uint8_t path, bool stored) {
    uint8_t i = (path == FLASH_RTC_PATH_PCF) ? 1u : 0u;
    rtc_last_proposed      = period;
    rtc_last_proposed_path = path;
    rtc_proposals[i]++;
    if (stored) {
        rtc_last_stored      = period;
        rtc_last_stored_path = path;
        rtc_stores[i]++;
    }
}

void flash_stats_rtc_store(uint16_t period, uint8_t path) {
    rtc_last_stored      = period;
    rtc_last_stored_path = path;
    rtc_stores[(path == FLASH_RTC_PATH_PCF) ? 1u : 0u]++;
}

void flash_stats_reset(void) {
    memset(sessions, 0, sizeof sessions);
    memset(programs, 0, sizeof programs);
    memset(erases, 0, sizeof erases);
    memset(kb_field_sessions, 0, sizeof kb_field_sessions);
    kb_mixed = kb_unknown = 0;
    last_erase_ms = 0;
    rtc_proposals[0] = rtc_proposals[1] = rtc_stores[0] = rtc_stores[1] = 0;
    /* rtc_last_* are states, not counts: kept. */
}

bool flash_stats_fill(uint8_t page, uint8_t *out28) {
    uint32_t v;
    uint8_t *p = out28;
#define PUT32(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF; } while (0)
#define PUT16(x) do { v = (x); *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; } while (0)
    memset(out28, 0, 28);
    switch (page) {
        case 0:
            for (uint8_t i = 0; i < FW_COUNT; i++) PUT32(sessions[i]);   /* 16 */
            for (uint8_t i = 0; i < FW_COUNT; i++) PUT16(erases[i]);     /* 24 */
            PUT16(kb_mixed); PUT16(kb_unknown);                          /* 28 */
            break;
        case 1:
            for (uint8_t i = 0; i < FW_COUNT; i++) PUT32(programs[i]);   /* 16 */
            for (uint8_t i = 0; i < KBF_COUNT; i++) PUT16(kb_field_sessions[i]);   /* 28 */
            break;
        case 2:
            PUT16(rtc_last_proposed);                                    /* 2 */
            *p++ = rtc_last_proposed_path; *p++ = rtc_last_stored_path;  /* 4 */
            PUT16(rtc_last_stored);                                      /* 6 */
            PUT32(rtc_proposals[0]); PUT32(rtc_proposals[1]);            /* 14 */
            PUT32(rtc_stores[0]); PUT32(rtc_stores[1]);                  /* 22 */
            PUT32(last_erase_ms);                                        /* 26 */
            PUT16(WEAR_LEVELING_BACKING_SIZE);                           /* 28: an erase is all of it */
            break;
        default:
            return false;
    }
#undef PUT16
#undef PUT32
    return true;
}
