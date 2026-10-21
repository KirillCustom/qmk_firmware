// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Internal-flash write attribution (Phase 1b, C1; plans/BATTERY-GAUGE-REFINE-PLAN.md).
 *
 * Every internal-flash write blanks the LED matrix for a few ms (docs/leds.md),
 * and the board made ~816 write sessions in ~47 h where the saved battery level
 * accounts for ~6 a charge cycle. These counters name the writer.
 *
 * The WRITER is decided where QMK writes the emulated EEPROM, by the logical
 * address (eeprom_wear_leveling_write_hook, called from eeprom_write_block):
 * the kb datablock (kb_eeconfig.c), the RGB matrix config, VIA's region (its
 * magic, custom config, dynamic keymap and macros), or anything else. For the
 * kb datablock it also notes WHICH FIELDS the write changes, from the bytes
 * that differ from the cache. wear_leveling_write() skips a write that changes
 * nothing, so a field that changes and changes back before kb_eeconfig's 5 s
 * settle costs nothing and counts nothing.
 *
 * The PHYSICAL counts are taken at the backing store: a write SESSION at each
 * unlock (backing_store_pre_write_hook), and each line PROGRAM and each ERASE
 * at backing_store_operation_begin. ⚠️ An ERASE here is one WHOLE-STORE erase:
 * wear_leveling_efl.c calls the hook once before looping over every sector of
 * the 2 KB backing store -- two 1 KB pages on the SN32F299 (codex, gate 2).
 * For endurance, each page takes one erase cycle per count; do not halve it. A session is attributed to the writer whose
 * EEPROM write opened it; one not opened by an EEPROM write (boot-time
 * consolidation, a format) is OTHER. Wear-levelling consolidation happens
 * inside the session that overflowed the log, and shows as that writer's
 * erases.
 *
 * Counters are since boot or since health_reset() (HC_RESET, hold Fn+D), like
 * health.c's flash_writes, which counts the same sessions. Main-loop only. */

enum flash_writer { FW_OTHER = 0, FW_KB, FW_RGB, FW_VIA, FW_COUNT };

/* kb datablock fields (kb_eeconfig.c's layout), as bits of a field mask. */
#define KBF_BT    0x01u
#define KBF_RTC   0x02u
#define KBF_LCD   0x04u
#define KBF_CLOCK 0x08u
#define KBF_BATT  0x10u
#define KBF_PWR   0x20u
#define KBF_COUNT 6
/* Stock lighting flags (byte 7). Past KBF_COUNT on purpose: health page 1 is
 * full, so its sessions show only in FW_KB's total, not in a field counter. */
#define KBF_FX    0x40u

/* backing_store_pre_write_hook: a write session starts. */
void flash_stats_session(void);
/* Every main-loop pass (health_loop_tick): an EEPROM write that changed
 * nothing opened no session and must not lend its writer to a later one. A
 * write and the session it opens are synchronous, inside one pass. */
void flash_stats_pass(void);
/* backing_store_operation_begin: one program (erase false) or erase. */
void flash_stats_op(bool erase);

/* rtc.c: the persisted RTC period was considered for saving. path 1 = the USB
 * SOF loop, 2 = the PCF trim; `period` is the proposed value; `stored` is true
 * when it differed from the saved one by enough to be written (the write
 * itself is kb_eeconfig's, deferred and coalesced). */
#define FLASH_RTC_PATH_SOF 1u
#define FLASH_RTC_PATH_PCF 2u
void flash_stats_rtc_proposal(uint16_t period, uint8_t path, bool stored);
/* C2: the persistence scheduler (rtc_persist.c) saved `period`, a candidate
 * from `path`. Since C2 the proposals pass stored = false and only this counts
 * stores. */
void flash_stats_rtc_store(uint16_t period, uint8_t path);

void flash_stats_reset(void);

/* Raw HID HC_FLASHW pages (hid_protocol.c), 28 bytes each, little-endian.
 *   page 0: u32 sessions[FW_COUNT] (other, kb, rgb, via)
 *           u16 erases[FW_COUNT]
 *           u16 kb_mixed     -- kb sessions that changed more than one field
 *           u16 kb_unknown   -- kb sessions with no field mask (should be 0)
 *   page 1: u32 programs[FW_COUNT]
 *           u16 kb_field_sessions[KBF_COUNT] (bt, rtc, lcd, clock, batt,
 *               power mode): kb sessions that changed that field ALONE (the
 *               power mode took the page's last, reserved, u16 in 2026-10)
 *   page 2: u16 rtc_last_proposed, u8 rtc_last_proposed_path,
 *           u8 rtc_last_stored_path, u16 rtc_last_stored,
 *           u32 rtc_proposals_sof, u32 rtc_proposals_pcf,
 *           u32 rtc_stores_sof, u32 rtc_stores_pcf,
 *           u32 last_erase_uptime_ms (0: none),
 *           u16 the backing store's size in bytes (WEAR_LEVELING_BACKING_SIZE:
 *               an erase erases all of it)
 * Returns false for an unknown page. */
bool flash_stats_fill(uint8_t page, uint8_t *out28);
