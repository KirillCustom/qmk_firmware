// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* The battery, as far as this board can know it. See battery.c for why each
 * signal is used where it is:
 *
 *  - VDD (the SN32's own ADC) says which supply feeds the rail. On battery it
 *    is a buck-boost's output, pinned at ~3.90 V, and says nothing about the
 *    pack.
 *  - `5C` (the CH582F's battery percent) is a linear voltmeter on the pack,
 *    clamped at ~4.036 V and ~3.161 V. It says how full the pack is. */

typedef enum {
    BATTERY_SUPPLY_UNKNOWN = 0,   /* boot, before the dwell */
    BATTERY_SUPPLY_BATTERY,       /* the pack powers the board */
    BATTERY_SUPPLY_EXTERNAL,      /* USB powers the board */
} battery_supply_t;

typedef enum {
    BATTERY_STATE_NONE = 0,   /* supply not decided yet */
    BATTERY_STATE_BATTERY,    /* running from the pack */
    BATTERY_STATE_CHARGING,   /* on USB, charger running */
    BATTERY_STATE_FULL,       /* on USB, charger terminated with `5C` at 100 */
    BATTERY_STATE_USB,        /* on USB, charger idle */
} battery_state_t;

/* HC_BATTLOG reply format, and HC_CONN byte 31 (the battery fields' layout). */
#define BATTERY_LOG_VERSION    5
#define BATTERY_LOG_ENTRY_SIZE 18
#define BATTERY_PROTO_VERSION  5

/* Enable the ADC. Call once from post-init. Blocks ~100 us (ADC warm-up). */
void battery_init(void);

/* 10 Hz housekeeping: one VDD sample, the supply decision, `5C` intake, the
 * level, the low-battery protection and the log. */
void battery_task(void);

/* Filtered VDD in mV; 0 until the first sample. Display and diagnostics only. */
uint16_t battery_vdd_mv(void);
/* The last raw 12-bit ADC code for AIN16; 0 if the conversion timed out. */
uint16_t battery_vdd_raw(void);
/* Lowest and highest filtered VDD since boot, in mV; 0 before the first sample. */
uint16_t battery_vdd_min_mv(void);
uint16_t battery_vdd_max_mv(void);

/* What powers the board. battery_on_battery() is supply == BATTERY; UNKNOWN
 * counts as false, so nothing battery-only acts before the decision. */
battery_supply_t battery_supply(void);
bool             battery_on_battery(void);
/* The board has run from the pack at some point this boot: a pack is fitted. */
bool             battery_ran_from_pack(void);

battery_state_t battery_state(void);

/* The level on the panel: 0-100 in whole percent, or 0xFF when nothing is
 * known yet. 100 only after a full charge. */
uint8_t  battery_level_pct(void);
/* The same level in per mille, 0xFFFF unknown. */
uint16_t battery_level_permille(void);

/* Every `5C` frame the CH582F driver parses, as it parses it. */
void     battery_5c_report(uint8_t pct);
/* The estimate from the recent fresh `5C` reports (a trimmed mean), in
 * hundredths of a count; 0xFFFF when there is none. */
uint16_t battery_5c_x100(void);
/* The same, rounded to a whole count; 0xFF when there is none. 100 and 0 are
 * the clamps. */
uint8_t  battery_5c_rounded(void);
/* That estimate as pack mV; 0 when there is none. ⚠️ At the clamps this is
 * roughly the clamp's BOUND (3161 = "at most", 4036 = "at least"). */
uint16_t battery_pack_mv(void);

/* LED drive over the last second, per mille of the maximum (after the power cap). */
uint16_t battery_led_pm(void);

/* Status bits for HC_CONN and the log: see BATT_F_* in battery.c. */
uint8_t battery_flags(void);

/* Raw-HID readouts. battery_log_read fills `out` (29 bytes): version u8 (BATTERY_LOG_VERSION),
 * stored count u16, the requested index u16, ms since the newest entry u32,
 * entries ever written (low 16 bits) u16, then the 18-byte entry at `idx`
 * (0 = oldest). v5 dropped the period from the reply to fit the entry: the
 * host reads it from battery_cfg's out[6..7]. battery_cfg applies op 1 (set warn/cut in PACK
 * mV, 0 disables either) and fills `out` (8 bytes) with warn u16, cut u16,
 * lights_cut u8, warned u8, and the log period in s (u16). */
void battery_log_read(uint16_t idx, uint8_t *out);
/* FROM-FLAT's test on the boot's first estimate, in hundredths of a count. */
bool battery_first_estimate_is_flat(uint16_t first_x100);
void battery_cfg(uint8_t op, uint16_t warn_mv, uint16_t cut_mv, uint8_t *out);
