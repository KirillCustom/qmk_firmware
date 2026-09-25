// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* The SN32's own supply voltage, measured with its ADC -- the only battery
 * measurement this board has that is not the CH582F's `5C` percentage, which
 * read 100 on a pack that was flat enough to brown the board out (2026-09-24).
 *
 * AIN16 is an internal 2.0 V reference sampled against VDD as the reference
 * high, so the code falls as VDD rises: VDD = 2.0 V * 4096 / code.
 *
 * Measured 2026-09-24: 4.79 V in the cable position and 4.47 V in BT with the
 * cable in -- both USB-derived -- and 3.90 V unplugged, where it can only come
 * from the pack. How VDD relates to the pack voltage on battery is still being
 * settled (task 8.1), so the percentage below is PROVISIONAL. */

/* Enable the ADC. Call once from post-init. Blocks ~100 us (ADC warm-up). */
void battery_init(void);

/* Take one sample, fold it into the filter, run the low-battery protection and
 * the once-a-minute log. 10 Hz housekeeping. */
void battery_task(void);

/* Filtered VDD in mV; 0 until the first sample. */
uint16_t battery_vdd_mv(void);

/* The last raw 12-bit ADC code for AIN16; 0 if the conversion timed out. */
uint16_t battery_vdd_raw(void);

/* Lowest and highest filtered VDD since boot, in mV; 0 before the first sample. */
uint16_t battery_vdd_min_mv(void);
uint16_t battery_vdd_max_mv(void);

/* True when VDD says the board runs from the pack: no reading yet counts as
 * false. With the cable in, VDD is USB-derived (>= 4.4 V) and says nothing
 * about the pack. */
bool battery_on_battery(void);

/* Provisional level, 0-100, from a generic lithium-ion curve applied to VDD.
 * 0xFF when there is no pack reading: on USB power, or before the first
 * sample. */
uint8_t battery_level_pct(void);

/* VDD for display in centivolts, with hysteresis so the panel does not repaint
 * on every millivolt of ripple. 0 before the first sample. */
uint16_t battery_display_cv(void);

/* Status bits for HC_CONN and the log: see BATT_F_* in battery.c. */
uint8_t battery_flags(void);

/* Raw-HID readouts. battery_log_read fills `out` (18 bytes): stored count
 * u16, the requested index u16, ms since the newest entry u32, then two 8-byte
 * entries starting at `idx` (0 = oldest). battery_cfg applies op 1 (set
 * warn/cut mV, 0 disables either) and fills `out` (8 bytes) with the current
 * warn u16, cut u16, lights_cut u8, warned u8, and the log period in s (u16). */
void battery_log_read(uint16_t idx, uint8_t *out);
void battery_cfg(uint8_t op, uint16_t warn_mv, uint16_t cut_mv, uint8_t *out);
