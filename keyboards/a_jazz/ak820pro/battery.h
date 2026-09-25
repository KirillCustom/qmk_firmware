// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* The battery, as far as this board can know it. The CH582F's `5C` percentage
 * read 100 on a pack flat enough to brown the board out (2026-09-24), so the
 * level comes from the SN32's own VDD instead: the internal 2.0 V reference
 * (AIN16) sampled against VDD as the reference high, VDD = 2.0 V * 4096 / code.
 *
 * Measured: 4.79 V in the cable position and 4.47 V in BT with the cable in --
 * both USB-derived -- and, unplugged, 3.90 V HELD BY A REGULATOR while the pack
 * sat at 4.18 -> 4.13 V (meter at the connector, 2026-09-25). VDD follows the
 * pack only once the pack drops below the regulator's output plus dropout, so
 * the level is an estimate: see battery.c. */

typedef enum {
    BATTERY_STATE_NONE = 0,   /* no reading yet */
    BATTERY_STATE_BATTERY,    /* running from the pack */
    BATTERY_STATE_CHARGING,   /* on USB, charger running */
    BATTERY_STATE_FULL,       /* on USB, the charger has terminated this session */
    BATTERY_STATE_USB,        /* on USB, nothing else known */
} battery_state_t;

/* HC_BATTLOG reply format. */
#define BATTERY_LOG_VERSION    2
#define BATTERY_LOG_ENTRY_SIZE 12

/* Enable the ADC. Call once from post-init. Blocks ~100 us (ADC warm-up). */
void battery_init(void);

/* Take one sample, fold it into the filter, run the low-battery protection,
 * the level estimator (1 Hz) and the once-a-minute log. 10 Hz housekeeping. */
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

battery_state_t battery_state(void);

/* Estimated level 0-100 with 2-point hysteresis, or 0xFF when nothing is
 * known yet (booted on USB and the charger has not terminated). */
uint8_t battery_level_pct(void);

/* The estimator's raw state, for HC_CONN: level in 0.01 % (0xFFFF unknown),
 * this unit's learned regulator output in mV, and the LED drive in per mille
 * of the maximum. */
uint16_t battery_soc_x100(void);
uint16_t battery_clamp_mv(void);
uint16_t battery_led_pm(void);

/* VDD for display in centivolts, with hysteresis so the panel does not repaint
 * on every millivolt of ripple. 0 before the first sample. */
uint16_t battery_display_cv(void);

/* Status bits for HC_CONN and the log: see BATT_F_* in battery.c. */
uint8_t battery_flags(void);

/* Raw-HID readouts. battery_log_read fills `out` (21 bytes): version u8,
 * stored count u16, the requested index u16, ms since the newest entry u32,
 * then the 12-byte entry at `idx` (0 = oldest). battery_cfg applies op 1 (set
 * warn/cut mV, 0 disables either) and fills `out` (8 bytes) with the current
 * warn u16, cut u16, lights_cut u8, warned u8, and the log period in s (u16). */
void battery_log_read(uint16_t idx, uint8_t *out);
void battery_cfg(uint8_t op, uint16_t warn_mv, uint16_t cut_mv, uint8_t *out);
