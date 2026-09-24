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
 * high, so the code falls as VDD rises: VDD = 2.0 V * 4096 / code. Whether VDD
 * IS the pack in the wireless positions is the question this module exists to
 * answer; see battery.c. */

/* Enable the ADC. Call once from post-init. Blocks ~100 us (ADC warm-up). */
void battery_init(void);

/* Take one sample and fold it into the filter. 10 Hz housekeeping. */
void battery_task(void);

/* Filtered VDD in mV; 0 until the first sample. */
uint16_t battery_vdd_mv(void);

/* The last raw 12-bit ADC code for AIN16; 0 if the conversion timed out. */
uint16_t battery_vdd_raw(void);

/* Lowest and highest filtered VDD since boot, in mV; 0 before the first sample. */
uint16_t battery_vdd_min_mv(void);
uint16_t battery_vdd_max_mv(void);
