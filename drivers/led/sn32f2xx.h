/* Copyright 2024 Dimitris Mantzouranis <d3xter93@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#if !defined(SN32F2XX_LED_OUTPUT_LUMINOSITY_R)
#    define SN32F2XX_LED_OUTPUT_LUMINOSITY_R 1
#endif
#if !defined(SN32F2XX_LED_OUTPUT_LUMINOSITY_G)
#    define SN32F2XX_LED_OUTPUT_LUMINOSITY_G 1
#endif
#if !defined(SN32F2XX_LED_OUTPUT_LUMINOSITY_B)
#    define SN32F2XX_LED_OUTPUT_LUMINOSITY_B 1
#endif

#if defined(RGB_MATRIX_SN32F2XX)
#    define SN32F2XX_LED_COUNT RGB_MATRIX_LED_COUNT
#endif

#if !defined(SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL)
#    define SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL SN32F2XX_PWM_OUTPUT_ACTIVE_LOW
#endif

#if !defined(SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL)
#    define SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH
#endif

#if !defined(SN32F2XX_PWM_CONTROL)
#    define SN32F2XX_PWM_CONTROL HARDWARE_PWM
#endif

#if !defined(SN32F2XX_PWM_DIRECTION)
#    define SN32F2XX_PWM_DIRECTION DIODE_DIRECTION
#endif

#if !defined(SN32F2XX_RGB_MATRIX_ROW_CHANNELS)
#    define SN32F2XX_RGB_MATRIX_ROW_CHANNELS 3
#endif

#if !defined(SN32F2XX_RGB_MATRIX_ROWS)
#    define SN32F2XX_RGB_MATRIX_ROWS MATRIX_ROWS
#endif

#if !defined(SN32F2XX_RGB_MATRIX_COLS)
#    define SN32F2XX_RGB_MATRIX_COLS MATRIX_COLS
#endif

#if !defined(SN32F2XX_RGB_MATRIX_ROW_PINS)
#    error Missing definition for SN32F2XX_RGB_MATRIX_ROW_PINS
#endif

#if !defined(SN32F2XX_RGB_MATRIX_COL_PINS)
#    define SHARED_MATRIX
#    define SN32F2XX_RGB_MATRIX_COL_PINS MATRIX_COL_PINS
#endif

#if !defined(SN32F2XX_RGB_MATRIX_ROWS_HW)
#    define SN32F2XX_RGB_MATRIX_ROWS_HW (SN32F2XX_RGB_MATRIX_ROWS * SN32F2XX_RGB_MATRIX_ROW_CHANNELS)
#endif

void sn32f2xx_init(void);
void sn32f2xx_flush(void);
void sn32f2xx_set_color(int index, uint8_t r, uint8_t g, uint8_t b);
void sn32f2xx_set_color_all(uint8_t r, uint8_t g, uint8_t b);

/* Sum of every LED channel's requested PWM value, 0 .. LED_COUNT * 3 * 255.
 * Proportional to the matrix's average drive, so a board can estimate its
 * current draw from it. Main-thread only, like sn32f2xx_set_color(). */
uint32_t sn32f2xx_led_load(void);

/* The drive actually shown, time-weighted: the sum of every channel's value in
 * the COMMITTED frame (what the row ISR drives), times the ms it was shown,
 * accumulated since the last call, which closes the interval at `now_ms` and
 * starts the next. Divided by the elapsed ms it is the mean drive over the
 * interval, whatever the effect's frame pattern. `lit_ms`, if not NULL, gets
 * the ms of the interval in which any channel was lit. Main-thread only. */
uint64_t sn32f2xx_led_drive_take(uint32_t now_ms, uint32_t *lit_ms);

/* Scale every LED's output by scale/256 (255 = unscaled) below the rgb_matrix
 * config: dim or darken the matrix without touching the user's brightness.
 * Takes effect as the next frame renders. Main-thread only. */
void sn32f2xx_set_power_scale(uint8_t scale);

/* Turn every LED output off and leave it off. Call before stopping the row
 * ISR, or the mux freezes with one row/channel driving at full duty. */
void sn32f2xx_blank(void);

/* Sleep hooks. Scan-only is compiled only where the board defines
 * SN32F2XX_SLEEP_HOOKS; the deep-sleep pair where it defines
 * SN32F2XX_DEEP_SLEEP (or SN32F2XX_SLEEP_HOOKS).
 *
 * Scan-only, for a board's light sleep: while on, the row ISR drives no LED
 * and scans one key row per call at a longer period (SN32F2XX_SCAN_ONLY_TICKS),
 * each key sampled every ~3.9 ms against the field's ~4.6. The ISR applies it at
 * its next call, and the LED field resumes where it was when it ends. Only for
 * a dark matrix: the caller's job. */
void sn32f2xx_set_scan_only(bool on);
bool sn32f2xx_scan_only_active(void);

/* Deep sleep, for a board's, with interrupts locked (I-class): every
 * LED output off and the row ISR stopped, the key columns left as inputs with
 * pull-ups; and back. */
void sn32f2xx_sleep_enterI(void);
void sn32f2xx_sleep_exitI(void);
