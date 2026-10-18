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

#include <string.h>
#include "matrix.h"
#include "rgb_matrix.h"
#include "sn32f2xx.h"

#define ROWS_PER_HAND (MATRIX_ROWS)

#if !defined(MATRIX_IO_DELAY)
#    define MATRIX_IO_DELAY 30
#endif

#define SN32F2XX_PWM_OUTPUT_ACTIVE_HIGH PWM_OUTPUT_ACTIVE_HIGH
#define SN32F2XX_PWM_OUTPUT_ACTIVE_LOW PWM_OUTPUT_ACTIVE_LOW

#define SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH PWM_OUTPUT_ACTIVE_HIGH
#define SN32F2XX_RGB_OUTPUT_ACTIVE_LOW PWM_OUTPUT_ACTIVE_LOW

#define HARDWARE_PWM 0
#define SOFTWARE_PWM 1

/*
    Default configuration example

    COLS key / led
    SS8050 transistors NPN driven low
    base      - GPIO
    collector - LED Col pins
    emitter   - VDD

    VDD     GPIO
    (E)     (B)
     |  PNP  |
     |_______|
         |
         |
        (C)
        LED

    ROWS RGB
    SS8550 transistors PNP driven high
    base      - GPIO
    collector - LED RGB row pins
    emitter   - GND

        LED
        (C)
         |
         |
      _______
     |  NPN  |
     |       |
    (B)     (E)
    GPIO    GND
*/
#if (SN32F2XX_PWM_DIRECTION == COL2ROW)
static uint8_t chan_col_order[SN32F2XX_RGB_MATRIX_COLS] = {0}; // track the channel col order
static uint8_t current_row                              = 0;   // LED row scan counter
static uint8_t current_key_row                          = 0;   // key row scan counter
#    if (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
static uint8_t led_duty_cycle[SN32F2XX_RGB_MATRIX_COLS] = {0}; // track the channel duty cycle
#    endif
/* Opt-in multi-timer hardware PWM. A board whose columns do not all fit on a single
   CT16 timer (e.g. SN32F290: 15 columns > 12 CT16B1 channels) supplies
   SN32F2XX_PWM_COL_MAP as an initialiser of {PWMDriver*, channel} pairs in COL_PINS
   order. Without it every column stays on PWMD1 and behaviour is unchanged. */
#    if defined(SN32F2XX_PWM_MULTI_TIMER)
typedef struct {
    PWMDriver *drv;
    uint8_t    ch;
} sn32_pwm_col_t;
static const sn32_pwm_col_t sn32_pwm_col_map[SN32F2XX_RGB_MATRIX_COLS] = SN32F2XX_PWM_COL_MAP;
#        define COL_PWM_DRV(col) (sn32_pwm_col_map[col].drv)
#        define COL_PWM_CH(col)  (sn32_pwm_col_map[col].ch)
#    else
#        define COL_PWM_DRV(col) (&PWMD1)
#        define COL_PWM_CH(col)  (chan_col_order[col])
#    endif
#elif (SN32F2XX_PWM_DIRECTION == ROW2COL)
/* make sure to `#define MATRIX_UNSELECT_DRIVE_HIGH` in this configuration*/
static uint8_t chan_row_order[SN32F2XX_RGB_MATRIX_ROWS_HW] = {0}; // track the channel row order
static uint8_t current_key_col                             = 0;   // key col scan counter
static uint8_t last_key_col                                = 0;   // key col scan counter
#    if (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
static uint8_t led_duty_cycle[SN32F2XX_RGB_MATRIX_ROWS_HW] = {0}; // track the channel duty cycle
#    endif
#endif
#if (DIODE_DIRECTION == ROW2COL)
static matrix_row_t row_shifter = MATRIX_ROW_SHIFTER;
#endif
#if defined(SHARED_MATRIX)
extern matrix_row_t  raw_matrix[MATRIX_ROWS];    // raw values
extern matrix_row_t  matrix[MATRIX_ROWS];        // debounced values
static matrix_row_t  shared_matrix[MATRIX_ROWS]; // scan values
/* Dirty flag: the ISR sets it when it has scanned something new, the main loop
 * clears it after consuming. NOT a scan gate -- see shared_matrix_scan_keys().
 * (The old matrix_locked interlock that paired with it is gone; the scheme it
 * balanced coverage for no longer exists.) */
static volatile bool matrix_scanned = false;
#endif // SHARED MATRIX
/* +1 so a channel at FULL brightness still has a match to hit.
 *
 * pwm_lld_start writes the period match as (period - 1), and pwm_lld_mr_value is
 * identity, so with periodticks == RGB_MATRIX_MAXIMUM_BRIGHTNESS the counter
 * resets at 254 while a full-value colour channel writes MR = 255. That match
 * never fires, the channel never asserts, and the LED goes DARK at exactly 100%
 * -- which reads as the colour lurching when brightness reaches maximum, and as
 * brief colour flashes while stepping through it.
 *
 * Costs one extra tick per PWM period (field rate 498 -> 496 Hz on the AK820 Pro,
 * i.e. nothing) and makes full duty actually mean full duty. */
static const uint32_t periodticks                               = RGB_MATRIX_MAXIMUM_BRIGHTNESS + 1;
/* PWM clock. Overridable, because deriving it from the UI step sizes is an
 * arbitrary coupling and a genuine wart.
 *
 * freq feeds exactly one thing -- the prescaler, psc = PWM_CLK/freq - 1 -- so
 * there is no physical reason it should be the PRODUCT of HUE/SAT/VAL/SPD steps
 * and LED_PROCESS_LIMIT. But because it is, every UI granularity choice
 * silently retunes the LED field rate: halving HUE_STEP for finer colour
 * control halves the field rate, which on boards where R/G/B are time-sliced
 * brings back visible colour fringing on eye movement. Nothing in the UI
 * connects those two things, and the coupling forces a board to trade one axis
 * of granularity against another for no reason.
 *
 * Defining SN32F2XX_RGB_PWM_FREQ pins the clock and frees all four step sizes
 * to be chosen purely for the UI. The default is the original expression, so
 * every board that does not set it behaves exactly as before. */
#ifndef SN32F2XX_RGB_PWM_FREQ
#    define SN32F2XX_RGB_PWM_FREQ (RGB_MATRIX_HUE_STEP * RGB_MATRIX_SAT_STEP * RGB_MATRIX_VAL_STEP * RGB_MATRIX_SPD_STEP * RGB_MATRIX_LED_PROCESS_LIMIT)
#endif
static const uint32_t freq                                      = SN32F2XX_RGB_PWM_FREQ;
static const pin_t    led_row_pins[SN32F2XX_RGB_MATRIX_ROWS_HW] = SN32F2XX_RGB_MATRIX_ROW_PINS; // We expect a R,B,G order here
static const pin_t    led_col_pins[SN32F2XX_RGB_MATRIX_COLS]    = SN32F2XX_RGB_MATRIX_COL_PINS;
static RGB            led_state[SN32F2XX_LED_COUNT];     // led state buffer
static RGB            led_state_buf[SN32F2XX_LED_COUNT]; // led state buffer
bool                  led_state_buf_update_required = false;
#ifdef UNDERGLOW_RBG // handle underglow with flipped B,G channels
static const uint8_t underglow_leds[UNDERGLOW_LEDS] = UNDERGLOW_IDX;
#endif

#if defined(SHARED_MATRIX)
void matrix_output_unselect_delay(uint8_t line, bool key_pressed) {
    for (int i = 0; i < TIME_US2I(MATRIX_IO_DELAY); ++i) {
        __asm__ volatile("" ::: "memory");
    }
}
bool matrix_can_read(void) {
    return matrix_scanned;
}
#endif // SHARED_MATRIX

static void rgb_callback(PWMDriver *pwmp);

#if !defined(SN32F2)
#    error Driver is MCU specific to the Sonix SN32F2 family.
#endif // !defined(SN32F2)

#if (defined(SN32F240B) || defined(SN32F240C) || defined(SN32F290))
/* PWM configuration structure. On F240B/C this is CT16B1 with 24 channels; on
 * F290 CT16B1 has only 12 usable channels, but with SOFTWARE_PWM the timer is
 * used solely as a free-running counter + periodic ISR, so the channel count is
 * irrelevant (rgb_callback bit-bangs the LED pins directly). */
static PWMConfig pwmcfg = {
    freq,        /* PWM clock frequency. */
    periodticks, /* PWM period = periodticks * (1 / CH_CFG_ST_FREQUENCY) ≈ 255 * (1 / 187500) ≈ 1.36 ms */
    NULL,        /* RGB Callback */
    {
        /* Default all channels to disabled - Channels will be configured during init */
        [0 ... PWM_CHANNELS - 1] = {PWM_OUTPUT_DISABLED, NULL, 0},
    },
    0 /* HW dependent part.*/
};

#    if defined(SN32F2XX_PWM_MULTI_TIMER)
/* Extra per-timer configs for the columns that spill over to CT16B0 / CT16B2.
   Channel modes are filled in by rgb_ch_ctrl() from SN32F2XX_PWM_COL_MAP. */
static PWMConfig pwmcfg_b0 = {
    freq,
    periodticks,
    NULL,
    {[0 ... PWM_CHANNELS - 1] = {PWM_OUTPUT_DISABLED, NULL, 0}},
    0};
static PWMConfig pwmcfg_b2 = {
    freq,
    periodticks,
    NULL,
    {[0 ... PWM_CHANNELS - 1] = {PWM_OUTPUT_DISABLED, NULL, 0}},
    0};

/* Resolve a column's PWMDriver to its owning PWMConfig. */
static PWMConfig *sn32_pwm_cfg_for(PWMDriver *drv) {
    if (drv == &PWMD0) return &pwmcfg_b0;
    if (drv == &PWMD2) return &pwmcfg_b2;
    return &pwmcfg; /* PWMD1 */
}
#    endif // SN32F2XX_PWM_MULTI_TIMER

static void rgb_ch_ctrl(PWMConfig *cfg) {
#    if defined(SN32F2XX_PWM_MULTI_TIMER)
    /* Multi-timer: the column->(timer,channel) mapping and pin-mux are fixed by the
       board (SN32F2XX_PWM_COL_MAP + the SN_PFPA writes in sn32f2xx_init). Just mark
       each used channel active on its owning timer's config. */
    for (uint8_t i = 0; i < SN32F2XX_RGB_MATRIX_COLS; i++) {
        if (led_col_pins[i] > C15) continue; // Only P0.0..P2.15 can be PWM outputs
        chan_col_order[i]                                        = sn32_pwm_col_map[i].ch;
        sn32_pwm_cfg_for(sn32_pwm_col_map[i].drv)->channels[sn32_pwm_col_map[i].ch].mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL;
    }
    return;
#    endif // SN32F2XX_PWM_MULTI_TIMER
    /* Enable PWM function, IOs and select the PWM modes for the LED pins */
#    if (SN32F2XX_PWM_DIRECTION == COL2ROW)
    for (uint8_t i = 0; i < SN32F2XX_RGB_MATRIX_COLS; i++) {
#        if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        // Only P0.0 to P2.15 can be used as pwm output
        if (led_col_pins[i] > C15) continue;
#        endif // SN32F2XX_PWM_CONTROL
        /* We use a trick here, according to pfpa table of sn32f240b datasheet,
           pwm channel and pfpa of pin Px.y can be calculated as below:
             channel = (x*16+y)%24
             pfpa = 1, when (x*16+y)>23
        */
        uint8_t pio_value = ((uint32_t)(PAL_PORT(led_col_pins[i])) - (uint32_t)(PAL_PORT(A0))) / ((uint32_t)(PAL_PORT(B0)) - (uint32_t)(PAL_PORT(A0))) * PAL_IOPORTS_WIDTH + PAL_PAD(led_col_pins[i]);
        uint8_t ch_idx    = pio_value % PWM_CHANNELS;
        chan_col_order[i] = ch_idx;
#    elif (SN32F2XX_PWM_DIRECTION == ROW2COL)
    for (uint8_t i = 0; i < SN32F2XX_RGB_MATRIX_ROWS_HW; i++) {
#        if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        // Only P0.0 to P2.15 can be used as pwm output
        if (led_row_pins[i] > C15) continue;
#        endif // SN32F2XX_PWM_CONTROL
        /* We use a trick here, according to pfpa table of sn32f240b datasheet,
           pwm channel and pfpa of pin Px.y can be calculated as below:
             channel = (x*16+y)%24
             pfpa = 1, when (x*16+y)>23
        */
        uint8_t pio_value = ((uint32_t)(PAL_PORT(led_row_pins[i])) - (uint32_t)(PAL_PORT(A0))) / ((uint32_t)(PAL_PORT(B0)) - (uint32_t)(PAL_PORT(A0))) * PAL_IOPORTS_WIDTH + PAL_PAD(led_row_pins[i]);
        uint8_t ch_idx    = pio_value % PWM_CHANNELS;
        chan_row_order[i] = ch_idx;
#    endif     // SN32F2XX_PWM_DIRECTION
#    if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        cfg->channels[ch_idx].pfpamsk = pio_value > (PWM_CHANNELS - 1);
        cfg->channels[ch_idx].mode    = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL;
#    endif // SN32F2XX_PWM_CONTROL
    }
}
#elif defined(SN32F260)
/* PWM configuration structure. We use timer CT16B1 with 23 channels. */
static const PWMConfig pwmcfg = {
    freq,         /* PWM clock frequency. */
    periodticks,  /* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
    rgb_callback, /* led Callback */
    .channels =
        {
            /* Default all channels to disabled */
            [0 ... PWM_CHANNELS - 1] = {.mode = PWM_OUTPUT_DISABLED},
/* Enable selected channels */
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_0)
            [0] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_0
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_1)
            [1] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_1
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_2)
            [2] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_2
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_3)
            [3] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_3
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_4)
            [4] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_4
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_5)
            [5] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_5
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_6)
            [6] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_6
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_7)
            [7] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_7
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_8)
            [8] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_8
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_9)
            [9] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_9
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_10)
            [10] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_10
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_11)
            [11] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_11
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_12)
            [12] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_12
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_13)
            [13] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_13
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_14)
            [14] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_14
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_15)
            [15] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_15
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_16)
            [16] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_16
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_17)
            [17] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_17
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_18)
            [18] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_18
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_19)
            [19] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_19
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_20)
            [20] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_20
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_21)
            [21] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_21
#    if defined(SN32F2XX_ACTIVATE_PWM_CHAN_22)
            [22] = {.mode = SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL},
#    endif // SN32F2XX_ACTIVATE_PWM_CHAN_22
        },
    0 /* HW dependent part.*/
};

static void rgb_ch_ctrl(void) {
    /* Enable PWM function, IOs and select the PWM modes for the LED pins */
#    if (SN32F2XX_PWM_DIRECTION == COL2ROW)
    for (uint8_t i = 0; i < SN32F2XX_RGB_MATRIX_COLS; i++) {
#        if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        // Only P0.0 to P0.15 and P3.0 to P3.8 can be used as pwm output
        if (led_col_pins[i] > A15 && led_col_pins[i] < D0) continue;
#        endif // SN32F2XX_PWM_CONTROL
        /* We use a trick here, according to pfpa table of sn32f260 datasheet,
           pwm channel and pfpa of pin Px.y can be calculated as below:
             channel = (x*16+y)%23
        */
        uint8_t pio_value = ((uint32_t)(PAL_PORT(led_col_pins[i])) - (uint32_t)(PAL_PORT(A0))) / ((uint32_t)(PAL_PORT(D0)) - (uint32_t)(PAL_PORT(A0))) * PAL_IOPORTS_WIDTH + PAL_PAD(led_col_pins[i]);
        uint8_t ch_idx    = pio_value % PWM_CHANNELS;
        chan_col_order[i] = ch_idx;
#    elif (SN32F2XX_PWM_DIRECTION == ROW2COL)
    for (uint8_t i = 0; i < SN32F2XX_RGB_MATRIX_ROWS_HW; i++) {
#        if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        // Only P0.0 to P0.15 and P3.0 to P3.8 can be used as pwm output
        if (led_row_pins[i] > A15 && led_row_pins[i] < D0) continue;
#        endif // SN32F2XX_PWM_CONTROL
        /* We use a trick here, according to pfpa table of sn32f260 datasheet,
           pwm channel and pfpa of pin Px.y can be calculated as below:
             channel = (x*16+y)%23
        */
        uint8_t pio_value = ((uint32_t)(PAL_PORT(led_row_pins[i])) - (uint32_t)(PAL_PORT(A0))) / ((uint32_t)(PAL_PORT(D0)) - (uint32_t)(PAL_PORT(A0))) * PAL_IOPORTS_WIDTH + PAL_PAD(led_row_pins[i]);
        uint8_t ch_idx    = pio_value % PWM_CHANNELS;
        chan_row_order[i] = ch_idx;
#    endif     // SN32F2XX_PWM_DIRECTION
    }
}
#else
#    error Unsupported MCU. Driver instance cant be configured.
#endif // chip selection

static void shared_matrix_rgb_enable(void) {
#if !defined(SN32F260)
    pwmcfg.callback = rgb_callback;
#endif // SN32F260 needs static allocation
    pwmEnablePeriodicNotification(&PWMD1);
}

#if defined(SHARED_MATRIX)
/* Per-row input instrumentation (weak; no-op on every other SN32 board).
 *
 * input_note_row_scan() runs in the row ISR on every key-row sample and is
 * where per-row timing lives (an ISR-safe tick read, no locks); the consume
 * hook runs in main-loop context and counts raw edges. Until 2026-09-03 this
 * driver published after scanning ONE row per main-loop consume, so `scan_rate`
 * -- which counts matrix_scan() calls -- was NOT a full-matrix refresh rate.
 * Every row is now sampled every cycle (see shared_matrix_scan_keys); the hooks
 * remain so the fix stays measurable. */
__attribute__((weak)) void input_note_row_scan(uint8_t row) { (void)row; }
__attribute__((weak)) void input_note_consume(const matrix_row_t *raw, const matrix_row_t *fresh, uint8_t rows) {
    (void)raw; (void)fresh; (void)rows;
}

/* Scan on EVERY key-row transition, straight into shared_matrix.
 *
 * WHAT THIS REPLACES: the old code gated scanning on `!matrix_scanned`, so at
 * most one row was sampled per main-loop consume. Which row that was depended
 * on the phase between the consume rate and the PWM row cycle -- an ALIASING
 * BEAT of 1/|1041.667 - 3*f_consume|, which near resonance is enormous. Measured
 * on hardware 2026-09-03: a key went 156-169 ms without being sampled on an idle
 * board, with the main loop running perfectly and every stall counter at zero.
 * A keypress lasts 25-80 ms, so presses fell into that window and were lost
 * before debounce ever saw them. See docs/hardware.md and
 * plans/MATRIX-PUBLISH-PLAN.md.
 *
 * NOW: the key row advances every third PWM ISR, so every row is sampled once
 * per 18-slot cycle regardless of the consume rate or its phase. The timer
 * arithmetic (4.8 MHz / 256 ticks) says 18,750 ISR/s and ~1042 samples/s per
 * row; MEASURED is ~3,870 ISR/s and ~215 samples/s per row (4.6 ms between
 * looks at any key, down from up to 169 ms). The gap is the ISR's own duration:
 * rgb_callback() re-arms the counter at its END, so its period is ISR time plus
 * ~53 us, not 53 us -- see the sn32f2xx_isr_*_hook() instrumentation and
 * docs/hardware.md. matrix_scanned becomes a DIRTY FLAG rather than a gate; the
 * main loop's compare/copy/clear runs under chSysLock() so the ISR cannot land
 * mid-copy (same pattern as sn32f2xx_flush() below).
 *
 * The old matrix_locked/first_scanned interlock existed to keep row coverage
 * fair under the one-row scheme. Every row is now sampled every cycle, so it is
 * dead and removed rather than left to disagree with this. */
static void shared_matrix_scan_keys(matrix_row_t current_matrix[], uint8_t current_key, uint8_t last_key) {
    /* Bit per row; suppress readiness until every row has been sampled at least
     * once, so the main loop never consumes uninitialised rows after boot. */
    static uint8_t rows_seen = 0;
#    define SHARED_MATRIX_ALL_ROWS ((uint8_t)((1u << ROWS_PER_HAND) - 1u))

    if (last_key != current_key) {   /* a genuinely new row is energised */
        {
#    if (DIODE_DIRECTION == COL2ROW)
#        if (SN32F2XX_PWM_DIRECTION == DIODE_DIRECTION)
            matrix_read_cols_on_row(current_matrix, current_key);
#        else
            // For each row...
            for (uint8_t row_index = 0; row_index < ROWS_PER_HAND; row_index++) {
                matrix_read_cols_on_row(current_matrix, row_index);
            }

#        endif // DIODE_DIRECTION == SN32F2XX_PWM_DIRECTION
#    elif (DIODE_DIRECTION == ROW2COL)
#        if (SN32F2XX_PWM_DIRECTION == DIODE_DIRECTION)
            matrix_read_rows_on_col(current_matrix, current_key, row_shifter);
#        else
            // For each col...
            matrix_row_t row_shifter = MATRIX_ROW_SHIFTER;
            for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++, row_shifter <<= 1) {
                matrix_read_rows_on_col(current_matrix, current_key, row_shifter);
            }
#        endif // SN32F2XX_PWM_DIRECTION
#    endif     // DIODE_DIRECTION
            input_note_row_scan(current_key);
        }
        if (rows_seen != SHARED_MATRIX_ALL_ROWS) {
            rows_seen |= (uint8_t)(1u << current_key);
            if (rows_seen != SHARED_MATRIX_ALL_ROWS) return;   /* not complete yet */
        }
        matrix_scanned = true;   /* dirty flag: something new since last consume */
    }
#    undef SHARED_MATRIX_ALL_ROWS
}
#endif // SHARED_MATRIX

#if (SN32F2XX_PWM_DIRECTION == COL2ROW)

static void shared_matrix_rgb_disable_output(void) {
    // Disable PWM outputs on column pins
    for (uint8_t y = 0; y < SN32F2XX_RGB_MATRIX_COLS; y++) {
#    if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        pwmDisableChannel(COL_PWM_DRV(y), COL_PWM_CH(y));
#    elif (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
        gpio_set_pin_input(led_col_pins[y]);
#    endif // SN32F2XX_PWM_CONTROL
    }
    // Disable LED outputs on RGB channel pins
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_low(led_row_pins[x]);
#    elif (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_high(led_row_pins[x]);
#    endif // SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL
    }
}

static void update_pwm_channels(PWMDriver *pwmp) {
    // Advance to the next LED RGB channels
    current_row++;
    /* Check if counter has wrapped around, reset before the next pass */
    if (current_row == SN32F2XX_RGB_MATRIX_ROWS_HW) current_row = 0;
#    if defined(SHARED_MATRIX)
    uint8_t last_key_row = current_key_row;
#    endif // SHARED_MATRIX
    // Advance to the next key matrix row
#    if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
    if (current_row % SN32F2XX_RGB_MATRIX_ROW_CHANNELS == 2) current_key_row++;
#    elif (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
    if (current_row % SN32F2XX_RGB_MATRIX_ROW_CHANNELS == 0) current_key_row++;
#    endif // SN32F2XX_PWM_CONTROL
    /* Check if counter has wrapped around, reset before the next pass */
    if (current_key_row == SN32F2XX_RGB_MATRIX_ROWS) current_key_row = 0;
    // Disable LED output before scanning the key matrix
    if (current_key_row < ROWS_PER_HAND) {
        shared_matrix_rgb_disable_output();
#    if defined(SHARED_MATRIX)
        shared_matrix_scan_keys(shared_matrix, current_key_row, last_key_row);
#    endif // SHARED_MATRIX
    }
    bool enable_pwm_output = false;
    for (uint8_t current_key_col = 0; current_key_col < SN32F2XX_RGB_MATRIX_COLS; current_key_col++) {
        uint8_t led_index = g_led_config.matrix_co[current_key_row][current_key_col];
#    if (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
        if (led_index >= SN32F2XX_LED_COUNT) continue;
#    endif // SN32F2XX_PWM_CONTROL
        // Check if we need to enable RGB output
        if (led_state[led_index].b > 0) enable_pwm_output |= true;
        if (led_state[led_index].g > 0) enable_pwm_output |= true;
        if (led_state[led_index].r > 0) enable_pwm_output |= true;
        // Update matching RGB channel PWM configuration
#    if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        switch (current_row % SN32F2XX_RGB_MATRIX_ROW_CHANNELS) {
            case 0:
                pwmEnableChannel(COL_PWM_DRV(current_key_col), COL_PWM_CH(current_key_col), led_state[led_index].b);
                break;
            case 1:
                pwmEnableChannel(COL_PWM_DRV(current_key_col), COL_PWM_CH(current_key_col), led_state[led_index].g);
                break;
            case 2:
                pwmEnableChannel(COL_PWM_DRV(current_key_col), COL_PWM_CH(current_key_col), led_state[led_index].r);
                break;
            default:;
        }
#    elif (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
        switch (current_row % SN32F2XX_RGB_MATRIX_ROW_CHANNELS) {
            case 0:
                led_duty_cycle[current_key_col] = led_state[led_index].r;
                break;
            case 1:
                led_duty_cycle[current_key_col] = led_state[led_index].b;
                break;
            case 2:
                led_duty_cycle[current_key_col] = led_state[led_index].g;
                break;
            default:;
        }
#    endif
    }
    // Enable RGB output
    if (enable_pwm_output) {
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_high(led_row_pins[current_row]);
#    elif (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_low(led_row_pins[current_row]);
#    endif
    }
}
#elif (SN32F2XX_PWM_DIRECTION == ROW2COL)

static void shared_matrix_rgb_disable_output(void) {
    // Disable LED outputs on RGB channel pins
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_COLS; x++) {
#    if (DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION)
        gpio_set_pin_input(led_col_pins[x]);
#    endif // DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION
        // Unselect all columns before scanning the key matrix
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_high(led_col_pins[x]);
#    elif (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_low(led_col_pins[x]);
#    endif
    }
#    if (DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION)
    // Disable PWM outputs on row pins
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
#        if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        pwmDisableChannel(&PWMD1, chan_row_order[x]);
#        endif // SN32F2XX_PWM_CONTROL
#        if (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_low(led_row_pins[x]);
#        elif (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_high(led_row_pins[x]);
#        endif // SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL
    }
#    endif     // DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION
}

static void update_pwm_channels(PWMDriver *pwmp) {
    /* Advance to the next LED RGB channel and get ready for the next pass */
    last_key_col = current_key_col;
    current_key_col++;
    /* Check if counter has wrapped around, reset before the next pass */
#    if (DIODE_DIRECTION == ROW2COL)
    if (current_key_col < MATRIX_COLS) row_shifter <<= 1;
    if (current_key_col == MATRIX_COLS) row_shifter = MATRIX_ROW_SHIFTER;
#    endif // DIODE_DIRECTION == ROW2COL
    if (current_key_col == SN32F2XX_RGB_MATRIX_COLS) current_key_col = 0;
    // Disable LED output before scanning the key matrix
    if (current_key_col < MATRIX_COLS) {
        shared_matrix_rgb_disable_output();
#    if defined(SHARED_MATRIX)
        shared_matrix_scan_keys(shared_matrix, current_key_col, last_key_col);
#    endif // SHARED_MATRIX
    }

    bool enable_pwm_output = false;
    for (uint8_t current_key_row = 0; current_key_row < MATRIX_ROWS; current_key_row++) {
        uint8_t led_index = g_led_config.matrix_co[current_key_row][current_key_col];
#    if (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
        if (led_index >= SN32F2XX_LED_COUNT) continue;
#    endif
        uint8_t led_row_id = (current_key_row * SN32F2XX_RGB_MATRIX_ROW_CHANNELS);
        // Check if we need to enable RGB output
        if (led_state[led_index].b > 0) enable_pwm_output |= true;
        if (led_state[led_index].g > 0) enable_pwm_output |= true;
        if (led_state[led_index].r > 0) enable_pwm_output |= true;
        // Update matching RGB channel PWM configuration
#    if (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 0)], led_state[led_index].r);
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 1)], led_state[led_index].b);
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 2)], led_state[led_index].g);
    }
    // Enable RGB output
    if (enable_pwm_output) {
        gpio_set_pin_output_push_pull(led_col_pins[last_key_col]);
#        if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_high(led_col_pins[last_key_col]);
#        elif (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_low(led_col_pins[last_key_col]);
#        endif // SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL
    }
#    elif (SN32F2XX_PWM_CONTROL == SOFTWARE_PWM)
        led_duty_cycle[(led_row_id + 0)] = led_state[led_index].r;
        led_duty_cycle[(led_row_id + 1)] = led_state[led_index].b;
        led_duty_cycle[(led_row_id + 2)] = led_state[led_index].g;
    }
    // Enable RGB output
    if (enable_pwm_output) {
        gpio_set_pin_output_push_pull(led_col_pins[last_key_col]);
#        if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_high(led_col_pins[current_key_col]);
#        elif (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW)
        gpio_write_pin_low(led_col_pins[current_key_col]);
#        endif // SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL
    }
#    endif     // SN32F2XX_PWM_CONTROL
}
#endif         // SN32F2XX_PWM_DIRECTION == ROW2COL

/* Row-ISR cost instrumentation (weak; no-op elsewhere). The counter is re-armed
 * at the END of rgb_callback(), so the ISR's own duration adds to its period
 * and sets the real row rate. These hooks let a board measure that instead of
 * deriving it from the timer configuration -- the two differ by ~4.8x on the
 * AK820 Pro. Cost when implemented: two tick reads and a few adds per ISR. */
__attribute__((weak)) void sn32f2xx_isr_enter_hook(void) {}
__attribute__((weak)) void sn32f2xx_isr_exit_hook(void) {}

/* Scan-only (a board's light sleep, sn32f2xx_set_scan_only): no LED work, one
 * key row per ISR at a longer period, on its own row counter so the LED field's
 * row pairing is untouched and resumes where it was. ISR-owned but for the
 * request flag. */
#ifndef SN32F2XX_SCAN_ONLY_TICKS
#    define SN32F2XX_SCAN_ONLY_TICKS 3000u   /* 0.625 ms at 4.8 MHz, plus the body: each key every ~3.9 ms,
                                               * under sym_defer_pk's 5 ms so debounce sees a fresh sample */
#endif
static volatile bool scan_only_req = false;
static bool          scan_only     = false;
static uint8_t       scan_only_row = 0;

void sn32f2xx_set_scan_only(bool on) {
    scan_only_req = on;
}

bool sn32f2xx_scan_only_active(void) {
    return scan_only;
}

static void rgb_callback(PWMDriver *pwmp) {
    sn32f2xx_isr_enter_hook();
    // Disable the interrupt
    pwmDisablePeriodicNotification(pwmp);
#if ((SN32F2XX_PWM_CONTROL == SOFTWARE_PWM) && (SN32F2XX_PWM_DIRECTION == COL2ROW))
    for (uint8_t pwm_cnt = 0; pwm_cnt < (SN32F2XX_RGB_MATRIX_COLS * RGB_MATRIX_HUE_STEP); pwm_cnt++) {
        uint8_t pwm_index = (pwm_cnt % SN32F2XX_RGB_MATRIX_COLS);
        if (((uint16_t)SN32_CT_PWM_GET(pwmp, config.TC) < ((uint16_t)(led_duty_cycle[pwm_index] + periodticks))) && (led_duty_cycle[pwm_index] > 0)) {
            gpio_set_pin_output_push_pull(led_col_pins[pwm_index]);
#    if (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_LOW)
            gpio_write_pin_low(led_col_pins[pwm_index]);
        } else {
            gpio_set_pin_input_high(led_col_pins[pwm_index]);
#    elif (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_HIGH)
            gpio_write_pin_high(led_col_pins[pwm_index]);
        } else {
            gpio_set_pin_input_low(led_col_pins[pwm_index]);
#    endif // SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL
        }
    }
#elif ((SN32F2XX_PWM_CONTROL == SOFTWARE_PWM) && (SN32F2XX_PWM_DIRECTION == ROW2COL))
    for (uint8_t pwm_cnt = 0; pwm_cnt < (SN32F2XX_RGB_MATRIX_ROWS_HW * RGB_MATRIX_HUE_STEP); pwm_cnt++) {
        uint8_t pwm_index = (pwm_cnt % SN32F2XX_RGB_MATRIX_ROWS_HW);
        if (((uint16_t)SN32_CT_PWM_GET(pwmp, config.TC) < ((uint16_t)(led_duty_cycle[pwm_index] + periodticks))) && (led_duty_cycle[pwm_index] > 0)) {
#    if (DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION)
            gpio_set_pin_output_push_pull(led_row_pins[pwm_index]);
#    endif // DIODE_DIRECTION != SN32F2XX_PWM_DIRECTION

#    if (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_LOW)
            gpio_write_pin_low(led_row_pins[pwm_index]);
        } else {
            gpio_write_pin_high(led_row_pins[pwm_index]);
#    elif (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_HIGH)
            gpio_write_pin_high(led_row_pins[pwm_index]);
        } else {
            gpio_write_pin_low(led_row_pins[pwm_index]);
#    endif // SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL
        }
    }
#endif
#if defined(SHARED_MATRIX) && (SN32F2XX_PWM_DIRECTION == COL2ROW) && (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
    /* Scan-only: the switch is applied here, between periods. Entering blanks
     * every output once (the columns stay released from PWM, so they read as
     * the inputs every scan needs) and lengthens the period; leaving restores
     * the field's period, and the next update_pwm_channels() drives it again. */
    if (scan_only_req != scan_only) {
        scan_only = scan_only_req;
        pwmChangePeriodI(pwmp, scan_only ? SN32F2XX_SCAN_ONLY_TICKS : periodticks);
        if (scan_only) shared_matrix_rgb_disable_output();
    }
    if (scan_only) {
        if (EFLD1.state != FLASH_PGM) {   /* as below: no scan inside a flash program */
            uint8_t last = scan_only_row;
            if (++scan_only_row >= ROWS_PER_HAND) scan_only_row = 0;
            shared_matrix_scan_keys(shared_matrix, scan_only_row, last);
        }
    } else
#endif
    // Scan the rgb and key matrix
    if (EFLD1.state != FLASH_PGM) {
        update_pwm_channels(pwmp);
    } else {
        /* A flash program is in progress. The old guard merely SKIPPED the
         * advance, which left the currently-selected mux row energized --
         * and the masked program windows then freeze it there for the whole
         * multi-ms operation, so that row shows its live colour slot at
         * ~18x normal brightness: the long-standing "row of LEDs flashes
         * red/green while adjusting" artifact (reproduced on demand
         * 2026-09-01 by firing EEPROM writes and watching the matrix).
         * De-select every mux pin instead -- plain GPIO writes, ISR-safe
         * (sn32f2xx_blank() is NOT: it calls non-I-class pwmDisableChannel)
         * -- so the frozen state is dark. The first tick after the
         * operation re-drives the mux via update_pwm_channels. */
#if (SN32F2XX_PWM_DIRECTION == COL2ROW)
        for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
            gpio_write_pin_low(led_row_pins[x]);
#    else
            gpio_write_pin_high(led_row_pins[x]);
#    endif
        }
#else /* ROW2COL: the column pins are the mux select */
        for (uint8_t y = 0; y < SN32F2XX_RGB_MATRIX_COLS; y++) {
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
            gpio_write_pin_low(led_col_pins[y]);
#    else
            gpio_write_pin_high(led_col_pins[y]);
#    endif
        }
#endif
    }
    sn32f2xx_isr_exit_hook();
    chSysLockFromISR();
    // Advance the timer to just before the wrap-around, that will start a new PWM cycle
    pwm_lld_change_counter(pwmp, UINT16_MAX);
#if defined(SN32F2XX_PWM_MULTI_TIMER)
    // Re-arm the auxiliary PWM timers on the same boundary as PWMD1. This is what
    // actually keeps their hardware PWM phase-locked to the row scan every period;
    // without it they free-run and drift (irregular flicker + one-phase colour
    // shift). Both share PWMD1's clock and period, so a co-located re-arm holds them
    // in lockstep.
    pwm_lld_change_counter(&PWMD0, UINT16_MAX);
    pwm_lld_change_counter(&PWMD2, UINT16_MAX);
#endif
    // Enable the interrupt
    pwmEnablePeriodicNotificationI(pwmp);
    chSysUnlockFromISR();
}

#if (SN32F2XX_PWM_DIRECTION == COL2ROW) && (SN32F2XX_PWM_CONTROL == HARDWARE_PWM)
/* Deep sleep (a board's experiment): every output off and the row ISR's
 * notification off, the columns left released from PWM (inputs with their
 * pull-ups, as for a scan); then put back, the counters re-armed on one
 * boundary as the ISR's end does. I-class: called with interrupts locked, so
 * no lock call (pwmDisableChannel would unlock on its way out). */
void sn32f2xx_sleep_enterI(void) {
    pwmDisablePeriodicNotificationI(&PWMD1);
    for (uint8_t y = 0; y < SN32F2XX_RGB_MATRIX_COLS; y++) {
        pwmDisableChannelI(COL_PWM_DRV(y), COL_PWM_CH(y));
    }
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
#    if (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH)
        gpio_write_pin_low(led_row_pins[x]);
#    else
        gpio_write_pin_high(led_row_pins[x]);
#    endif
    }
}

void sn32f2xx_sleep_exitI(void) {
    pwm_lld_change_counter(&PWMD1, UINT16_MAX);
#    if defined(SN32F2XX_PWM_MULTI_TIMER)
    pwm_lld_change_counter(&PWMD0, UINT16_MAX);
    pwm_lld_change_counter(&PWMD2, UINT16_MAX);
#    endif
    pwmEnablePeriodicNotificationI(&PWMD1);
}
#endif

void sn32f2xx_init(void) {
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
        gpio_set_pin_output_push_pull(led_row_pins[x]);
#if ((SN32F2XX_PWM_DIRECTION == COL2ROW) && (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_HIGH) || (SN32F2XX_PWM_DIRECTION == ROW2COL) && (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_HIGH))
        gpio_write_pin_low(led_row_pins[x]);
#elif ((SN32F2XX_PWM_DIRECTION == COL2ROW) && (SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL == SN32F2XX_RGB_OUTPUT_ACTIVE_LOW) || (SN32F2XX_PWM_DIRECTION == ROW2COL) && (SN32F2XX_PWM_OUTPUT_ACTIVE_LEVEL == SN32F2XX_PWM_OUTPUT_ACTIVE_LOW))
        gpio_write_pin_high(led_row_pins[x]);
#endif // SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL
    }

    // Determine which PWM channels we need to control
#if (defined(SN32F240B) || defined(SN32F240C) || defined(SN32F290))
    rgb_ch_ctrl(&pwmcfg);
#elif defined(SN32F260)
    rgb_ch_ctrl();
#endif // chip selection

#if defined(SHARED_MATRIX)
    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        shared_matrix[i] = 0;
    }
#endif // SHARED_MATRIX

#if defined(SN32F2XX_PWM_MULTI_TIMER)
    /* Program the pin-mux for the columns spread across CT16B0/B1/B2. The three
       constants are board-specific (see docs/HARDWARE_PWM.md). */
#    if defined(SN32F2XX_PWM_PFPA_CT16B0)
    SN_PFPA->CT16B0 = SN32F2XX_PWM_PFPA_CT16B0;
#    endif
#    if defined(SN32F2XX_PWM_PFPA_CT16B1)
    SN_PFPA->CT16B1 = SN32F2XX_PWM_PFPA_CT16B1;
#    endif
#    if defined(SN32F2XX_PWM_PFPA_CT16B2)
    SN_PFPA->CT16B2 = SN32F2XX_PWM_PFPA_CT16B2;
#    endif
    /* All three timers share HCLK. Only PWMD1 drives the row-advance callback,
       which reloads the duty of every column (across all three timers) once per
       period. For that to land in the right PWM cycle on PWMD0/PWMD2 they must be
       phase-locked to PWMD1. */
    pwmStart(&PWMD0, &pwmcfg_b0);
    pwmStart(&PWMD1, &pwmcfg);
    pwmStart(&PWMD2, &pwmcfg_b2);
    /* pwmStart() launches the counters a few instructions apart, leaving a fixed
       phase offset; the PWMD1 ISR then writes PWMD0/PWMD2 duties mid-cycle, which
       glitches (irregular flicker + one-phase colour shift). Reset all three
       counters together so they share a cycle boundary; identical periods on a
       shared clock keep them locked afterwards. */
    chSysLock();
    pwm_lld_change_counter(&PWMD0, 0);
    pwm_lld_change_counter(&PWMD1, 0);
    pwm_lld_change_counter(&PWMD2, 0);
    chSysUnlock();
#else
    pwmStart(&PWMD1, &pwmcfg);
#endif // SN32F2XX_PWM_MULTI_TIMER
    shared_matrix_rgb_enable();
}

/* Blank the matrix and leave it blank.
 *
 * The row scan is a MUX: exactly one of the 18 row slots is energised at any
 * instant, and the ISR moves it on. Stop the ISR -- by masking interrupts, or
 * by the CPU stalling on an internal-flash write -- and whichever row and
 * channel happened to be live stays live, at full duty instead of its 1/18
 * share. That is the stuck bright row seen during flash writes, and after
 * jumping to the ROM bootloader it is permanent.
 *
 * Callers that are about to stop the ISR should call this FIRST. It is the
 * driver's own teardown, exported: outputs off on every column and every row
 * channel, so no slot can be left driving. */
void sn32f2xx_blank(void) {
#if defined(SHARED_MATRIX)
    shared_matrix_rgb_disable_output();
#endif
    /* Then force every LED pin to HIGH-IMPEDANCE.
     *
     * The driver's own teardown above writes row pins to an inactive LEVEL,
     * which depends on SN32F2XX_RGB_OUTPUT_ACTIVE_LEVEL -- a define this board
     * never sets, so it takes a default that may not match the hardware. If it
     * is wrong, "blanking" drives rows to the ACTIVE state instead, which is
     * how a row stayed lit after the mux was stopped.
     *
     * High-Z sidesteps the question: an input pin cannot source or sink, so
     * there is no current path through any LED regardless of polarity. Only
     * safe when the scan is being stopped for good -- which is the one caller
     * here, on the way into the ROM bootloader. */
    for (uint8_t y = 0; y < SN32F2XX_RGB_MATRIX_COLS; y++) {
        gpio_set_pin_input(led_col_pins[y]);
    }
    for (uint8_t x = 0; x < SN32F2XX_RGB_MATRIX_ROWS_HW; x++) {
        gpio_set_pin_input(led_row_pins[x]);
    }
}

/* The committed frame's drive, time-weighted (sn32f2xx_led_drive_take). The
 * sum is taken after the masked copy, not inside it: only this function
 * writes led_state[], and the ISR only reads it. */
static uint32_t drive_now   = 0;   /* the committed frame's channel sum */
static uint32_t drive_since = 0;   /* when it was committed, ms */
static uint64_t drive_acc   = 0;   /* sum x ms since the last take */
static uint32_t drive_lit   = 0;   /* ms with any channel lit, since the last take */

static void drive_close(uint32_t now) {
    uint32_t dt = now - drive_since;
    drive_acc += (uint64_t)drive_now * dt;
    if (drive_now) drive_lit += dt;
    drive_since = now;
}

uint64_t sn32f2xx_led_drive_take(uint32_t now_ms, uint32_t *lit_ms) {
    drive_close(now_ms);
    uint64_t r = drive_acc;
    drive_acc  = 0;
    if (lit_ms) *lit_ms = drive_lit;
    drive_lit = 0;
    return r;
}

void sn32f2xx_flush(void) {
    if (led_state_buf_update_required) {
        /* The row ISR reads led_state[] to set each channel's duty, and this
         * memcpy runs on the MAIN LOOP -- so without a lock the ISR can land
         * mid-copy and drive a frame that is part new and part old. The split
         * falls at whatever byte was reached, which shows on the panel as a
         * single row briefly lit in a wrong colour. Visible while sweeping hue,
         * where consecutive frames differ enough to notice.
         *
         * Masking for the duration is cheap and bounded: ~10-16 us for a
         * 82-128 LED copy at 48 MHz, against a 53 us row-ISR period at
         * 18,750/s. Worst case the current row is armed a few microseconds
         * late, which is invisible; a torn frame is not. */
        chSysLock();
        memcpy(led_state, led_state_buf, sizeof(RGB) * SN32F2XX_LED_COUNT);
        led_state_buf_update_required = false;
        chSysUnlock();
        uint32_t sum = 0;
        for (int i = 0; i < SN32F2XX_LED_COUNT; i++) {
            sum += (uint32_t)led_state[i].r + led_state[i].g + led_state[i].b;
        }
        drive_close(timer_read32());
        drive_now = sum;
    }
}

/* Board power scale, 255 = as requested. Applied here, below the rgb_matrix
 * config, so a board can dim or darken the matrix to save power without
 * touching the user's brightness (or its EEPROM copy). No divide: this runs
 * for every channel of every LED on every frame, on a core without one. */
static uint8_t power_scale = 255;

void sn32f2xx_set_power_scale(uint8_t scale) {
    power_scale = scale;
}

void sn32f2xx_set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t color_r = r * SN32F2XX_LED_OUTPUT_LUMINOSITY_R;
    uint8_t color_g = g * SN32F2XX_LED_OUTPUT_LUMINOSITY_G;
    uint8_t color_b = b * SN32F2XX_LED_OUTPUT_LUMINOSITY_B;
    if (power_scale != 255) {
        color_r = (uint8_t)(((uint16_t)color_r * (power_scale + 1u)) >> 8);
        color_g = (uint8_t)(((uint16_t)color_g * (power_scale + 1u)) >> 8);
        color_b = (uint8_t)(((uint16_t)color_b * (power_scale + 1u)) >> 8);
    }

#ifdef UNDERGLOW_RBG
    bool flip_gb = false;
    for (uint8_t led_id = 0; led_id < UNDERGLOW_LEDS; led_id++) {
        if (underglow_leds[led_id] == index) {
            flip_gb = true;
        }
    }
    if (flip_gb) {
        if (led_state_buf[index].r == color_r && led_state_buf[index].b == color_g && led_state_buf[index].g == color_b) {
            return;
        }

        led_state_buf[index].r        = color_r;
        led_state_buf[index].b        = color_g;
        led_state_buf[index].g        = color_b;
        led_state_buf_update_required = true;
    } else {
#endif // UNDERGLOW_RBG
        if (led_state_buf[index].r == color_r && led_state_buf[index].b == color_b && led_state_buf[index].g == color_g) {
            return;
        }

        led_state_buf[index].r        = color_r;
        led_state_buf[index].b        = color_b;
        led_state_buf[index].g        = color_g;
        led_state_buf_update_required = true;
#ifdef UNDERGLOW_RBG
    }
#endif // UNDERGLOW_RBG
}

void sn32f2xx_set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < SN32F2XX_LED_COUNT; i++) {
        sn32f2xx_set_color(i, r, g, b);
    }
}

uint32_t sn32f2xx_led_load(void) {
    uint32_t sum = 0;
    for (int i = 0; i < SN32F2XX_LED_COUNT; i++) {
        sum += (uint32_t)led_state_buf[i].r + led_state_buf[i].g + led_state_buf[i].b;
    }
    return sum;
}

#if defined(SHARED_MATRIX)
bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    if (!matrix_scanned) return false; // Nothing to process until we have the matrix scanned

    /* Instrumentation first and UNLOCKED on purpose: it reads the timer, which
     * must not be called inside chSysLock(). A torn read here only skews a
     * counter, never behaviour. */
    input_note_consume(raw_matrix, shared_matrix, MATRIX_ROWS);

    /* The ISR now writes shared_matrix on every row transition (~6250/s), so the
     * compare/copy must exclude it or the loop can consume a matrix that is part
     * new and part old -- the same hazard sn32f2xx_flush() locks against. */
    chSysLock();
    bool changed = memcmp(raw_matrix, shared_matrix, sizeof(shared_matrix)) != 0;
    if (changed) memcpy(raw_matrix, shared_matrix, sizeof(shared_matrix));
    matrix_scanned = false;
    chSysUnlock();

    return changed;
}
#endif // SHARED_MATRIX