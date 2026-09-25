// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Battery: VDD from the SN32's own ADC, a level estimate built on it, the
 * low-battery protection, and a once-a-minute log. See battery.h.
 *
 * Why this exists: the CH582F's `5C` battery percentage held 100 through a
 * whole discharge at full RGB brightness, until the board browned out, and
 * still read 100 six minutes into recharging the flat pack (2026-09-24). It is
 * not a level. It is not nothing either -- a board with no pack fitted reads 0
 * (indicators.c) -- so the module does sense the pack, but its number
 * saturates across the range we have seen.
 *
 * The SN32 cannot see the pack through a pin. AIN0..AIN15 are P2.0..P2.15 and
 * every one is a matrix column, an RGB row or the Win Lock LED; the op-amp and
 * comparator inputs are RGB rows, the flash's WP, the encoder and the BOOT pin.
 * The stock firmware never touches the ADC, the comparator or the op-amps
 * either (no reference to their base addresses in v1.13 or v1.14, nor in seven
 * sibling SN32F290 boards): stock's battery gauge was the module's number.
 * What the SN32 can see is its own VDD, by converting the internal 2.0 V
 * reference (AIN16) against VDD as the reference high.
 *
 * And VDD is REGULATED on battery. Measured 2026-09-24/25 with a meter at the
 * pack connector: pack 4.18 V, VDD 3.90 V; then the pack fell to 4.13 V over
 * 50 minutes at full RGB while VDD did not move off 3.90. So VDD says nothing
 * while the pack is above the regulator's output plus its dropout, and follows
 * the pack below that. Hence the estimator below.
 *
 * Accuracy: the 2.0 V reference is 1.96-2.04 V, so the absolute reading is
 * good to +-2% (about +-80 mV at 4 V). The error is a fixed gain per part.
 *
 * Cost: one conversion per 10 Hz tick, ~11 us with interrupts masked. The mask
 * is caution, not a known need: with the global channel enabled the datasheet
 * hands "the ADC shared pins" to the ADC, and those are the matrix and RGB
 * pins. It is almost certainly only the selected pin, and AIN16 has none, but
 * masking keeps the row ISR from ever sampling the matrix inside that window. */
#include "quantum.h"
#include "battery.h"
#include "indicators.h"
#include "bluetooth/ch582f_ajazz.h"
#include "graphics/display.h"
#include "drivers/led/sn32f2xx.h"

/* ADC_ADM fields (SN32F290 datasheet 7.5.1). Written as a whole word so no
 * read-modify-write carries a stale EOC or ADS back into the register. */
#define ADM_CHS_AIN16   16u          /* internal reference channel, no pin */
#define ADM_GCHS        (1u << 5)    /* global channel enable */
#define ADM_EOC         (1u << 6)    /* end of conversion */
#define ADM_ADS         (1u << 7)    /* start */
#define ADM_ADLEN       (1u << 8)    /* 12-bit */
#define ADM_ADCKS(n)    ((uint32_t)(n) << 9)
#define ADM_ADENB       (1u << 12)
#define ADM_VHS(n)      ((uint32_t)(n) << 14)
#define VHS_VDD_2V0_AIN16 4u         /* VDD is the reference high; 2.0 V on AIN16 */

/* ADC_PCLK must stay under 16 MHz: HCLK 48 MHz / 4 = 12 MHz. ADCKS /2 then
 * gives a 6 MHz ADC clock and a ~11 us 12-bit conversion. */
#define ADC_PCLK_PRE_DIV4 2u
#define ADC_ADCKS_DIV2    1u

#define ADM_BASE (ADM_VHS(VHS_VDD_2V0_AIN16) | ADM_ADENB | ADM_ADCKS(ADC_ADCKS_DIV2) | \
                  ADM_ADLEN | ADM_CHS_AIN16)

#define REF_MV        2000u
#define ADC_FULLSCALE 4096u
#define EOC_SPINS     2000u          /* ~10x the conversion; a timeout reads 0 */

/* Plausibility window for one sample. VDD is specified 2.5-5.5 V; anything
 * outside is a bad conversion, not a reading. */
#define VDD_MIN_MV 2000u
#define VDD_MAX_MV 6000u

/* EMA in mV * 64, alpha 1/16: a ~1.6 s time constant at 10 Hz, which smooths
 * the RGB field's load ripple without hiding a real sag. */
#define EMA_SHIFT 4
#define EMA_SCALE 64
/* A USB <-> battery handover moves VDD by ~0.6 V at once, and sliding the EMA
 * across it walked the level bar through every value in between. Three
 * samples in a row this far from the filter reseed it instead. */
#define SNAP_MV      300u
#define SNAP_SAMPLES 3u

/* Above this VDD the board is on USB power: 4.47 V in BT with the cable in,
 * 4.79 V in the cable position, while a lithium cell tops out at 4.2 V. */
#define ON_USB_MIN_MV 4300u

/* Status bits (HC_CONN byte 23 and each log entry). */
#define BATT_F_ON_BATTERY 0x01u   /* VDD says the pack is powering the board */
#define BATT_F_CHARGING   0x02u   /* charger CHRG pin active (low) */
#define BATT_F_DONE       0x04u   /* charger STDBY/DONE pin active (low) */
#define BATT_F_RGB_ON     0x08u
#define BATT_F_LIGHTS_CUT 0x10u   /* the low-battery protection turned RGB off */
#define BATT_F_SLIDER_USB 0x20u   /* mode slider in the cable position */
#define BATT_F_LINKED     0x40u   /* module reports a live connection */
#define BATT_F_CLAMPED    0x80u   /* on battery with VDD at the regulator's output */

static bool     adc_ready  = false;
static uint16_t last_raw   = 0;
static uint32_t ema_x64    = 0;      /* 0 = no sample yet */
static uint8_t  snap_run   = 0;
static uint16_t vdd_min    = 0;
static uint16_t vdd_max    = 0;
static uint16_t shown_cv   = 0;      /* battery_display_cv()'s held value */

void battery_init(void) {
    sys1SelectADCPRE(ADC_PCLK_PRE_DIV4);
    sys1EnableADC();
    /* A prescale change takes effect through a peripheral reset. */
    sys1ResetADC();
    SN_ADC->ADM = ADM_BASE;
    wait_us(150);   /* 100 us minimum after ADENB before the first conversion */
    adc_ready = true;
}

static uint16_t adc_read_ain16(void) {
    uint32_t spins = 0;
    uint16_t code  = 0;
    chSysLock();
    SN_ADC->ADM = ADM_BASE | ADM_GCHS;
    SN_ADC->ADM = ADM_BASE | ADM_GCHS | ADM_ADS;
    while (!(SN_ADC->ADM & ADM_EOC) && ++spins < EOC_SPINS) {
    }
    if (spins < EOC_SPINS) code = (uint16_t)(SN_ADC->ADB & 0xFFFu);
    SN_ADC->ADM = ADM_BASE;
    chSysUnlock();
    return code;
}

uint16_t battery_vdd_mv(void) {
    return (uint16_t)((ema_x64 + EMA_SCALE / 2u) / EMA_SCALE);
}

uint16_t battery_vdd_raw(void)    { return last_raw; }
uint16_t battery_vdd_min_mv(void) { return vdd_min; }
uint16_t battery_vdd_max_mv(void) { return vdd_max; }

bool battery_on_battery(void) {
    uint16_t mv = battery_vdd_mv();
    return mv != 0 && mv < ON_USB_MIN_MV;
}

uint16_t battery_display_cv(void) {
    uint16_t mv = battery_vdd_mv();
    if (mv == 0) return 0;
    /* Move the shown value only when the filtered reading is 8 mV past it: a
     * glyph repaint per millivolt of ripple is LCD traffic for nothing. */
    int32_t diff = (int32_t)mv - (int32_t)shown_cv * 10;
    if (shown_cv == 0 || diff >= 8 || diff <= -8) shown_cv = (uint16_t)((mv + 5u) / 10u);
    return shown_cv;
}

/* --- Pack voltage to level ------------------------------------------------------
 * A generic lithium-ion (LCO/NMC) rest-voltage curve, applied to the PACK
 * voltage (VDD + dropout below the clamp). CAL: replace with the measured
 * discharge once the 2026-09-25 run is in. */
static const uint16_t curve_mv[]  = {3270, 3610, 3690, 3710, 3730, 3750, 3770, 3790, 3800, 3820,
                                     3840, 3850, 3870, 3910, 3950, 3980, 4020, 4080, 4110, 4150, 4200};
static const uint8_t  curve_pct[] = {   0,    5,   10,   15,   20,   25,   30,   35,   40,   45,
                                       50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100};
#define CURVE_N (sizeof(curve_mv) / sizeof(curve_mv[0]))

/* Level in hundredths of a percent, 0..10000. */
static uint16_t curve_x100(uint16_t pack_mv) {
    if (pack_mv <= curve_mv[0]) return 0;
    if (pack_mv >= curve_mv[CURVE_N - 1]) return 10000;
    for (uint8_t i = 1; i < CURVE_N; i++) {
        if (pack_mv < curve_mv[i]) {
            uint32_t span = curve_mv[i] - curve_mv[i - 1];
            uint32_t into = pack_mv - curve_mv[i - 1];
            return (uint16_t)(curve_pct[i - 1] * 100u + (into * (curve_pct[i] - curve_pct[i - 1]) * 100u) / span);
        }
    }
    return 10000;
}

/* --- Level estimator ------------------------------------------------------------
 * Three sources, each used only where it is true:
 *   - the charger stopping with USB in is a true 100%;
 *   - below the clamp, VDD + DROPOUT_MV on the curve above;
 *   - in the clamp, a countdown from the load: a base draw plus the LED drive
 *     (sn32f2xx_led_load(), the sum of every channel's PWM value), floored at
 *     the level where the clamp lets go, which the voltage then takes over.
 * On USB, VDD is USB-derived, so the level holds, creeps up while charging,
 * and jumps to 100 when the charger terminates.
 *
 * Every constant marked CAL is a placeholder until the 2026-09-25 discharge
 * (and a lights-off run) replace it. */
#define SOC_UNKNOWN      0xFFFFu
#define SOC_FULL         10000u
#define SOC_CHARGE_CAP   9900u      /* charging never claims 100 before termination */
#define CLAMP_MV_DEFAULT 3900u      /* measured on JD's unit; learned per unit below */
#define CLAMP_MV_LO      3700u
#define CLAMP_MV_HI      4050u
#define CLAMP_MARGIN_MV  15u        /* this far under the clamp = tracking the pack */
#define DROPOUT_MV       100u       /* CAL: pack minus VDD just below the clamp */
/* The pack: "JKJ606090 4000mAh 3.7V 14.8Wh", NMC (Li/Ni/Co/Mn, graphite anode),
 * printed on the cell in JD's unit. Currents below are converted to level per
 * hour through it: 1 mA is 0.025 % per hour. */
#define PACK_MAH         4000u
#define DRAW_BASE_MA       45u      /* CAL: MCU + LCD backlight + radio, LEDs dark */
#define DRAW_LED_FULL_MA  900u      /* CAL: extra at full LED drive (every channel 255) */
#define CHARGE_MA         800u      /* CAL: average charge current into the pack */
#define MA_TO_X100_PER_H(ma) ((uint32_t)(ma) * 10000u / PACK_MAH)
#define DRAIN_BASE_X100_PER_H MA_TO_X100_PER_H(DRAW_BASE_MA)
#define DRAIN_LED_X100_PER_H  MA_TO_X100_PER_H(DRAW_LED_FULL_MA)
#define CHARGE_X100_PER_H     MA_TO_X100_PER_H(CHARGE_MA)
#define CHARGE_CONFIRM_S 3u         /* charging this long, then stopped this long = full */
#define LED_LOAD_MAX ((uint32_t)RGB_MATRIX_LED_COUNT * 3u * 255u)

static uint16_t soc        = SOC_UNKNOWN;
static uint16_t clamp_mv   = CLAMP_MV_DEFAULT;
static uint32_t soc_acc    = 0;     /* sub-0.01 % remainder, in 0.01 %-per-hour-seconds */
static uint16_t chg_on_s   = 0;     /* this USB session: seconds charging */
static uint16_t chg_off_s  = 0;     /* ...and seconds since charging stopped */
static bool     full       = false;
static uint16_t batt_s     = 0;     /* seconds on battery since the last handover */
static uint16_t led_pm     = 0;     /* LED drive, per mille of the maximum, last second */

static bool clamped(uint16_t mv) {
    return mv + CLAMP_MARGIN_MV >= clamp_mv;
}

static uint16_t clamp_floor_x100(void) {
    return curve_x100((uint16_t)(clamp_mv + DROPOUT_MV));
}

/* Apply `rate` (0.01 % per hour) for one second, in the direction given. */
static void soc_step(uint32_t rate, bool up, uint16_t limit) {
    soc_acc += rate;
    uint16_t steps = (uint16_t)(soc_acc / 3600u);
    soc_acc %= 3600u;
    if (up) {
        soc = (uint16_t)((soc + steps > limit) ? limit : soc + steps);
    } else {
        soc = (uint16_t)((soc < limit + steps) ? limit : soc - steps);
    }
}

static void estimator_tick(void) {   /* 1 Hz */
    uint16_t mv = battery_vdd_mv();
    if (mv == 0) return;
    led_pm = (uint16_t)((sn32f2xx_led_load() * 1000u) / LED_LOAD_MAX);

    if (!battery_on_battery()) {
        batt_s = 0;
        if (charge_is_charging()) {
            if (chg_on_s < 60000u) chg_on_s++;
            chg_off_s = 0;
            full      = false;
            if (soc != SOC_UNKNOWN && soc < SOC_CHARGE_CAP) soc_step(CHARGE_X100_PER_H, true, SOC_CHARGE_CAP);
        } else {
            if (chg_off_s < 60000u) chg_off_s++;
            if (chg_on_s >= CHARGE_CONFIRM_S && chg_off_s >= CHARGE_CONFIRM_S && !full) {
                full = true;
                soc  = SOC_FULL;
            }
        }
        return;
    }

    /* On battery. */
    chg_on_s = chg_off_s = 0;
    full     = false;
    if (batt_s < 60000u) batt_s++;

    /* Learn this unit's clamp: the highest settled VDD on battery. */
    if (batt_s >= 30u && mv > clamp_mv && mv <= CLAMP_MV_HI) clamp_mv = mv;

    if (!clamped(mv)) {
        soc     = curve_x100((uint16_t)(mv + DROPOUT_MV));
        soc_acc = 0;
        return;
    }
    uint16_t floor = clamp_floor_x100();
    if (soc == SOC_UNKNOWN || soc < floor) {
        /* Nothing known about the top of the charge: claim only what the
         * clamp proves. The countdown then starts from there. */
        soc = floor;
    }
    uint32_t rate = DRAIN_BASE_X100_PER_H + (DRAIN_LED_X100_PER_H * led_pm) / 1000u;
    soc_step(rate, false, floor);
}

battery_state_t battery_state(void) {
    if (battery_vdd_mv() == 0) return BATTERY_STATE_NONE;
    if (battery_on_battery())   return BATTERY_STATE_BATTERY;
    if (charge_is_charging())   return BATTERY_STATE_CHARGING;
    if (full)                   return BATTERY_STATE_FULL;
    return BATTERY_STATE_USB;
}

uint8_t battery_level_pct(void) {
    static uint8_t shown = 0xFF;
    if (soc == SOC_UNKNOWN) {
        shown = 0xFF;
        return shown;
    }
    uint8_t now = (uint8_t)((soc + 50u) / 100u);
    /* Hysteresis: move on a 2-point change, or to the ends of the scale. */
    if (shown == 0xFF || now == 0 || now == 100 || now + 2u <= shown || now >= shown + 2u) shown = now;
    return shown;
}

/* --- Low-battery protection ---------------------------------------------------
 * On battery with the filtered VDD under `warn_mv` for WARN_HOLD_TICKS, say so
 * once. Under `cut_mv` for the same time, turn the RGB matrix off (noeeprom:
 * the saved setting is untouched) so the board keeps running on a fraction of
 * the current instead of browning out mid-typing. The lights come back when
 * USB power returns. Thresholds are RAM-only and settable over HC_BATTCFG;
 * 0 disables either. */
#define WARN_HOLD_TICKS    300u    /* 30 s at 10 Hz */
#define RESTORE_HOLD_TICKS 50u     /* 5 s of USB power */
#define WARN_REARM_MV      100u    /* hysteresis before a second warning */

static uint16_t warn_mv    = 3550;
static uint16_t cut_mv     = 3400;
static uint16_t warn_ticks = 0, cut_ticks = 0, usb_ticks = 0;
static bool     warned     = false;
static bool     lights_cut = false;
static bool     cut_was_on = false;

static void protection_task(void) {
    uint16_t mv = battery_vdd_mv();
    if (mv == 0) return;

    if (!battery_on_battery()) {
        warn_ticks = cut_ticks = 0;
        warned     = false;
        if (lights_cut && ++usb_ticks >= RESTORE_HOLD_TICKS) {
            if (cut_was_on) rgb_matrix_enable_noeeprom();
            lights_cut = false;
        }
        return;
    }
    usb_ticks = 0;

    if (warn_mv && mv < warn_mv) {
        if (warn_ticks < WARN_HOLD_TICKS) warn_ticks++;
        if (warn_ticks >= WARN_HOLD_TICKS && !warned) {
            warned = true;
            display_set_alert("Battery low");
        }
    } else {
        warn_ticks = 0;
        if (warned && (!warn_mv || mv > warn_mv + WARN_REARM_MV)) warned = false;
    }

    if (cut_mv && mv < cut_mv) {
        if (cut_ticks < WARN_HOLD_TICKS) cut_ticks++;
        if (cut_ticks >= WARN_HOLD_TICKS && !lights_cut) {
            lights_cut = true;
            cut_was_on = rgb_matrix_is_enabled();
            rgb_matrix_disable_noeeprom();
            display_set_alert("Low: RGB off");
        }
    } else {
        cut_ticks = 0;
    }
}

uint8_t battery_flags(void) {
    uint16_t mv = battery_vdd_mv();
    uint8_t  f  = 0;
    if (battery_on_battery()) {
        f |= BATT_F_ON_BATTERY;
        if (clamped(mv)) f |= BATT_F_CLAMPED;
    }
    if (!gpio_read_pin(CHARGE_CHRG_PIN))  f |= BATT_F_CHARGING;
    if (!gpio_read_pin(CHARGE_STDBY_PIN)) f |= BATT_F_DONE;
    if (rgb_matrix_is_enabled())          f |= BATT_F_RGB_ON;
    if (lights_cut)                       f |= BATT_F_LIGHTS_CUT;
    if (ch582_is_usb())                   f |= BATT_F_SLIDER_USB;
    if (ch582_is_connected())             f |= BATT_F_LINKED;
    return f;
}

uint16_t battery_soc_x100(void) { return soc; }
uint16_t battery_clamp_mv(void) { return clamp_mv; }
uint16_t battery_led_pm(void)   { return led_pm; }

/* --- Once-a-minute log --------------------------------------------------------
 * 12 hours in RAM, oldest overwritten. It exists to record discharges -- VDD
 * against time and load, beside the estimate and the module's number -- so the
 * CAL constants above can be measured rather than guessed. RAM, not flash, so
 * it survives anything but a reset; the protection above keeps a drained board
 * running rather than browning out. Read with HC_BATTLOG. */
#define LOG_LEN       720u
#define LOG_PERIOD_MS 60000u

typedef struct __attribute__((packed)) {
    uint16_t vdd_avg;      /* mean of the raw samples over the minute, mV */
    uint16_t vdd_min;      /* lowest raw sample over the minute, mV */
    uint16_t soc;          /* estimate at the end of the minute, 0.01 %, 0xFFFF unknown */
    uint16_t led_pm;       /* mean LED drive over the minute, per mille of the maximum */
    uint8_t  module_pct;   /* the CH582F's `5C` value */
    uint8_t  flags;        /* BATT_F_* */
    uint8_t  rgb_val;      /* RGB brightness 0-255 */
    uint8_t  module_age_s; /* seconds since the module's last `5C`, 255 = stale/never */
} log_entry_t;
_Static_assert(sizeof(log_entry_t) == BATTERY_LOG_ENTRY_SIZE, "HC_BATTLOG entry size");

static log_entry_t log_buf[LOG_LEN];
static uint16_t    log_head    = 0;   /* next slot to write */
static uint16_t    log_count   = 0;
static uint32_t    log_last_at = 0;   /* when the newest entry was written */
static uint32_t    acc_sum = 0, acc_start = 0, acc_led = 0;
static uint16_t    acc_n = 0, acc_min = 0, acc_led_n = 0;

static void log_task(uint16_t sample_mv) {
    if (sample_mv) {
        acc_sum += sample_mv;
        acc_n++;
        if (acc_min == 0 || sample_mv < acc_min) acc_min = sample_mv;
    }
    if (acc_start == 0) acc_start = timer_read32() | 1u;
    if (timer_elapsed32(acc_start) < LOG_PERIOD_MS || acc_n == 0) return;

    uint32_t     age = ch582_battery_age_ms();
    log_entry_t *e   = &log_buf[log_head];
    e->vdd_avg      = (uint16_t)(acc_sum / acc_n);
    e->vdd_min      = acc_min;
    e->soc          = soc;
    e->led_pm       = acc_led_n ? (uint16_t)(acc_led / acc_led_n) : led_pm;
    e->module_pct   = ch582_get_battery();
    e->flags        = battery_flags();
    e->rgb_val      = rgb_matrix_get_val();
    e->module_age_s = (age == UINT32_MAX || age / 1000u > 254u) ? 255u : (uint8_t)(age / 1000u);

    log_head = (uint16_t)((log_head + 1u) % LOG_LEN);
    if (log_count < LOG_LEN) log_count++;
    log_last_at = timer_read32();
    acc_sum = 0; acc_n = 0; acc_min = 0; acc_led = 0; acc_led_n = 0;
    acc_start = log_last_at | 1u;
}

void battery_log_read(uint16_t idx, uint8_t *out) {
    uint32_t since = log_count ? timer_elapsed32(log_last_at) : 0;
    out[0] = BATTERY_LOG_VERSION;
    out[1] = (uint8_t)(log_count & 0xFF); out[2] = (uint8_t)(log_count >> 8);
    out[3] = (uint8_t)(idx & 0xFF);       out[4] = (uint8_t)(idx >> 8);
    for (uint8_t i = 0; i < 4; i++) out[5 + i] = (uint8_t)(since >> (8 * i));
    uint8_t *dst = &out[9];
    if (idx >= log_count) {
        memset(dst, 0, BATTERY_LOG_ENTRY_SIZE);
        return;
    }
    /* idx 0 is the oldest entry still held. */
    uint16_t slot = (uint16_t)((log_head + LOG_LEN - log_count + idx) % LOG_LEN);
    memcpy(dst, &log_buf[slot], BATTERY_LOG_ENTRY_SIZE);
}

void battery_cfg(uint8_t op, uint16_t new_warn, uint16_t new_cut, uint8_t *out) {
    if (op == 1) {
        warn_mv    = new_warn;
        cut_mv     = new_cut;
        warn_ticks = cut_ticks = 0;
    }
    out[0] = (uint8_t)(warn_mv & 0xFF); out[1] = (uint8_t)(warn_mv >> 8);
    out[2] = (uint8_t)(cut_mv & 0xFF);  out[3] = (uint8_t)(cut_mv >> 8);
    out[4] = lights_cut ? 1 : 0;
    out[5] = warned ? 1 : 0;
    out[6] = (uint8_t)((LOG_PERIOD_MS / 1000u) & 0xFF);
    out[7] = (uint8_t)((LOG_PERIOD_MS / 1000u) >> 8);
}

void battery_task(void) {
    static uint8_t tick = 0;
    if (!adc_ready) return;
    uint16_t code = adc_read_ain16();
    last_raw      = code;
    uint16_t mv   = 0;

    if (code != 0) {
        uint32_t m = (REF_MV * ADC_FULLSCALE + code / 2u) / code;
        if (m >= VDD_MIN_MV && m <= VDD_MAX_MV) mv = (uint16_t)m;
    }
    if (mv) {
        if (ema_x64 == 0) {
            ema_x64 = (uint32_t)mv * EMA_SCALE;
        } else {
            uint16_t f    = battery_vdd_mv();
            uint16_t dist = (mv > f) ? (uint16_t)(mv - f) : (uint16_t)(f - mv);
            snap_run      = (dist > SNAP_MV) ? (uint8_t)(snap_run + 1u) : 0u;
            if (snap_run >= SNAP_SAMPLES) {
                ema_x64  = (uint32_t)mv * EMA_SCALE;
                snap_run = 0;
            } else {
                ema_x64 = ema_x64 - (ema_x64 >> EMA_SHIFT) + (((uint32_t)mv * EMA_SCALE) >> EMA_SHIFT);
            }
        }
        uint16_t f = battery_vdd_mv();
        if (vdd_min == 0 || f < vdd_min) vdd_min = f;
        if (f > vdd_max) vdd_max = f;
    }

    protection_task();
    if (++tick >= 10u) {
        tick = 0;
        estimator_tick();
        acc_led += led_pm;
        acc_led_n++;
    }
    log_task(mv);
}
