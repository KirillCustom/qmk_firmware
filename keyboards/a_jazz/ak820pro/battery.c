// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The SN32's own supply voltage, from its ADC. See battery.h.
 *
 * Why this exists: the CH582F's `5C` battery percentage held 100 through a
 * whole discharge at full RGB brightness, until the board browned out, and
 * still read 100 six minutes into recharging the flat pack (2026-09-24). It is
 * not a level. It is not nothing either -- a board with no pack fitted reads 0
 * (indicators.c) -- which fits an ADC pin on the module wired to the pack
 * through a divider that saturates for any real cell voltage.
 *
 * The SN32 cannot see the pack through a pin: AIN0..AIN15 are P2.0..P2.15, and
 * every one of them is a matrix column, an RGB row or the Win Lock LED. The
 * stock firmware never touches the ADC, the comparator or the op-amps either
 * (no reference to their base addresses in v1.13 or v1.14, nor in seven
 * sibling SN32F290 boards): stock's battery gauge was the module's number.
 * What the SN32 can see is its own VDD, by converting the internal 2.0 V
 * reference (AIN16) against VDD as the reference high. The datasheet calls that
 * "a good battery detector for battery system".
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
#include "bluetooth/ch582f_ajazz.h"
#include "graphics/display.h"

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

static bool     adc_ready = false;
static uint16_t last_raw  = 0;
static uint32_t ema_x64   = 0;       /* 0 = no sample yet */
static uint16_t vdd_min   = 0;
static uint16_t vdd_max   = 0;
static uint16_t shown_cv  = 0;       /* battery_display_cv()'s held value */

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

/* --- Provisional level -------------------------------------------------------
 * A generic lithium-ion (LCO/NMC) rest-voltage curve. It assumes VDD is the
 * pack, which is not established yet: this is a placeholder until a measured
 * discharge (the log below, plus a multimeter at the pack) replaces it. */
static const uint16_t curve_mv[]  = {3270, 3610, 3690, 3710, 3730, 3750, 3770, 3790, 3800, 3820,
                                     3840, 3850, 3870, 3910, 3950, 3980, 4020, 4080, 4110, 4150, 4200};
static const uint8_t  curve_pct[] = {   0,    5,   10,   15,   20,   25,   30,   35,   40,   45,
                                       50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100};
#define CURVE_N (sizeof(curve_mv) / sizeof(curve_mv[0]))

uint8_t battery_level_pct(void) {
    if (!battery_on_battery()) return 0xFF;
    uint16_t mv = battery_vdd_mv();
    if (mv <= curve_mv[0]) return 0;
    if (mv >= curve_mv[CURVE_N - 1]) return 100;
    for (uint8_t i = 1; i < CURVE_N; i++) {
        if (mv < curve_mv[i]) {
            uint32_t span = curve_mv[i] - curve_mv[i - 1];
            uint32_t into = mv - curve_mv[i - 1];
            return (uint8_t)(curve_pct[i - 1] + (into * (curve_pct[i] - curve_pct[i - 1])) / span);
        }
    }
    return 100;
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

/* --- Low-battery protection ---------------------------------------------------
 * On battery with the filtered VDD under `warn_mv` for WARN_HOLD_TICKS, say so
 * once. Under `cut_mv` for the same time, turn the RGB matrix off (noeeprom:
 * the saved setting is untouched) so the board keeps running on a fraction of
 * the current instead of browning out mid-typing. The lights come back when
 * USB power returns. Thresholds are RAM-only and settable over HC_BATTCFG,
 * because where they belong depends on how VDD relates to the pack (task 8.1);
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
    uint8_t f = 0;
    if (battery_on_battery())                f |= BATT_F_ON_BATTERY;
    if (!gpio_read_pin(CHARGE_CHRG_PIN))     f |= BATT_F_CHARGING;
    if (!gpio_read_pin(CHARGE_STDBY_PIN))    f |= BATT_F_DONE;
    if (rgb_matrix_is_enabled())             f |= BATT_F_RGB_ON;
    if (lights_cut)                          f |= BATT_F_LIGHTS_CUT;
    if (ch582_is_usb())                      f |= BATT_F_SLIDER_USB;
    if (ch582_is_connected())                f |= BATT_F_LINKED;
    return f;
}

/* --- Once-a-minute log --------------------------------------------------------
 * 12 hours in RAM (5.6 KB of the ~17 KB left unused), oldest overwritten. It
 * exists to record a discharge: VDD against time and load, beside the module's
 * number, so the curve above can be replaced by a measured one. RAM, not
 * flash, so it survives anything but a reset -- and the protection above keeps
 * a drained board running rather than browning out. Read with HC_BATTLOG. */
#define LOG_LEN       720u
#define LOG_PERIOD_MS 60000u

typedef struct __attribute__((packed)) {
    uint16_t vdd_avg;      /* mean of the raw samples over the minute, mV */
    uint16_t vdd_min;      /* lowest raw sample over the minute, mV */
    uint8_t  module_pct;   /* the CH582F's `5C` value */
    uint8_t  flags;        /* BATT_F_* */
    uint8_t  rgb_val;      /* RGB brightness 0-255 */
    uint8_t  module_age_s; /* seconds since the module's last `5C`, 255 = stale/never */
} log_entry_t;
_Static_assert(sizeof(log_entry_t) == 8, "HC_BATTLOG carries 8-byte entries");

static log_entry_t log_buf[LOG_LEN];
static uint16_t    log_head    = 0;   /* next slot to write */
static uint16_t    log_count   = 0;
static uint32_t    log_last_at = 0;   /* when the newest entry was written */
static uint32_t    acc_sum = 0, acc_start = 0;
static uint16_t    acc_n = 0, acc_min = 0;

static void log_task(uint16_t sample_mv) {
    if (sample_mv) {
        acc_sum += sample_mv;
        acc_n++;
        if (acc_min == 0 || sample_mv < acc_min) acc_min = sample_mv;
    }
    if (acc_start == 0) acc_start = timer_read32() | 1u;
    if (timer_elapsed32(acc_start) < LOG_PERIOD_MS || acc_n == 0) return;

    uint32_t   age = ch582_battery_age_ms();
    log_entry_t *e = &log_buf[log_head];
    e->vdd_avg      = (uint16_t)(acc_sum / acc_n);
    e->vdd_min      = acc_min;
    e->module_pct   = ch582_get_battery();
    e->flags        = battery_flags();
    e->rgb_val      = rgb_matrix_get_val();
    e->module_age_s = (age == UINT32_MAX || age / 1000u > 254u) ? 255u : (uint8_t)(age / 1000u);

    log_head = (uint16_t)((log_head + 1u) % LOG_LEN);
    if (log_count < LOG_LEN) log_count++;
    log_last_at = timer_read32();
    acc_sum = 0; acc_n = 0; acc_min = 0;
    acc_start = log_last_at | 1u;
}

void battery_log_read(uint16_t idx, uint8_t *out) {
    uint32_t since = log_count ? timer_elapsed32(log_last_at) : 0;
    out[0] = (uint8_t)(log_count & 0xFF); out[1] = (uint8_t)(log_count >> 8);
    out[2] = (uint8_t)(idx & 0xFF);       out[3] = (uint8_t)(idx >> 8);
    for (uint8_t i = 0; i < 4; i++) out[4 + i] = (uint8_t)(since >> (8 * i));
    for (uint8_t k = 0; k < 2; k++) {
        uint8_t *dst = &out[8 + 8 * k];
        uint16_t n   = (uint16_t)(idx + k);
        if (n >= log_count) {
            memset(dst, 0, 8);
            continue;
        }
        /* idx 0 is the oldest entry still held. */
        uint16_t slot = (uint16_t)((log_head + LOG_LEN - log_count + n) % LOG_LEN);
        memcpy(dst, &log_buf[slot], 8);
    }
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
            ema_x64 = ema_x64 - (ema_x64 >> EMA_SHIFT) + (((uint32_t)mv * EMA_SCALE) >> EMA_SHIFT);
        }
        uint16_t f = battery_vdd_mv();
        if (vdd_min == 0 || f < vdd_min) vdd_min = f;
        if (f > vdd_max) vdd_max = f;
    }

    protection_task();
    log_task(mv);
}
