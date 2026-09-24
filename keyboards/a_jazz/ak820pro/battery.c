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
 * every one of them is a matrix column, an RGB row or the Win Lock LED. What it
 * can see is its own VDD, by converting the internal 2.0 V reference (AIN16)
 * against VDD as the reference high. The datasheet calls that "a good battery
 * detector for battery system". VDD is specified 2.5-5.5 V and the slider is a
 * power-source switch (docs/hardware.md), so in the wireless positions VDD is
 * expected to be the pack itself -- to be confirmed on hardware before anything
 * is drawn from it.
 *
 * Accuracy: the 2.0 V reference is 1.96-2.04 V, so the absolute reading is
 * good to +-2% (about +-80 mV at 4 V). The error is a fixed gain per part, so
 * the charger's 4.2 V termination could calibrate it later.
 *
 * Cost: one conversion per 10 Hz tick, ~11 us with interrupts masked. The mask
 * is caution, not a known need: with the global channel enabled the datasheet
 * hands "the ADC shared pins" to the ADC, and those are the matrix and RGB
 * pins. It is almost certainly only the selected pin, and AIN16 has none, but
 * masking keeps the row ISR from ever sampling the matrix inside that window. */
#include "quantum.h"
#include "battery.h"

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

static bool     adc_ready = false;
static uint16_t last_raw  = 0;
static uint32_t ema_x64   = 0;       /* 0 = no sample yet */
static uint16_t vdd_min   = 0;
static uint16_t vdd_max   = 0;

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

void battery_task(void) {
    if (!adc_ready) return;
    uint16_t code = adc_read_ain16();
    last_raw      = code;
    if (code == 0) return;

    uint32_t mv = (REF_MV * ADC_FULLSCALE + code / 2u) / code;
    if (mv < VDD_MIN_MV || mv > VDD_MAX_MV) return;

    if (ema_x64 == 0) {
        ema_x64 = mv * EMA_SCALE;
    } else {
        ema_x64 = ema_x64 - (ema_x64 >> EMA_SHIFT) + ((mv * EMA_SCALE) >> EMA_SHIFT);
    }

    uint16_t f = battery_vdd_mv();
    if (vdd_min == 0 || f < vdd_min) vdd_min = f;
    if (f > vdd_max) vdd_max = f;
}

uint16_t battery_vdd_mv(void) {
    return (uint16_t)((ema_x64 + EMA_SCALE / 2u) / EMA_SCALE);
}

uint16_t battery_vdd_raw(void)    { return last_raw; }
uint16_t battery_vdd_min_mv(void) { return vdd_min; }
uint16_t battery_vdd_max_mv(void) { return vdd_max; }
