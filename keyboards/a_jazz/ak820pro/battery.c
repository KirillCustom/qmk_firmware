// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Battery: what powers the board, the pack voltage from the CH582F's `5C`, the
 * level on the panel, the low-battery protection, and a log. See battery.h,
 * docs/battery.md, and plans/BATTERY-GAUGE-PLAN.md for the evidence and the
 * review behind every number here.
 *
 * Two signals, each used only for what it can say.
 *
 * VDD, from the SN32's own ADC. On battery it is a BUCK-BOOST's output, pinned
 * at ~3.90 V while the pack falls 4.18 -> 3.30 V, so it says nothing about the
 * pack (an earlier level estimator built on it, 36be68f16a, is gone for that
 * reason). It does say which supply feeds the rail: ~3.90 V from the pack,
 * never under 4.1 V from USB (4.12-4.30 V charging, 4.47 V in the BT position,
 * 4.79 V in the cable position). With the charger's CHRG pin it answers "on
 * battery?".
 *
 * `5C`, the module's battery percent, is a LINEAR VOLTMETER on the pack:
 * 5C = 114.22 * V - 361.04, r2 0.99987 over five meter points 3.30-3.84 V
 * (history/battery-2026-09-25/readings.csv). It clamps at 100 above ~4.036 V
 * and at 0 below ~3.161 V, both extrapolated. It arrives 2-3 times per 5 s A6 53
 * poll cycle, bunched (see the intake below). It answers "how full?".
 *
 * VDD measurement: the internal 2.0 V reference (AIN16) converted against VDD
 * as the reference high, VDD = 2.0 V * 4096 / code. The reference is 1.96-2.04 V,
 * so the absolute reading is good to +-2%, a fixed gain per part. One conversion
 * per 10 Hz tick, ~11 us with interrupts masked. The mask is caution, not a
 * known need: with the global channel enabled the datasheet hands "the ADC
 * shared pins" to the ADC, and those are the matrix and RGB pins. It is almost
 * certainly only the selected pin, and AIN16 has none, but masking keeps the
 * row ISR from ever sampling the matrix inside that window. */
#include "quantum.h"
#include "battery.h"
#include "power.h"
#include "kb_eeconfig.h"
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

/* EMA in mV * 64, alpha 1/16: a ~1.6 s time constant at 10 Hz. For display and
 * HC_CONN only -- the supply decision uses raw samples with its own dwell. */
#define EMA_SHIFT 4
#define EMA_SCALE 64
/* A USB <-> battery handover moves VDD by ~0.6 V at once. Three samples in a
 * row this far from the filter reseed it instead of sliding across. */
#define SNAP_MV      300u
#define SNAP_SAMPLES 3u

/* Status bits (HC_CONN byte 23 and each log entry). ON_BATTERY and EXTERNAL are
 * the supply decision; neither set means UNKNOWN (boot, before the dwell). */
#define BATT_F_ON_BATTERY 0x01u   /* supply: the pack powers the board */
#define BATT_F_CHARGING   0x02u   /* charger CHRG pin low, raw */
#define BATT_F_DONE       0x04u   /* charger STDBY/DONE pin low, raw: never seen low
                                   * on USB, and ALWAYS low on battery -- meaningless */
#define BATT_F_RGB_ON     0x08u
#define BATT_F_LIGHTS_CUT 0x10u   /* the low-battery protection's RGB cap is on */
#define BATT_F_SLIDER_USB 0x20u   /* mode slider in the cable position */
#define BATT_F_LINKED     0x40u   /* module reports a live connection */
#define BATT_F_EXTERNAL   0x80u   /* supply: USB powers the board */

static bool     adc_ready  = false;
static uint16_t last_raw   = 0;
static uint32_t ema_x64    = 0;      /* 0 = no sample yet */
static uint8_t  snap_run   = 0;
static uint16_t vdd_min    = 0;
static uint16_t vdd_max    = 0;

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

/* --- Supply: what powers the rail ---------------------------------------------
 * A state machine, not a threshold. The old predicate (VDD < 4300 mV) flickered
 * as VDD crept through 4300 in the charge taper, and called a pack charging
 * hard at VDD 4193 mV "on battery" for four hours (2026-09-28).
 *
 *   CHRG low on 2 consecutive ticks   -> EXTERNAL evidence, held 1 s past the
 *                                        last low (covers the charger's ~2 Hz
 *                                        no-pack pulse). CHRG can only sink
 *                                        with USB present: a direct observation.
 *   raw VDD >= 4100 mV for 0.5 s      -> EXTERNAL
 *   raw VDD <  4000 mV for 1 s,
 *     with no EXTERNAL evidence       -> BATTERY
 *   otherwise                         -> keep the previous state
 *
 * On battery VDD is ~3.90 V; on USB no raw sample went under 4119 mV across a
 * 7.5 h charge into a dead pack. ⚠️ The ADC reference is +-2% per part:
 * re-check both margins on another unit. */
#define EXT_ENTER_MV       4100u
#define BATT_ENTER_MV      4000u
#define EXT_ENTER_TICKS    5u
#define BATT_ENTER_TICKS   10u
#define CHRG_QUALIFY_TICKS 2u
#define CHRG_HOLD_TICKS    10u
/* Unbroken charging this long is a real charging session: the level may rise,
 * and a brief plug-in cannot bump it. */
#define SESSION_TICKS      600u    /* 60 s */
/* CHRG released this long, on USB, with `5C` at 100: the charger terminated. */
#define FULL_SETTLE_TICKS  100u    /* 10 s */
/* A second of failed conversions and what VDD last said has expired. */
#define ADC_STALE_TICKS    10u

static battery_supply_t supply        = BATTERY_SUPPLY_UNKNOWN;
static uint8_t          ext_run       = 0;   /* raw VDD >= EXT_ENTER_MV, ticks */
static uint8_t          batt_run      = 0;   /* raw VDD <  BATT_ENTER_MV, ticks */
static uint16_t         chrg_run      = 0;   /* CHRG low, consecutive ticks */
static uint16_t         chrg_off_run  = 0;   /* CHRG high, consecutive ticks */
static uint8_t          chrg_hold     = 0;   /* ticks of EXTERNAL evidence left */
static uint8_t          adc_fail_run  = 0;   /* consecutive failed conversions */
/* Consecutive ticks with FRESH positive evidence of USB -- CHRG, or VDD read
 * this second. Not "supply is EXTERNAL": that is remembered, and a remembered
 * state must never be what lifts the low-battery cut (codex, 2026-09-28). */
static uint16_t         ext_evidence_ticks = 0;
static bool             chg_was       = false;   /* charging_now() last tick */
static bool             session       = false;
static bool             ran_from_pack = false;
/* Set when the board goes onto the pack after a real charging session: the
 * level re-seats once, from the first post-unplug estimate (see level_report).
 * Owed until consumed: a brief replug before the estimate exists must not cancel
 * it, or the discharge keeps the charging estimate and its guessed I*R. */
static bool             reseat        = false;

static void c5_forget(void);

static bool charging_now(void) {
    return supply == BATTERY_SUPPLY_EXTERNAL && chrg_hold != 0;
}

static void supply_tick(uint16_t raw_mv) {
    bool chrg_low = !gpio_read_pin(CHARGE_CHRG_PIN);
    if (chrg_low) {
        if (chrg_run < UINT16_MAX) chrg_run++;
        chrg_off_run = 0;
    } else {
        chrg_run = 0;
        if (chrg_off_run < UINT16_MAX) chrg_off_run++;
    }
    if (chrg_run >= CHRG_QUALIFY_TICKS) {
        chrg_hold = CHRG_HOLD_TICKS;
    } else if (chrg_hold) {
        chrg_hold--;
    }

    /* A failed conversion is no information: the runs hold rather than reset.
     * A second of them and the runs expire -- the supply then holds (only CHRG
     * can still move it), but nothing counts as fresh proof of USB. */
    if (raw_mv) {
        adc_fail_run = 0;
        ext_run  = (raw_mv >= EXT_ENTER_MV)  ? (uint8_t)(ext_run  < UINT8_MAX ? ext_run  + 1u : ext_run)  : 0u;
        batt_run = (raw_mv <  BATT_ENTER_MV) ? (uint8_t)(batt_run < UINT8_MAX ? batt_run + 1u : batt_run) : 0u;
    } else if (adc_fail_run < UINT8_MAX && ++adc_fail_run >= ADC_STALE_TICKS) {
        ext_run = batt_run = 0;
    }

    bool ext_evidence  = chrg_hold || ext_run >= EXT_ENTER_TICKS;
    ext_evidence_ticks = ext_evidence ? (uint16_t)(ext_evidence_ticks < UINT16_MAX ? ext_evidence_ticks + 1u : ext_evidence_ticks) : 0u;

    battery_supply_t was = supply;
    if (ext_evidence) {
        supply = BATTERY_SUPPLY_EXTERNAL;
    } else if (batt_run >= BATT_ENTER_TICKS) {
        supply = BATTERY_SUPPLY_BATTERY;
    }

    if (supply != was) {
        /* Every report in the estimate was taken on the other supply -- under
         * charge current, or without it -- so none describes this one. */
        c5_forget();
        if (was == BATTERY_SUPPLY_EXTERNAL && supply == BATTERY_SUPPLY_BATTERY && session) reseat = true;
    }
    /* The charger starting or stopping moves the terminal voltage by I*R as
     * surely as a supply change does: a charger fault mid-charge must not be
     * judged FULL on reports taken while it was still charging. */
    bool chg = charging_now();
    if (chg != chg_was) {
        c5_forget();
        chg_was = chg;
    }
    if (supply == BATTERY_SUPPLY_BATTERY) {
        ran_from_pack = true;
        session       = false;
    }
    if (chrg_run >= SESSION_TICKS) session = true;
}

battery_supply_t battery_supply(void) { return supply; }
bool battery_on_battery(void)         { return supply == BATTERY_SUPPLY_BATTERY; }
bool battery_ran_from_pack(void)      { return ran_from_pack; }

/* --- `5C`: intake, freshness, the estimate -------------------------------------
 * The CH582F driver hands over every report as it is parsed
 * (battery_5c_report), so two that land between ticks are both counted -- in
 * the estimate, the critical-zero run, and the log. Never re-read the cached
 * value instead: that only duplicates the one report it holds.
 *
 * The estimate is a TRIMMED MEAN of the last 64 reports, in hundredths of a
 * count, dropping the highest and lowest eighth: ~2-2.5 min at the 262-334
 * reports per 10 min the v4 log counts. They come 2-3 per 5 s A6 53 poll cycle,
 * bunched (the debug page's age resets every ~5 s), so 64 reports may be only
 * ~25 independent readings; how the module produces them is unconfirmed
 * (docs/battery.md). Not a median: a median of whole counts is a whole count, and one count
 * is ~9 mV -- 1.2% of level at 4.0 V, up to ~5% near 3.8 V -- so the level moved
 * in lumps and then sat still for an hour. `5C` dithers between neighbouring
 * counts (the drain log's means read like 96.62), so a mean resolves about a
 * tenth of a count; the trim keeps the median's indifference to a stray report.
 *
 * Emptied when the reports go stale, so a module that falls silent and returns
 * is not judged on minutes-old values, and at every supply or charger change. */
#define C5_RING        64u
#define C5_MIN_EST     5u
#define C5_STALE_MS    20000u    /* ~8 missed reports */
#define C5_NONE        0xFFu
#define C5_EST_NONE    0xFFFFu
/* The clamps, as the estimate sees them: most reports at 100 (>= 4036 mV, the
 * voltage says only "at least"), or most at 0 (<= 3161 mV). */
#define C5_TOP_X100    9950u
#define C5_BOTTOM_X100 50u

static uint8_t  c5_ring[C5_RING];
static uint8_t  c5_count    = 0;
static uint8_t  c5_pos      = 0;
static bool     c5_new      = false;         /* a report arrived since the last tick */
static uint16_t c5_est      = C5_EST_NONE;   /* hundredths of a count */
static uint8_t  c5_zero_run = 0;             /* consecutive reports of 0 */

/* The log's view of every report in the period (see log_task). */
static uint16_t lg_c5_sum = 0, lg_c5_n = 0;
static uint8_t  lg_c5_min = C5_NONE, lg_c5_max = 0;

/* The estimate reads c5_ring[0 .. c5_count-1], so writing must restart at 0 too:
 * resuming at the old c5_pos would sort new reports among forgotten ones. */
static void c5_forget(void) {
    c5_count    = 0;
    c5_pos      = 0;
    c5_est      = C5_EST_NONE;
    c5_zero_run = 0;
}

static bool c5_fresh(void) {
    uint32_t age = ch582_battery_age_ms();
    return age != UINT32_MAX && age < C5_STALE_MS;
}

static bool c5_top(uint16_t e)    { return e != C5_EST_NONE && e >= C5_TOP_X100; }
static bool c5_bottom(uint16_t e) { return e < C5_BOTTOM_X100; }

/* ~1000 compares for a full ring, at one report per ~2 s. */
static uint16_t c5_trimmed_mean_x100(void) {
    uint8_t s[C5_RING];
    for (uint8_t i = 0; i < c5_count; i++) {
        uint8_t v = c5_ring[i], j = i;
        while (j > 0 && s[j - 1] > v) {
            s[j] = s[j - 1];
            j--;
        }
        s[j] = v;
    }
    uint8_t  trim = (uint8_t)(c5_count / 8u);
    uint8_t  n    = (uint8_t)(c5_count - 2u * trim);
    uint16_t sum  = 0;
    for (uint8_t i = trim; i < c5_count - trim; i++) sum += s[i];
    return (uint16_t)(((uint32_t)sum * 100u + n / 2u) / n);
}

/* Called by the CH582F driver for every `5C` frame it parses (main loop). */
void battery_5c_report(uint8_t v) {
    if (v > 100u) return;
    c5_new          = true;
    c5_ring[c5_pos] = v;
    c5_pos          = (uint8_t)((c5_pos + 1u) % C5_RING);
    if (c5_count < C5_RING) c5_count++;
    c5_est      = (c5_count >= C5_MIN_EST) ? c5_trimmed_mean_x100() : C5_EST_NONE;
    c5_zero_run = (v == 0u) ? (uint8_t)(c5_zero_run < UINT8_MAX ? c5_zero_run + 1u : c5_zero_run) : 0u;

    if (lg_c5_n < UINT16_MAX && lg_c5_sum <= (uint16_t)(UINT16_MAX - v)) {
        lg_c5_sum += v;
        lg_c5_n++;
        if (v < lg_c5_min) lg_c5_min = v;
        if (v > lg_c5_max) lg_c5_max = v;
    }
}

uint16_t battery_5c_x100(void) {
    return c5_fresh() ? c5_est : C5_EST_NONE;
}

uint8_t battery_5c_rounded(void) {
    uint16_t e = battery_5c_x100();
    return (e == C5_EST_NONE) ? C5_NONE : (uint8_t)((e + 50u) / 100u);
}

/* --- `5C` -> pack millivolts ---------------------------------------------------
 * mV = (5C + 361.04) / 114.22 * 1000, integer, from the estimate in hundredths.
 * ⚠️ Valid below the clamps. At them it returns roughly the clamp's BOUND
 * (3161, 4036): "at most" and "at least", which the callers treat as such --
 * never as a reading, never on the curve. */
#define FIT_SLOPE_X100  11422u      /* 114.22 counts per volt */
#define FIT_OFFSET_X100 36104u      /* 361.04 counts */

static uint16_t c5_to_mv(uint16_t x100) {
    return (uint16_t)((((uint32_t)x100 + FIT_OFFSET_X100) * 1000u + FIT_SLOPE_X100 / 2u) / FIT_SLOPE_X100);
}

uint16_t battery_pack_mv(void) {
    uint16_t e = battery_5c_x100();
    return (e == C5_EST_NONE) ? 0u : c5_to_mv(e);
}

/* --- The level ------------------------------------------------------------------
 * What the number means: the fraction of RUNTIME left at the kind of load the
 * curve was measured at. Behind a buck-boost the board draws constant power, so
 * that -- not charge -- is what a timed run calibrates, and it is what a user
 * wants to know.
 *
 * FITTED from JD's unit on TWO drain tests at full white, 2026-09-28 (51.6
 * equivalent full-white hours from FULL to the cut) and 2026-10-02 (50.4 h),
 * by scripts/battery_fit.py --refit (history/battery-2026-10-01-drain/
 * refit-output.txt). Each knot is the mean of the two runs' smoothed voltages
 * at that fraction of runtime. 0% is the RGB cut at 3400 mV (JD, 2026-10-01);
 * the top knot keeps LEVEL_CLAMP_EXIT's 90% (the clamp's exit fell at 89.6%
 * and 90.3%). Leave-one-out, each run's own curve scored on the other: below
 * 3.95 V within 0.54-0.59 points rms (worst 2.1); on the 3.95-4.036 V plateau
 * 3.1-3.2 rms, worst 8.9 -- inside the plan's thresholds, with little room.
 * ⚠️ The plateau holds ~20% of the runtime in ~20 mV: a few mV there (lighter
 * load, less sag, one run's pack a little different) is ~10%, and 2 mV of
 * estimator offset moves the worst error to ~10. Two runs of one pack at one
 * load: a consistency check, not a confidence bound. The one-run curve this
 * replaces (759e265796) differed by at most 5 mV at any knot. */
static const uint16_t curve_mv[] = {3400, 3456, 3490, 3520, 3538, 3564, 3591, 3640, 3680,
                                    3723, 3748, 3785, 3814, 3831, 3846, 3856, 3868, 3896,
                                    3945, 3981, 3996, 4003, 4007, 4010, 4016, 4036};
static const uint16_t curve_pm[] = {   0,   10,   20,   30,   40,   50,   60,   80,  100,
                                     125,  150,  200,  250,  300,  350,  400,  450,  500,
                                     550,  600,  650,  700,  750,  800,  850,  900};
#define CURVE_N (sizeof(curve_mv) / sizeof(curve_mv[0]))
_Static_assert(sizeof(curve_pm) == sizeof(curve_mv), "curve tables must pair");

/* Level in per mille, 0..900, for a pack voltage below the top clamp. */
static uint16_t curve(uint16_t mv) {
    if (mv <= curve_mv[0]) return 0;
    if (mv >= curve_mv[CURVE_N - 1]) return curve_pm[CURVE_N - 1];
    for (uint8_t i = 1; i < CURVE_N; i++) {
        if (mv < curve_mv[i]) {
            uint32_t span = curve_mv[i] - curve_mv[i - 1];
            uint32_t into = mv - curve_mv[i - 1];
            return (uint16_t)(curve_pm[i - 1] + (into * (uint32_t)(curve_pm[i] - curve_pm[i - 1])) / span);
        }
    }
    return curve_pm[CURVE_N - 1];
}

/* The level for a `5C` estimate below the top clamp; 0 at the bottom clamp. */
static uint16_t c5_level(uint16_t e) {
    return c5_bottom(e) ? 0u : curve(c5_to_mv(e));
}

/* ⚠️ PROVISIONAL. While charging, `5C` reads the terminal voltage, which the
 * charge current lifts by I*R above the pack's own. Uncorrected, the discharge
 * curve read 5C 90 two hours into the 09-28 charge as ~77%, where the charge
 * time says ~40%, and unplugging there would drop the level by ~35 points.
 * ~0.8-0.9 A (4 h of constant current from flat) through ~180 mOhm (a
 * comparable 606090 pack's cell + protection board) is ~150 mV. Measure it as
 * the step in 5C at the moment of unplugging mid-charge; the re-seat at unplug
 * (level_report) bounds what a wrong value costs to one correction. */
#define CHARGE_IR_MV 150u

static uint16_t c5_level_charging(uint16_t e) {
    if (c5_bottom(e)) return 0u;
    uint16_t mv = c5_to_mv(e);
    return curve(mv > CHARGE_IR_MV ? (uint16_t)(mv - CHARGE_IR_MV) : 0u);
}

#define LEVEL_UNKNOWN      0xFFFFu
#define LEVEL_FULL         1000u
/* Charging never claims 100: it creeps to 97.0%, which shows as 95, and only
 * the charger's own termination (FULL) says 100. Everything >= 97.5% therefore
 * comes from a full charge, so the panel's rounding needs no special case. */
#define LEVEL_CHARGE_CAP   970u
/* The countdown in the top clamp never goes below this: the clamp's exit is
 * 90%, and it is the voltage's job to say when the pack has got there. Just
 * above the exit, so with a whole-percent (or tenths) panel the countdown runs
 * into the voltage's 90 without a step. (It was 925 while the panel showed 5%
 * steps, the lowest level that still rounded to 95.) */
#define LEVEL_CLAMP_FLOOR  905u
/* Booted at the top clamp: the voltage says only "90..100", so take the level
 * saved before the reboot (every cable <-> BT slider flip is one) if it is in
 * that band. With nothing saved -- only after a flash erased it -- guess the
 * middle. Below the clamp the voltage is the authority and nothing is restored. */
#define LEVEL_CLAMP_GUESS  950u
#define LEVEL_CLAMP_EXIT   900u
/* The clamp's span at full white, from the two drain tests: run 1 left the
 * clamp at 89.6% after 5.36 equivalent hours (5.36 h * 3600 / 104 pm = 185.5
 * s/pm), run 2 at 90.3% after 4.90 h (4.90 * 3600 / 97.2 = 181.5); the mean,
 * 183.5, rounded. From FULL the countdown reaches its 905 floor in 95 * 184 s
 * = 4.86 h, about when the voltage takes over. It matters little: the clamp
 * is only ~10 points wide. (It was 190, from the 09-25 run's 5.3 h.) */
#define COUNTDOWN_S_PER_PM 184u
/* Charging at the clamp (the CV phase, 5.5 h on 09-28): creep toward the cap.
 * Faster below the clamp floor, at about a 1 A charge's real rate (0.4 %/min):
 * plugged in near full, `5C` jumps straight to 100 on the charge current's
 * I*R, and the curve never gets the chance to raise the level. */
#define CV_CREEP_S_PER_PM  180u
#define CV_CATCHUP_S_PER_PM 15u
/* On battery the level moves down only after the curve has sat below it for
 * this many fresh estimates in a row (~15 s: the 09-29 post-flash walk stepped
 * 1% about every 15 s), then to the HIGHEST of them. Every move
 * outside the deliberate re-seat is at most 1%, so a correction (a countdown
 * that ran slow, a charge rising fast) eases instead of jumping. */
#define STEP_DOWN_MEDIANS  6u
#define STEP_MAX_PM        10u

static uint16_t level      = LEVEL_UNKNOWN;
static uint8_t  below_run  = 0;
static uint16_t below_best = 0;
static uint16_t clamp_s    = 0;
static bool     full       = false;

/* What is saved, and when. ⚠️ Every internal-flash write blanks the whole LED
 * matrix for a few ms: the row ISR de-selects every row while the flash is in
 * its program state, because each per-line program window masks interrupts
 * (the vector table and the row ISR live in flash, and the M0 has no VTOR) and
 * a row left lit across one shows at ~18x (docs/leds.md). JD sees the blink;
 * 2026-09-29. Saving every whole-percent change was ~100
 * writes a discharge. The restore only uses a value in the clamp's 90..100
 * band, so save only there, in 2.5% steps, rounded DOWN so a restore never
 * overstates: FULL (on USB), then 97.5, 95, 92.5, 90 on battery, and one marker
 * below the band so a stale 9x is not restored after a pack swap. Six writes a
 * charge cycle; nothing while charging. */
#define LEVEL_SAVE_BELOW_BAND 890u

static uint16_t level_save_point(void) {
    if (level >= LEVEL_FULL) return LEVEL_FULL;
    if (level >= LEVEL_CLAMP_EXIT) return (uint16_t)((level / 25u) * 25u);
    return LEVEL_SAVE_BELOW_BAND;
}

static void level_save_tick(void) {   /* 1 Hz */
    if (level == LEVEL_UNKNOWN) return;
    if (full || supply == BATTERY_SUPPLY_BATTERY) kb_eeconfig_set_batt_level(level_save_point());
}

static uint16_t level_at_clamp_boot(void) {
    uint16_t saved;
    if (kb_eeconfig_get_batt_level(&saved) && saved >= LEVEL_CLAMP_EXIT) return saved;
    return LEVEL_CLAMP_GUESS;
}

static void level_toward(uint16_t target) {
    if (target > level) {
        level = (target - level > STEP_MAX_PM) ? (uint16_t)(level + STEP_MAX_PM) : target;
    } else if (target < level) {
        level = (level - target > STEP_MAX_PM) ? (uint16_t)(level - STEP_MAX_PM) : target;
    }
}

static void full_tick(void) {
    full = supply == BATTERY_SUPPLY_EXTERNAL && chrg_off_run >= FULL_SETTLE_TICKS &&
           c5_top(battery_5c_x100());
}

/* One new fresh estimate, in hundredths of a count. */
static void level_report(uint16_t m) {
    if (supply == BATTERY_SUPPLY_BATTERY) {
        if (c5_top(m)) {
            /* The top clamp: the countdown (level_tick_1hz) owns it, from
             * wherever charging left the level. */
            if (level == LEVEL_UNKNOWN) level = level_at_clamp_boot();
            below_run = 0;
            reseat    = false;
            return;
        }
        uint16_t target = c5_level(m);
        if (level == LEVEL_UNKNOWN || reseat) {
            /* First reading, or the first after a real charge: the charging
             * level carried a guessed I*R, and this estimate is post-unplug
             * reports only (c5_forget at the supply change). */
            level  = target;
            reseat = false;
            return;
        }
        if (target >= level) {     /* the ratchet: never rises on battery */
            below_run = 0;
            return;
        }
        if (below_run == 0 || target > below_best) below_best = target;
        if (++below_run < STEP_DOWN_MEDIANS) return;
        below_run = 0;
        level_toward(below_best);
        return;
    }
    below_run = 0;
    if (supply != BATTERY_SUPPLY_EXTERNAL) return;   /* UNKNOWN: touch nothing */

    if (full) {
        level = LEVEL_FULL;
        return;
    }
    if (c5_top(m)) {
        /* Charging in the CV phase, or idle on USB with a nearly full pack:
         * level_tick_1hz creeps it. */
        if (level == LEVEL_UNKNOWN) level = level_at_clamp_boot();
        return;
    }
    uint16_t target = charging_now() ? c5_level_charging(m) : c5_level(m);
    if (target > LEVEL_CHARGE_CAP) target = LEVEL_CHARGE_CAP;
    if (level == LEVEL_UNKNOWN) {
        level = target;
        return;
    }
    if (charging_now()) {
        /* Only rising, and only in a real charging session: `5C` reads the
         * terminal voltage, which charge current lifts, so a brief plug-in
         * would bump the level and then hand it back.
         * ⚠️ A level already above the cap is NOT pulled down to it: that
         * only happens within ~80 min of a FULL (the countdown from 1000 takes
         * that long to pass 975), so replugging a pack that was just full
         * keeps showing 100 through the charger's top-up -- which is true.
         * Dropping it to 95 for the top-up would read as a glitch. */
        if (session && target > level && level < LEVEL_CHARGE_CAP) level_toward(target);
        return;
    }
    /* On USB, charger idle or faulted, below the clamp: follow the estimate. */
    level_toward(target);
}

static void level_tick_1hz(void) {
    uint16_t m = battery_5c_x100();
    if (c5_top(m) && level != LEVEL_UNKNOWN) {
        if (supply == BATTERY_SUPPLY_BATTERY) {
            if (level > LEVEL_CLAMP_FLOOR && ++clamp_s >= COUNTDOWN_S_PER_PM) {
                clamp_s = 0;
                level--;
            }
            return;
        }
        if (charging_now() && session && !full) {
            uint16_t per = (level < LEVEL_CLAMP_FLOOR) ? CV_CATCHUP_S_PER_PM : CV_CREEP_S_PER_PM;
            if (level < LEVEL_CHARGE_CAP && ++clamp_s >= per) {
                clamp_s = 0;
                level++;
            }
            return;
        }
    }
    clamp_s = 0;
}

uint16_t battery_level_permille(void) { return level; }

uint8_t battery_level_pct(void) {
    if (level == LEVEL_UNKNOWN) return 0xFF;
    /* Whole percent. It was 5% steps, which JD found hid how fast the level was
     * falling -- the thing worth watching. (Tenths: BATTERY_SHOW_TENTHS.) */
    uint16_t p = (uint16_t)((level + 5u) / 10u);
    return (uint8_t)(p > 100u ? 100u : p);
}

battery_state_t battery_state(void) {
    switch (supply) {
        case BATTERY_SUPPLY_BATTERY:
            return BATTERY_STATE_BATTERY;
        case BATTERY_SUPPLY_EXTERNAL:
            if (full) return BATTERY_STATE_FULL;
            if (charging_now()) return BATTERY_STATE_CHARGING;
            return BATTERY_STATE_USB;
        default:
            return BATTERY_STATE_NONE;
    }
}

/* --- Low-battery protection ---------------------------------------------------
 * On the PACK voltage from `5C`. It used to compare VDD, which the buck-boost
 * pins at ~3.90 V, so it never fired: the 09-25 run went to the pack's own
 * protection trip with VDD still at ~3.87 V.
 *
 * Warn once after WARN_HOLD_TICKS under `warn_mv`; cut the RGB after the same
 * under `cut_mv`. A fresh 0 is "critically low", not "unknown": three in a row
 * cut at once. The cut is a power CAP (power_set_lights_cut), so the user's own
 * RGB toggle cannot undo it -- rgb_matrix_disable_noeeprom() could -- and it
 * lifts only after EXT_RESTORE_TICKS of external power. A stale reading holds
 * every timer and never clears the cut. This sheds load; it does not
 * disconnect the cell -- the pack's protector does that.
 *
 * The warning fires once per discharge. It re-arms only after a REAL charging
 * session (`session`: CHRG low 60 s), never on a voltage: a brief dump
 * plug-in, or a pack relaxing upward after one, used to re-arm it and re-fire
 * it on unplugging (10-01 01:06). A new threshold over HC_BATTCFG re-arms it
 * too, so it can be tested on demand.
 *
 * Thresholds are RAM-only and settable over HC_BATTCFG (pack mV); 0 disables
 * either. The warning's default is the curve's 10%: 3680 mV, ~5 h before the
 * cut at full white (it was 3550 mV, ~4.5%, ~2.3 h). The cut's is the curve's
 * 0%, 3400 mV. */
#define WARN_HOLD_TICKS    300u    /* 30 s at 10 Hz */
#define EXT_RESTORE_TICKS  50u     /* 5 s of external power */
#define CRITICAL_ZEROES    3u

static uint16_t warn_mv    = 3680;
static uint16_t cut_mv     = 3400;
static uint16_t warn_ticks = 0, cut_ticks = 0;
static bool     warned     = false;

static void cut_lights(void) {
    if (power_lights_cut()) return;
    power_set_lights_cut(true);
    display_set_alert("Low: RGB off");
}

static void protection_tick(void) {
    if (supply == BATTERY_SUPPLY_EXTERNAL) {
        warn_ticks = cut_ticks = 0;
        if (session) warned = false;
        if (power_lights_cut() && ext_evidence_ticks >= EXT_RESTORE_TICKS) power_set_lights_cut(false);
        return;
    }
    if (supply != BATTERY_SUPPLY_BATTERY) return;

    uint16_t m = battery_5c_x100();
    if (m == C5_EST_NONE) return;
    if (c5_zero_run >= CRITICAL_ZEROES) {
        cut_lights();
        /* At or under 3.161 V: the panel says "Low" now, not after the
         * ratchet's 1%-per-~15-s walk down from wherever the level was. */
        level = 0;
    }
    /* At the top clamp the pack is only known to be >= 4036 mV, which is not
     * "under" any threshold -- even one set above 4036. At the bottom clamp
     * the bound (<= 3161) IS under any threshold above it. */
    bool     top = c5_top(m);
    uint16_t mv  = c5_to_mv(m);

    if (warn_mv && !top && mv < warn_mv) {
        if (warn_ticks < WARN_HOLD_TICKS) warn_ticks++;
        if (warn_ticks >= WARN_HOLD_TICKS && !warned) {
            warned = true;
            display_set_alert("Battery low");
        }
    } else {
        warn_ticks = 0;
    }

    if (cut_mv && !top && mv < cut_mv) {
        if (cut_ticks < WARN_HOLD_TICKS) cut_ticks++;
        if (cut_ticks >= WARN_HOLD_TICKS) cut_lights();
    } else {
        cut_ticks = 0;
    }
}

uint8_t battery_flags(void) {
    uint8_t f = 0;
    if (supply == BATTERY_SUPPLY_BATTERY)  f |= BATT_F_ON_BATTERY;
    if (supply == BATTERY_SUPPLY_EXTERNAL) f |= BATT_F_EXTERNAL;
    if (!gpio_read_pin(CHARGE_CHRG_PIN))   f |= BATT_F_CHARGING;
    if (!gpio_read_pin(CHARGE_STDBY_PIN))  f |= BATT_F_DONE;
    if (rgb_matrix_is_enabled())           f |= BATT_F_RGB_ON;
    if (power_lights_cut())                f |= BATT_F_LIGHTS_CUT;
    if (ch582_is_usb())                    f |= BATT_F_SLIDER_USB;
    if (ch582_is_connected())              f |= BATT_F_LINKED;
    return f;
}

/* LED drive, per mille of the maximum, over the last second. sn32f2xx_led_load()
 * sums the driver's buffer, which is after the power cap: the real drive. */
#define LED_LOAD_MAX ((uint32_t)RGB_MATRIX_LED_COUNT * 3u * 255u)
static uint16_t led_pm = 0;
uint16_t battery_led_pm(void) { return led_pm; }

/* --- The log ---------------------------------------------------------------------
 * 720 ten-minute entries: 120 hours in RAM, oldest overwritten. Long enough for
 * a whole discharge at full white (~52-63 h) with room to spare. It records
 * what a curve fit and a load comparison need: every `5C` report in the period
 * (sum, count, min, max -- not the last value, which threw away 119 of 120),
 * flags at the end AND over the period, so a transition cannot fall between
 * samples, and the effective LED drive and backlight, which see the caps.
 * ⚠️ RAM, so it dies with the board: dump before the pack runs out, and plug in
 * with the slider on BT -- a BT -> cable flip resets the board. HC_BATTLOG. */
#define LOG_LEN       720u
#define LOG_PERIOD_MS 600000u

typedef struct __attribute__((packed)) {
    uint16_t vdd_avg;      /* mean raw VDD over the period, mV */
    uint16_t vdd_min;      /* lowest raw VDD, mV */
    uint16_t c5_sum;       /* sum of every fresh `5C` report in the period */
    uint8_t  c5_n_lo;      /* how many, low byte (0: none arrived) */
    uint8_t  c5_min;       /* 0xFF when none */
    uint8_t  c5_max;
    uint8_t  flags;        /* BATT_F_* at the end of the period */
    uint8_t  flags_any;    /* BATT_F_* OR-ed over every tick of the period */
    uint8_t  level;        /* the level at the end, in 0.5 % (0..200), 0xFF unknown */
    uint16_t led_pm;       /* mean LED drive, per mille of the maximum */
    uint8_t  bkl;          /* effective LCD backlight level at the end */
    uint8_t  c5_n_hi;      /* how many, high byte: 224-334 a period measured, which
                            * a byte (v3) nearly overflowed */
} log_entry_t;
_Static_assert(sizeof(log_entry_t) == BATTERY_LOG_ENTRY_SIZE, "HC_BATTLOG entry size");

static log_entry_t log_buf[LOG_LEN];
static uint16_t    log_head    = 0;   /* next slot to write */
static uint16_t    log_count   = 0;
static uint32_t    log_written = 0;   /* entries ever written */
static uint32_t    log_last_at = 0;   /* the newest entry's period end */
static bool        log_started = false;
static uint32_t    acc_sum = 0, acc_start = 0, acc_led = 0;
static uint16_t    acc_n = 0, acc_min = 0, acc_led_n = 0;
static uint8_t     acc_flags = 0;

static void log_task(uint16_t sample_mv) {
    if (sample_mv) {
        acc_sum += sample_mv;
        acc_n++;
        if (acc_min == 0 || sample_mv < acc_min) acc_min = sample_mv;
    }
    acc_flags |= battery_flags();
    if (!log_started) {
        acc_start   = timer_read32();
        log_started = true;
        return;
    }
    /* Every period closes on schedule, sample or not: the host dates entry n
     * as newest - (count-1-n) * period, so a period that stretched to cover an
     * ADC outage would misdate everything before it. */
    if (timer_elapsed32(acc_start) < LOG_PERIOD_MS) return;

    log_entry_t *e = &log_buf[log_head];
    e->vdd_avg   = acc_n ? (uint16_t)(acc_sum / acc_n) : 0u;   /* 0: no conversion in the period */
    e->vdd_min   = acc_min;
    e->c5_sum    = lg_c5_sum;
    e->c5_n_lo   = (uint8_t)(lg_c5_n & 0xFFu);
    e->c5_n_hi   = (uint8_t)(lg_c5_n >> 8);
    e->c5_min    = lg_c5_min;
    e->c5_max    = lg_c5_max;
    e->flags     = battery_flags();
    e->flags_any = acc_flags;
    e->level     = (level == LEVEL_UNKNOWN) ? 0xFFu : (uint8_t)(level / 5u);
    e->led_pm    = acc_led_n ? (uint16_t)(acc_led / acc_led_n) : led_pm;
    e->bkl       = display_backlight_effective();

    log_head = (uint16_t)((log_head + 1u) % LOG_LEN);
    if (log_count < LOG_LEN) log_count++;
    log_written++;
    acc_start  += LOG_PERIOD_MS;   /* a fixed cadence: no drift from loop jitter */
    log_last_at = acc_start;
    acc_sum = 0; acc_n = 0; acc_min = 0; acc_led = 0; acc_led_n = 0; acc_flags = 0;
    lg_c5_sum = 0; lg_c5_n = 0; lg_c5_min = C5_NONE; lg_c5_max = 0;
}

void battery_log_read(uint16_t idx, uint8_t *out) {
    uint32_t since  = log_count ? timer_elapsed32(log_last_at) : 0;
    uint16_t period = (uint16_t)(LOG_PERIOD_MS / 1000u);
    out[0] = BATTERY_LOG_VERSION;
    out[1] = (uint8_t)(log_count & 0xFF); out[2] = (uint8_t)(log_count >> 8);
    out[3] = (uint8_t)(idx & 0xFF);       out[4] = (uint8_t)(idx >> 8);
    for (uint8_t i = 0; i < 4; i++) out[5 + i] = (uint8_t)(since >> (8 * i));
    out[9]  = (uint8_t)(period & 0xFF);      out[10] = (uint8_t)(period >> 8);
    out[11] = (uint8_t)(log_written & 0xFF); out[12] = (uint8_t)((log_written >> 8) & 0xFF);
    uint8_t *dst = &out[13];
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
        warned     = false;   /* a new threshold is a new warning to give */
    }
    out[0] = (uint8_t)(warn_mv & 0xFF); out[1] = (uint8_t)(warn_mv >> 8);
    out[2] = (uint8_t)(cut_mv & 0xFF);  out[3] = (uint8_t)(cut_mv >> 8);
    out[4] = power_lights_cut() ? 1 : 0;
    out[5] = warned ? 1 : 0;
    out[6] = (uint8_t)((LOG_PERIOD_MS / 1000u) & 0xFF);
    out[7] = (uint8_t)((LOG_PERIOD_MS / 1000u) >> 8);
}

/* --- 10 Hz --------------------------------------------------------------------- */
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

    supply_tick(mv);
    if (!c5_fresh()) c5_forget();
    bool fresh_report = c5_new;
    c5_new            = false;
    full_tick();
    if (fresh_report) {
        uint16_t m = battery_5c_x100();
        if (m != C5_EST_NONE) level_report(m);
    }
    protection_tick();

    if (++tick >= 10u) {
        tick   = 0;
        led_pm = (uint16_t)((sn32f2xx_led_load() * 1000u) / LED_LOAD_MAX);
        acc_led += led_pm;
        acc_led_n++;
        level_tick_1hz();
        level_save_tick();
    }
    log_task(mv);
}
