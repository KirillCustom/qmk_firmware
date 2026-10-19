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
#define BATTERY_LOG_VERSION    7
#define BATTERY_LOG_ENTRY_SIZE 18
/* HC_BATTLOG's version byte while a deep sleep's entries are not yet laid in:
 * the reply's age would date the old entries hours too recently, so the host
 * retries. */
#define BATTERY_LOG_PENDING    0xFFu
#define BATTERY_PROTO_VERSION  5

/* The charging model's constants (Phase 1b, B; battery.c has the design).
 * Here so the simulator tests the values the firmware ships. */
#define BATTERY_MODEL_CAP     990u    /* per mille: the display's ceiling before FULL */
#define BATTERY_LOST_PM       50u     /* M - U beyond this is a contradiction */
#define BATTERY_LOST_S        1800u   /* ... for 30 min, accumulated: LOST */
#define BATTERY_STALE_LOST_S  600u    /* no fresh estimate for 10 min: LOST */
#define BATTERY_RISE_PM_PER_S 1u      /* the display rises at most this a second */
#define BATTERY_T_OVERRUN_S   3600u   /* termination this long after the model's: LOST */
/* After an unplug, or a pause's stop, the voltage relaxes; reports before
 * RELAX_S are not the pack. B3 trial 1 (10-07, 40 min from 2%) read the
 * level it implies at the unplug ~1 point high 6-10 min after, ~0.6 at 15 and
 * ~0.3 at 30: 30 min. Confirmed by B3's three relaxations (trial 1, the 10-08
 * recharge, trial 3): at 30 min the implied level sits 0.1-0.35 point above its
 * 60-min value. */
#define BATTERY_RELAX_S       1800u
/* A charger pause this long hands over to the idle path. Never less than
 * RELAX_S, so it is RELAX_S (the plan's 10 min predates B3's RELAX_S). */
#define BATTERY_PAUSE_S       1800u
/* FROM-FLAT: a boot on USB whose first estimate is <= 10.00 counts. That is
 * <= 3248 mV UNDER CHARGE, >= 150 mV below the 3400 mV cut (itself a loaded
 * voltage), with room for the voltmeter's +-11 mV. */
#define BATTERY_FLAT_X100     1000u

/* Phase 2's voltage band (battery.c, "The energy count", has why): the curve
 * this many mV above and below the estimate, widened BATTERY_BAND_PM.
 * Overridable for the simulator's sensitivity runs. */
#ifndef BATTERY_BAND_MV_UP
#    define BATTERY_BAND_MV_UP   25
#endif
#ifndef BATTERY_BAND_MV_DOWN
#    define BATTERY_BAND_MV_DOWN 40
#endif
#define BATTERY_BAND_PM          15

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

/* Phase 2's energy count (plans/PHASE2-BUILD-PLAN.md). The modelled draw over
 * the last second and its average (uW; the average over the minutes on
 * battery so far, an exponential one with a 12 h constant from 720 of them),
 * how many minutes it has, the time left to the cut at that average (minutes,
 * 0xFFFF none: on USB, the level unknown or cut, before the first minute),
 * the modelled energy since FULL (0.1 mWh), the count's state, and the
 * voltage band's edges in pm (-1 with no estimate below the clamp). */
enum {
    BATTERY_COUNT_OFF = 0,   /* not on battery */
    BATTERY_COUNT_RUN,       /* counting, inside or above the band */
    BATTERY_COUNT_CLAMP,     /* counting at the top clamp, which bounds nothing */
    BATTERY_COUNT_HELD,      /* below the band: waiting for the voltage */
    BATTERY_COUNT_RELAX,     /* after an unplug, until the re-seat */
    BATTERY_COUNT_CUT,       /* 0%: the protection's cut */
};
uint32_t battery_power_uw(void);
uint32_t battery_power_avg_uw(void);
uint16_t battery_avg_minutes(void);
uint16_t battery_time_left_min(void);
uint32_t battery_energy_used_mwh_x10(void);
uint8_t  battery_count_state(void);
int16_t  battery_band_lower(void);
int16_t  battery_band_upper(void);
/* Deep sleep (sleep.c; plans/DEEP-SLEEP-BUILD-PLAN.md, sections 4 and 6).
 * QMK's timer stands still asleep, so the sleep is handed over explicitly:
 *  - battery_sleep_enter(): just before the lock. The awake energy is
 *    integrated up to the sleep, and the state the log's entries need is
 *    noted (the flags, the level, the charging state);
 *  - battery_sleep_wake(): at the wake, before any other battery work. The
 *    log's open period is frozen and the live intake reset (the radio's `5C`
 *    reports and the samples after the wake land in it), the `5C` estimate
 *    forgotten, HC_BATTLOG pending, and no period closes;
 *  - battery_sleep_account(): once the slept time is known (the PCF's, or the
 *    counted seconds). The sleep's energy at BATTERY_P_SLEEP_UW goes into
 *    the count and the log, never into the awake average; the relaxation is
 *    aged; the log's entries are laid in on its own time base; time left is
 *    recomputed.
 * The asleep draw: 3.24 mA at 4.12 V on the logger over the hour asleep
 * (history/deep-sleep-20261019-1h/), assumed constant power. */
#define BATTERY_P_SLEEP_UW 13350u
void battery_sleep_enter(void);
void battery_sleep_wake(void);
void battery_sleep_account(uint32_t slept_ms);
bool battery_sleep_pending(void);   /* woken, the sleep not yet accounted */

/* Real charging sessions (CHRG low 60 s) since boot, wrapping: power.c re-arms
 * its low-battery notice on a change, as the warning re-arms on a session. */
uint16_t battery_session_count(void);
/* Provided by the board (ak820pro.c): USB configured by a host. */
bool     battery_usb_configured(void);

/* Status bits for HC_CONN and the log: see BATT_F_* in battery.c. */
uint8_t battery_flags(void);

/* Raw-HID readouts. battery_log_read fills `out` (29 bytes): version u8 (BATTERY_LOG_VERSION,
 * or BATTERY_LOG_PENDING while a sleep is being laid in),
 * stored count u16, the requested index u16, ms since the newest entry u32,
 * entries ever written (low 16 bits) u16, then the 18-byte entry at `idx`
 * (0 = oldest). v5 dropped the period from the reply to fit the entry: the
 * host reads it from battery_cfg's out[6..7]. battery_cfg applies op 1 (set warn/cut in PACK
 * mV, 0 disables either) and fills `out` (8 bytes) with warn u16, cut u16,
 * lights_cut u8, warned u8, and the log period in s (u16). */
void battery_log_read(uint16_t idx, uint8_t *out);
/* FROM-FLAT's test on the boot's first estimate, in hundredths of a count. */
bool battery_first_estimate_is_flat(uint16_t first_x100);
/* The charging model (Phase 1b, B): M in pm, 0xFFFF when none runs; the
 * charging state, 0 none, 1 UNKNOWN-CHG ("Charge"), 2 FROM-FLAT, 3 MODEL,
 * 4 LOST ("Charge"), 5 FULL; and the discharge curve, pm for a pack mV. */
uint16_t battery_model_pm(void);
uint8_t  battery_chg_state(void);
uint16_t battery_curve_pm(uint16_t mv);
void battery_cfg(uint8_t op, uint16_t warn_mv, uint16_t cut_mv, uint8_t *out);
