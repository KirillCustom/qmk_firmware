#include "watchdog_record.h"
// Copyright 2026 Fernando Birra, JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The raw-HID protocol: the RTC (0x10), flash-provisioning (0x11), host-text
 * (0x12) and health (0x13) channels, and the dispatch -- VIA builds route
 * through via_custom_value_command_kb, non-VIA through raw_hid_receive.
 * Moved verbatim from ak820pro.c in the phase-3 module split. */
#include "quantum.h"
#include "ak820pro.h"
#include "raw_hid.h"
#include "rtc/rtc.h"
#include "graphics/display.h"
#include "graphics/lcd_bus.h"
#include "health.h"
#include "kb_eeconfig.h"
#include "bluetooth/ch582f_ajazz.h"   /* HC_CONN readout + fault injection */
#include "watchdog.h"                 /* boot reset cause for HC_CONN */
#include "indicators.h"               /* battery_is_absent for HC_CONN */
#include "battery.h"                  /* VDD for HC_CONN */
#include "flash_stats.h"              /* HC_FLASHW */
#include "loop_acct.h"                /* HC_ACCT */
#include "power.h"                    /* HC_DEVHOOK op 14 */
#include "sleep.h"                    /* HC_DEVHOOK op 14, subs 3 and 4 */
#include "drivers/led/sn32f2xx.h"     /* scan-only, in sub 3's reply */

// Apply a 7-byte time payload to the RTC:
//   [0]=year-2000 [1]=month [2]=day [3]=weekday [4]=hour [5]=min [6]=sec
// Sets both the PCF8563 (persist) and the live SN32 clock; the display picks it up
// within a second via rtc_get_time(). Returns the PCF (persistence) write status.
static bool rtc_apply_bytes(const uint8_t *p) {
    /* Validate BEFORE touching hardware: these bytes go into a BATTERY-BACKED
     * part, so garbage persists across power cycles -- and dec2bcd(sec >= 80)
     * would even set the PCF8563's VL (voltage-low) flag via bit 7 of the
     * seconds register. Any local HID-capable process can send this packet;
     * "the host script is well-behaved" is not a guard. (Audit finding
     * IV-1, hardening-plan/findings-input-validation.md.) */
    if (p[0] > 99 ||               /* year -- the PCF stores year %100 but
                                    * reads reconstruct 2000+yy, so 100..255
                                    * would set one year live and persist
                                    * another (Codex phase-2 review #1) */
        p[1] < 1 || p[1] > 12 ||   /* month   */
        p[2] < 1 || p[2] > 31 ||   /* day     */
        p[3] > 6 ||                /* weekday */
        p[4] > 23 ||               /* hours   */
        p[5] > 59 ||               /* minutes */
        p[6] > 59) {               /* seconds */
        return false;
    }
    rtc_time_t t = {
        .year    = (uint16_t)(2000 + p[0]),
        .month   = p[1],
        .day     = p[2],
        .weekday = p[3],
        .hours   = p[4],
        .minutes = p[5],
        .seconds = p[6],
    };
    return rtc_set_time(&t);
}

// Clock-set command framing, identical for VIA and non-VIA builds so the host
// set-clock utility speaks ONE protocol. It's VIA's custom-value layout:
//   [SET_VALUE, RTC_CHANNEL, RTC_SET_TIME, year-2000, month, day, weekday,
//    hour, min, sec]
// The reply echoes the packet: data[0] stays SET_VALUE when handled, or becomes
// UNHANDLED (0xFF) when rejected. SET_VALUE/UNHANDLED mirror VIA's
// id_custom_set_value / id_unhandled so the same bytes work against either build.
enum {
    RTC_SET_VALUE = 0x07, // == VIA id_custom_set_value
    RTC_UNHANDLED = 0xFF, // == VIA id_unhandled
    RTC_CHANNEL   = 0x10,
    RTC_SET_TIME  = 0x01,
    /* Read the live clock back. Exists so post-set phase error is MEASURABLE:
     * without it the only way to see the offset is to film the panel next to a
     * screen showing `date` and step frames. Reply reuses the request buffer:
     *   [SET_VALUE, RTC_CHANNEL, RTC_GET_TIME, ok, yy, mm, dd, wday, hh, mm, ss]
     * `ok` is 1 when rtc_get_time() succeeded.
     *
     * Whole seconds only, deliberately -- the sub-second phase is recovered by
     * POLLING this fast and watching for the increment, which needs no extra
     * protocol and no sub-second field on the wire. */
    RTC_GET_TIME  = 0x02,
    /* Phase-correct set (clock-sync plan 3.7):
     *   [SET_VALUE, RTC_CHANNEL, RTC_SET_TIME_MS, yy mm dd wd hh mi ss,
     *    ms_lo ms_hi, flags, sof_bias_lo sof_bias_hi]
     * meaning "at the instant this packet was received, true time was
     * t + ms". Reply: [3] = RTC_SET_* status, [4..5] offset before the
     * correction (s16 ms, board vs target), [11] = RTC_PROTO_VERSION. */
    RTC_SET_TIME_MS = 0x03,
    RTC_PROTO_VERSION = 2,
};

static const uint8_t days_in_month[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

/* Full validation for RTC_SET_TIME_MS, BEFORE anything is written: calendar
 * incl. day-of-month and leap year, year 2026..2098 (the PCF stores year%100
 * and the +1 s branch must stay representable), ms <= 999, reserved flag
 * bits zero, sof_bias in +-600 ppm or the 0x7FFF "unknown" sentinel. */
static bool rtc_ms_payload_valid(const uint8_t *p) {
    uint8_t yy = p[0], mo = p[1], dd = p[2];
    if (yy < 26 || yy > 98) return false;
    if (mo < 1 || mo > 12) return false;
    uint8_t dim = days_in_month[mo - 1];
    uint16_t year = 2000 + yy;
    if (mo == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) dim = 29;
    if (dd < 1 || dd > dim) return false;
    if (p[3] > 6 || p[4] > 23 || p[5] > 59 || p[6] > 59) return false;
    uint16_t ms = (uint16_t)(p[7] | (p[8] << 8));
    if (ms > 999) return false;
    if (p[9] & 0xFC) return false;
    int16_t bias = (int16_t)(p[10] | (p[11] << 8));
    if (bias != 0x7FFF && (bias < -600 || bias > 600)) return false;
    return true;
}

static void rtc_set_ms_command(uint8_t *data, uint8_t length) {
    if (length < 15 || !rtc_ms_payload_valid(&data[3])) {
        data[3] = RTC_SET_REJECT;
        data[11] = RTC_PROTO_VERSION;
        return;
    }
    rtc_time_t t = {
        .year = (uint16_t)(2000 + data[3]), .month = data[4], .day = data[5],
        .weekday = data[6], .hours = data[7], .minutes = data[8], .seconds = data[9],
    };
    uint16_t ms    = (uint16_t)(data[10] | (data[11] << 8));
    uint8_t  flags = data[12] & (uint8_t)~RTC_SETF_INTERNAL;   /* the host's are never internal */
    int16_t  bias  = (int16_t)(data[13] | (data[14] << 8));
    int16_t  off   = 0;
    uint8_t  st    = rtc_set_time_ms(&t, ms, flags, bias, &off);
    memset(&data[3], 0, 29);
    data[3]  = st;
    data[4]  = (uint8_t)((uint16_t)off & 0xFF);
    data[5]  = (uint8_t)((uint16_t)off >> 8);
    data[11] = RTC_PROTO_VERSION;
}

static inline bool rtc_is_set_ms_cmd(const uint8_t *data, uint8_t length) {
    return length >= 3 && data[0] == RTC_SET_VALUE &&
           data[1] == RTC_CHANNEL && data[2] == RTC_SET_TIME_MS;
}

// ---------------------------------------------------------------------------
// Flash provisioning channel (Stage D). Same VIA custom-value framing as the
// clock above, on its own channel, so one host tool speaks one protocol:
//   [SET_VALUE, FLASH_CHANNEL, cmd, payload...]
// Replies are written back into the same buffer (VIA echoes it); data[3] is a
// status byte, with any returned data from data[4].
//
// Nothing here ever blocks on the chip. A page program is ~1-3 ms and a sector
// erase 50-300 ms; waiting for either inside the HID callback would stall the
// matrix scan (measured: a blocking erase costs ~6% of one scan window). So a
// command that needs an idle chip returns FS_BUSY and the host re-sends.
enum {
    FLASH_CHANNEL   = 0x11,
    // commands
    FC_INFO         = 0x01,  // -> jedec[3], asset_base[3]
    FC_ERASE        = 0x02,  // addr[3]            (4K sector)
    FC_WRITE_BEGIN  = 0x03,  // addr[3]
    FC_WRITE_DATA   = 0x04,  // len[1], bytes...
    FC_WRITE_END    = 0x05,  // flush a partial page
    FC_CRC32        = 0x06,  // addr[3], len[3]  -> crc[4]
    FC_STATUS       = 0x07,  // -> busy[1]
    FC_UNLOCK       = 0x08,  // on[1]  (animation slots)
    FC_CRC_NEXT     = 0x09,  // continue a running CRC -> crc[4] when done
    // status codes returned in data[3]
    FS_OK           = 0x00,
    FS_BUSY         = 0x01,  // chip busy -- resend this packet
    FS_REFUSED      = 0x02,  // write floor / locked / animation owns the bus
    FS_BADARG       = 0x03,
    FS_MORE         = 0x04,  // CRC still running -- send FC_CRC_NEXT
};

// CRC is computed in slices. Reading a whole range inside one HID callback
// blocks the matrix scan for the entire read -- a 184 KB verify measured a drop
// from ~1396 Hz to ~300 Hz. Everything else in this channel is non-blocking, so
// the CRC must be too: each call folds at most CRC_SLICE bytes (~0.3 ms of SPI)
// and returns FS_MORE until the range is consumed.
#define CRC_SLICE 1024u
static uint32_t crc_addr, crc_left, crc_acc;

// Streaming write state. Bytes accumulate here until a 256-byte page boundary,
// because the chip WRAPS rather than continuing when a program crosses one.
static uint32_t fw_addr  = 0;      // flash address of pg[0]
static uint16_t fw_fill  = 0;      // bytes buffered
static uint8_t  fw_pg[256];
static bool     fw_open  = false;

static uint8_t flash_flush_page(void) {
    if (!fw_fill) return FS_OK;
    if (flash_busy()) return FS_BUSY;
    if (!flash_page_program(fw_addr, fw_pg, fw_fill)) return FS_REFUSED;
    fw_addr += fw_fill;
    fw_fill  = 0;
    return FS_OK;
}

static void flash_command(uint8_t *data, uint8_t length) {
    uint8_t  cmd = data[2];
    uint8_t *p   = &data[3];
    uint32_t a   = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];

    switch (cmd) {
        case FC_INFO: {
            uint32_t id = flash_jedec_id();
            data[3] = FS_OK;
            data[4] = (uint8_t)(id >> 16); data[5] = (uint8_t)(id >> 8); data[6] = (uint8_t)id;
            data[7] = (uint8_t)(FLASH_ASSET_BASE >> 16);
            data[8] = (uint8_t)(FLASH_ASSET_BASE >> 8);
            data[9] = (uint8_t)(FLASH_ASSET_BASE);
            return;
        }
        case FC_STATUS:
            data[3] = FS_OK;
            data[4] = flash_busy() ? 1 : 0;
            return;

        case FC_UNLOCK:
            flash_set_unlocked(p[0] != 0);
            data[3] = FS_OK;
            return;

        case FC_ERASE:
            if (flash_busy())               { data[3] = FS_BUSY;    return; }
            data[3] = flash_erase_sector(a) ? FS_OK : FS_REFUSED;
            return;

        case FC_WRITE_BEGIN:
            // A page-aligned start keeps every later flush inside one page.
            if (a & 0xFFu)                  { data[3] = FS_BADARG;  return; }
            if (!flash_writable(a, 1))      { data[3] = FS_REFUSED; return; }
            fw_addr = a; fw_fill = 0; fw_open = true;
            data[3] = FS_OK;
            return;

        case FC_WRITE_DATA: {
            if (!fw_open)                   { data[3] = FS_BADARG;  return; }
            uint8_t n = p[0];
            if (n == 0 || n > length - 4)   { data[3] = FS_BADARG;  return; }
            // Buffer, flushing whenever a full page is ready. On FS_BUSY nothing
            // is consumed, so the host simply re-sends the identical packet.
            for (uint8_t i = 0; i < n; i++) {
                fw_pg[fw_fill++] = p[1 + i];
                if (fw_fill == sizeof fw_pg) {
                    uint8_t st = flash_flush_page();
                    if (st != FS_OK) { fw_fill -= (uint16_t)(i + 1); data[3] = st; return; }
                }
            }
            data[3] = FS_OK;
            return;
        }

        case FC_WRITE_END:
            data[3] = flash_flush_page();
            if (data[3] == FS_OK) fw_open = false;
            return;

        case FC_CRC32: {
            uint32_t len = ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5];
            if (!len)                       { data[3] = FS_BADARG;  return; }
            crc_addr = a; crc_left = len; crc_acc = 0xFFFFFFFFu;
        }
        /* fall through: fold the first slice immediately */
        case FC_CRC_NEXT: {
            if (!crc_left)                  { data[3] = FS_BADARG;  return; }
            uint32_t n = crc_left < CRC_SLICE ? crc_left : CRC_SLICE;
            crc_acc   = flash_crc32_acc(crc_acc, crc_addr, n);
            crc_addr += n;
            crc_left -= n;
            if (crc_left) { data[3] = FS_MORE; return; }
            uint32_t c = ~crc_acc;
            data[3] = FS_OK;
            data[4] = (uint8_t)(c >> 24); data[5] = (uint8_t)(c >> 16);
            data[6] = (uint8_t)(c >> 8);  data[7] = (uint8_t)c;
            return;
        }
        default:
            data[0] = RTC_UNHANDLED;
            return;
    }
}

static inline bool is_flash_cmd(const uint8_t *data, uint8_t length) {
    return length >= 4 && data[0] == RTC_SET_VALUE && data[1] == FLASH_CHANNEL;
}

/* --- Host text channel ----------------------------------------------------
 *   [SET_VALUE, TEXT_CHANNEL, TEXT_SET,   icon, bytes...]   up to 12 bytes
 *   [SET_VALUE, TEXT_CHANNEL, TEXT_CLEAR]
 *
 * One packet carries the whole payload: the band fits 12 glyphs and a raw-HID
 * report has ~27 usable bytes, so there is no framing to design. The firmware
 * assigns no meaning to the text -- a host script decides what it says.
 * Platform-agnostic on purpose: a launchd agent on macOS and a Task Scheduler
 * script on Windows produce the same bytes, so moving machines needs no reflash. */
enum {
    TEXT_CHANNEL = 0x12,
    TEXT_SET     = 0x01,
    TEXT_CLEAR   = 0x02,
    /* Per-line set: [.., TEXT_SET_LINE, line, icon, ASCII...].
     * A second line does not fit in one packet -- 32 bytes leaves ~27 for text
     * after framing, and two 16-char lines is 32 -- so each line gets its own
     * report. Torn updates are harmless: the lines are independently meaningful
     * (title / artist) and the producer polls every 3 s. */
    TEXT_SET_LINE = 0x03,
    /* Playback position: [.., TEXT_PLAYBACK, state, pos_hi, pos_lo, dur_hi,
     * dur_lo]. Seconds, big-endian, 16 bits -> 18.2 h, well past any track.
     * state 0 hands the band back to the clock. */
    TEXT_PLAYBACK = 0x04,
};

static inline bool is_text_cmd(const uint8_t *data, uint8_t length) {
    return length >= 3 && data[0] == RTC_SET_VALUE && data[1] == TEXT_CHANNEL;
}

static void text_command(uint8_t *data, uint8_t length) {
    switch (data[2]) {
        case TEXT_SET:
            /* data[3] = icon id, data[4..] = ASCII. Length is whatever the host
             * sent; display_set_text() clamps and sanitises. */
            if (length >= 4) {
                display_set_text(data[3], (const char *)&data[4],
                                 (uint8_t)(length - 4));
            }
            break;
        case TEXT_SET_LINE:
            /* data[3] = line, data[4] = icon, data[5..] = ASCII. */
            if (length >= 5) {
                display_set_text_line(data[3], data[4], (const char *)&data[5],
                                      (uint8_t)(length - 5));
            }
            break;
        case TEXT_PLAYBACK:
            if (length >= 8) {
                display_set_playback(data[3],
                                     (uint16_t)((data[4] << 8) | data[5]),
                                     (uint16_t)((data[6] << 8) | data[7]));
            }
            break;
        case TEXT_CLEAR:
            display_clear_text();
            break;
        default:
            data[0] = RTC_UNHANDLED;
            break;
    }
}

// ---------------------------------------------------------------------------
// Health channel: read the unified health counters over raw HID (health.c).
// Same VIA custom-value framing as the channels above:
//   [SET_VALUE, HEALTH_CHANNEL, HC_GET] -> [.., .., HC_GET, version, 28 bytes]
// Raw HID is the PRIMARY health readout -- it exists in every build flavor,
// where the console exists only in instrumented ones. Replies route through
// the active host driver, so like ak820ctl this needs the dip switch in wired
// mode; the counters themselves accumulate in any mode.
enum {
    HEALTH_CHANNEL = 0x13,
    HC_GET         = 0x01,
    /* Connection readout: [.., .., HC_CONN] ->
     *   [.., .., HC_CONN, conn_state, target_slot, battery, module_flags]
     * module_flags bit0 = connected, bit1 = pairing, bit2 = usb mode.
     * Exists so host scripts (and the CH582F fault-injection tests) can
     * assert link state without eyeballing the panel.
     * Then [7] boot RSTST, [8..11] RTC periods, and the battery (all u16 LE):
     *   [12..13] VDD mV, filtered   [14..15] last raw AIN16 code
     *   [16..17] VDD min mV         [18..19] VDD max mV (since boot)
     *   [20] charger pins: bit0 = CHRG level, bit1 = STDBY level (both
     *        active low)
     *   [21] the `5C` estimate rounded to a count (0xFF none)
     *   [22] s since the module's last `5C` (255 = stale or never)
     *   [23] battery_flags() (BATT_F_* in battery.c)
     *   [24..25] the level, per mille (0xFFFF unknown)
     *   [26..27] pack mV from that estimate (0 none; at an estimate of 0 or 100,
     *            the clamp's bound)      [28..29] LED drive, per mille
     *   [30] battery_state()
     *   [31] BATTERY_PROTO_VERSION: 3, 4 or 5; from 4 on it equals the
     *        log's version (HC_BATTLOG). Older firmware leaves the host's own
     *        byte here (0). From 5 the level reads 0xFFFF ("Charge") through
     *        a charge with no trusted start, and the log carries the charging
     *        model (Phase 1b, B). */
    /* Stall-measurement page 2 (LOOP-BUDGET-PLAN phase 1): [.., .., HC_GET2]
     * -> [.., .., HC_GET2, version, 28 bytes]. A SECOND page exists because
     * HC_GET's payload was already exactly full at 28 bytes -- there was no
     * room to extend it. */
    HC_GET2        = 0x04,
    /* Clear the resettable counters: [.., .., HC_RESET] -> echo. Watchdog
     * counters are boot facts and survive. Without this every reading carries
     * boot's deliberate blocking (lcd_init alone spends 240 ms in wait_ms)
     * and no measurement can be repeated. */
    HC_RESET       = 0x05,
    /* Per-row input sampling page. Added when this driver still published ONE
     * row per matrix_scan() call (fixed 2026-09-03); scan_rate on page 1 never
     * could answer how often a given key is actually looked at. */
    HC_GET3        = 0x06,
    /* Row-ISR cost page: entries, summed/min/max duration in system ticks, a
     * firmware timebase. Exists because the row rate is set by the ISR's own
     * duration (the counter is re-armed at its end), not by the timer -- and
     * the timer-derived figure was 4.8x wrong. */
    HC_GET4        = 0x07,
    HC_GET5        = 0x08, /* retained watchdog operation; watchdog_record.h */
    HC_GET6        = 0x09, /* crash-hunt vitals, protocol >= 7; health.h */
    HC_CONN        = 0x02,
    /* Battery log (battery.c), one entry per period (15 min from v6, 10
     * before): [.., ..,
     * HC_BATTLOG, idx_lo, idx_hi] -> [.., .., HC_BATTLOG, version, count u16,
     * idx u16, ms since the newest entry u32, entries ever written u16, the
     * entry at idx (0 = oldest)]. battery_log_read() and battery.h have the
     * layout. v5 (Phase 1b): an 18-byte entry (v4's 16 plus M and the
     * charging state), and the period no longer in the reply -- the host
     * reads it from HC_BATTCFG -- so 3 + 11 + 18 fills the 32-byte report.
     * v4: the period u16 before the written count, a 16-byte entry. Version
     * 1 (b35d8672b3) had no version byte and two 8-byte entries. v6 (Phase
     * 2): the period's modelled energy in vdd_min's place, 15-min periods. */
    HC_BATTLOG     = 0x0A,
    /* Low-battery thresholds, RAM only: [.., .., HC_BATTCFG, op, warn u16,
     * cut u16] with op 0 = read, 1 = set (0 mV disables either) ->
     * [.., .., HC_BATTCFG, op, warn u16, cut u16, lights_cut, warned,
     * log period s u16]. */
    HC_BATTCFG     = 0x0B,
    /* Phase 1b diagnostics, protocol >= 8. Paged: [.., .., cmd, page] ->
     * [.., .., cmd, page, 28 bytes]; an unknown page answers RTC_UNHANDLED.
     *   HC_FLASHW: internal-flash write sessions, programs and erases by
     *              writer, kb fields, the RTC period's proposals (C1;
     *              flash_stats.h has the pages).
     *   HC_ACCT:   per-task accounting of slow passes (D1; loop_acct.h).
     *              Page 0xF0 with [4] = 0/1 turns the accounting off/on,
     *              page 0xF1 clears it; both then answer page 0.
     *   HC_LINK:   [.., .., HC_LINK] -> [.., .., HC_LINK, version, u32 sent,
     *              timeouts, queue_full, giveups, replaced, u16 uart overrun,
     *              framing, parity, u16 reserved] -- every way the CH582F link
     *              can lose a frame (D1; ch582_link_stats). With a page
     *              0x21-0x2A at [3]: [.., .., HC_LINK, page, 28 bytes], the
     *              profile of slow ch582_task calls (D2 round 2, flash 1c;
     *              ch582_prof_fill). Firmware without it answers page 0's
     *              layout with [3] = the version (8), which a host checking the
     *              echoed page rejects -- hence pages from 0x21. */
    HC_FLASHW      = 0x0C,
    HC_ACCT        = 0x0D,
    HC_LINK        = 0x0E,
    /* Phase 2's power figures (battery.c's energy count, power.c's ladder;
     * plans/PHASE2-BUILD-PLAN.md): [.., .., HC_POWER] -> [.., .., HC_POWER,
     * version (1), mode, stage, count state (BATTERY_COUNT_*), level pm u16
     * (0xFFFF unknown), the draw now u32 uW, its average u32 uW, the
     * average's minutes u16, time left u16 min (0xFFFF none), energy since
     * FULL u32 0.1 mWh, the band's lower and upper edges i16 pm (-1 none),
     * on battery u8] at [3..29]. HC_CONN is full, hence its own command. */
    HC_POWER       = 0x0F,
    /* Clock-sync status (PLAN.md 3.7): [.., .., HC_RTC, page] ->
     *   [.., .., HC_RTC, page, block...]
     * page 1 = the 21-byte RTC_GET_TIME[11..31] tail at [4..24];
     * page 2 = 28 bytes of counters at [4..31] (stale_count u16,
     *   i2c_fail u16, deferred_passes u16, i2c_max_cycles u32,
     *   window_rejects u16, ref_transitions u16, isr_lat min/max/mean/n
     *   u16 x4, sof_d_zero u16, sof_d_reject u16, sizeof(time_t) u8,
     *   usb_active|fn_valid<<1 u8);
     * page 3 = the last 14 FRMNO-per-second deltas, u16 each. */
    HC_RTC         = 0x03,
#ifdef WDT_TEST_HOOKS
    /* Phase 0 hardware-fact tests (rtc_test_op): [.., .., HC_RTCTEST, op,
     * args...] -> [.., .., HC_RTCTEST, op, reply...]. These deliberately
     * move the RTC phase / PCF registers -- resync afterwards. */
    HC_RTCTEST     = 0x7A,
    /* Fault injection: [.., .., HC_INJECT, len, bytes...] -- feed up to 27
     * bytes to the CH582F parser as if received from the module. */
    HC_INJECT      = 0x7D,
    /* Outbound A6 trace: -> [.., .., HC_TXTRACE, count_lo, count_hi, n,
     * params...(n, newest last)]. The observable for pending-action tests. */
    HC_TXTRACE     = 0x7C,
    /* Drive the user-command entry points so pending actions can be ARMED
     * from a host test: [.., .., HC_DRIVE, op, arg]. op 1 = set_profile(arg),
     * 2 = enter_pairing, 3 = cancel_connect. Test builds only -- these
     * 4 = rx_mute(arg) -- discard real module bytes so injected traffic is
     * deterministic. Test builds only -- these really do move the module. */
    HC_DRIVE       = 0x7B,
#endif
#ifdef WDT_TEST_HOOKS
    /* Test-only, instrumented builds: deliberately wedge the main loop to
     * prove the watchdog resets the board and the boot accounting works.
     *   [SET_VALUE, HEALTH_CHANNEL, HC_STALL, mode]
     * mode 1: spin forever (interrupts still running -- the historical hang
     *         signature). mode 2: force a kb-eeconfig flash write, then spin,
     *         so the reset lands as close after a program cycle as this test
     *         can arrange. The reply never arrives, by design. */
    HC_STALL       = 0x7E,
    /* Test-only, instrumented builds: prove the terminal records end to end.
     *   [SET_VALUE, HEALTH_CHANNEL, HC_FAULT, mode]
     * mode 1: UDF in thread context -> hard_fault, exception 0.
     * mode 2: a fault INSIDE an ISR (Vector50, test builds only) -> hard_fault,
     *         exception 20 -- the MSP path mode 1 never takes.
     * mode 3: pend a vector nobody handles (I2S0) -> unhandled_exception, 19.
     * mode 4: mode 1, then fault again inside HardFault -> LOCKUP. Answers
     *         whether the watchdog recovers a locked-up core; if it does not,
     *         the way out is a cold power-off.
     * mode 5: write 1 KB of stack and return normally -- page 6's PSP
     *         watermark must move.
     * mode 6: commit a halt record from THREAD context, then wedge.
     * mode 7: the same, then fault: the handler finds the record written and
     *         must leave it alone. 6 and 7 split the commit from the
     *         exception; they found the lost final write (watchdog_record.c,
     *         commit_terminal) and now guard it.
     * Modes 1-4, 6 and 7 never reply, by design, and are refused while
     * degraded. The record's parent is test_fault (this scope), not raw_hid. */
    HC_FAULT       = 0x7F,
    /* Test-only: read one aligned RAM word. [.., .., HC_PEEK, a0..a3 LE] ->
     * [.., .., HC_PEEK, v0..v3 LE]. SRAM only. */
    HC_PEEK        = 0x78,
    /* Test-only: [.., .., HC_BLITFAULT, n] drops the completion interrupt of
     * the next n (<= 3) blits (lcd_bus.c) -- proves the lost-completion
     * recovery and the display's repaint of a blit given up for good. */
    HC_BLITFAULT   = 0x77,
#endif
#if defined(WDT_TEST_HOOKS) || defined(AK820_RAW_BOOTLOADER)
    /* Jump to the bootloader, so a flash needs no Fn+Esc (flash.sh sends it
     * through hostagent/ak820bootloader.py after its backups):
     *   [SET_VALUE, HEALTH_CHANNEL, HC_BOOTLOADER, 'B', 'O', 'O', 'T']
     * The four bytes keep a stray or garbled frame from jumping; anything else
     * is unhandled. It replies only to refuse: [.., .., HC_BOOTLOADER, 'B',
     * 'U', 'S', 'Y', why], why 1 = a key moved in the last 2 s, 2 = the BT
     * releases were not delivered; the host retries. Instrumented builds, and `dev` builds
     * (build.sh) for JD's board while development lasts. ⚠️ Never daily or
     * fpb, the flavors releases ship: any process that can open raw HID could
     * then drop the board into a bootloader that looks dead (build.sh checks
     * for the marker below). VIA's own jump command is gone from QMK; this is
     * not a replacement for it. */
    HC_BOOTLOADER  = 0x79,
#endif
#ifdef AK820_DEV_HOOKS
    /* Dev builds only (build.sh dev), for unattended display testing:
     *   [.., .., HC_DEVHOOK, op, arg]
     * op 1: press Fn+D (display_debug_toggle), so the bench can cycle the
     *       pages and exit to the dashboard with no hands;
     * op 2: the display probe, page arg (0 or 1): 14 u16 worst times in
     *       microseconds at [4..31] (display.c, DP_*);
     * op 3: reset the probe.
     * op 4: set the panel backlight level to arg (0..max, clamped), RAM only:
     *       display_set_brightness() never persists (the keys' paths do), so
     *       a reboot restores the saved level and no flash write blinks the
     *       LEDs. For the bench's per-load current map (docs/test-bench.md).
     * op 5: panel backlight off (arg 0) or on (arg != 0), as Fn+Home does but
     *       without its overlay: nothing is drawn and nothing persists.
     * op 6: read only.
     * Ops 4-6 reply [level, max, on, effective, saved+1] at [4..8]: effective
     * is after the power cap (power.c) and 0 when off; saved+1 is the level
     * the keys last persisted, 0 if none.
     * op 7: the inbound 5A trace's header (ch582f_ajazz.c), at [4..18]:
     *       [frames u32][accepted u32][host_leds][last byte][ms since the
     *       last frame u32, UINT32_MAX if none][ring size]. LE, like the rest.
     * op 8: its ring, page arg (0..3): [4] the page again, [5] 0, then slots
     *       arg*13 .. arg*13+12 as {byte, gap ms} pairs at [6..31]; frame n is
     *       in slot n % ring size. Op 7, the pages, then op 7 again, is a
     *       consistent read only if the count did not move between them.
     * op 9: reset the trace.
     * op 10: the lock-LED channel's receiver (lockchan.c), at [4..28]:
     *       [frames u32][preambles u32][busy][last frame: profile, type, seq,
     *       len][its anchor ms u32][its commit ms u32][the board's ms now u32].
     * op 11: its abort counts, LOCKCHAN_NABORTS u16s from [4], in
     *       lockchan_abort_t order.
     * op 12: the last frame's payload, page arg (0..1): [4] the page again,
     *       [5] 0, then payload[arg*26 .. arg*26+25] at [6..31].
     * op 13: zero the receiver's counters and last frame.
     * For the lock-LED channel's tests (plans/BACKLOG.md): what the module
     * passes on from the host's LED report, with no hands and no display.
     * op 14: the power ladder (power.c), sub-op at [4]: 0 read; 1 set the mode
     *       to [5] (RAM and the coalesced EEPROM write, no overlay); 2 force
     *       the stage to [5], 0xFF to release it (RAM only; the battery-only
     *       rule does not apply to a forced stage). Replies [mode][stage]
     *       [val cap][forced stage, 0xFF none][on battery] at [4..8]. For the
     *       bench's load map of the stages (PHASE2-BUILD-PLAN.md).
     * op 15: the RX tap (ch582f_ajazz.c): every raw byte from the module
     *       while armed. arg 0 reads, 1 clears and arms, 2 stops; any other
     *       arg is unhandled. Replies [armed][bytes since arming u32][ring
     *       size u16] at [4..10]. For an undocumented module frame type (the
     *       2.4G dongle's FF60 interface, plans/GREMLIN-RESULTS.md).
     * op 16: its ring, page arg (0..9): [4] the page again, [5] 0, then
     *       slots arg*26 .. arg*26+25 at [6..31]; byte n is in slot n % ring
     *       size. Op 15 (arg 0), the pages, then op 15 again, is a consistent
     *       read only if the count did not move between them.
     * Anything else is unhandled. */
    HC_DEVHOOK     = 0x76,
#endif
};

#ifdef AK820_DEV_HOOKS
/* build.sh's flavor check: present exactly when HC_DEVHOOK is compiled in. */
const volatile uint8_t ak820_dev_hooks = 1;

/* HC_DEVHOOK ops 4-6's reply, at out[0..4]. */
static void devhook_lcd_state(uint8_t *out) {
    uint8_t saved;
    out[0] = display_get_brightness();
    out[1] = display_get_brightness_max();
    out[2] = display_get_power() ? 1 : 0;
    out[3] = display_backlight_effective();
    out[4] = kb_eeconfig_get_lcd_brightness(&saved) ? (uint8_t)(saved + 1) : 0;
}
#endif

#if defined(WDT_TEST_HOOKS) || defined(AK820_RAW_BOOTLOADER)
/* build.sh's flavor check: present exactly when HC_BOOTLOADER is compiled in.
 * The jump reads it, through the volatile, so --gc-sections cannot drop it;
 * `used` alone keeps it in the object file but not in the link. */
const volatile uint8_t ak820_raw_bootloader = 1;
#endif
#define HEALTH_PROTO_VERSION 8   /* 8: HC_FLASHW, HC_ACCT, HC_LINK (Phase 1b) */

#ifdef WDT_TEST_HOOKS
extern volatile uint8_t wdt_test_fault_in_handler;   /* fault.c */

/* HC_FAULT mode 2's interrupt: I2S1 is unused on this board, so its vector is
 * free for a test-build-only handler that faults in handler mode. */
void Vector50(void);
void Vector50(void) { __asm__ volatile("udf #0"); }

/* HC_FAULT mode 5: WRITE every byte, or a frame that only reserves space leaves
 * the paint intact and the watermark cannot see it (finding 10). */
static __attribute__((noinline)) void test_deep_stack(void) {
    volatile uint8_t deep[1024];
    for (unsigned i = 0; i < sizeof(deep); i++) deep[i] = (uint8_t)i;
}
#endif

static inline bool is_health_cmd(const uint8_t *data, uint8_t length) {
    return length >= 3 && data[0] == RTC_SET_VALUE && data[1] == HEALTH_CHANNEL;
}

static void health_command(uint8_t *data, uint8_t length) {
    switch (data[2]) {
        case HC_GET:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                health_fill(&data[4]);   /* 28 bytes: exactly fills the report */
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_GET2:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                health_fill2(&data[4]);   /* 28 bytes: exactly fills the report */
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_GET3:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                health_fill3(&data[4]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_GET4:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                health_fill4(&data[4]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_GET5:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                watchdog_record_fill(&data[4]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_GET6:
            if (length >= 32) {
                data[3] = HEALTH_PROTO_VERSION;
                health_fill6(&data[4]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_RESET:
            health_reset();
            break;
        case HC_FLASHW:
            if (length < 32 || !flash_stats_fill(data[3], &data[4])) data[0] = RTC_UNHANDLED;
            break;
        case HC_ACCT:
            if (length < 32) {
                data[0] = RTC_UNHANDLED;
                break;
            }
            if (data[3] == 0xF0) {
                loop_acct_set_enabled(data[4] != 0);
                data[3] = 0;
            } else if (data[3] == 0xF1) {
                loop_acct_reset();
                data[3] = 0;
            }
            if (!loop_acct_fill(data[3], &data[4])) data[0] = RTC_UNHANDLED;
            break;
        case HC_LINK:
            if (length >= 32 && data[3] != 0) {
                /* Pages 0x21-0x2A: D2's profile of slow ch582_task calls (flash
                 * 1c), echoing the page; page 0 (what older hosts send) is
                 * below. Any other nonzero page is unhandled. */
                if (ch582_prof_fill(data[3], &data[4])) {
                    /* data[3] stays the page */
                } else {
                    data[0] = RTC_UNHANDLED;
                }
            } else if (length >= 32) {
                ch582_link_stats_t ls;
                ch582_link_stats(&ls);
                uint8_t *p = &data[4];
                const uint32_t w[5] = {ls.sent, ls.timeouts, ls.queue_full, ls.giveups, ls.replaced};
                for (uint8_t i = 0; i < 5; i++)
                    for (uint8_t b = 0; b < 4; b++) *p++ = (uint8_t)(w[i] >> (8 * b));
                const uint32_t h[3] = {ls.uart_overrun, ls.uart_framing, ls.uart_parity};
                for (uint8_t i = 0; i < 3; i++) {
                    uint16_t x = h[i] > 0xFFFFu ? 0xFFFFu : (uint16_t)h[i];
                    *p++ = (uint8_t)x; *p++ = (uint8_t)(x >> 8);
                }
                *p++ = 0; *p++ = 0;
                data[3] = HEALTH_PROTO_VERSION;
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_CONN:
            data[3] = (uint8_t)ch582_get_conn_state();
            data[4] = ch582_get_target_slot();
            data[5] = ch582_get_battery();
            data[6] = (ch582_is_connected() ? 1 : 0) |
                      (ch582_is_pairing()   ? 2 : 0) |
                      (ch582_is_usb()       ? 4 : 0) |
                      /* bit3: no pack fitted. The level in data[5] reads 0 in
                       * that case and cannot be told from a flat cell without
                       * it -- see indicators.c. */
                      (battery_is_absent()  ? 8 : 0);
            /* Boot reset cause (raw RSTST bits): names what kind of reset a
             * slider flip produces (POR vs LVD brownout vs external). */
            data[7] = watchdog_boot_rstst();
            /* RTC divider periods, LE u16: [8..9] persisted seed (0 = unset),
             * [10..11] live trimmed register value. A large gap between them,
             * or either far from ~33600 on this unit, explains a clock that
             * runs fast/slow from boot. */
            {
                uint16_t sp = kb_eeconfig_get_rtc_period();
                uint16_t lp = (uint16_t)rtc_get_period();
                data[8]  = (uint8_t)(sp & 0xFF); data[9]  = (uint8_t)(sp >> 8);
                data[10] = (uint8_t)(lp & 0xFF); data[11] = (uint8_t)(lp >> 8);
            }
            {
                uint16_t w[4] = {battery_vdd_mv(), battery_vdd_raw(),
                                 battery_vdd_min_mv(), battery_vdd_max_mv()};
                for (uint8_t i = 0; i < 4; i++) {
                    data[12 + 2 * i] = (uint8_t)(w[i] & 0xFF);
                    data[13 + 2 * i] = (uint8_t)(w[i] >> 8);
                }
                data[20] = (gpio_read_pin(CHARGE_CHRG_PIN)  ? 1 : 0) |
                           (gpio_read_pin(CHARGE_STDBY_PIN) ? 2 : 0);
                data[21] = battery_5c_rounded();
                uint32_t age = ch582_battery_age_ms();
                data[22] = (age == UINT32_MAX || age / 1000u > 254u) ? 255u : (uint8_t)(age / 1000u);
                data[23] = battery_flags();
                uint16_t e[3] = {battery_level_permille(), battery_pack_mv(), battery_led_pm()};
                for (uint8_t i = 0; i < 3; i++) {
                    data[24 + 2 * i] = (uint8_t)(e[i] & 0xFF);
                    data[25 + 2 * i] = (uint8_t)(e[i] >> 8);
                }
                data[30] = (uint8_t)battery_state();
                data[31] = BATTERY_PROTO_VERSION;
            }
            break;
        case HC_BATTLOG:
            if (length >= 32) battery_log_read((uint16_t)(data[3] | (data[4] << 8)), &data[3]);
            break;
        case HC_POWER:
            if (length >= 32) {
                /* The page, read before the memset: 0 is the reply as it has
                 * always been (older hosts send 0 there), 1 the deep sleep's
                 * figures, which begin 0x81 so an older firmware's page-0
                 * reply (1) is never read as them (codex, deep-sleep review). */
                uint8_t page = data[3];
                memset(&data[3], 0, 29);
                if (page == 1) {
                    _Static_assert(SLEEP_PAGE1_SIZE <= 29u, "HC_POWER page 1 must fit");
                    sleep_page1_fill(&data[3]);
                    break;
                }
                uint8_t *p = &data[3];
#define PUT16(x) do { uint16_t v_ = (uint16_t)(x); *p++ = (uint8_t)(v_ & 0xFF); *p++ = (uint8_t)(v_ >> 8); } while (0)
#define PUT32(x) do { uint32_t v_ = (uint32_t)(x); for (uint8_t k_ = 0; k_ < 4; k_++) *p++ = (uint8_t)(v_ >> (8 * k_)); } while (0)
                *p++ = 1;   /* the reply's version */
                *p++ = (uint8_t)power_mode();
                *p++ = (uint8_t)power_stage();
                *p++ = battery_count_state();
                PUT16(battery_level_permille());
                PUT32(battery_power_uw());
                PUT32(battery_power_avg_uw());
                PUT16(battery_avg_minutes());
                PUT16(battery_time_left_min());
                PUT32(battery_energy_used_mwh_x10());
                PUT16((uint16_t)battery_band_lower());
                PUT16((uint16_t)battery_band_upper());
                *p++ = battery_on_battery() ? 1 : 0;
#undef PUT16
#undef PUT32
            }
            break;
        case HC_BATTCFG:
            if (length >= 32) {
                uint8_t  op = data[3];
                uint16_t w  = (uint16_t)(data[4] | (data[5] << 8));
                uint16_t c  = (uint16_t)(data[6] | (data[7] << 8));
                data[3] = op;
                battery_cfg(op, w, c, &data[4]);
            }
            break;
        case HC_RTC:
            if (length >= 32) {
                uint8_t page = data[3];
                memset(&data[4], 0, 28);
                rtc_status_fill(page, &data[4]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
#ifdef WDT_TEST_HOOKS
        case HC_RTCTEST:
            if (length >= 32) {
                uint8_t op = data[3];
                uint8_t arg[8];
                memcpy(arg, &data[4], sizeof arg);   /* the reply overwrites [3..] */
                rtc_test_op(op, arg, &data[3]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_INJECT:
            if (length >= 4 && data[3] <= length - 4) {
                ch582_inject(&data[4], data[3]);
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_TXTRACE: {
            uint16_t cnt;
            uint8_t  n = ch582_a6_trace(&data[6], &cnt);
            data[3] = (uint8_t)(cnt & 0xFF);
            data[4] = (uint8_t)(cnt >> 8);
            data[5] = n;
            break;
        }
        case HC_DRIVE:
            if (length >= 5) {
                switch (data[3]) {
                    case 1: ch582_set_profile((ch582_profile_t)data[4]); break;
                    case 2: ch582_enter_pairing();                       break;
                    case 3: ch582_cancel_connect();                      break;
                    case 4: ch582_rx_mute(data[4] != 0);                 break;
                    default: data[0] = RTC_UNHANDLED;                    break;
                }
            } else {
                data[0] = RTC_UNHANDLED;
            }
            break;
        case HC_STALL: {
            WDT_SCOPE(WDT_SITE_TEST_STALL);
            /* With the watchdog off (degraded) a wedge is permanent: refuse. */
            if (watchdog_degraded()) { data[0] = RTC_UNHANDLED; break; }
            if (length >= 4) {
                if (data[3] == 2) {
                    kb_eeconfig_test_write();   /* a REAL flash program first */
                }
                for (;;) { /* wedge: the watchdog must get us out of here */ }
            }
            break;
        }
        case HC_PEEK: {
            if (length < 7) { data[0] = RTC_UNHANDLED; break; }
            uint32_t a = (uint32_t)data[3] | ((uint32_t)data[4] << 8) |
                         ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 24);
            if ((a & 3u) || a < 0x20000000u || a > 0x20007FFCu) { data[0] = RTC_UNHANDLED; break; }
            uint32_t v = *(volatile uint32_t *)a;
            data[3] = v; data[4] = v >> 8; data[5] = v >> 16; data[6] = v >> 24;
            break;
        }
        case HC_BLITFAULT: {
            extern volatile uint8_t lcd_test_drop_tc;
            if (length >= 4) lcd_test_drop_tc = data[3] > 3 ? 3 : data[3];
            data[3] = lcd_test_drop_tc;
            break;
        }
        case HC_FAULT: {
            WDT_SCOPE(WDT_SITE_TEST_FAULT);
            if (length < 4) break;
            /* Three test resets inside ten minutes reach degraded mode and the
             * watchdog stays OFF for the boot: a fault then spins forever, and
             * the lockup test would "fail" for the wrong reason (implementation
             * review, finding 2). Mode 5 never resets, so it is allowed. */
            if (watchdog_degraded() && data[3] != 5) { data[0] = RTC_UNHANDLED; return; }
            switch (data[3]) {
                case 4:
                    wdt_test_fault_in_handler = 1;
                    __attribute__((fallthrough));
                case 1:
                    __asm__ volatile("udf #0");
                    break;
                case 2:
                    NVIC_EnableIRQ(I2S1_IRQn);
                    NVIC_SetPendingIRQ(I2S1_IRQn);
                    break;
                case 3:
                    NVIC_EnableIRQ(I2S0_IRQn);
                    NVIC_SetPendingIRQ(I2S0_IRQn);
                    break;
                case 5:
                    test_deep_stack();
                    return;   /* the one mode that replies */
                case 6:   /* commit from THREAD context, then a plain wedge */
                    watchdog_record_stop(WDT_SITE_HALT, 0, 0xCAFE0006u);
                    break;
                case 7:   /* commit from thread context, THEN fault: the
                           * handler finds ready == false and only spins */
                    watchdog_record_stop(WDT_SITE_HALT, 0, 0xCAFE0007u);
                    __asm__ volatile("udf #0");
                    break;
                default:
                    data[0] = RTC_UNHANDLED;
                    return;
            }
            for (;;) { /* not reached if the fault fired; the watchdog if not */ }
        }
#endif
#ifdef AK820_DEV_HOOKS
        case HC_DEVHOOK:
            if (!ak820_dev_hooks || length < 5) { data[0] = RTC_UNHANDLED; break; }
            switch (data[3]) {
                case 1: display_debug_toggle(); break;
                case 2: { uint8_t pg = data[4]; display_probe_fill(pg, &data[4]); break; }
                case 3: display_probe_reset(); break;
                case 4: display_set_brightness(data[4]); devhook_lcd_state(&data[4]); break;
                case 5: display_set_power(data[4] != 0); devhook_lcd_state(&data[4]); break;
                case 6: devhook_lcd_state(&data[4]); break;
                case 7:
                    if (length < 32) { data[0] = RTC_UNHANDLED; break; }
                    memset(&data[4], 0, 28);
                    ch582_led5a_fill(&data[4]);
                    break;
                case 8:
                    if (length < 32 || !ch582_led5a_page(data[4], &data[4])) data[0] = RTC_UNHANDLED;
                    break;
                case 9: ch582_led5a_reset(); break;
                case 10: {
                    if (length < 32) { data[0] = RTC_UNHANDLED; break; }
                    const lockchan_t *lc  = ch582_lockchan();
                    uint32_t          now = timer_read32();
                    memset(&data[4], 0, 28);
                    memcpy(&data[4], &lc->frames, 4);
                    memcpy(&data[8], &lc->preambles, 4);
                    data[12] = lockchan_busy(lc) ? 1 : 0;
                    data[13] = lc->last.profile;
                    data[14] = lc->last.type;
                    data[15] = lc->last.seq;
                    data[16] = lc->last.len;
                    memcpy(&data[17], &lc->last.anchor_ms, 4);
                    memcpy(&data[21], &lc->last.commit_ms, 4);
                    memcpy(&data[25], &now, 4);
                    break;
                }
                case 11:
                    if (length < 32) { data[0] = RTC_UNHANDLED; break; }
                    memset(&data[4], 0, 28);
                    memcpy(&data[4], ch582_lockchan()->aborts, sizeof(ch582_lockchan()->aborts));
                    break;
                case 12: {
                    uint8_t pg = data[4];
                    if (length < 32 || pg > 1) { data[0] = RTC_UNHANDLED; break; }
                    const uint8_t *src = ch582_lockchan()->last.payload;
                    memset(&data[4], 0, 28);
                    data[4] = pg;
                    memcpy(&data[6], &src[pg * 26], pg ? LOCKCHAN_MAX_PAYLOAD - 26 : 26);
                    break;
                }
                case 13: ch582_lockchan_clear(); break;
                case 14: {
                    if (length < 32) {   /* before reading the arguments (codex) */
                        data[0] = RTC_UNHANDLED;
                        break;
                    }
                    uint8_t sub = data[4], arg = data[5];
                    if (sub > 6 || (sub == 1 && arg >= POWER_MODE_COUNT) ||
                        (sub == 2 && arg != 0xFF && arg > POWER_STAGE_SCREEN_OFF) ||
                        (sub == 3 && (arg & ~SLEEP_LS_ALL)) || (sub == 4 && arg > 2) || (sub == 6 && arg != 1)) {
                        data[0] = RTC_UNHANDLED;
                        break;
                    }
                    if (sub == 3) {
                        /* Light sleep's sizing bits (sleep.h): arg the mask, [6]
                         * the yield in ms, [7] PMU Sleep's window in s, [8] the
                         * mask's own end in s. Reply: 3, the mask, the bits in
                         * force, the WFI count (u32), scan-only in force, the
                         * yield in effect. */
                        sleep_light_set(arg, data[6], data[7], data[8]);
                        uint32_t w = ak820_wfi_count;
                        memset(&data[4], 0, 28);
                        data[4]  = 3;
                        data[5]  = sleep_light_mask();
                        data[6]  = sleep_light_applied();
                        data[7]  = (uint8_t)w;
                        data[8]  = (uint8_t)(w >> 8);
                        data[9]  = (uint8_t)(w >> 16);
                        data[10] = (uint8_t)(w >> 24);
                        data[11] = sn32f2xx_scan_only_active() ? 1 : 0;
                        data[12] = sleep_light_yield();
                        break;
                    }
                    if (sub == 4) {
                        /* The bench's deep sleep (record v2, 32-bit counts): arg
                         * 1 arms it ([6] the delay s, [7..10] the cap in counted
                         * s, 0 the 24 h backstop, [11] 1 = leave the watchdog
                         * unfed, [12..15] the counted seconds' preset), 2
                         * disarms it, 0 only reads. Reply: 4, then the record. */
                        if (arg == 1) {
                            uint32_t cap = (uint32_t)data[7] | (uint32_t)data[8] << 8 | (uint32_t)data[9] << 16 |
                                           (uint32_t)data[10] << 24;
                            uint32_t pre = (uint32_t)data[12] | (uint32_t)data[13] << 8 | (uint32_t)data[14] << 16 |
                                           (uint32_t)data[15] << 24;
                            sleep_deep_arm(data[6], cap, data[11] != 0, pre);
                        }
                        if (arg == 2) sleep_deep_cancel();
                        memset(&data[4], 0, 28);
                        data[4] = 4;
                        _Static_assert(SLEEP_RECORD_SIZE <= 27u, "the record must fit data[5..31]");
                        sleep_deep_record(&data[5]);
                        break;
                    }
                    if (sub == 5) {
                        /* The bench's sleep time ([6..9] ms, 0 clears; at least
                         * 30 s), so the soak runs through the ladder's own path.
                         * Not persisted. Reply: 5. */
                        power_set_bench_sleep_ms((uint32_t)data[6] | (uint32_t)data[7] << 8 |
                                                 (uint32_t)data[8] << 16 | (uint32_t)data[9] << 24);
                        memset(&data[4], 0, 28);
                        data[4] = 5;
                        break;
                    }
                    if (sub == 6) {
                        /* The acquisition's split, tested: the next full read is
                         * held 1 s past its bracket's edge, once; it must
                         * reacquire, never set the clock a second wrong. */
                        rtc_dev_delay_full_once();
                        memset(&data[4], 0, 28);
                        data[4] = 6;
                        break;
                    }
                    if (sub == 1) power_set_mode((power_mode_t)arg);
                    if (sub == 2) power_force_stage(arg);
                    memset(&data[4], 0, 28);
                    data[4] = (uint8_t)power_mode();
                    data[5] = (uint8_t)power_stage();
                    data[6] = power_val_cap();
                    data[7] = power_forced_stage();
                    data[8] = battery_on_battery() ? 1 : 0;
                    break;
                }
                case 15:
                    if (length < 32 || data[4] > 2) { data[0] = RTC_UNHANDLED; break; }
                    if (data[4] != 0) ch582_rxtap_arm(data[4] == 1);
                    memset(&data[4], 0, 28);
                    ch582_rxtap_fill(&data[4]);
                    break;
                case 16:
                    if (length < 32 || !ch582_rxtap_page(data[4], &data[4])) data[0] = RTC_UNHANDLED;
                    break;
                default: data[0] = RTC_UNHANDLED; break;
            }
            break;
#endif
#if defined(WDT_TEST_HOOKS) || defined(AK820_RAW_BOOTLOADER)
        case HC_BOOTLOADER:
            if (!ak820_raw_bootloader || length < 7 || memcmp(&data[3], "BOOT", 4) != 0) {
                data[0] = RTC_UNHANDLED;
                break;
            }
            /* Never mid-typing, and never with a key left down at the host
             * (codex, flash 2b finding 1). Over BT, clear_keyboard()'s releases
             * are frames in the CH582F's TX queue, which only the main loop
             * pumps: pump them here and see them ACKed before the reset. Fn+Esc
             * has the same gap, but there the hands are on the keys. */
            if (last_input_activity_elapsed() < 2000u) {
                memcpy(&data[3], "BUSY", 4); data[7] = 1;
                break;
            }
            clear_keyboard();
            /* Fresh, on every attempt, and refused if a full queue dropped one. */
            bool queued = ch582_is_usb() || ch582_send_releases();
            if (!queued || !ch582_tx_drain(500)) {
                memcpy(&data[3], "BUSY", 4); data[7] = 2;
                break;
            }
            /* Then what Fn+Esc does (ak820pro.c, QK_BOOT): the splash, held
             * 1.5 s so it can be read, then QMK's own reset path, which ends
             * in our bootloader_jump() (watchdog stopped first). The one path
             * every flash so far has taken. */
            display_bootloader_splash();
            wait_ms(1500);
            reset_keyboard();   /* does not return */
            break;
#endif
        default:
            data[0] = RTC_UNHANDLED;
            break;
    }
}

static inline bool rtc_is_set_time_cmd(const uint8_t *data, uint8_t length) {
    return length >= 10 && data[0] == RTC_SET_VALUE &&
           data[1] == RTC_CHANNEL && data[2] == RTC_SET_TIME;
}

static inline bool rtc_is_get_time_cmd(const uint8_t *data, uint8_t length) {
    return length >= 3 && data[0] == RTC_SET_VALUE &&
           data[1] == RTC_CHANNEL && data[2] == RTC_GET_TIME;
}

/* Fill the reply in place with the live clock. */
static void rtc_read_into(uint8_t *data) {
    /* Zeroed up front: rtc_get_time() leaves t untouched when it fails, and the
     * fields below are serialized regardless of ok. Without this the reply
     * carries stack garbage -- nondeterministic, and a small leak of whatever
     * was on the stack. data[3] already tells the host the reading is invalid. */
    rtc_time_t t = {0};
    bool ok = rtc_get_time(&t);
    data[3] = ok ? 1 : 0;
    data[4] = (uint8_t)(t.year >= 2000 ? t.year - 2000 : 0);
    data[5] = t.month;
    data[6] = t.day;
    data[7] = t.weekday;
    data[8] = t.hours;
    data[9] = t.minutes;
    data[10] = t.seconds;
    /* Clock-sync plan 3.7: the sub-second tail. [11] is RTC_PROTO_VERSION
     * (2); old firmware echoed zeros here, which is how a host tells them
     * apart. Layout: [12..15] SECCNT u32, [16..17] period_active,
     * [18..19] period_nominal, [20] flags, [21..22] last_host_offset_ms,
     * [23..24] sof_bias_ppm, [25] ref_state, [26] sync_age_min,
     * [27] sof_epoch, [28..31] sof_frames_total. */
    rtc_status_fill(1, &data[11]);
}

#if defined(VIA_ENABLE)

// VIA owns raw_hid_receive() and dispatches custom-value commands here. VIA echoes
// the buffer back itself -- do NOT call raw_hid_send().
void via_custom_value_command_kb(uint8_t *data, uint8_t length) {
    WDT_SCOPE(WDT_SITE_RAW_HID);
    if (rtc_is_get_time_cmd(data, length)) {
        rtc_read_into(data);
        return;
    }
    if (rtc_is_set_time_cmd(data, length)) {
        // Honest reply: a rejected packet or failed I2C write must not echo
        // back as "handled" -- the host's next poll is its only truth.
        if (!rtc_apply_bytes(&data[3])) data[0] = RTC_UNHANDLED;
        return;
    }
    if (rtc_is_set_ms_cmd(data, length)) {
        rtc_set_ms_command(data, length);
        return;
    }
    if (is_flash_cmd(data, length)) {
        flash_command(data, length);
        return;
    }
    if (is_text_cmd(data, length)) {
        text_command(data, length);
        return;
    }
    if (is_health_cmd(data, length)) {
        health_command(data, length);
        return;
    }
    data[0] = RTC_UNHANDLED;
}

#else // no VIA: handle the same packet directly and echo it back like VIA would.

void raw_hid_receive(uint8_t *data, uint8_t length) {
    WDT_SCOPE(WDT_SITE_RAW_HID);
    if (rtc_is_get_time_cmd(data, length)) {
        rtc_read_into(data);
    } else if (rtc_is_set_time_cmd(data, length)) {
        if (!rtc_apply_bytes(&data[3])) data[0] = RTC_UNHANDLED;
    } else if (rtc_is_set_ms_cmd(data, length)) {
        rtc_set_ms_command(data, length);
    } else if (is_flash_cmd(data, length)) {
        flash_command(data, length);
    } else if (is_text_cmd(data, length)) {
        text_command(data, length);
    } else if (is_health_cmd(data, length)) {
        health_command(data, length);
    } else {
        data[0] = RTC_UNHANDLED;
    }
    raw_hid_send(data, length);
}

#endif
