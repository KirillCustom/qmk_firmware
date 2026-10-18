// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#include "ak820pro.h"

#include "gpio.h"
#include "connection.h"

#include "graphics/display.h"
#include "graphics/lcd_bus.h"
#include "bluetooth/ch582f_ajazz.h"
#include "rtc/rtc.h"
#include "raw_hid.h"
#include "watchdog.h"
#include "health.h"
#ifdef USB_WAKEUP_ON_KEYPRESS
#    include "usb_main.h"   /* USB_DRIVER */
#endif

// Wireless-mode/slot UI lives in bt_ui.c; the persisted kb datablock in
// kb_eeconfig.c (phase-3 module split -- code moved verbatim).
#include "bt_ui.h"
#include "kb_eeconfig.h"
#include "consumer_mod.h"
#include "param_overlay.h"
#include "indicators.h"
#include "battery.h"
#include "power.h"
#include "loop_acct.h"     /* D1: ACCT() around each board task */
#include "flash_stats.h"   /* C1: flash write sessions by writer */

void early_hardware_init_post(void) {
    // Configure SPI0 pins for the LCD panel. SEL0 is left UNMUXED: our bare-metal bus
    // (graphics/lcd_bus.c) drives CS (B8) as a plain GPIO and must hold it low across
    // a whole DMA frame, so B8 must be free of the SPI SEL function.
    SN_PFPA->SPI_b.MISO0 = 0b11;
    SN_PFPA->SPI_b.MOSI0 = 0b11;
    SN_PFPA->SPI_b.SCK0  = 0b11;

    // Configure UART2 pins for the CH582F wireless module
    SN_PFPA->UART_b.UTXD2 = 0b11;
    SN_PFPA->UART_b.URXD2 = 0b11;
}


 void keyboard_post_init_kb(void) {
#ifdef FORCE_EEPROM_RESET
    /* ONE-SHOT RECOVERY BUILD -- not for normal use.
     * An interrupted `flash write` erased the whole application region,
     * including the wear-levelling EEPROM store. A corrupt store makes QMK
     * rewrite it continually; every write stalls the main loop while the flash
     * array is busy, which presents as stuck and repeating keys. This wipes it
     * back to a known state. Flash the normal build immediately after. */
    eeconfig_init();
#endif

    // Windows Lock and Charging LEDs: outputs, off initially. update_leds() then
    // tracks their real state, writing only on a change.
    pwm_tick_init();

    gpio_set_pin_output(LED_WINLOCK_PIN);
    gpio_write_pin(LED_WINLOCK_PIN, false);
    gpio_set_pin_output(LED_CHARGING_PIN);
    gpio_write_pin(LED_CHARGING_PIN, false);

    // Set up GPIO pins for the charging status inputs
    gpio_set_pin_input_high(CHARGE_CHRG_PIN);   // input with pull-up
    gpio_set_pin_input_high(CHARGE_STDBY_PIN);  // input with pull-up

    // VDD measurement: the SN32 ADC's internal reference against VDD.
    battery_init();

    // Restore persisted config, then re-select the saved BT slot if the
    // slider booted in the BT position (dip_switch_init ran before EEPROM
    // was read). Details in kb_eeconfig.c / bt_ui.c.
    kb_eeconfig_init();
    bt_ui_resume_saved_profile();
    power_init();   // the persisted power mode (Normal, and says so, after a flash)

    // Bring up the clock: I2C, the SN32 1 Hz counter, and a first PCF8563 seed.
    rtc_init();

    // Initialize the display subsystem (painter, fonts, images, etc.) and draw the splash screen.
    display_init_kb();

    // Reset-cause bookkeeping (consecutive-WDT-reset counter, degraded-mode
    // decision) -- before the watchdog arms, after the console is plumbed.
    watchdog_boot_check();

    // A watchdog recovery is otherwise SILENT in the daily build (no
    // console): say so on the panel, loudly enough to notice, briefly
    // enough not to nag. DEGRADED means "it kept happening and the
    // watchdog is now off" -- the one state that really needs eyes.
    if (watchdog_degraded()) {
        display_set_alert("WDT DEGRADED");
    } else if (watchdog_fired_last_boot()) {
        char buf[16];
        snprintf(buf, sizeof(buf), "WDT reset x%u", (unsigned)watchdog_reset_count());
        display_set_alert(buf);
    }

    // Chain the user hook: overriding keyboard_post_init_kb() replaces QMK's
    // default, which is what normally calls keyboard_post_init_user().
    keyboard_post_init_user();

    // Arm the watchdog LAST: boot blocks deliberately (lcd_init's 240 ms of
    // wait_ms, the asset index read, the RTC seed), and none of that should
    // count against the timeout. See watchdog.c for the 12 s rationale.
    watchdog_start();
 }

 bool dip_switch_update_kb(uint8_t index, bool active) {
    // Let the keymap handle layer logic (Mac/Win, no_gui) first.
    if(!dip_switch_update_user(index, active)) {
        return false;
    }

    if (index == 0) {  // Mac/Windows switch -- icon only (layer set by keymap)
        if (active) display_draw_windows_logo();
        else        display_draw_mac_logo();
    } else if (index == 1 || index == 2) {
        bt_ui_mode_slider(index, active);   // tri-state slider; see bt_ui.c
    }
    return true;
 }

#ifdef USB_WAKEUP_ON_KEYPRESS
/* Ask a sleeping host to wake. QMK's own version is compiled out here -- see the
 * NO_USB_STARTUP_CHECK note in config.h -- and we cannot restore it, because it
 * wakes the host from inside a blocking `while (USB_SUSPENDED)` loop that would
 * stall the main loop and drop the wireless link.
 *
 * Gated on the host having ENABLED remote wakeup (bit 1 of the USB status): a
 * host that does not want to be woken by this device never sets it, so this is
 * self-limiting rather than something to police ourselves.
 *
 * Rate-limited because usbWakeupHost() sleeps SN32_USB_HOST_WAKEUP_DURATION (2 ms)
 * driving the K-state. Costs nothing in normal use -- it returns immediately
 * unless USB is actually suspended. */
#define USB_STATUS_REMOTE_WAKEUP 2U   /* USB_GETSTATUS_REMOTE_WAKEUP_ENABLED; that
                                       * define is private to chibios.c */
#define USB_WAKEUP_RETRY_MS      500

/* battery.c's model counts the bench's USB data (R2: 7.1 mW) only while the
 * host has configured the device; on battery that is the bench, never real
 * use, where USB means USB power. */
bool battery_usb_configured(void) {
    return USB_DRIVER.state == USB_ACTIVE;
}

static void usb_wakeup_try(void) {
    if (USB_DRIVER.state != USB_SUSPENDED) return;
    if (!(USB_DRIVER.status & USB_STATUS_REMOTE_WAKEUP)) return;

    static uint16_t last_try = 0;
    static bool     tried    = false;
    if (tried && timer_elapsed(last_try) < USB_WAKEUP_RETRY_MS) return;
    last_try = timer_read();
    tried    = true;
    usbWakeupHost(&USB_DRIVER);
}
#endif

/* ---- Worst-case main-loop gap (Stage 2 instrument) ------------------------
 *
 * Measures the LONGEST interval between consecutive housekeeping passes, not
 * the average scan rate. A single 100 ms stall is invisible in a per-second
 * average -- 400 Hz drops to 360 and looks like ordinary jitter -- which is
 * exactly why the console blocking bug went unnoticed.
 *
 * Reported on the LCD, deliberately NOT over the console: the console path is
 * the thing under test, and printing to it would perturb the measurement.
 * Fires only when a NEW maximum is set, so it costs nothing while quiet.
 *
 * Remove with LOOPGAP_INSTRUMENT once the console work is validated. */

/* UNCONDITIONAL (LOOP-BUDGET-PLAN phase 1): a stall the daily build cannot
 * attribute is a stall nobody can act on, and this is one byte store on paths
 * that are already milliseconds long. health_loop_tick() is its sole consumer;
 * it reads and clears once per pass and republishes via health_last_mark(). */
volatile uint8_t loop_stall_mark = 0;   /* see LOOP_MARK_* in ak820pro.h */

#ifdef LOOPGAP_INSTRUMENT

/* WHICH long operation was running when the loop stalled.
 *
 * The probe measured a 59 ms in-use stall -- easily enough to swallow a
 * keystroke, since a character arrives roughly every 125 ms at typing speed.
 * Magnitude alone does not say what to fix, and the three suspects need
 * completely different remedies, so each marks itself on the way in. */

static const char *loop_site_name(uint8_t s) {
    static const char *const names[LOOP_SITE_COUNT] = {
        "none", "leds", "pair", "consumer",
        "param", "rtctask", "anim", "conn", "status", "text", "locks", "clockF",
        "clock", "batt", "eecfg", "health",
    };
    return s < LOOP_SITE_COUNT ? names[s] : "?";
}

static uint8_t  site_open = LOOP_SITE_NONE, site_worst = LOOP_SITE_NONE;
static uint32_t site_t0 = 0, site_worst_ms = 0;
static uint32_t hk_worst_ms = 0;      /* the whole 10 Hz block */

void loop_site_end(void) {
    if (site_open == LOOP_SITE_NONE) return;
    uint32_t e = timer_elapsed32(site_t0);
    if (e > site_worst_ms) { site_worst_ms = e; site_worst = site_open; }
    site_open = LOOP_SITE_NONE;
}

void loop_site_begin(uint8_t site) {
    loop_site_end();
    site_open = site;
    site_t0   = timer_read32();
}

static const char *loop_mark_name(uint8_t m) {
    switch (m) {
        case LOOP_MARK_FLASH: return "flash";   /* internal-flash program/erase */
        case LOOP_MARK_BLIT:  return "blit";    /* flash->LCD DMA wait */
        case LOOP_MARK_I2C:   return "i2c";     /* bit-banged PCF8563 */
        default:              return "?";
    }
}

static void loop_gap_task(void) {
    static uint32_t last = 0, report_at = 0;
    static uint16_t n_gaps = 0;
    static uint32_t worst = 0;
    static uint8_t  worst_mark = 0;

    if (timer_read32() < LOOPGAP_SETTLE_MS) {   /* boot blocks deliberately */
        last = timer_read32();
        return;
    }
    if (last) {
        uint32_t gap = timer_elapsed32(last);
        if (gap >= 4) {                          /* main loop is ~2.5 ms */
            n_gaps++;
            if (gap > worst) { worst = gap; worst_mark = health_last_mark(); }
        }
    }
    last = timer_read32();
    /* health_loop_tick() owns loop_stall_mark and already cleared it. */

    /* Report at most once a second, and only when something happened.
     *
     * A MAX-HOLD WAS THE WRONG INSTRUMENT for the observed fault: typing goes
     * janky for a stretch, then recovers. Once a max latches, every later stall
     * is invisible, so a burst of thirty 12 ms stalls looks identical to
     * silence. The COUNT is what distinguishes a periodic storm from a one-off,
     * and the count is what the symptom actually is. */
    /* Report on the CONSOLE, never the panel.
     *
     * The previous version called display_set_param_status(), which AT THE TIME
     * drew ~12 glyphs, each a blocking DMA blit -- i.e. it reported by doing
     * the exact thing it was measuring. That fed back and ran away to a 154 ms
     * stall and an unusable board. An instrument must not touch the subsystem
     * under test.
     *
     * ⚠️ THE COST IS HISTORICAL, THE LESSON IS NOT. display_set_param_status()
     * now only sets text_dirty; the band draws through the glyph queue, one
     * glyph per main-loop pass. Verified 2026-09-03, after this stale warning
     * was quoted as current fact in a later comment. Report on the console here
     * anyway -- the reasoning below about counts vs. visibility stands on its
     * own, and an instrument still should not touch what it measures.
     *
     * Console writes are one line per second and the endpoint's sticky
     * timed_out flag bounds the worst case; that path was measured earlier and
     * exonerated. Also prints uptime, because the fault is WORST RIGHT AFTER
     * BOOT and settles over minutes -- so when it happens matters as much as
     * what. */
    if (n_gaps && timer_elapsed32(report_at) >= 1000) {
        report_at = timer_read32();
        dprintf("[stall] t=%lus %ux worst=%lums %s hk=%lums site=%s:%lums\n",
                (unsigned long)(timer_read32() / 1000), (unsigned)n_gaps,
                (unsigned long)worst, loop_mark_name(worst_mark),
                (unsigned long)hk_worst_ms,
                loop_site_name(site_worst), (unsigned long)site_worst_ms);
        n_gaps = 0; worst = 0; worst_mark = LOOP_MARK_NONE;
        hk_worst_ms = 0; site_worst_ms = 0; site_worst = LOOP_SITE_NONE;
    }
}
#endif

#ifdef CONSOLE_ENABLE
/* Key-event counter, to localise dropped keystrokes.
 *
 * Measured 2026-08-30: ~2.3% of characters lost on this board, wired, all pure
 * drops with no transpositions. That could be the matrix/debounce failing to
 * SEE the press, or the USB path failing to DELIVER it. Those need completely
 * different fixes, and counting is the only way to tell them apart:
 *
 *   firmware count == characters received  -> the matrix missed it
 *   firmware count >  characters received  -> the report was lost downstream
 *
 * Counted in process_record_kb, which is after debounce and after the matrix
 * has resolved the event, so it measures what the firmware BELIEVES it sent.
 *
 * Printed at most once a second and only when the count moves, i.e. one USB
 * write per second of typing rather than one per keystroke -- deliberately, so
 * the instrument cannot cause the drops it is measuring. */
static uint16_t key_press_count = 0;

static void key_stat_task(void) {
    static uint16_t last = 0;
    static uint32_t last_at = 0;
    if (key_press_count == last) return;
    if (timer_elapsed32(last_at) < 1000) return;
    last_at = timer_read32();
    dprintf("[keys] presses=%u\n", (unsigned)key_press_count);
    last = key_press_count;
}
#endif
/* Hold-to-reset on DBG_PAGE. 800 ms, and the hold task runs at ~10 Hz so the
 * effective threshold is ~200 ms longer -- the same caveat bt_ui.c records for
 * BT_PAIR_HOLD_MS. Long enough that no tap reaches it, short enough not to feel
 * like a hang. */
#define DBG_RESET_HOLD_MS 800
static uint16_t dbg_hold_timer = 0;
static bool     dbg_hold_armed = false;
static bool     dbg_hold_fired = false;

/* Fires the instant the threshold passes while the key is STILL DOWN. */
void dbg_hold_task(void) {
    if (!dbg_hold_armed || dbg_hold_fired) return;
    if (timer_elapsed(dbg_hold_timer) < DBG_RESET_HOLD_MS) return;
    dbg_hold_fired = true;          /* release must not also toggle the page */
    health_reset();
    display_debug_refresh();        /* show the zeros now, not on the next second */
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    WDT_SCOPE(WDT_SITE_KEY_EVENT);
    /* Counted on EVERY build: this is what splits "the matrix never saw it"
     * from "the report was lost downstream", and it is useless if it only
     * exists in a flavour nobody types on. */
    if (record->event.pressed) health_note_key_press();
    if (record->event.pressed && keycode == KC_SCRL) lock_note_scroll_key();
#ifdef CONSOLE_ENABLE
    if (record->event.pressed) key_press_count++;
#endif
    param_repeat_process_record(keycode, record);   // arm/disarm hold-to-repeat

    if (!process_modified_consumer(keycode, record)) return false;

#ifdef USB_WAKEUP_ON_KEYPRESS
    if (record->event.pressed) usb_wakeup_try();   // typing should wake a sleeping host
#endif
    // Let the keymap handle/override keycodes first (e.g. Mac media keys).
    if (!process_record_user(keycode, record)) {
        return false;
    }

    switch (keycode) {
        case SCR_TOG:
            if (record->event.pressed) {
                display_toggle_power();
#ifdef PARAM_OVERLAY
                /* Only the "On" case is ever legible -- turning the panel off
                 * takes the message with it. Pushed anyway so the action is
                 * consistent with every other adjustment, and so switching back
                 * on confirms itself rather than just appearing. */
                if (display_get_power()) display_set_param_status("LCD On");
#endif
            }
            return false;
        case ANIM_TOG:
            if (record->event.pressed) anim_toggle();
            return false;
        case SCR_UP:
        case SCR_DN:
            if (record->event.pressed) {
                if (keycode == SCR_UP) display_brightness_up();
                else                   display_brightness_down();
                /* Shared with the hold-repeat step so a tap and a sweep produce
                 * the same readout from one place; see param_overlay.c. */
                param_show_lcd_brightness();
            }
            return false;
        case CLK_MODE:
            if (record->event.pressed) {
                /* 24h -> 12h -> off -> date -> 24h. Persisted like the backlight
                 * level: through kb_eeconfig's coalesced deferred flush. */
                uint8_t m = (uint8_t)((display_get_clock_mode() + 1) % DISPLAY_CLOCK_MODE_COUNT);
                display_set_clock_mode(m);
                kb_eeconfig_set_clock_mode(m);
#ifdef PARAM_OVERLAY
                static const char *const clock_mode_names[DISPLAY_CLOCK_MODE_COUNT] = {
                    "Clock 24h", "Clock 12h", "Clock off", "Clock date",
                };
                display_set_param_status(clock_mode_names[m]);
#endif
            }
            return false;
        case PWR_MODE:
            if (record->event.pressed) power_cycle_mode();
            return false;
        case DBG_PAGE:
            /* Tap toggles the page; HOLD resets the counters.
             *
             * Hold fires from dbg_hold_task() at the threshold, under the
             * finger -- not here on key-up. bt_ui.c records why: firing a hold
             * on release means holding the key appears to do nothing until you
             * let go, and you cannot tell a taken hold from an ignored one.
             *
             * Feedback is the numbers going to zero on the page, not an
             * overlay. NOT because display_set_param_status() is expensive --
             * VERIFIED 2026-09-03 that it is not: it copies a string and sets
             * text_dirty, and the drawing goes through the glyph queue like
             * everything else in that band. The "~12 blocking DMA blits"
             * warning above the stall reporter in this file describes an OLDER
             * version of it and is now STALE; this comment previously repeated
             * that claim as current fact.
             *
             * The real reason is simpler: the page is already showing the
             * counters, so zeroing them in place is the clearest possible
             * confirmation, and an overlay would cover the thing you are
             * looking at. */
            if (record->event.pressed) {
                dbg_hold_timer = timer_read();
                dbg_hold_armed = true;
                dbg_hold_fired = false;
            } else {
                dbg_hold_armed = false;
                if (!dbg_hold_fired) display_debug_toggle();   /* a tap */
            }
            return false;
        case QK_BOOT:
            /* Draw the bootloader notice BEFORE handing off to QMK, which then
             * resets into the ROM bootloader.
             *
             * Bootloader mode is otherwise indistinguishable from a dead board
             * -- no RGB, dark LCD, no typing, no indicator of any kind -- which
             * has cost real diagnostic time. The GC9107 keeps its own GRAM
             * across the MCU reset, so the picture survives; whether the
             * BACKLIGHT does is the open question (PANEL_BKL goes high-Z on
             * reset). If it goes dark this is simply a no-op, which is why it
             * is worth trying in this form first.
             *
             * Returns TRUE so QMK still performs the reset -- we are only
             * decorating it. This covers Fn+Esc; holding ESC while plugging in
             * is handled by the ROM before any of our code runs, and cannot be
             * decorated. */
            /* Draw the notice, then let QMK do its normal magic-flag reset.
             *
             * ⚠️ DO NOT "improve" this by jumping straight to the ROM at
             * SN32_BOOTLOADER_ADDRESS. That DOES keep the splash lit -- GPIOs
             * retain state without a reset, so the backlight survives -- but it
             * hands the ROM a live machine (48 MHz clocks, SPI/PWM/UART/timers
             * configured, USB half torn down) and flashing then becomes
             * unreliable: sonixflasher stalls partway through the erase and
             * only a REPLUG clears the ROM's ISP session. Tried 2026-08-31 and
             * reverted. Entry via a clean power-on reset worked every time;
             * entry via the direct jump failed every time.
             *
             * The splash is therefore only visible for the moment before the
             * reset. A barely-visible message that flashes reliably beats a
             * readable one that does not. */
            if (record->event.pressed) {
                display_bootloader_splash();
                /* Hold it long enough to READ before QMK resets into the ROM.
                 * The reset drops PANEL_BKL to high-Z so the panel goes dark
                 * the instant it fires; without this pause the notice is
                 * present for a few milliseconds and effectively invisible.
                 * Costs 1.5 s on the way into the bootloader, which is nothing
                 * against a flash cycle. */
                wait_ms(1500);
            }
            return true;
        case KC_MEDIA_PLAY_PAUSE:
            /* Guess the new state so the icon flips instantly instead of after
             * the host's next poll. Returns TRUE: the keypress must still reach
             * the host, we are only decorating it. Works wherever the key is
             * bound, since process_record_kb sees the resolved keycode. */
            if (record->event.pressed) {
                display_toggle_play_icon();
                display_playback_key();   // freeze/resume the timer with it
            }
            return true;
#ifdef RGB_MATRIX_ENABLE
        // VIA-assignable RGB-matrix controls (see ak820pro.h). One step per press.
        case RGBM_TOG:  if (record->event.pressed) rgb_matrix_toggle();         return false;
        case RGBM_MOD:  if (record->event.pressed) rgb_matrix_step();           return false;
        case RGBM_RMOD: if (record->event.pressed) rgb_matrix_step_reverse();   return false;
        case RGBM_HUI:  if (record->event.pressed) rgb_matrix_increase_hue();   return false;
        case RGBM_HUD:  if (record->event.pressed) rgb_matrix_decrease_hue();   return false;
        case RGBM_SAI:  if (record->event.pressed) rgb_matrix_increase_sat();   return false;
        case RGBM_SAD:  if (record->event.pressed) rgb_matrix_decrease_sat();   return false;
        case RGBM_VAI:  if (record->event.pressed) rgb_matrix_increase_val();   return false;
        case RGBM_VAD:  if (record->event.pressed) rgb_matrix_decrease_val();   return false;
        case RGBM_SPI:  if (record->event.pressed) rgb_matrix_increase_speed(); return false;
        case RGBM_SPD:  if (record->event.pressed) rgb_matrix_decrease_speed(); return false;
#endif
        // BT slot / 2.4G / pair keys: tap-select + hold-to-pair, in bt_ui.c.
        case BT1:      // Fn+Q -> tap: select BT1 | hold: pair BT1
        case BT2:      // Fn+W -> tap: select BT2 | hold: pair BT2
        case BT3:      // Fn+E -> tap: select BT3 | hold: pair BT3
            return bt_ui_slot_key(keycode, record);
        case BT24G:    // Fn+R -> select 2.4G (2.4G mode only)
            return bt_ui_24g_key(record);
        case BT_PAIR:  // Fn+P (long press) -> pair, BT/2.4G modes only
            return bt_ui_pair_key(record);
        default:
            return true;
    }
}

// Raw-HID protocol (RTC/flash/text/health channels + dispatch) lives in
// hid_protocol.c (phase-3 module split -- code moved verbatim).



__attribute__((weak)) void display_housekeeping_task(void) {}


/* Health line for the never-started-DMA fault, printed ONLY when a failure
 * actually happens -- i.e. when timeouts or retries move.
 *
 * This used to print blits/s every second. That served its purpose: it measured
 * the blit rate (median 1/s idle, bursts to ~85) and killed the theory that the
 * two-line text slot and playback readout had raised the hang rate by raising
 * blit volume. Once answered it was a USB write per second for a number nobody
 * reads, so only the failure signal is left.
 *
 * i2c-overlap was dropped from the line too: it stayed 0 across hundreds of
 * pcf_read opportunities, so the RTC bus guard is confirmed to be doing nothing
 * for this fault. The guard STAYS -- it is correct per the contract documented
 * in lcd_bus.c -- but it does not need reporting.
 *
 * blits/s is still included as context on the failure line, where it is free. */
#ifdef CONSOLE_ENABLE
static void blit_stat_task(void) {
    static uint32_t last_at = 0;
    static uint16_t last_to = 0, last_re = 0;
    if (timer_elapsed32(last_at) < 1000) return;
    last_at = timer_read32();

    static uint8_t minute = 0;
    if (++minute >= 60) { minute = 0; lcd_blit_arm_report(); }

    uint32_t rate = lcd_blit_count_take();
    uint16_t to   = lcd_blit_timeouts();
    uint16_t re   = lcd_blit_retries();
    if (to != last_to || re != last_re) {
        dprintf("[lcd] timeouts=%u retries=%u (blits/s=%lu)\n",
                (unsigned)to, (unsigned)re, (unsigned long)rate);
        last_to = to;
        last_re = re;
    }
}
#endif

void housekeeping_task_kb(void) {
    WDT_SCOPE(WDT_SITE_HOUSEKEEPING);
    health_loop_tick();    // worst-gap max-hold; one timer read per pass
#ifdef LOOPGAP_INSTRUMENT
    loop_gap_task();
#endif
    /* Per-pass work is deliberately NOT sited: see the LOOP_SITE note in
     * ak820pro.h -- the timer reads cost ~2 ms a pass on this MCU. */
    /* ACCT(): D1's per-task accounting (loop_acct.h), one tick-register read
     * either side; mutually exclusive scopes, everything else unaccounted.
     * The param repeat is not timed: a per-pass scope costs every pass, and
     * it is near-empty unless a parameter key is held (trimmed after gate 3). */
    WDT_CALL(WDT_SITE_PARAM_REPEAT, param_repeat_task());
    WDT_CALL(WDT_SITE_RTC_FAST, ACCT(ACCT_RTC_FAST, rtc_fast_task()));          // <= one queued PCF I2C transaction, only when no blit is in flight (R4)
    WDT_CALL(WDT_SITE_SECOND_EDGE, ACCT(ACCT_EDGE, display_second_edge_task()));  // clock digits repaint on the tick, not at the 10 Hz cadence (3.9)
    /* One queued glyph per iteration, arming a DMA and returning -- EXCEPT that
     * it also drives the Fn+D page's banded clears and the staged restore, and
     * lcd_clear_rect() ends in lcd_blit_wait(): bounded (a 128x16 band is
     * ~5.5 ms) but it does wait. "Never waits on the DMA" stopped being true
     * on 2026-09-03. It is also the ONLY thing that advances debug_exit_step,
     * which is why display_housekeeping_task() may gate on that flag: keep this
     * call OUT of that function (see the gate's comment in display.c). */
    WDT_CALL(WDT_SITE_DISPLAY_PUMP, ACCT(ACCT_PUMP, display_blit_pump()));
#ifdef CONSOLE_ENABLE
    blit_stat_task();
#endif

    // Throttle the housekeeping to 10 Hz
    static uint32_t last_t = 0;
    if (timer_elapsed32(last_t) >= 100) {
        last_t = timer_read32();
#ifdef LOOPGAP_INSTRUMENT
        uint32_t hk_t0 = last_t;
#endif

        LOOP_SITE(LOOP_SITE_LEDS,     ACCT(ACCT_LEDS, update_leds()));
        LOOP_SITE(LOOP_SITE_BATT,     ACCT(ACCT_BATT, battery_task()));   // one ~11 us VDD conversion
        LOOP_SITE(LOOP_SITE_PAIR,     ACCT(ACCT_HK_OTHER, bt_pair_hold_task()));   // hold-to-pair fires under the finger, not on release
        LOOP_SITE(LOOP_SITE_PAIR,     ACCT(ACCT_HK_OTHER, dbg_hold_task()));       // hold-Fn+D resets the health counters
        LOOP_SITE(LOOP_SITE_CONSUMER, ACCT(ACCT_HK_OTHER, modified_consumer_task()));  // drop held mods once a knob spin stops
#ifdef PARAM_OVERLAY
        LOOP_SITE(LOOP_SITE_PARAM,    ACCT(ACCT_HK_OTHER, param_status_task()));   // surface setting changes in the info band
#endif
        /* The idle ladder, battery only. After param_status_task(): a setting's
         * readout posted this tick makes the band busy, so power.c's notices
         * wait for it rather than be replaced by it before they paint. */
        LOOP_SITE(LOOP_SITE_BATT,     ACCT(ACCT_BATT, power_task()));
#ifdef CONSOLE_ENABLE
        key_stat_task();       // dropped-keystroke localisation; see key_press_count
#endif

        if (!anim_active()) LOOP_SITE(LOOP_SITE_RTCTASK, ACCT(ACCT_RTC, rtc_task()));   // RTC I2C (port A) glitches the flash SPI1 pins (A12/A13) mid-DMA
        LOOP_SITE(LOOP_SITE_ANIM, ACCT(ACCT_ANIM, anim_task()));                   // one animation frame per 100 ms
        WDT_CALL(WDT_SITE_DISPLAY_HK, ACCT(ACCT_DISP_HK, display_housekeeping_task()));   // its sub-draws are sited individually
        LOOP_SITE(LOOP_SITE_EECFG,  ACCT(ACCT_EECFG, kb_eeconfig_task()));          // settled, coalesced kb-config flush
        LOOP_SITE(LOOP_SITE_HEALTH, ACCT(ACCT_HEALTH, health_task()));              // [health] console line, on change only
#ifdef LOOPGAP_INSTRUMENT
        uint32_t hk = timer_elapsed32(hk_t0);
        if (hk > hk_worst_ms) hk_worst_ms = hk;
#endif
    }

    // Chain the user hook
    WDT_CALL(WDT_SITE_USER_HK, housekeeping_task_user());   /* untimed, as the param repeat */

    display_pass_end();   /* the battery icon's one-component-a-pass limit (display.c) */

    // Kick the watchdog LAST -- after the 10 Hz block and the user hook, so a
    // kick certifies a COMPLETED pass. A kick at entry would hand a freshly
    // wedged pass another full timeout and prove nothing about what follows.
    watchdog_kick();
}

/*
 * Defer eeconfig flushes away from LCD DMA activity.
 *
 * Confirmed failure: adjusting RGB writes eeconfig on every step, and an SN32 EFL
 * program/erase overlapping an in-flight flash->LCD DMA blit wedges the board --
 * LED row mux frozen mid-cycle, raw HID dead, USB still enumerated, recoverable
 * only by a power cycle. Console capture caught it ending on
 * "rgb matrix set hsv [EEPROM]" with the next line never arriving.
 *
 * efl_ramtext.diff (flash routines in SRAM) narrows this but does not close it,
 * and CORTEX_ENABLE_WFI_IDLE FALSE does not fix it either -- that only removed
 * the lost-wakeup half of the failure described in config.h; the missed SPI0 DMA
 * completion is the half that actually bites.
 *
 * So simply do not start a flash write while a blit is in flight. Paired with
 * RGB_MATRIX_EEPROM_WRITE_DELAY this also cuts the write rate from ~8/s while a
 * key is held to at most one per interval, which matters for flash wear too.
 *
 * Deliberately NOT gated on anim_active(): the animation player runs continuous
 * DMA, and blocking on it would mean RGB settings never persist while an
 * animation is on. Blits are short, so per-blit gating still finds gaps.
 */
/* Drain any flash->LCD DMA before the wear-levelling layer programs or erases
 * internal flash. See backing_store_pre_write_hook() in
 * platforms/chibios/drivers/wear_leveling/wear_leveling_efl.c.
 *
 * WHY THIS IS BROADER THAN rgb_matrix_eeprom_flush_allowed() BELOW: that hook
 * only gates RGB's own eeconfig flush, which was the trigger we happened to
 * notice first. VIA's dynamic-keymap writes take a completely different path and
 * were never covered -- assigning keys in VIA reproduced the hang (one LED row
 * stuck lit, board dead until a power cycle) on 2026-08-29. The bug was never
 * about RGB; it is about ANY internal-flash write overlapping the LCD DMA.
 *
 * Waiting is sufficient rather than merely narrowing the window: flash writes are
 * synchronous on the main loop, and blits are started from the main loop too, so
 * once the in-flight blit has drained no new one can begin before the write
 * completes.
 *
 * Bounded like every other blit wait here -- an unbounded spin would trade an
 * intermittent hang for a guaranteed one if a completion interrupt is ever
 * genuinely lost. */
void backing_store_pre_write_hook(void) {
    WDT_SCOPE(WDT_SITE_EFL_DRAIN);
    loop_stall_mark = LOOP_MARK_FLASH;   /* every internal-flash writer passes here */
    health_note_flash_write();
    flash_stats_session();               /* and whose write this session is (C1) */
    /* lcd_blit_wait() rather than a bare spin: a blit whose completion IRQ was
     * missed never clears, so the old loop burned its full second here on every
     * single flash write and left the bus asserted. See lcd_blit_wait(). */
    lcd_blit_wait();
}

bool rgb_matrix_eeprom_flush_allowed(void) {
    /* Two gates.
     *
     * 1. Never start an internal-flash write while a flash->LCD DMA blit is in
     *    flight -- the original reason this hook exists.
     *
     * 2. Wait until the RGB settings have actually SETTLED.
     *
     * QMK's eeconfig_flush_rgb_matrix_task() is a RATE LIMIT, not a debounce:
     *
     *     if (timer_elapsed(flush_timer) > timeout) { flush(); flush_timer = timer_read(); }
     *
     * so it fires every RGB_MATRIX_EEPROM_WRITE_DELAY ms for as long as the
     * config stays dirty -- including throughout a held adjust key. Each write
     * makes the flash array busy for milliseconds, during which the row ISR
     * cannot run to arm the next row and the matrix goes DARK. Observed on
     * hardware as a dark flash mid-hold while sweeping brightness.
     *
     * Holding the flush off until the values stop moving turns "a write every
     * 750 ms while adjusting" into "one write once you settle". That also cuts
     * flash wear, and cuts exposure to the still-unexplained RGB-adjust hang,
     * whose trigger is an internal-flash write.
     *
     * Cheap enough to run from rgb_task_sync: six byte compares and a timer
     * read. Self-contained on purpose -- it must not depend on PARAM_OVERLAY,
     * which is a removable nicety. */
    if (lcd_blit_busy()) return false;

    static uint8_t  last_mode = 0, last_h = 0, last_s = 0, last_v = 0, last_sp = 0;
    static bool     last_on = false, primed = false;
    static uint32_t settled_since = 0;

    uint8_t mode = rgb_matrix_get_mode();
    uint8_t h    = rgb_matrix_get_hue();
    uint8_t sa   = rgb_matrix_get_sat();
    uint8_t v    = rgb_matrix_get_val();
    uint8_t sp   = rgb_matrix_get_speed();
    bool    on   = rgb_matrix_is_enabled();

    if (!primed) {
        primed = true;
        last_mode = mode; last_h = h; last_s = sa; last_v = v; last_sp = sp; last_on = on;
        settled_since = timer_read32();
    } else if (mode != last_mode || h != last_h || sa != last_s ||
               v != last_v || sp != last_sp || on != last_on) {
        last_mode = mode; last_h = h; last_s = sa; last_v = v; last_sp = sp; last_on = on;
        settled_since = timer_read32();      // still moving -- restart the settle window
    }

    return timer_elapsed32(settled_since) >= RGB_SETTLE_MS;
}

/* EFL write/erase hooks bracket the actual HAL operation, not just its
 * preceding LCD drain. Wear levelling calls these serially on the main loop. */
static uint32_t efl_saved_path;
void backing_store_operation_begin(bool erase) {
    efl_saved_path = watchdog_record_enter(erase ? WDT_SITE_EFL_ERASE : WDT_SITE_EFL_WRITE);
    flash_stats_op(erase);
}
void backing_store_operation_end(void) {
    watchdog_record_leave(&efl_saved_path);
}
