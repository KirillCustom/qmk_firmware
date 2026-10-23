// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* RGB matrix (hardware PWM on SN32F299). The 18 hardware row pins are 6 key-rows
 * x 3 colour channels in R,B,G order per group. Columns are shared with the key
 * matrix (COL_PINS omitted -> SHARED_MATRIX uses MATRIX_COL_PINS). */
#define SN32F2XX_RGB_MATRIX_ROW_PINS { A11, B4, B5, A8, A9, D8, D9, D10, D11, D12, D13, D16, D17, D18, C10, C11, C12, C13 }
#define SN32F2XX_PWM_CONTROL   HARDWARE_PWM
#define SN32F2XX_PWM_DIRECTION COL2ROW      // hardware PWM on columns, rows are the mux select
#define SN32F2XX_RGB_MATRIX_ROW_CHANNELS 3  // R, B, G
// ROWS defaults to MATRIX_ROWS (6); ROWS_HW = ROWS * ROW_CHANNELS = 18; COLS = MATRIX_COLS (15).

/* The 15 columns exceed CT16B1's 12 PWM channels, so hardware PWM spreads them over
 * three timers (CT16B0/B1/B2). Map is in COL_PINS order:
 *   A4 A5 C0 C1 C2 C3 A6 A7 C4 C5 C6 C7 C14 C8 C9
 * and the SN_PFPA pin-mux constants are datasheet-verified (docs/HARDWARE_PWM.md). */
#define SN32F2XX_PWM_MULTI_TIMER
#define SN32F2XX_DEEP_SLEEP        // the deep-sleep hooks, in every build (sleep.c, the ladder's last stage)
#define SN32F2XX_SLEEP_HOOKS       // scan-only, for light sleep (sleep.c; every build since 2026-10-23)
#define SN32F2XX_PWM_COL_MAP { \
    {&PWMD1, 4}, {&PWMD1, 5}, {&PWMD1, 8}, {&PWMD2, 1}, {&PWMD2, 2}, {&PWMD2, 3}, \
    {&PWMD1, 6}, {&PWMD1, 7}, {&PWMD1, 9}, {&PWMD0, 1}, {&PWMD0, 0}, {&PWMD1, 10}, \
    {&PWMD0, 3}, {&PWMD0, 2}, {&PWMD1, 11} }
#define SN32F2XX_PWM_PFPA_CT16B0 0x3332
#define SN32F2XX_PWM_PFPA_CT16B1 0x0F00
#define SN32F2XX_PWM_PFPA_CT16B2 0x0000

// The per-key LED map (81 LEDs; knob at matrix [0][14] has none) and
// RGB_MATRIX_LED_COUNT are defined by the rgb_matrix "layout" in keyboard.json.

// --- Field rate vs PWM resolution ------------------------------------------
// The 18 hardware rows (6 key rows x R,B,G) are lit one at a time, one row per
// PWM period, so a key's three colour channels fire in separate time slots. The
// full R->B->G cycle rate is:
//
//     freq / periodticks / SN32F2XX_RGB_MATRIX_ROWS_HW
//
// where drivers/led/sn32f2xx.c sets periodticks = RGB_MATRIX_MAXIMUM_BRIGHTNESS
// and, oddly, derives the PWM clock from the UI step sizes:
//     freq = HUE_STEP * SAT_STEP * VAL_STEP * SPD_STEP * LED_PROCESS_LIMIT
//
// Stock (8*16*16*16*17 = 557056, /255, /18) = 121 Hz. That clears flicker fusion
// when the eyes are still, but the ~0.46 ms channel spacing smears into visible
// colour fringes during a saccade -- the DLP "rainbow" artifact.
//
// There are two ways to raise the field rate, and they are NOT equivalent:
//
//   - Lower periodticks. Costs PWM resolution, because periodticks is also the
//     val ceiling and the driver feeds the raw colour byte straight in as duty
//     (pwmEnableChannel(..., led_state[i].b)). An earlier revision did this
//     (128 -> 242 Hz) and it was the wrong trade: this board is run dim in a
//     dark room, so coarsening the low end is exactly the wrong thing to spend.
//     (Duty also sets how wide each colour's image is while the R->B->G spacing
//     stays fixed, so a short pulse gives crisper, better-separated bars -- but
//     that is a second-order effect. The reason the artifact stands out in a
//     dark room is mostly dark adaptation and contrast, not the duty cycle.)
//   - Raise freq. Costs only UI step granularity, and PWM_CLK is SN32_HCLK
//     (48 MHz) against a current freq of ~0.56 MHz, so there is ample headroom.
//
// Raising freq is strictly better here, and SPD_STEP is the right factor to
// spend: it sets only the animation-speed step, meaningless on solid_color.
// SPD_STEP 16 -> 64 quadruples freq, which lets periodticks go back to the full
// 255 (all 256 brightness levels restored, and VAL_STEP 8 now yields 31 dimming
// steps instead of 16 -- finer low-end control, which is the point).
//
//   freq = 16*16*8*64*17 = 2228224
//   psc  = 48e6/2228224 - 1 = 20  (integer division; hal_pwm_lld.c:293)
//   effective freq = 48e6/21 = 2285714 Hz
//   field rate = 2285714 / 256 / 18 = 496 Hz   (4.1x the 121 Hz stock)
//
// SPD_STEP 128 is the current setting: freq 4456448, psc 9, effective 4.8 MHz,
// field rate 4.8e6/255/18 = 1046 Hz, row ISR ~18800/s. This is the highest
// field rate reachable without spending periodticks (i.e. without giving up
// dimming resolution), and the rainbow artifact is at its least visible here.
//
// IT WAS PREVIOUSLY BACKED OUT because it broke Bluetooth -- you could out-type
// the link. That diagnosis was half right and the stated mechanism was WRONG:
//
//   - WRONG: "the RX FIFO is DISABLED, so a byte must be serviced within ~87us."
//     The FIFO is enabled. UART_RxFIFOThreshold_1 is 0x00 and the LLD
//     unconditionally ORs UART_FIFO_Enable, so the threshold is a trigger level,
//     not a disable. The real budget is the FIFO depth, not one byte time.
//
//   - RIGHT: the row ISR really was stealing UART servicing. But that was an
//     interrupt PRIORITY inversion, not a raw rate problem: SN32_SERIAL_UART2
//     defaulted to priority 3 while the PWM row ISR sat at 2, so the row scan
//     could preempt byte servicing no matter how fast or slow it ran.
//
// mcuconf.h now sets UART2 to priority 1, GPT CT16B3 to 2, and the PWM row ISR
// to 3. The row ISR can no longer preempt the UART at any rate, which is what
// makes 128 safe again. If Bluetooth throughput ever regresses, verify that
// ordering FIRST -- do not reflexively drop SPD_STEP.
//
// The old coupling to BKL_PWM_TICKS / IND_PWM_TICKS IS GONE. The backlight and
// indicator software PWM used to tick from the row ISR, so their switching
// rates moved with this constant. They now run from a dedicated 20 kHz GPT
// (CT16B3/GPTD4, see pwm_tick_init in ak820pro.c), measured at 20000 Hz. This
// constant no longer affects them at all.
//
// Reference points: matrix scan 1396 Hz at 2189/s, ~1050 Hz at 8963/s, ~585 Hz
// at 18800/s. Measure with `qmk console` (DEBUG_MATRIX_SCAN_RATE is on).
//
// NOTE: changing periodticks reinterprets every STORED val -- EEPROM survives a
// flash, so the board comes back at a different brightness and needs re-dimming
// by hand. Keep RGB_MATRIX_DEFAULT_VAL in keyboard.json scaled to the ceiling
// (64/255 ~= the same 25% output as the old 32/128).
#define RGB_MATRIX_MAXIMUM_BRIGHTNESS 255
/* ⚠️ THE STEP SIZES NO LONGER AFFECT THE FIELD RATE. Read this before tuning.
 *
 * The SonixQMK driver derived the LED PWM clock from the PRODUCT of the four UI
 * step sizes and LED_PROCESS_LIMIT, so making any one of them finer silently
 * halved the field rate and brought the DLP-rainbow artifact back. That forced
 * every granularity choice to be paid for by coarsening another axis.
 *
 * SN32F2XX_RGB_PWM_FREQ (added to drivers/led/sn32f2xx.c) pins the clock
 * instead, so the steps below are now purely a UI decision. 4,800,000 gives
 * psc = 48e6/4.8e6 - 1 = 9, an effective 4.8 MHz, and 4.8e6/256/18 = 1042 Hz --
 * the same field rate this board ran before, with none of the trade.
 *
 * The old coupling is why the notes above talk about "spending" one step to buy
 * another. That reasoning is now obsolete; it is kept for the history because
 * the rebalance table explains where the constants came from. */
#define SN32F2XX_RGB_PWM_FREQ 4800000
/* The row ISR writes the column registers directly (~60 us instead of ~160) and
 * a dark tail keeps each slot at 1085 PWM ticks = 226 us, the period the old
 * ISR ran at on d70e9c68b3: the same field rate (~245 Hz) and brightness, with
 * the saved ~100 us per slot given to the main loop (drivers/led/sn32f2xx.c,
 * sn32f2xx_fast_cols / sn32f2xx_slot_ticks; checked by eye 2026-10-21: same
 * brightness and colour, pure R/G/B even, a lone key white). */
#define SN32F2XX_RGB_FAST_COLS  true
#define SN32F2XX_RGB_SLOT_TICKS 1085

/* 256 values, 1.4 deg -- the FINEST POSSIBLE. Hue is an 8-bit value in QMK, so
 * 256 steps over 360 deg is the hardware floor; true single-degree steps would
 * need a wider hue type throughout rgb_matrix. Only worth it because holding
 * traverses the whole range in ~4 s. */
#define RGB_MATRIX_HUE_STEP 1
#define RGB_MATRIX_SAT_STEP 4    /* 64 values */
#define RGB_MATRIX_VAL_STEP 2    /* 128 values -- the dim end is where this board lives */
#define RGB_MATRIX_SPD_STEP 4    /* 64 values -- 1.6% per press.
                                  * Fine because hold-to-repeat covers the
                                  * distance, and because speed is the SECOND
                                  * HUE in ALPHAS_MODS (5.6 deg per press) and
                                  * the drop density in RAINFALL. Free only
                                  * because SN32F2XX_RGB_PWM_FREQ pins the PWM
                                  * clock -- before that this would have HALVED
                                  * the LED field rate. */

// Columns are shared between the key matrix and the (column-active-LOW) LED matrix.
// Drive unselected key-rows HIGH (instead of leaving them high-Z): a pressed switch
// then pulls its shared column to INACTIVE (high) rather than active (low), so a
// keypress no longer lights up its whole LED column ("column leak").
#define MATRIX_UNSELECT_DRIVE_HIGH

#define SPI_DRIVER SPID0

#define SPI_MOSI_PIN    D2
#define SPI_SCK_PIN     D0
#define SPI_MISO_PIN    NO_PIN
#define SPI_SS_PIN      B8

#define CH582_SERIAL_DRIVER SD2

/* The LCD clock is seeded once from the external PCF8563 (I2C), then advanced by
 * the SN32 internal RTC's per-second interrupt (rtc.c). RTC_TICK_SECCNTV (rtc.c)
 * sets the nominal 1 Hz divider. */

/* Auto-calibration: periodically read the PCF8563 (crystal) and discipline the
 * SN32 RTC to it -- snap the phase and trim the divider so the clock self-locks
 * on any hardware (no hardcoded SECCNTV) and tracks temperature drift. Costs one
 * I2C read per RTC_CHECK_INTERVAL_S. Comment out to free-run the RTC. */
#define RTC_AUTO_CALIBRATION

/* rtc.c defaults this to 3600 -- once an HOUR. (The comment above used to claim
 * "~1/min" and name a nonexistent RTC_CAL_INTERVAL_S; both were wrong.) An hour
 * of free-running SN32 RTC between corrections is a lot of rope, and the
 * correction only fires past RTC_DRIFT_THRESHOLD_S anyway, so error can sit
 * uncorrected well past a minute. One bit-banged I2C read per minute is cheap.
 *
 * NOTE this only disciplines the SN32 RTC *to the PCF8563*. If the PCF itself is
 * off -- and this unit's is a CHMC D8563F clone -- no calibration interval helps;
 * that needs a host resync (ak820ctl clock). */
/* Log the phase error on EVERY discipline pass, not just when it exceeds the
 * snap threshold. ~1440 lines/day at a 60 s interval -- for debug sessions. */
// #define RTC_LOG_EVERY_PASS

#define RTC_CHECK_INTERVAL_S 60

/* NO_USB_STARTUP_CHECK is enabled automatically by BLUETOOTH_ENABLE (custom
 * driver); it keeps the main loop (matrix scan + key processing) running when
 * USB is suspended/unplugged so wireless typing works on battery.
 *
 * SIDE EFFECT, and it is a real one: QMK's ENTIRE remote-wakeup path lives
 * inside that same `#if !defined(NO_USB_STARTUP_CHECK)` block in
 * tmk_core/protocol/chibios/chibios.c. So enabling Bluetooth silently removes
 * the ability to WAKE A SLEEPING HOST BY TYPING -- on USB, permanently, even in
 * wired mode. The hardware is fine: the SN32 LLD implements usb_lld_wakeup_host()
 * and drives the K-state correctly; QMK just never calls it.
 *
 * QMK cannot keep its version, because it wakes the host from inside a BLOCKING
 * `while (USB_SUSPENDED)` loop -- exactly the loop that must not exist here, or
 * the main loop stalls and the wireless link dies. So we request the wakeup
 * ourselves from a keypress instead. See usb_wakeup_try() in ak820pro.c. */
#define USB_WAKEUP_ON_KEYPRESS

/* The mode slider drives the connection host explicitly (dip_switch_update_user),
 * but define a sane boot default before the first slider callback fires. */
#define CONNECTION_HOST_DEFAULT CONNECTION_HOST_USB

#define LED_WINLOCK_PIN     C15
#define LED_CHARGING_PIN    B18

#define CHARGE_CHRG_PIN   B16
#define CHARGE_STDBY_PIN  B17


// Idle thread sleeps on WFI (lower idle current). Safe here only because
// efl_ramtext.diff makes the SN32 EFL flash program/erase run from SRAM: with
// the flash routines in flash, a VIA/eeconfig flash write stalls instruction/
// vector fetch long enough that the SPI0 DMA completion IRQ is missed and the
// WFI-idle wakeup is lost -> hang. RAM-resident flash ops keep IRQs serviced.
// (If you drop efl_ramtext.diff, set this back to FALSE.)
//
// SET TO FALSE 2026-08-28: efl_ramtext.diff narrows that race but does NOT close
// it. Reproduced on this unit while stepping RGB values (each step writes
// eeconfig to flash), and caught live on the console:
//
//   16:58:37  console attached to an already-hung board
//   16:58:37..17:00:51  TOTAL silence -- no "matrix scan frequency" line
//   17:00:51  unplugged; 17:00:54 back to ~1130 Hz immediately
//
// During the hang: USB still enumerated (0x8009, all 6 HID interfaces), but a
// raw-HID round-trip returned "no reply" and the LED row mux was frozen with one
// hardware row still energised. So the USB peripheral was alive while nothing at
// the application level ran -- the signature of the CPU parked in WFI with no
// pending interrupt left to wake it. Only a power cycle recovered it.
//
// Spinning in the idle thread cannot lose a wakeup, so this removes the failure
// mode rather than narrowing it. Cost is power: the MCU never sleeps, which is
// irrelevant on USB but shortens BT/2.4G battery life. Accepted deliberately --
// this board lives plugged in, and a freeze beats a shorter runtime.
//
// UPDATE, same day: this did NOT fix the hang. It reproduced again on the very
// next build, with the console log ending on "rgb matrix set hsv [EEPROM]". The
// failure config.h describes above has two halves -- a missed SPI0 DMA completion
// IRQ *and* a lost WFI wakeup -- and only the second is addressed here. The first
// is the one that actually bites. The working fix is the eeconfig flush guard in
// ak820pro.c (rgb_matrix_eeprom_flush_allowed) plus RGB_MATRIX_EEPROM_WRITE_DELAY.
//
// Kept FALSE anyway: it removes one of the two failure halves for free on a board
// that lives plugged in. Do not mistake it for the fix.
#define CORTEX_ENABLE_WFI_IDLE FALSE

/* NOT a debug leftover -- keep it in every build. It makes QMK count matrix
 * scans, and health.c reads get_matrix_scan_rate() for the scan_rate counter
 * (health page 1, the agent's history) and the Fn+D debug page. Without a
 * console its once-a-second print is compiled out; the count is what stays. */
#define DEBUG_MATRIX_SCAN_RATE
// HH:MM:SS rather than HH:MM. display.c defaults this ON; the board shipped it
// off with no stated reason. The per-second path is already cheap -- it redraws
// only the character cells that actually changed, which the monospace clock font
// makes exact -- so in practice this is one extra glyph blit per second.
// Also makes RTC drift measurable by eye against a reference clock.
#define DISPLAY_CLOCK_SHOW_SECONDS  TRUE

// Persist a small keyboard-specific config block in EEPROM. On SN32F290 QMK's
// default "vendor" EEPROM driver is wear-leveling backed by MCU internal flash.
// Layout is owned by kb_eeconfig.c (BT slot, RTC period, LCD brightness,
// clock format, battery level, power mode, stock lighting flags). Bump EECONFIG_KB_DATA_VERSION if the layout
// changes -- QMK validates the block by version, not size, and a mismatch
// zero-fills it once at the next boot (a flash erases the whole emulated
// EEPROM anyway).
#define EECONFIG_KB_DATA_SIZE    8
// The battery row in whole percent. Define BATTERY_SHOW_TENTHS for tenths
// ("85.9%"), the view the 2026-09-28 drain test was calibrated with.
#define EECONFIG_KB_DATA_VERSION 5

// Debounce eeconfig flushes from RGB adjustment. Without this QMK writes MCU
// flash on every single step (~8/s while a key is held), which is both needless
// wear and a wide window for the LCD-DMA collision documented in ak820pro.c.
#define RGB_MATRIX_EEPROM_WRITE_DELAY 750
/* How long the RGB settings must be UNCHANGED before the eeconfig write is
 * allowed. QMK's own delay above is a rate limit, so without this a write
 * fires every 750 ms throughout a held adjust key -- and each one blanks the
 * LEDs for milliseconds. See rgb_matrix_eeprom_flush_allowed() in ak820pro.c. */
#define RGB_SETTLE_MS 900

/* Hold-to-repeat for the RGB adjust keys (hue/sat/val/speed). 32 hue values at
 * one press each is a lot of pressing; holding sweeps the range in ~2 s.
 *
 * ⚠️ COUNTERINTUITIVE: this REDUCES internal-flash writes rather than
 * multiplying them. rgb_matrix's eeconfig flush is debounced by
 * RGB_MATRIX_EEPROM_WRITE_DELAY (750 ms) after the LAST change, so a single
 * hold produces ONE write at the end no matter how many steps it walked --
 * whereas twenty deliberate taps spaced beyond the debounce produce twenty.
 * Flash writes are the trigger for the unresolved RGB-adjust hang, so holding
 * is the safer gesture, not the riskier one.
 *
 * The real cost is display blits: the parameter overlay repaints on every
 * value change, so a hold runs ~16 blits/s against a ~1/s idle baseline. The
 * never-started-DMA fault scales with blit count, but it is absorbed by the
 * retry in lcd_blit_wait(), so the worst case is an extra invisible retry
 * during a long hold. */
/* Stage 2 of the console investigation: report the worst main-loop gap on the
 * LCD. Comment out once the console change is validated -- see the plan. */
/* ⚠️ DISABLED. The probe reported by DRAWING to the panel, and drawing is what
 * it was measuring: each report issues ~12 glyph blits, each blit blocks the
 * main loop, which produces more stalls, which triggers another report. On
 * hardware 2026-08-31 that ran away to "8x 154 blit" -- a 154 ms stall -- and
 * left the board unusable. Any future probe must report somewhere that is not
 * the thing under test. */
// #define LOOPGAP_INSTRUMENT   /* diagnostics off for daily use */
#define LOOPGAP_SETTLE_MS 10000  /* skip boot; measure steady state only */

#define PARAM_REPEAT_DELAY_MS    400   /* hold this long before repeating starts */
#define PARAM_REPEAT_INTERVAL_MS  60   /* first cadence: ~16 steps/s, fine for nudging */
/* Then ACCELERATE. Fine steps and a fixed repeat rate are in direct conflict:
 * 128 brightness values at 16/s is an 8-second sweep. Ramping after a moment
 * of holding keeps small corrections precise (you release before it speeds up)
 * while making a full traverse quick. */
#define PARAM_REPEAT_FAST_AFTER_MS 800  /* holding this long past the initial delay... */
#define PARAM_REPEAT_FAST_MS        12  /* ...switches to ~83 steps/s (256 hue values in ~4 s) */

/* Measured ILRC divider for THIS unit, seeding the RTC so it does not have to
 * climb from the LLD's nominal 32000 on every boot (the trimmed value is not
 * persisted). Three independent windows on 2026-08-28 gave 33391 / 33343 / 33484
 * -- mean ~33400, spread 0.2% -- i.e. the ILRC runs ~34.3 kHz, not the 32 kHz
 * hal_rtc_lld.h assumes.
 *
 * Starting here lands within ~0.2% instead of 4%, so the clock is usable within
 * a minute of boot rather than after a ~40 minute convergence with the display
 * jumping several seconds every few minutes throughout.
 *
 * RC oscillator: per-unit and temperature-dependent. This is a seed, not a
 * calibration -- RTC_AUTO_CALIBRATION still does the real work. */
/* MEASURED ON THIS UNIT, 2026-08-30. Two independent estimates agree:
 *   - the firmware's own trim computed 33587 from a 360-ticks / 358-s window
 *   - a host phase measurement after that trim implied 33607
 * The previous 33400 left the clock ~0.5% FAST from boot (~7 ms/s), taking
 * about seven minutes of trimming to converge -- visible as the display
 * drifting to the 2 s snap threshold and jumping back.
 *
 * PER-UNIT AND TEMPERATURE-DEPENDENT: the ILRC is an untrimmed on-chip RC
 * oscillator that varies part to part. NEVER upstream this value -- it would
 * start other units further off than the nominal does. Measure yours with
 * hostagent/clock-phase.py, or read the value the trim converges to with
 * RTC_LOG_EVERY_PASS enabled. */
#define RTC_PERIOD_INITIAL 33600

// LCD backlight level at boot: index into bkl_duty[] in graphics/display.c.
// 1 = the dimmest lit step (1/64 duty, ~1.6%), chosen on hardware as "about
// right for a dark room". 0 would be off; BKL_MAX_LEVEL (11) is full.
// Not persisted deliberately -- see display_set_brightness().
/* --- Parameter readout overlay --------------------------------------------
 * Show a setting in the LCD info band while you adjust it: RGB brightness, hue,
 * saturation, speed and effect, plus LCD backlight level.
 * SELF-CONTAINED AND REMOVABLE: comment out this one define and every part of
 * the feature compiles out (poll task and key handlers in ak820pro.c, string
 * slot in display.c). Nothing else depends on it. */
/* Encoder press/release gap. ENCODER_MAP_KEY_DELAY defaults to TAP_CODE_DELAY,
 * which defaults to 0 -- and at 0 the delay is #if'd OUT of quantum/encoder.c
 * entirely, so an encoder press and its release are adjacent instructions.
 *
 * That breaks MODIFIED consumer keycodes on the knob. LSA(KC_VOLU) registers
 * Shift+Alt into the KEYBOARD report, sends the volume usage in the CONSUMER
 * report, then clears the modifiers -- and with no gap all of that can fall
 * inside one USB poll interval, so the host may sample a partial or already-
 * cleared modifier state. Measured on macOS with the knob set to LSA(KC_VOLD/U):
 *
 *   both mods seen  -> quarter-step volume (correct)     ~2/3 of clicks
 *   no mods seen    -> full-step volume                  ~1/3
 *   Alt only seen   -> opens the Sound settings dialog   rare
 *
 * Three different results from one gesture is the signature of a host sampling a
 * transient state, not of a mapping error.
 *
 * 10 ms follows QMK's own encoder fallback in the same file, which uses
 * tap_code_delay(KC_VOLU, 10) for exactly this reason. COST: wait_ms BLOCKS the
 * main loop and is applied after BOTH the press and the release, so a detent
 * costs ~20 ms and a fast spin will stall matrix scanning. Lower it if spinning
 * feels sluggish; raise it if modified encoder keycodes are still unreliable. */
/* wait_ms() here BLOCKS THE MAIN LOOP, which stops the encoder being sampled --
 * so this delay does not merely slow the knob down, it makes fast spins DROP
 * DETENTS outright. At 10 it was ~36 ms per click and the knob had to be turned
 * very slowly; at 5 it was still noticeably behind.
 *
 * It turns out to be largely unnecessary. usb_endpoint_in_send() writes into an
 * OUTPUT BUFFER QUEUE (obqWriteTimeout), so consecutive consumer reports are
 * already serialised and drain on successive USB frames -- they cannot coalesce
 * the way a same-frame overwrite would. QMK's own comment says these delays
 * "cater for Windows", so 1 keeps a token gap rather than compiling the delay
 * out entirely (at 0 the whole block is #if'd away).
 *
 * Modifier ORDERING is a separate problem and is handled properly by
 * process_modified_consumer() in ak820pro.c -- do not put that job back here. */
#define ENCODER_MAP_KEY_DELAY 1

/* The encoder event queue defaults to MAX(4, NUM_ENCODERS_MAX_PER_SIDE + 1) = 4,
 * and it is a RING buffer, so the usable depth is 3. Events past that are DROPPED
 * OUTRIGHT, not delayed -- which is why a fast spin lost a lot of clicks while a
 * moderate one was fine.
 *
 * Deepening it is nearly free: each event is an index plus a direction, so 32
 * costs tens of bytes of RAM and absorbs any burst a hand can produce. */
#define MAX_QUEUED_ENCODER_EVENTS 32

/* Gap between the MODIFIER report and the CONSUMER usage for a modified consumer
 * keycode such as LSA(KC_VOLU). See process_modified_consumer() in ak820pro.c --
 * they travel on DIFFERENT USB ENDPOINTS and the host does not guarantee ordering
 * between them, so back-to-back sends let the host sample the wrong modifier
 * state. Raise if full-step volume still slips through; lower if the knob feels
 * sluggish (each detent costs 2x this plus 2x ENCODER_MAP_KEY_DELAY). */
#define MODIFIED_CONSUMER_GAP_MS 8

/* How long the modifiers stay held after the last modified-consumer event.
 *
 * Paying MODIFIED_CONSUMER_GAP_MS on EVERY detent made a fast spin unusable --
 * ~36 ms of blocked main loop per click, which also overflowed the encoder queue
 * and dropped clicks outright. The modifiers do not need re-sending per detent:
 * hold them across the spin, exactly as a person holds Shift+Alt and taps volume
 * repeatedly. Only the FIRST click pays the ordering cost.
 *
 * Kept short because these are real modifiers -- anything typed inside this
 * window would carry them. In practice nobody types while spinning the volume
 * knob, and 150 ms is below the gap between deliberate keystrokes. */
#define MODIFIED_CONSUMER_HOLD_MS 150

#define PARAM_OVERLAY
#define PARAM_OVERLAY_HOLD_MS 2000   /* how long a change stays on screen */

/* Boot brightness, as an index into bkl_duty[] in graphics/display.c (0..9,
 * which the Fn+PgUp/PgDn readout shows as level*100/9 -- so 5 reads "LCD 56%").
 * Level 5 is duty 8/48, i.e. 17% of the actual PWM period; the levels are
 * spaced perceptually, so the index and the duty do not track each other.
 *
 * WAS 1 ("LCD 11%"), chosen in a dark room where the dimmest lit step looked
 * right. Against real content it reads as too dim to be comfortable -- the panel
 * is mostly black anyway, so the lit pixels are a small fraction of the area and
 * carry the whole impression. Middle of the range is the better compromise, and
 * it is still far below the stock always-on-full behaviour.
 *
 * Not persisted: every kb-eeconfig write is an internal-flash program/erase,
 * which is the thing that wedges this board. Change it here, not at runtime. */
#define DISPLAY_BRIGHTNESS_DEFAULT 5

// Indicator LED brightness (Caps Lock, Win Lock, Charging) -- index into
// ind_duty[] in ak820pro.c. These are software-PWM'd on the same tick as the
// LCD backlight; at full drive they are searchlights on a dark desk, and the
// charging one is not under user control at all. 1 = dimmest lit step (1/64).
// 0 would turn them off entirely.
#define INDICATOR_BRIGHTNESS_DEFAULT 9   /* stock-like: full (kirillcustom; JD runs 1) */
#define CHARGING_LED_BRIGHTNESS 9        /* stock-like: the charging LED lit while charging (JD: 0, off) */
