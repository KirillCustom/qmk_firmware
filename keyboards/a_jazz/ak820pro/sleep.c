// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The deep sleep, the idle ladder's last stage, in every build
 * (plans/DEEP-SLEEP-BUILD-PLAN.md, revision 5, cleared by codex's plan review
 * and four verifies); and, in dev builds, light sleep's sizing bits and the
 * bench's controls (plans/SLEEP-PLAN.md). Register facts are from the SN32F299
 * manual (V1.8); page numbers in the plans and docs/sn32f299.md. */
#include "quantum.h"
#include "sleep.h"
#include <hal.h>
#include "usb_main.h"   /* USB_DRIVER */
#include "power.h"
#include "battery.h"
#include "indicators.h"
#include "watchdog_record.h"
#include "kb_eeconfig.h"
#include "rtc/rtc.h"
#include "graphics/display.h"
#include "bluetooth/ch582f_ajazz.h"
#include "watchdog.h"
#include "health.h"
#include "drivers/led/sn32f2xx.h"
#include "overrun_diag.h"   /* dev: a UART overrun's context */

/* The encoder driver's decoder state, read only (drivers/encoder/
 * encoder_quadrature.c, a local accessor): A in bit 0, B in bit 1. */
uint8_t encoder_quadrature_state(uint8_t index);
/* quantum/keyboard.c's activity marks (defined there, not in its header). */
void last_matrix_activity_trigger(void);
void last_encoder_activity_trigger(void);

/* ---- The pass budget ---------------------------------------------------------
 * The system time's counter (CT16B5, 187.5 kHz, 16-bit): a pass is far under
 * its 350 ms wrap. Marked at the end of each pass, which is where the next
 * one starts: QMK's keyboard and raw-HID work run before housekeeping. */
static systime_t pass_t0;

sysinterval_t sleep_pass_ticks(void) {
    return (sysinterval_t)(systime_t)(chVTGetSystemTimeX() - pass_t0);
}

/* ---- USB presence ------------------------------------------------------------
 * Is a host on the bus? Not the USB driver's state: with the data cut (the
 * bench's uhubctl) it stays USB_ACTIVE, nothing telling it the bus went away
 * (10-18). The frame number (FRMNO, a SOF every 1 ms) moves only while a host
 * is talking: a bus whose frame number has not moved for 100 ms has no host, or
 * a suspended one, which the frame number cannot tell apart. Sampled every
 * main-loop pass. */
#define USB_IDLE_MS 100u
static uint16_t usb_fn_last     = 0xFFFFu;
static uint32_t usb_fn_moved_at = 0;

static void usb_watch(void) {
    uint16_t fn = (uint16_t)(SN_USB->FRMNO & 0x7FFu);
    if (fn != usb_fn_last) {
        usb_fn_last     = fn;
        usb_fn_moved_at = timer_read32();
    }
}

static bool usb_bus_live(void) {
    usb_watch();
    return timer_elapsed32(usb_fn_moved_at) < USB_IDLE_MS;
}

/* PMU Sleep and Deep-sleep turn the USB transceiver off (manual p. 245,
 * USB_CFG.PHY_EN, bit 30); setting it back was enough on every bench wake.
 * Returns whether it was off. */
#define USB_CFG_PHY_EN (1u << 30)
static bool usb_phy_onI(void) {
    if (SN_USB->CFG & USB_CFG_PHY_EN) return false;
    SN_USB->CFG |= USB_CFG_PHY_EN;
    return true;
}

/* ---- Light sleep (dev builds) -------------------------------------------------- */
#ifdef AK820_DEV_HOOKS
static uint8_t  ls_mask      = 0;
static uint8_t  ls_yield_ms  = 2;
static uint32_t ls_pmu_until = 0;   /* timer ms; bit 4 runs until then */
static uint32_t ls_until     = 0;   /* timer ms; the whole mask ends then (codex: a probe that
                                     * breaks USB must end by itself, or nothing can end it) */
static uint8_t  ls_applied   = 0;
/* Read by the idle thread's hooks (chconf.h). */
volatile bool     ak820_wfi       = false;
volatile bool     ak820_pmu_sleep = false;
volatile uint32_t ak820_wfi_count = 0;   /* WFIs the idle thread executed */

void sleep_light_set(uint8_t mask, uint8_t yield_ms, uint8_t pmu_s, uint8_t ttl_s) {
    uint32_t now = timer_read32();
    ls_mask      = mask & SLEEP_LS_ALL;
    ls_yield_ms  = (yield_ms >= 1 && yield_ms <= 4) ? yield_ms : 2;   /* the RX queue wants a pass every ~5.5 ms */
    ls_pmu_until = now + (uint32_t)(pmu_s ? pmu_s : 60) * 1000u;
    ls_until     = now + (uint32_t)(ttl_s ? ttl_s : 240) * 1000u;
}

uint8_t sleep_light_yield(void) {
    return ls_yield_ms;
}

uint8_t sleep_light_mask(void) {
    return ls_mask;
}

uint8_t sleep_light_applied(void) {
    return ls_applied;
}

static void light_task(void) {
    if (ls_mask && (int32_t)(timer_read32() - ls_until) >= 0) ls_mask = 0;
    uint8_t m = power_rgb_dark() ? ls_mask : 0;
    /* Bit 4 only inside its window, and never with a host's frames: the
     * transceiver turns itself off in PMU Sleep (manual p. 245). */
    if ((m & SLEEP_LS_PMU) && ((int32_t)(timer_read32() - ls_pmu_until) >= 0 || usb_bus_live()))
        m &= (uint8_t)~SLEEP_LS_PMU;
    /* Bit 3 only with the screen dark and no indicator lit. */
    if ((m & SLEEP_LS_TICK) && (display_backlight_effective() > 0 || indicators_any_lit()))
        m &= (uint8_t)~SLEEP_LS_TICK;
    if ((m ^ ls_applied) & SLEEP_LS_TICK) pwm_tick_run(!(m & SLEEP_LS_TICK));
    /* Leaving PMU Sleep, or the plain-WFI probe (manual p. 79 documents
     * mode 0 only as "disable": whether a WFI there touches USB is what bit 2
     * finds out; codex): the transceiver bit back, a no-op if it is on. */
    if ((ls_applied & (SLEEP_LS_PMU | SLEEP_LS_WFI)) & ~m) {
        chSysLock();
        usb_phy_onI();
        chSysUnlock();
    }
    sn32f2xx_set_scan_only((m & SLEEP_LS_SCAN) != 0);
    ak820_wfi       = (m & (SLEEP_LS_WFI | SLEEP_LS_PMU)) != 0;
    ak820_pmu_sleep = (m & SLEEP_LS_PMU) != 0;
    ls_applied      = m;
    if (m & SLEEP_LS_YIELD) chThdSleepMilliseconds(ls_yield_ms);
}
#else
void    sleep_light_set(uint8_t mask, uint8_t yield_ms, uint8_t pmu_s, uint8_t ttl_s) { (void)mask; (void)yield_ms; (void)pmu_s; (void)ttl_s; }
uint8_t sleep_light_yield(void) { return 0; }
uint8_t sleep_light_mask(void) { return 0; }
uint8_t sleep_light_applied(void) { return 0; }
#endif

/* ---- The deep sleep -----------------------------------------------------------
 * Plan section 2. The locked sequence is the experiment's, as codex cleared it
 * five times (PRIMASK held through the clock restore; I-class calls and
 * register writes only; each wake source cleared in the peripheral and the
 * NVIC; the temporary vectors disabled before the old enables return; the
 * IHRC, then the PLL off, then the flash's slow mode, and back in reverse),
 * with: the system time's counter stopped, the knob and the charger as wake
 * sources, 32-bit counts and a daily backstop. */

/* The preconditions that can fail (the record's and the page's u16). */
enum {
    DSB_KEY = 0x0001, DSB_USB = 0x0002, DSB_TX = 0x0004, DSB_DISPLAY = 0x0008, DSB_FLASH = 0x0010,
    DSB_RTC = 0x0020, DSB_PINS = 0x0040, DSB_WDT = 0x0080, DSB_LIT = 0x0100, DSB_RGB = 0x0200,
    DSB_PAIRING = 0x0400, DSB_RECOVERY = 0x0800, DSB_KNOB = 0x1000, DSB_CHARGE = 0x2000,
};

/* Register bits (the SN32F299 manual; sn32_uart.h and battery.c agree). */
#define UART_LS_TEMT   (1u << 6)    /* transmitter empty, not just THR (bit 5) */
#define UART_CTRL_RXEN (1u << 6)
#define ADC_ADM_EOC    (1u << 6)
#define ADC_ADM_ADS    (1u << 7)
#define ADC_ADM_ADENB  (1u << 12)
#define UART_LS_RDR    (1u << 0)    /* receive data ready */
#define SPI_STAT_BUSY  (1u << 4)
#define SPI0_DMACTRL_DMAEN (1u << 0)
#define CLK_WAIT_LOOPS 100000u      /* a clock switch reports in a few cycles */

#define DS_STRAYS_MAX  1000u        /* a wake that is neither a source nor a second, this often: leave */
#define DS_PLL_LOOPS   200000u      /* ~ms at 12 MHz; the PLL locks in ~100 us (p. 58) */
#define DS_BACKSTOP_S  86400u       /* counted seconds: ~27.5 h real at the hour's rate */

static const pin_t ds_rows[] = MATRIX_ROW_PINS;
static const pin_t ds_cols[] = MATRIX_COL_PINS;
static const pin_t ds_enc_a[] = ENCODER_A_PINS;
static const pin_t ds_enc_b[] = ENCODER_B_PINS;

/* The wake pins' masks per port: the columns (falling), the knob's two pins
 * (both edges, those bits only) and the charger's CHRG (falling). P0, P1 and
 * P2 have interrupts wired here; anything else fails the precondition. */
typedef struct {
    uint32_t col0, col2;     /* the columns: P0, P2 */
    uint32_t knob0, knob1;   /* the knob: P0, P1 */
    uint32_t chrg1;          /* CHRG: P1 */
} wake_pins_t;

static bool pin_mask(pin_t pin, uint32_t *p0, uint32_t *p1, uint32_t *p2) {
    ioportid_t port = PAL_PORT(pin);
    uint32_t   bit  = 1u << PAL_PAD(pin);
    if (port == PAL_PORT(A0)) *p0 |= bit;
    else if (port == PAL_PORT(B0)) *p1 |= bit;
    else if (port == PAL_PORT(C0)) *p2 |= bit;
    else return false;
    return true;
}

static bool wake_pins(wake_pins_t *w) {
    uint32_t c1 = 0, k2 = 0, x0 = 0, x2 = 0;
    memset(w, 0, sizeof(*w));
    bool ok = true;
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_cols); i++) ok &= pin_mask(ds_cols[i], &w->col0, &c1, &w->col2);
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_enc_a); i++) {
        ok &= pin_mask(ds_enc_a[i], &w->knob0, &w->knob1, &k2);
        ok &= pin_mask(ds_enc_b[i], &w->knob0, &w->knob1, &k2);
    }
    ok &= pin_mask(CHARGE_CHRG_PIN, &x0, &w->chrg1, &x2);
    /* the columns on P0/P2 only, the knob on P0/P1, CHRG on P1 */
    return ok && c1 == 0 && k2 == 0 && x0 == 0 && x2 == 0;
}

/* The knob's two-bit state on its pins now (A bit 0, B bit 1). */
static uint8_t knob_pins(uint8_t i) {
    return (uint8_t)((gpio_read_pin(ds_enc_a[i]) ? 1u : 0u) | (gpio_read_pin(ds_enc_b[i]) ? 2u : 0u));
}

/* The knob at rest: its pins as the decoder last saw them (codex, plan
 * review 18: A == B before and after misses 00 -> 11). */
static bool knob_at_rest(void) {
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_enc_a); i++)
        if (knob_pins(i) != encoder_quadrature_state(i)) return false;
    return true;
}

/* The radio's side is quiet: nothing queued or awaiting an ACK, the serial
 * output queue empty (the module's ACKs go straight to it, bypassing the
 * protocol queue; codex), and UART2's transmitter empty. I-class, and a
 * thread form that takes the lock. */
static bool tx_quietI(void) {
    return ch582_tx_idle() && oqIsEmptyI(&SD2.oqueue) && (SN_UART2->LS & UART_LS_TEMT);
}

static bool tx_quietI_locked(void) {
    chSysLock();
    bool q = tx_quietI();
    chSysUnlock();
    return q;
}

static bool rc_active;   /* a wake's recovery is under way (below) */

static uint16_t blockers(void) {
    uint16_t b = 0;
    for (uint8_t r = 0; r < MATRIX_ROWS; r++)
        if (matrix_get_row(r)) b |= DSB_KEY;
    if (usb_bus_live()) b |= DSB_USB;
    if (!tx_quietI_locked()) b |= DSB_TX;
    if (!display_bus_idle() || (SN_SPI0->STAT & SPI_STAT_BUSY) || (SN_SPI0->DMACTRL & SPI0_DMACTRL_DMAEN) ||
        (SN_SPI1->STAT & SPI_STAT_BUSY))
        b |= DSB_DISPLAY;
    /* The EFL driver is STOPPED between writes (wear_leveling_efl.c starts it
     * for a write and stops it after), so idle is STOP or READY: block only a
     * program or an erase in flight, and a deferred write still to come. */
    if (kb_eeconfig_write_pending() || EFLD1.state == FLASH_PGM || EFLD1.state == FLASH_ERASE) b |= DSB_FLASH;
    if (!rtc_sleep_ok()) b |= DSB_RTC;
    wake_pins_t w;
    if (!wake_pins(&w)) b |= DSB_PINS;
    /* The way back if a wake fails: refuse without a counting watchdog
     * (degraded after three resets, or never armed; codex). */
    if (!watchdog_running() || !(SN_WDT->CFG & 1u)) b |= DSB_WDT;   /* CFG.WDTEN */
    /* A hard rule, not a timing accident: a lit screen needs the 20 kHz
     * tick, which a sleep stops (plan section 1). */
    if (display_backlight_effective() > 0) b |= DSB_LIT;
    if (!power_rgb_dark()) b |= DSB_RGB;
    if (ch582_is_pairing()) b |= DSB_PAIRING;
    if (rc_active) b |= DSB_RECOVERY;
    if (!knob_at_rest()) b |= DSB_KNOB;
    if (!gpio_read_pin(CHARGE_CHRG_PIN)) b |= DSB_CHARGE;   /* charging: on USB power, never asleep */
    return b;
}

/* Each wake source, consumed in the peripheral AND the NVIC: no handler runs
 * (PRIMASK), so nothing else clears the pending bit, and a pending enabled
 * interrupt ends every later WFI at once (codex). */
static bool take_second(void) {
    if (!(SN_RTC->RIS & mskRTC_SECIF)) return false;
    SN_RTC->IC    = mskRTC_SECIC;
    NVIC->ICPR[0] = 1u << RTC_IRQn;
    return true;
}

/* What the last attempt did (the record, the page). */
static uint8_t  ds_reason, ds_flags;
static uint16_t ds_strays, ds_pll_loops, ds_kp_before;
static uint32_t ds_secs, ds_p0_ris, ds_p1_ris, ds_p2_ris;
static uint32_t ds_sleeps, ds_slept_total;   /* since boot */
static uint16_t ds_wanted_blocked;   /* the blockers while a sleep was last wanted */

/* The recovery's state (below). */
static uint8_t  rc_open;             /* SLEEP_RC_* still pending */
static uint32_t rc_woke_at;
static uint32_t rc_enc_t0;
static uint16_t rc_reports0;
static uint32_t rc_poll_at, rc_gauge_at;
static bool     rc_poll_sent;
static uint8_t  ur_state;            /* the USB re-attach */
static uint32_t ur_at;
static uint16_t ur_fn0;
static uint32_t rc_pcf_s;
static bool     rc_usb_watch;
static uint16_t rc_samples0;         /* battery_supply_samples() at the wake */
/* The steps that ended on their own timeout or the 10 s bound, not by doing
 * their job (SLEEP_RC_* bits; codex, deep-sleep build review 7). */
static uint8_t  rc_timed_out;

static void recovery_begin(uint8_t reason);

/* The Deep-sleep itself, run from SRAM (.ramtext, copied at boot with .data;
 * plans/DEEP-SLEEP-REGRESSION-2026-10-20.md). On b2a07e0d37 every deep sleep
 * ended after one counted second. Its WFI was the second half of the first
 * word in an 8-byte flash block, with straight-line code after it, and which
 * builds failed followed only where the linker put the WFI: at 2 mod 8 with
 * no branch after it, it failed every time; so any change anywhere could
 * bring it back. The evidence fits the next block's fetch after the wake
 * returning the stale one (the DSB's tail, the WFI and the CTRL write): the
 * core ran 0x8f4f as `ldrh r7, [r1, #58]` (r7 = 0xbf30 sent the wake's store
 * astray) and skipped the masks' loads. That is a hypothesis, not measured.
 * From SRAM, nothing is fetched from the flash until ~1,300 cycles have
 * passed and one flash word has been read and dropped. How long the flash
 * needs is not measured either: do not shorten the wait without measuring
 * it. noinline and long_call: the call from the flash is a blx to SRAM. */
static const uint32_t ds_flash_word = 0x5A5AA5A5u;

__attribute__((section(".ramtext"), noinline, long_call)) static void ds_deep_wfi(volatile uint32_t *pmu_ctrl, const volatile uint32_t *flash) {
    *pmu_ctrl = 2u;   /* Deep-sleep (p. 75) */
    __DSB();
    __WFI();
    *pmu_ctrl = 0u;
    for (uint32_t d = 260u; d; d--) __asm volatile("");   /* ~5 cycles a pass: ~108 us at the IHRC's 12 MHz */
    (void)*flash;
}

#ifdef AK820_DEV_HOOKS
/* Each teardown's wake, the last eight (sleep.h), since the page keeps only
 * the last sleep's reason: on 10-19 a ~3 s wake an hour in, with no key,
 * went unnamed because the rig's tap woke the next sleep. */
typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint8_t  reason, flags;
    uint32_t counted, p0, p1, p2, ms;
    uint16_t strays;
} wake_entry_t;
_Static_assert(sizeof(wake_entry_t) == 26, "an entry and the count fill 28 bytes");
static wake_entry_t wake_ring[8];
static uint16_t     wake_seq;

static void wake_ring_put(uint32_t counted) {
    wake_entry_t *e = &wake_ring[wake_seq % 8u];
    e->seq     = wake_seq++;
    e->reason  = ds_reason;
    e->flags   = ds_flags;
    e->counted = counted;
    e->p0      = ds_p0_ris;
    e->p1      = ds_p1_ris;
    e->p2      = ds_p2_ris;
    e->ms      = timer_read32();
    e->strays  = ds_strays;
}

/* The last sleep with a stray (plans/BACKLOG.md, "A stray-wake storm"): what
 * was pending when a WFI first returned with no wake pin and no second, how
 * many more came within that counted second (a storm, or sparse returns),
 * whether anything outside the wake set could assert an enabled IRQ, and the
 * pending state at the latest stray. Op 24's slot 8. Reads only: RIS, IE,
 * ISER, ISPR and ICSR clear nothing when read. An empty snapshot means an
 * unexplained return, not proven noise (codex, the stray-diagnostic review). */
typedef struct __attribute__((packed)) {
    uint16_t seq;        /* the ring entry this sleep will write */
    uint16_t same_s;     /* strays within the first one's counted second */
    uint32_t first_s;    /* the counted second of the first */
    uint32_t ispr;       /* NVIC->ISPR[0] at the first (enabled: P0-P2 and the RTC) */
    uint32_t icsr;       /* SCB->ICSR at the first: VECTPENDING, PENDSVSET, PENDSTSET */
    uint16_t flags;      /* STRAY_F_*, at the first */
    uint16_t last_n;     /* the strays counted at the latest snapshot */
    uint32_t last_ispr;  /* NVIC->ISPR[0] at the latest stray */
    uint32_t last_icsr;  /* SCB->ICSR at the latest stray */
} stray_diag_t;
_Static_assert(sizeof(stray_diag_t) == 28, "op 24's slot 8 is 28 bytes");
#define STRAY_F_IE(port)   (1u << (port))         /* IE set outside the port's wake mask */
#define STRAY_F_MIS(port)  (1u << (3 + (port)))   /* RIS & IE outside it: could assert the port IRQ */
#define STRAY_F_ISER       (1u << 6)              /* NVIC enabled != the wake set */
static stray_diag_t stray_diag;

static void stray_note(uint32_t strays, uint32_t secs, uint32_t p0w, uint32_t p1w, uint32_t p2w, uint32_t irqs) {
    uint32_t ispr = NVIC->ISPR[0], icsr = SCB->ICSR;
    if (strays == 0) {
        uint32_t ie0 = SN_GPIO0->IE & ~p0w, ie1 = SN_GPIO1->IE & ~p1w, ie2 = SN_GPIO2->IE & ~p2w;
        uint16_t f   = 0;
        if (ie0) f |= STRAY_F_IE(0);
        if (ie1) f |= STRAY_F_IE(1);
        if (ie2) f |= STRAY_F_IE(2);
        if (SN_GPIO0->RIS & ie0) f |= STRAY_F_MIS(0);
        if (SN_GPIO1->RIS & ie1) f |= STRAY_F_MIS(1);
        if (SN_GPIO2->RIS & ie2) f |= STRAY_F_MIS(2);
        if (NVIC->ISER[0] != irqs) f |= STRAY_F_ISER;
        stray_diag.seq     = wake_seq;
        stray_diag.same_s  = 0;
        stray_diag.first_s = secs;
        stray_diag.ispr    = ispr;
        stray_diag.icsr    = icsr;
        stray_diag.flags   = f;
    }
    if (secs == stray_diag.first_s && stray_diag.same_s < 0xFFFFu) stray_diag.same_s++;
    stray_diag.last_n    = (uint16_t)(strays + 1u > 0xFFFFu ? 0xFFFFu : strays + 1u);
    stray_diag.last_ispr = ispr;
    stray_diag.last_icsr = icsr;
}

bool sleep_wake_ring_fill(uint8_t slot, uint8_t *out) {
    if (slot == 8u) {
        memcpy(out, &stray_diag, sizeof(stray_diag));
        return true;
    }
    if (slot >= 8u) return false;
    memcpy(out, &wake_ring[slot], sizeof(wake_entry_t));
    memcpy(&out[26], &wake_seq, 2);
    return true;
}
#endif

/* The sleep: false if it unwound before the teardown (a host's frame, the
 * radio busy), true otherwise, whether it slept (ds_flags) or a source was
 * already active. It wakes after `cap_s` counted seconds, counting from
 * `preset_s`; `no_feed` leaves the watchdog unfed. The ladder's sleep passes
 * the backstop, fed, from 0; only the dev bench's arm passes its own, so an
 * arm cancelled or left over never reaches the ladder's (codex, deep-sleep
 * build review 5). */
static bool deep_sleep(uint32_t cap_s, bool no_feed, uint32_t preset_s) {
    WDT_SCOPE(WDT_SITE_DEEP_SLEEP);
    static volatile uint32_t sacrificial;
    sacrificial++;   /* the breadcrumb above must reach RAM before the CPU stops */

    wake_pins_t w;
    wake_pins(&w);
    uint32_t p0w = w.col0 | w.knob0, p1w = w.knob1 | w.chrg1, p2w = w.col2;
    ds_kp_before = health_key_presses();
    OD_ACTIVITY_SET(OD_ACT_SLEEP);   /* dev: an overrun seen across it is the sleep's */

    /* Everything from here to the unlock is lock-free (I-class calls and
     * register writes): a lock call would unlock on its way out, and a wake
     * interrupt taken has no handler (codex, must-fix 1). */
    chSysLock();
    /* The admission again, under the lock: nothing can be queued for the
     * radio, or a host's frames come, between the check and the teardown. */
    if ((uint16_t)(SN_USB->FRMNO & 0x7FFu) != usb_fn_last || !tx_quietI()) {
        chSysUnlock();
        OD_ACTIVITY_CLEAR(OD_ACT_SLEEP);
        return false;
    }
    /* Outputs off: the LED matrix, the 20 kHz tick with the pins it drives. */
    sn32f2xx_sleep_enterI();
    pwm_tick_stopI();
    /* UART2's receiver off across the clock change and the sleep (TX stays
     * enabled, so its line idles high); the ADC off (~100 uA, p. 96). */
    uint32_t uart_ctrl = SN_UART2->CTRL;
    SN_UART2->CTRL     = uart_ctrl & ~UART_CTRL_RXEN;
    OD_RX_OFF();   /* dev: what arrives now is lost by design, counted apart */
    uint32_t adm       = SN_ADC->ADM & ~(ADC_ADM_EOC | ADC_ADM_ADS);
    SN_ADC->ADM        = adm & ~ADC_ADM_ADENB;
    /* Every key row low, every column an input with its pull-up, so a press
     * pulls its column low. The knob's and CHRG's pins are inputs already. */
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_rows); i++) {
        gpio_set_pin_output_push_pull(ds_rows[i]);
        gpio_write_pin_low(ds_rows[i]);
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_cols); i++) gpio_set_pin_input_high(ds_cols[i]);
    /* Only the wake sources in the NVIC; SysTick's alarm off; the system
     * time's counter stopped, so it stands still through the whole sleep,
     * the core's microseconds awake each second included (codex, plan review
     * 1: a week would otherwise pass QMK's wrap unobserved). CEN holds the
     * counter and the prescale counter (manual p. 125). */
    uint32_t iser = NVIC->ISER[0];
    NVIC->ICER[0] = 0xFFFFFFFFu;
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;
    SN32_ST_TIM->config.TMRCTRL &= ~mskCT16_CEN_EN;
    /* The wake pins: edge sensed (IS 0). The columns and CHRG one edge,
     * falling (IBS 0, IEV 1); the knob both edges (IBS 1), its bits only
     * (pp. 81-84). Cleared after the pins settle, then enabled. */
    uint32_t p0f = w.col0, p1f = w.chrg1, p2f = w.col2;
    SN_GPIO0->IS &= ~p0w;  SN_GPIO0->IBS &= ~p0f;  SN_GPIO0->IEV |= p0f;  SN_GPIO0->IBS |= w.knob0;
    SN_GPIO1->IS &= ~p1w;  SN_GPIO1->IBS &= ~p1f;  SN_GPIO1->IEV |= p1f;  SN_GPIO1->IBS |= w.knob1;
    SN_GPIO2->IS &= ~p2w;  SN_GPIO2->IBS &= ~p2f;  SN_GPIO2->IEV |= p2f;
    for (volatile uint32_t d = 0; d < 1200u; d++) {}   /* ~100 us at 48 MHz */
    SN_GPIO0->IC = p0w;    SN_GPIO1->IC = p1w;    SN_GPIO2->IC = p2w;
    SN_GPIO0->IE |= p0w;   SN_GPIO1->IE |= p1w;   SN_GPIO2->IE |= p2w;
    uint32_t wake_irqs = (1u << P0_IRQn) | (1u << P1_IRQn) | (1u << P2_IRQn) | (1u << RTC_IRQn);
    NVIC->ICPR[0] = wake_irqs;
    NVIC->ISER[0] = wake_irqs;

    uint32_t secs = preset_s, strays = 0;
    if (take_second()) secs++;   /* one that came since the lock: its handler cannot run now */
    /* A source already active is a wake with no sleep (codex, must-fix 2): a
     * column low (a key down, or pressed before the clear erased its edge);
     * the knob moved since the admission (its state against the decoder's,
     * or an edge latched since the clear); CHRG low. */
    uint32_t low0 = ~SN_GPIO0->DATA & w.col0, low2 = ~SN_GPIO2->DATA & w.col2;
    uint32_t chg1 = ~SN_GPIO1->DATA & w.chrg1;
    bool     knob = (SN_GPIO0->RIS & w.knob0) || (SN_GPIO1->RIS & w.knob1);
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_enc_a); i++)
        if (knob_pins(i) != encoder_quadrature_state(i)) knob = true;
    bool     slept = false;
    uint32_t clkcfg = SN_SYS0->CLKCFG & 0x7u, pllctrl = SN_SYS0->PLLCTRL;
    uint32_t lpctrl = SN_FLASH->LPCTRL & 0xFFFFu;
    ds_p0_ris = ds_p1_ris = ds_p2_ris = 0;
    if (low0 || low2) {
        ds_p0_ris = low0;
        ds_p2_ris = low2;
        ds_reason = SLEEP_WAKE_KEY;
    } else if (knob) {
        ds_reason = SLEEP_WAKE_KNOB;
    } else if (chg1) {
        ds_p1_ris = chg1;
        ds_reason = SLEEP_WAKE_CHARGE;
    } else if (!(SN_SYS0->CSST & 0x1u)) {   /* IHRCRDY: the clock to drop to */
        ds_reason = SLEEP_WAKE_STRAYS;
    } else {
        /* The system clock to the IHRC, then the PLL off (never a selected
         * PLL, p. 64), then the flash to its slow mode for <= 24 MHz. */
        SN_SYS0->CLKCFG = 0x0;
        for (uint32_t k = 0; (SN_SYS0->CLKCFG & 0x70u) != 0x0u; k++)
            if (k >= CLK_WAIT_LOOPS) NVIC_SystemReset();
        SN_SYS0->PLLCTRL = pllctrl & ~(1u << 15);   /* PLLEN */
        SN_FLASH->LPCTRL = 0x5AFA0000u;
        slept = true;
        /* Asleep. PRIMASK is set: a pending wake source ends the WFI, but no
         * handler runs; these vectors have none. */
        for (;;) {
            ds_deep_wfi(&SN_PMU->CTRL, &ds_flash_word);   /* from SRAM, see above */
            uint32_t r0 = SN_GPIO0->RIS & p0w, r1 = SN_GPIO1->RIS & p1w, r2 = SN_GPIO2->RIS & p2w;
            bool second = take_second();
            if (second) {
                if (!no_feed) SN_WDT->FEED = (mskWDT_WDKEY | mskWDT_FV);   /* no lock calls in here */
                secs++;
                if (!(r0 || r1 || r2) && secs >= cap_s) {
                    ds_reason = SLEEP_WAKE_BACKSTOP;
                    break;
                }
            }
            if (r0 || r1 || r2) {
                ds_p0_ris = r0;
                ds_p1_ris = r1;
                ds_p2_ris = r2;
                ds_reason = (r0 & w.col0) || (r2 & w.col2) ? SLEEP_WAKE_KEY
                          : ((r0 & w.knob0) || (r1 & w.knob1)) ? SLEEP_WAKE_KNOB
                                                               : SLEEP_WAKE_CHARGE;
                break;
            }
            if (second) continue;   /* a second is not a stray */
#ifdef AK820_DEV_HOOKS
            stray_note(strays, secs, p0w, p1w, p2w, wake_irqs);
#endif
            if (++strays >= DS_STRAYS_MAX) {
                ds_reason = SLEEP_WAKE_STRAYS;
                break;
            }
        }
    }

    bool pll_ceiling = false;
    if (slept) {
        /* Awake: the flash's speed first, then the PLL, and only then select it. */
        SN_FLASH->LPCTRL = 0x5AFA0000u | lpctrl;
        SN_SYS0->PLLCTRL = pllctrl;
        uint32_t loops = 0;
        while ((SN_SYS0->CSST & 0x40u) != 0x40u) {   /* PLLRDY */
            if (++loops >= DS_PLL_LOOPS) break;
        }
        ds_pll_loops = loops > 0xFFFFu ? 0xFFFFu : (uint16_t)loops;
        pll_ceiling  = loops >= DS_PLL_LOOPS;
        if (pll_ceiling) NVIC_SystemReset();   /* the UART, USB and timers all assume 48 MHz */
        SN_SYS0->CLKCFG = clkcfg;
        for (uint32_t k = 0; (SN_SYS0->CLKCFG & 0x70u) != (clkcfg << 4); k++)
            if (k >= CLK_WAIT_LOOPS) NVIC_SystemReset();
    }
    /* The wake pins quiet again; the temporary vectors off BEFORE the old
     * enables go back (writing ISER cannot disable one; codex). P1 too. */
    SN_GPIO0->IE &= ~p0w;  SN_GPIO1->IE &= ~p1w;  SN_GPIO2->IE &= ~p2w;
    SN_GPIO0->IC = p0w;    SN_GPIO1->IC = p1w;    SN_GPIO2->IC = p2w;
    NVIC->ICER[0] = wake_irqs;
    NVIC->ICPR[0] = (1u << P0_IRQn) | (1u << P1_IRQn) | (1u << P2_IRQn);
    if (take_second()) secs++;   /* one coincident with the wake: counted here, once */
    NVIC->ISER[0] = iser;
    /* Rows to the unselected level the matrix scan expects (driven high). */
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_rows); i++) gpio_write_pin_high(ds_rows[i]);
    /* The ADC and UART2's receiver back, at 48 MHz; whatever the receiver
     * holds from before is dropped, with its error flags (reading LS clears
     * them), and the radio's parser starts its window over (codex). */
    SN_ADC->ADM    = adm;
    SN_UART2->CTRL = uart_ctrl;
    for (uint8_t k = 0; k < 32u && (SN_UART2->LS & UART_LS_RDR); k++) (void)SN_UART2->RB;
    (void)SN_UART2->LS;
    iqResetI(&SD2.iqueue);
    for (volatile uint32_t d = 0; d < 1200u; d++) {}   /* the ADC's 100 us before a conversion */
    /* The system time's counter runs again, on every exit; the kernel's
     * alarm left off and pended, so its handler serves the timers now and
     * re-arms the next one (codex's verify: no VT or QMK adjustment needed). */
    SN32_ST_TIM->config.TMRCTRL |= mskCT16_CEN_EN;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    sn32f2xx_sleep_exitI();
    pwm_tick_startI();
    bool     phy_off = usb_phy_onI();
    uint32_t counted = secs - preset_s;
    rtc_account_sleepI(counted);   /* both software clocks, before any second is served */
    if (slept) rtc_wake_beginI(counted);
    chSysUnlock();

    ch582_rx_resync();
#ifdef AK820_DEV_HOOKS
    ls_applied &= (uint8_t)~SLEEP_LS_TICK;   /* the tick runs again; light_task stops it if wanted */
#endif
    ds_secs   = counted;
    ds_strays = (uint16_t)(strays > 0xFFFFu ? 0xFFFFu : strays);
    ds_flags  = (uint8_t)((slept ? SLEEP_V_SLEPT : 0u) | (pll_ceiling ? SLEEP_V_PLL_CEIL : 0u));
    OD_ACTIVITY_CLEAR(OD_ACT_SLEEP);
#ifdef AK820_DEV_HOOKS
    wake_ring_put(counted);
#endif
    (void)phy_off;
    if (slept) recovery_begin(ds_reason);
    return true;
}

/* ---- The wake's recovery (plan section 4) ---------------------------------------
 * One owner, in steps, each bounded, so no pass carries two of them; the
 * board stays awake until every step has ended:
 *  1. the waking input first: no recovery I/O and no battery poll until the
 *     key's press is counted (deferred debounce needs more than one pass) or
 *     the knob's event decoded, or 50 ms;
 *  2. the coarse clock (rtc.c, once released): the PCF's slept seconds;
 *  3. the gauge and the log (battery.c), in a pass with no I2C;
 *  4. the phase (rtc.c's acquisition, re-armed): rtc_wake_busy() until done;
 *  5. the radio's battery poll, once the waking report has left; its reply
 *     or 2 s;
 *  6. host content expired (at the wake);
 *  7. the supply: 1.5 s of fresh samples after the wake, or 3 s whatever
 *     they were (on EXTERNAL the ladder is off, so the board stays awake);
 *  8. USB: one re-attach if no host's frame came within 3 s, through to D+
 *     back;
 *  9. the display: the backlight held dark until the clock and the battery
 *     row have repainted after steps 2 and 3 (or 500 ms after step 3). */
#define RC_INPUT_MAX_MS   50u
#define RC_POLL_AFTER_MS  100u
#define RC_POLL_WAIT_MS   2000u
#define RC_SUPPLY_SAMPLES 15u        /* 1.5 s of the battery task's 10 Hz samples */
#define RC_SUPPLY_MAX_MS  3000u
#define RC_HOLD_MAX_MS    500u
#define UR_WAIT_MS        3000u
#define UR_OFF_MS         100u
#define RC_LONG_SLEEP_S   7200u      /* a wake after this long asleep says time left */
#define RC_MAX_MS         10000u     /* the whole recovery's bound: the board never stays awake on a stuck step */
enum { UR_NONE = 0, UR_WAIT, UR_OFF, UR_DONE };

static void recovery_begin(uint8_t reason) {
    rc_active    = true;
    rc_open      = SLEEP_RC_INPUT | SLEEP_RC_CLOCK | SLEEP_RC_GAUGE | SLEEP_RC_RADIO | SLEEP_RC_SUPPLY |
              SLEEP_RC_USB | SLEEP_RC_DISPLAY;
    rc_woke_at   = timer_read32();
    rc_poll_sent = false;
    rc_reports0  = ch582_battery_reports();
    rc_samples0  = battery_supply_samples();
    rc_timed_out = 0;
    /* A key or knob wake is input: the ladder's idle restarts even if the
     * first detent decodes nothing (codex, plan review 4). */
    if (reason == SLEEP_WAKE_KEY) last_matrix_activity_trigger();
    if (reason == SLEEP_WAKE_KNOB) last_encoder_activity_trigger();
    rc_enc_t0 = last_encoder_activity_time();
    /* Before any other battery or radio work this pass or the next. */
    battery_sleep_wake();
    ch582_sleep_wake();
    display_sleep_wake();   /* host content expired; the backlight held */
    ur_state     = UR_WAIT;
    ur_at        = rc_woke_at;
    ur_fn0       = (uint16_t)(SN_USB->FRMNO & 0x7FFu);
    rc_usb_watch = true;
    ds_sleeps++;
}

static bool rc_done(uint8_t step) {
    return !(rc_open & step);
}

static void rc_close(uint8_t step) {
    rc_open &= (uint8_t)~step;
}

static void recovery_task(void) {
    uint32_t since = timer_elapsed32(rc_woke_at);
    /* The 20 s USB observation (the page): a host's frames again within it. */
    if (rc_usb_watch && ((uint16_t)(SN_USB->FRMNO & 0x7FFu) != ur_fn0 || since >= 20000u)) {
        rc_usb_watch = false;
        if ((uint16_t)(SN_USB->FRMNO & 0x7FFu) != ur_fn0) ds_flags |= SLEEP_V_USB_20S;
    }
    if (!rc_active) return;

    /* The bound: every step normally ends within ~3.5 s (the USB re-attach's
     * 3 s and 100 ms the longest). A step that has not by 10 s is ended:
     * the counted seconds stand for the slept time, the screen and the
     * radio's poll are released, D+ is back on. */
    if (since >= RC_MAX_MS) {
        rtc_wake_allow();
        rtc_wake_abandon();
        if (!rc_done(SLEEP_RC_GAUGE)) {
            uint32_t s;
            bool     pcf;
            if (rtc_wake_slept(&s, &pcf)) {
                rc_pcf_s = s;
                ds_slept_total += s;
                battery_sleep_account(s * 1000u);
                rtc_wake_gauge_taken();
            }
        }
        if (!rc_done(SLEEP_RC_DISPLAY)) display_wake_release();
        ch582_battery_poll_release();
        if (ur_state == UR_OFF) {
            chSysLock();
            usb_lld_connect_bus(&USB_DRIVER);
            chSysUnlock();
        }
        ur_state  = UR_DONE;
        rc_timed_out |= rc_open;
        rc_open   = 0;
        rc_active = false;
        ds_flags |= SLEEP_V_BOUND;
        return;
    }

    /* 1. The waking input. */
    if (!rc_done(SLEEP_RC_INPUT)) {
        bool in = ds_reason == SLEEP_WAKE_KEY    ? health_key_presses() != ds_kp_before
                : ds_reason == SLEEP_WAKE_KNOB   ? last_encoder_activity_time() != rc_enc_t0
                                                 : true;
        if (in || since >= RC_INPUT_MAX_MS) {
            if (!in) rc_timed_out |= SLEEP_RC_INPUT;
            rc_close(SLEEP_RC_INPUT);
            rtc_wake_allow();
        }
    }
    /* 2 and 3. The clock's coarse read, then the gauge: in a pass with no
     * PCF transaction (rtc.c counts them from the last pass's end; the
     * display's deferral flag is cleared before this runs, so it cannot say:
     * codex, deep-sleep build review 2). rtc.c holds the acquisition until
     * the gauge has its seconds, so the pass after the coarse read is free. */
    if (rc_done(SLEEP_RC_INPUT) && !rc_done(SLEEP_RC_GAUGE) && !rtc_pass_used_bus()) {
        uint32_t s;
        bool     pcf;
        if (rtc_wake_slept(&s, &pcf)) {
            rc_pcf_s = s;
            if (pcf) ds_flags |= SLEEP_V_PCF;
            ds_slept_total += s;
            battery_sleep_account(s * 1000u);
            rtc_wake_gauge_taken();
            rc_gauge_at = timer_read32();
            rc_close(SLEEP_RC_GAUGE);
            display_wake_corrected();   /* the clock and the row to repaint, now */
        }
    }
    /* 4. The phase. */
    if (!rc_done(SLEEP_RC_CLOCK) && rc_done(SLEEP_RC_GAUGE) && !rtc_wake_busy()) {
        if (rtc_wake_acq_result() == 2u) ds_flags |= SLEEP_V_ACQ_DONE;   /* ACQ_DONE */
        rc_close(SLEEP_RC_CLOCK);
    }
    /* 5. The radio. */
    if (rc_done(SLEEP_RC_INPUT) && !rc_poll_sent && (ch582_tx_idle() || since >= RC_POLL_AFTER_MS)) {
        ch582_battery_poll_release();
        rc_poll_sent = true;
        rc_poll_at   = timer_read32();
    }
    if (rc_poll_sent && !rc_done(SLEEP_RC_RADIO)) {
        if (ch582_battery_reports() != rc_reports0) {
            rc_close(SLEEP_RC_RADIO);
        } else if (timer_elapsed32(rc_poll_at) >= RC_POLL_WAIT_MS) {
            rc_timed_out |= SLEEP_RC_RADIO;
            rc_close(SLEEP_RC_RADIO);
        }
    }
    /* 7. The supply: 1.5 s of fresh samples since the wake (the battery
     * task's conversions that read), or 3 s whatever they were, flagged
     * (codex, deep-sleep build review 7). */
    if (!rc_done(SLEEP_RC_SUPPLY)) {
        if ((uint16_t)(battery_supply_samples() - rc_samples0) >= RC_SUPPLY_SAMPLES) {
            rc_close(SLEEP_RC_SUPPLY);
        } else if (since >= RC_SUPPLY_MAX_MS) {
            rc_timed_out |= SLEEP_RC_SUPPLY;
            rc_close(SLEEP_RC_SUPPLY);
        }
    }
    /* 8. USB: latched, a frame since this wake; else one re-attach. D+ off
     * and on are read-modify-writes of CFG, which the USB ISR also writes:
     * under the lock (codex). Never restart_usb_driver(): its 50 ms wait
     * loses a tap. */
    if ((uint16_t)(SN_USB->FRMNO & 0x7FFu) != ur_fn0) ds_flags |= SLEEP_V_USB_SOF;
    if (ur_state == UR_WAIT) {
        if (ds_flags & SLEEP_V_USB_SOF) {
            ur_state = UR_DONE;
        } else if (since >= UR_WAIT_MS) {
            chSysLock();
            usb_lld_disconnect_bus(&USB_DRIVER);
            chSysUnlock();
            ur_at    = timer_read32();
            ur_state = UR_OFF;
            ds_flags |= SLEEP_V_REATTACH;
        }
    } else if (ur_state == UR_OFF && timer_elapsed32(ur_at) >= UR_OFF_MS) {
        chSysLock();
        usb_lld_connect_bus(&USB_DRIVER);
        chSysUnlock();
        ur_state = UR_DONE;
    }
    if (ur_state == UR_DONE) rc_close(SLEEP_RC_USB);
    /* 9. The display. */
    if (!rc_done(SLEEP_RC_DISPLAY) && rc_done(SLEEP_RC_GAUGE)) {
        bool painted = display_wake_repainted();
        if (painted || timer_elapsed32(rc_gauge_at) >= RC_HOLD_MAX_MS) {
            if (!painted) rc_timed_out |= SLEEP_RC_DISPLAY;
            display_wake_release();
            rc_close(SLEEP_RC_DISPLAY);
            if (rc_pcf_s >= RC_LONG_SLEEP_S) power_wake_notice();   /* "6h use left", if it matters */
        }
    }
    if (rc_open == 0) rc_active = false;
}

bool sleep_recovering(void) {
    return rc_active;
}

void sleep_page1_fill(uint8_t *out) {
    memset(out, 0, SLEEP_PAGE1_SIZE);
    uint8_t *p = out;
#define PUT16(x) do { uint16_t v_ = (uint16_t)(x); *p++ = (uint8_t)v_; *p++ = (uint8_t)(v_ >> 8); } while (0)
#define PUT32(x) do { uint32_t v_ = (uint32_t)(x); for (uint8_t k_ = 0; k_ < 4; k_++) *p++ = (uint8_t)(v_ >> (8 * k_)); } while (0)
    *p++ = 0x81;   /* page 1, so an older firmware's page-0 reply (1) is never read as this */
    *p++ = ds_reason;
    *p++ = ds_flags;
    *p++ = rc_open;
    PUT32(ds_sleeps);
    PUT32(ds_slept_total);
    PUT32(rc_pcf_s);
    PUT32(ds_secs);
    PUT16((uint16_t)rtc_wake_acq_step_ms());
    PUT16(ds_pll_loops);
    PUT16(ds_strays);
    PUT16(ds_wanted_blocked);
    *p++ = rc_timed_out;
#undef PUT16
#undef PUT32
}

/* ---- The dev bench's arm ---------------------------------------------------- */
enum { DS_IDLE = 0, DS_ARMED = 1, DS_DONE = 2 };
#ifdef AK820_DEV_HOOKS
#    define DS_GIVE_UP_MS 60000u   /* armed and never safe this long after the delay: give up */
static uint8_t  ds_state = DS_IDLE;
static uint16_t ds_blocked, ds_kp_gain;
static uint32_t ds_armed_at, ds_woke_at;
static uint8_t  ds_delay_s;
static bool     ds_kp_pending;
static uint32_t ds_arm_cap_s, ds_arm_preset_s;   /* this arm's, never the ladder's */
static bool     ds_arm_no_feed;

void sleep_deep_arm(uint8_t delay_s, uint32_t cap_s, bool no_feed, uint32_t preset_s) {
    /* The counted seconds past the preset at most the backstop's, so the
     * slept time's ms (x 1000, u32) and the cap (preset + span) never wrap
     * (codex, deep-sleep build review 4). 0: the backstop. */
    if (preset_s > UINT32_MAX - DS_BACKSTOP_S) preset_s = UINT32_MAX - DS_BACKSTOP_S;
    uint32_t span = cap_s > preset_s ? cap_s - preset_s : 1u;
    if (cap_s == 0 || span > DS_BACKSTOP_S) span = DS_BACKSTOP_S;
    ds_state        = DS_ARMED;
    ds_reason       = SLEEP_WAKE_NONE;
    ds_blocked      = 0;
    ds_armed_at     = timer_read32();
    ds_delay_s      = delay_s;
    ds_arm_cap_s    = preset_s + span;
    ds_arm_no_feed  = no_feed;
    ds_arm_preset_s = preset_s;
    ds_kp_gain      = 0;
    ds_kp_pending = false;
    ls_mask       = 0;   /* light sleep off: the deep run starts from the plain board */
}

void sleep_deep_cancel(void) {
    if (ds_state == DS_ARMED) ds_state = DS_IDLE;
}

static void arm_task(void) {
    if (ds_kp_pending && timer_elapsed32(ds_woke_at) >= 1000u) {
        ds_kp_gain    = (uint16_t)(health_key_presses() - ds_kp_before);
        ds_kp_pending = false;
    }
    if (ds_state != DS_ARMED || rc_active) return;
    uint32_t waited = timer_elapsed32(ds_armed_at);
    if (waited < (uint32_t)ds_delay_s * 1000u) return;
    ds_blocked = blockers() & (uint16_t)~(DSB_LIT | DSB_RGB);   /* the bench forces the stage */
    if (ds_blocked == 0 && rtc_sleep_prepare()) {
        battery_sleep_enter();
        if (deep_sleep(ds_arm_cap_s, ds_arm_no_feed, ds_arm_preset_s)) {
            ds_state      = DS_DONE;
            ds_woke_at    = timer_read32();
            ds_kp_pending = true;
        }
    } else if (waited >= (uint32_t)ds_delay_s * 1000u + DS_GIVE_UP_MS) {
        ds_reason = SLEEP_WAKE_BLOCKED;
        ds_state  = DS_DONE;
    }
}

void sleep_deep_record(uint8_t *o) {
    memset(o, 0, SLEEP_RECORD_SIZE);
    uint8_t *p = o;
    *p++ = 2;   /* the record's version */
    *p++ = ds_state;
    *p++ = ds_reason;
    *p++ = (uint8_t)ds_blocked; *p++ = (uint8_t)(ds_blocked >> 8);
    *p++ = ds_flags;
    for (uint8_t k = 0; k < 4; k++) *p++ = (uint8_t)(ds_secs >> (8 * k));
    *p++ = (uint8_t)ds_strays; *p++ = (uint8_t)(ds_strays >> 8);
    *p++ = (uint8_t)ds_p0_ris; *p++ = (uint8_t)(ds_p0_ris >> 8);
    *p++ = (uint8_t)ds_p2_ris; *p++ = (uint8_t)(ds_p2_ris >> 8);
    for (uint8_t k = 0; k < 4; k++) *p++ = (uint8_t)(ds_p1_ris >> (8 * k));
    *p++ = (uint8_t)ds_pll_loops; *p++ = (uint8_t)(ds_pll_loops >> 8);
    *p++ = (uint8_t)ds_kp_gain;   *p++ = (uint8_t)(ds_kp_gain >> 8);
    *p++ = (uint8_t)ds_sleeps;    *p++ = (uint8_t)(ds_sleeps >> 8);
    *p++ = rc_timed_out;
}
#else
void sleep_deep_arm(uint8_t delay_s, uint32_t cap_s, bool no_feed, uint32_t preset_s) {
    (void)delay_s; (void)cap_s; (void)no_feed; (void)preset_s;
}
void sleep_deep_cancel(void) {}
void sleep_deep_record(uint8_t *o) { memset(o, 0, SLEEP_RECORD_SIZE); }
#endif

/* ---- Each pass ---------------------------------------------------------------- */
void sleep_task(void) {
    usb_watch();
    recovery_task();
#ifdef AK820_DEV_HOOKS
    arm_task();
    light_task();
#endif
    /* The ladder's sleep, at the first pass where it can (the dev arm above
     * may have slept already this pass). The PCF's "before" read goes by the
     * bus arbiter: not admitted, the sleep waits a pass. */
    if (!rc_active && power_sleep_wanted()) {
        uint16_t b = blockers();
        ds_wanted_blocked = b;
        if (b == 0 && rtc_sleep_prepare()) {
            battery_sleep_enter();
            deep_sleep(DS_BACKSTOP_S, false, 0);
        }
    }
    rtc_pass_mark();                  /* the PCF transactions of the next pass count from here */
    pass_t0 = chVTGetSystemTimeX();   /* the next pass starts here */
}
