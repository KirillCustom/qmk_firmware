// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* Sleep experiments (plans/SLEEP-PLAN.md): light sleep's sizing bits and the
 * deep-sleep experiment, dev builds only, both off unless HC_DEVHOOK op 14
 * turns them on. Register facts are from the SN32F299 manual (V1.8); page
 * numbers in the plan and docs/sn32f299.md. */
#include "quantum.h"
#include "sleep.h"

#ifdef AK820_DEV_HOOKS
#    include <hal.h>
#    include "usb_main.h"   /* restart_usb_driver, USB_DRIVER */
#    include "power.h"
#    include "battery.h"   /* battery_usb_configured */
#    include "indicators.h"
#    include "watchdog_record.h"
#    include "kb_eeconfig.h"
#    include "rtc/rtc.h"
#    include "graphics/display.h"
#    include "bluetooth/ch582f_ajazz.h"
#    include "watchdog.h"
#    include "health.h"
#    include "drivers/led/sn32f2xx.h"

/* ---- Light sleep ---------------------------------------------------------- */

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

/* PMU Sleep and Deep-sleep turn the USB transceiver off (manual p. 245,
 * USB_CFG.PHY_EN, bit 30); whether it comes back by itself is not written:
 * set it back. Returns whether it was off. */
#    define USB_CFG_PHY_EN (1u << 30)
static bool usb_phy_onI(void) {
    if (SN_USB->CFG & USB_CFG_PHY_EN) return false;
    SN_USB->CFG |= USB_CFG_PHY_EN;
    return true;
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
    /* Bit 4 only inside its window, and never with USB up: the transceiver
     * turns itself off in PMU Sleep (manual p. 245). */
    if ((m & SLEEP_LS_PMU) && ((int32_t)(timer_read32() - ls_pmu_until) >= 0 || battery_usb_configured()))
        m &= (uint8_t)~SLEEP_LS_PMU;
    /* Bit 3 only with the screen dark and no indicator lit. */
    if ((m & SLEEP_LS_TICK) && (display_backlight_effective() > 0 || indicators_any_lit()))
        m &= (uint8_t)~SLEEP_LS_TICK;
    if ((m ^ ls_applied) & SLEEP_LS_TICK) pwm_tick_run(!(m & SLEEP_LS_TICK));
    if ((ls_applied & SLEEP_LS_PMU) && !(m & SLEEP_LS_PMU)) {
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

/* ---- Deep sleep ----------------------------------------------------------- */

enum { DS_IDLE = 0, DS_ARMED = 1, DS_DONE = 2 };
enum { DS_NONE = 0, DS_KEY = 1, DS_CAP = 2, DS_STRAYS = 3, DS_BLOCKED = 4 };
enum { DSB_KEY = 1, DSB_USB = 2, DSB_TX = 4, DSB_DISPLAY = 8, DSB_FLASH = 16, DSB_RTC = 32, DSB_PINS = 64, DSB_WDT = 128 };

/* Register bits (the SN32F299 manual; sn32_uart.h and battery.c agree). */
#    define UART_LS_TEMT   (1u << 6)    /* transmitter empty, not just THR (bit 5) */
#    define UART_CTRL_RXEN (1u << 6)
#    define ADC_ADM_EOC    (1u << 6)
#    define ADC_ADM_ADS    (1u << 7)
#    define ADC_ADM_ADENB  (1u << 12)
#    define UART_LS_RDR    (1u << 0)    /* receive data ready */
#    define SPI_STAT_BUSY  (1u << 4)
#    define SPI0_DMACTRL_DMAEN (1u << 0)
#    define CLK_WAIT_LOOPS 100000u      /* a clock switch reports in a few cycles */

#    define DS_GIVE_UP_MS 60000u   /* armed and never safe this long after the delay: give up */
#    define DS_STRAYS_MAX 1000u    /* a wake that is neither a key nor a second, this often: leave */
#    define DS_PLL_LOOPS  200000u  /* ~ms at 12 MHz; the PLL locks in ~100 us (p. 58) */

static uint8_t  ds_state = DS_IDLE, ds_reason = DS_NONE, ds_blocked = 0;
static uint32_t ds_armed_at;
static uint8_t  ds_delay_s;
static uint16_t ds_cap_s;
static bool     ds_no_feed;
static uint16_t ds_secs, ds_strays, ds_pll_loops, ds_kp_gain;
static uint32_t ds_p0_ris, ds_p2_ris;
static bool     ds_pll_ceiling;
static bool     ds_slept;
static bool     ds_phy_was_off;   /* the transceiver was off after the wake */
static bool     ds_usb_restarted; /* the 20 s fallback ran */
static bool     ds_usb_watch;     /* waiting to see USB come back */
static uint16_t ds_kp_before;
static uint32_t ds_woke_at;
static bool     ds_kp_pending;


static const pin_t ds_rows[] = MATRIX_ROW_PINS;
static const pin_t ds_cols[] = MATRIX_COL_PINS;

void sleep_deep_arm(uint8_t delay_s, uint16_t cap_s, bool no_feed) {
    ds_state    = DS_ARMED;
    ds_reason   = DS_NONE;
    ds_blocked  = 0;
    ds_armed_at = timer_read32();
    ds_delay_s  = delay_s;
    ds_cap_s    = cap_s ? cap_s : 120;
    ds_no_feed  = no_feed;
    ds_secs = ds_strays = ds_pll_loops = ds_kp_gain = 0;
    ds_p0_ris = ds_p2_ris = 0;
    ds_pll_ceiling = false;
    ds_slept       = false;
    ds_phy_was_off = ds_usb_restarted = ds_usb_watch = false;
    ds_kp_pending  = false;
    ls_mask        = 0;   /* light sleep off: the deep run starts from the plain board */
}

void sleep_deep_cancel(void) {
    if (ds_state == DS_ARMED) ds_state = DS_IDLE;
}

void sleep_deep_record(uint8_t *o) {
    memset(o, 0, 24);
    o[0] = ds_state;
    o[1] = ds_reason;
    o[2] = ds_blocked;
    o[3] = ds_slept ? 1 : 0;
    o[4] = (uint8_t)ds_secs;      o[5] = (uint8_t)(ds_secs >> 8);
    o[6] = (uint8_t)ds_strays;    o[7] = (uint8_t)(ds_strays >> 8);
    for (uint8_t k = 0; k < 4; k++) {
        o[8 + k]  = (uint8_t)(ds_p0_ris >> (8 * k));
        o[12 + k] = (uint8_t)(ds_p2_ris >> (8 * k));
    }
    o[16] = (uint8_t)ds_pll_loops; o[17] = (uint8_t)(ds_pll_loops >> 8);
    o[18] = (uint8_t)ds_kp_gain;   o[19] = (uint8_t)(ds_kp_gain >> 8);
    o[20] = ds_pll_ceiling ? 1 : 0;
    o[21] = ds_phy_was_off ? 1 : 0;
    o[22] = ds_usb_restarted ? 1 : 0;
}

/* The wake pins: the matrix's columns, which must all sit on P0 or P2 (the
 * ports whose interrupts are wired here); false if one does not. */
static bool col_masks(uint32_t *p0, uint32_t *p2) {
    *p0 = *p2 = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_cols); i++) {
        ioportid_t port = PAL_PORT(ds_cols[i]);
        uint32_t   bit  = 1u << PAL_PAD(ds_cols[i]);
        if (port == PAL_PORT(A0)) *p0 |= bit;
        else if (port == PAL_PORT(C0)) *p2 |= bit;
        else return false;
    }
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

static uint8_t blockers(void) {
    uint8_t b = 0;
    for (uint8_t r = 0; r < MATRIX_ROWS; r++)
        if (matrix_get_row(r)) b |= DSB_KEY;
    if (battery_usb_configured()) b |= DSB_USB;
    if (!tx_quietI_locked()) b |= DSB_TX;
    if (!display_bus_idle() || (SN_SPI0->STAT & SPI_STAT_BUSY) || (SN_SPI0->DMACTRL & SPI0_DMACTRL_DMAEN) ||
        (SN_SPI1->STAT & SPI_STAT_BUSY))
        b |= DSB_DISPLAY;
    if (kb_eeconfig_write_pending() || EFLD1.state != FLASH_READY) b |= DSB_FLASH;
    if (!rtc_sleep_ok()) b |= DSB_RTC;
    uint32_t p0, p2;
    if (!col_masks(&p0, &p2)) b |= DSB_PINS;
    /* The way back if nothing wakes it: refuse without a counting watchdog
     * (degraded after three resets, or never armed; codex). */
    if (!watchdog_running() || !(SN_WDT->CFG & 1u)) b |= DSB_WDT;   /* CFG.WDTEN */
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

static bool deep_sleep(void) {
    WDT_SCOPE(WDT_SITE_DEEP_SLEEP);
    static volatile uint32_t sacrificial;
    sacrificial++;   /* the breadcrumb above must reach RAM before the CPU stops */

    uint32_t p0m, p2m;
    col_masks(&p0m, &p2m);
    ds_kp_before = health_key_presses();

    /* Everything from here to the unlock is lock-free (I-class calls and
     * register writes): a lock call would unlock on its way out, and a wake
     * interrupt taken has no handler (codex, must-fix 1). */
    chSysLock();
    /* The admission again, under the lock: nothing can be queued for the
     * radio, or USB come up, between the check and the teardown (codex). */
    if (battery_usb_configured() || !tx_quietI()) {
        chSysUnlock();
        return false;
    }
    /* Outputs off: the LED matrix, the 20 kHz tick with the pins it drives. */
    sn32f2xx_sleep_enterI();
    pwm_tick_stopI();
    /* UART2's receiver off across the clock change and the sleep (TX stays
     * enabled, so its line idles high); the ADC off (~100 uA, p. 96). */
    uint32_t uart_ctrl = SN_UART2->CTRL;
    SN_UART2->CTRL     = uart_ctrl & ~UART_CTRL_RXEN;
    uint32_t adm       = SN_ADC->ADM & ~(ADC_ADM_EOC | ADC_ADM_ADS);
    SN_ADC->ADM        = adm & ~ADC_ADM_ADENB;
    /* Every key row low, every column an input with its pull-up, so a press
     * pulls its column low. */
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_rows); i++) {
        gpio_set_pin_output_push_pull(ds_rows[i]);
        gpio_write_pin_low(ds_rows[i]);
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(ds_cols); i++) gpio_set_pin_input_high(ds_cols[i]);
    /* Only the wake sources in the NVIC; SysTick's alarm off. */
    uint32_t iser = NVIC->ISER[0];
    NVIC->ICER[0] = 0xFFFFFFFFu;
    NVIC->ICPR[0] = 0xFFFFFFFFu;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;
    /* The column pins: edge sensed, one edge, falling (IS 0, IBS 0, IEV 1;
     * pp. 81-84), cleared after the rows settle, then enabled. */
    SN_GPIO0->IS &= ~p0m;  SN_GPIO0->IBS &= ~p0m;  SN_GPIO0->IEV |= p0m;
    SN_GPIO2->IS &= ~p2m;  SN_GPIO2->IBS &= ~p2m;  SN_GPIO2->IEV |= p2m;
    for (volatile uint32_t d = 0; d < 1200u; d++) {}   /* ~100 us at 48 MHz */
    SN_GPIO0->IC = p0m;    SN_GPIO2->IC = p2m;
    SN_GPIO0->IE |= p0m;   SN_GPIO2->IE |= p2m;
    uint32_t wake_irqs = (1u << P0_IRQn) | (1u << P2_IRQn) | (1u << RTC_IRQn);
    NVIC->ICPR[0] = wake_irqs;
    NVIC->ISER[0] = wake_irqs;

    uint16_t secs = 0, strays = 0;
    if (take_second()) secs++;   /* one that came since the lock: its handler cannot run now */
    /* A column already low is a key down, or one pressed before the clear
     * above erased its edge (codex, must-fix 2): no sleep, a key wake. */
    uint32_t low0 = ~SN_GPIO0->DATA & p0m, low2 = ~SN_GPIO2->DATA & p2m;
    bool     slept = false;
    uint32_t clkcfg = SN_SYS0->CLKCFG & 0x7u, pllctrl = SN_SYS0->PLLCTRL;
    uint32_t lpctrl = SN_FLASH->LPCTRL & 0xFFFFu;
    if (low0 || low2) {
        ds_p0_ris = low0;
        ds_p2_ris = low2;
        ds_reason = DS_KEY;
    } else if (!(SN_SYS0->CSST & 0x1u)) {   /* IHRCRDY: the clock to drop to */
        ds_reason = DS_STRAYS;
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
            SN_PMU->CTRL = 2;   /* Deep-sleep (p. 75) */
            __DSB();
            __WFI();
            SN_PMU->CTRL = 0;
            uint32_t r0 = SN_GPIO0->RIS & p0m, r2 = SN_GPIO2->RIS & p2m;
            bool second = take_second();
            if (second) {
                if (!ds_no_feed) SN_WDT->FEED = (mskWDT_WDKEY | mskWDT_FV);   /* no lock calls in here */
                secs++;
                if (!(r0 || r2) && secs >= ds_cap_s) {
                    ds_reason = DS_CAP;
                    break;
                }
            }
            if (r0 || r2) {
                ds_p0_ris = r0;
                ds_p2_ris = r2;
                ds_reason = DS_KEY;
                break;
            }
            if (second) continue;   /* a second is not a stray */
            if (++strays >= DS_STRAYS_MAX) {
                ds_reason = DS_STRAYS;
                break;
            }
        }
    }

    if (slept) {
        /* Awake: the flash's speed first, then the PLL, and only then select it. */
        SN_FLASH->LPCTRL = 0x5AFA0000u | lpctrl;
        SN_SYS0->PLLCTRL = pllctrl;
        uint32_t loops = 0;
        while ((SN_SYS0->CSST & 0x40u) != 0x40u) {   /* PLLRDY */
            if (++loops >= DS_PLL_LOOPS) break;
        }
        ds_pll_loops   = loops > 0xFFFFu ? 0xFFFFu : (uint16_t)loops;
        ds_pll_ceiling = loops >= DS_PLL_LOOPS;
        if (ds_pll_ceiling) NVIC_SystemReset();   /* the UART, USB and timers all assume 48 MHz */
        SN_SYS0->CLKCFG = clkcfg;
        for (uint32_t k = 0; (SN_SYS0->CLKCFG & 0x70u) != (clkcfg << 4); k++)
            if (k >= CLK_WAIT_LOOPS) NVIC_SystemReset();
    }
    /* The wake pins quiet again; the temporary vectors off BEFORE the old
     * enables go back (writing ISER cannot disable one; codex). */
    SN_GPIO0->IE &= ~p0m;  SN_GPIO2->IE &= ~p2m;
    SN_GPIO0->IC = p0m;    SN_GPIO2->IC = p2m;
    NVIC->ICER[0] = wake_irqs;
    NVIC->ICPR[0] = (1u << P0_IRQn) | (1u << P2_IRQn);
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
    /* The kernel's alarm: left off and pended, so its handler serves the
     * timers now and re-arms the next one. */
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    sn32f2xx_sleep_exitI();
    pwm_tick_startI();
    bool phy_off = usb_phy_onI();
    rtc_account_sleepI(secs);   /* both software clocks, before any second is served */
    chSysUnlock();

    ch582_rx_resync();
    ls_applied &= (uint8_t)~SLEEP_LS_TICK;   /* the tick runs again; light_task stops it if wanted */
    ds_secs        = secs;
    ds_strays      = strays;
    ds_slept       = slept;
    ds_phy_was_off = phy_off;
    /* No USB restart here: its 50 ms wait would hold the main loop while the
     * waking key is down (codex). The transceiver is back on; if the host has
     * not configured it 20 s after the wake, deep_task() restarts it once. */
    ds_usb_watch  = slept;
    ds_woke_at    = timer_read32();
    ds_kp_pending = true;
    ds_state      = DS_DONE;
    return true;
}

static void deep_task(void) {
    if (ds_kp_pending && timer_elapsed32(ds_woke_at) >= 1000u) {
        ds_kp_gain    = (uint16_t)(health_key_presses() - ds_kp_before);
        ds_kp_pending = false;
    }
    if (ds_usb_watch && timer_elapsed32(ds_woke_at) >= 20000u) {
        ds_usb_watch = false;
        if (!battery_usb_configured()) {
            ds_usb_restarted = true;
            restart_usb_driver(&USB_DRIVER);   /* 50 ms, long after the waking key */
        }
    }
    if (ds_state != DS_ARMED) return;
    uint32_t waited = timer_elapsed32(ds_armed_at);
    if (waited < (uint32_t)ds_delay_s * 1000u) return;
    ds_blocked = blockers();
    if (ds_blocked == 0 && deep_sleep()) {
        return;
    } else if (waited >= (uint32_t)ds_delay_s * 1000u + DS_GIVE_UP_MS) {
        ds_reason = DS_BLOCKED;
        ds_state  = DS_DONE;
    }
}

void sleep_task(void) {
    deep_task();
    light_task();
}

#else
void    sleep_light_set(uint8_t mask, uint8_t yield_ms, uint8_t pmu_s, uint8_t ttl_s) { (void)mask; (void)yield_ms; (void)pmu_s; (void)ttl_s; }
uint8_t sleep_light_yield(void) { return 0; }
void    sleep_deep_cancel(void) {}
uint8_t sleep_light_mask(void) { return 0; }
uint8_t sleep_light_applied(void) { return 0; }
void    sleep_deep_arm(uint8_t delay_s, uint16_t cap_s, bool no_feed) { (void)delay_s; (void)cap_s; (void)no_feed; }
void    sleep_deep_record(uint8_t *out24) { memset(out24, 0, 24); }
void    sleep_task(void) {}
#endif
