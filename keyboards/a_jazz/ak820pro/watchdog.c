// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware watchdog -- caps the blast radius of the "main loop dead until
 * power cycle" failure class (the eeconfig/blit hang, a wedged wait, any
 * future sibling) by turning it into a self-recovering reset with a
 * breadcrumb, instead of a keyboard that is bricked until someone reaches
 * the dip switch.
 *
 * TIMEOUT: ~12 s nominal, deliberately LONG. This board has recoverable
 * main-loop stalls measured at up to EIGHT SECONDS (2026-08-30 console
 * capture: an 8 s gap, then output resumed -- the board was alive the whole
 * time and healed on its own). A watchdog inside that envelope converts a
 * recoverable crawl into a reset loop, which is strictly worse than the
 * disease. 12 s clears the worst observed stall by 50% while still turning
 * an infinite hang into a blip. Also note a WDT reset costs the RTC divider
 * trim (not yet persisted), so false resets are far from free.
 *
 * CLOCKING (SN32F299 datasheet ch.11): the counting clock WDT_PCLK is the
 * ILRC (~32 kHz nominal, untrimmed RC) through the SYS1_APBCP1 WDTPRE
 * prescaler; the counter itself divides by a fixed 128 and reloads an 8-bit
 * constant, so timeout = TC * 128 / (ILRC / presc).
 *
 * ⚠️ WDTPRE ENCODING IS ASSUMED, NOT DATASHEET-CONFIRMED: the APBCP1 field
 * table did not survive PDF text extraction. pr=4 is assumed to mean /16 by
 * analogy with the family's other APB prescaler fields (0=/1,1=/2,2=/4,
 * 3=/8,4=/16). With ILRC 32 kHz that gives 64 ms/tick, TC=188 -> 12.0 s.
 * The hardware checklist measures the real stall-to-reset delay; if it
 * comes out ~2x off, this is the assumption that was wrong -- adjust
 * WDT_TC, not the structure.
 */
#include "quantum.h"
#include <hal.h>
#include "watchdog.h"
#include "watchdog_record.h"

#if HAL_USE_WDG

#define WDT_PRESC_SEL 4u  /* APBCP1.WDTPRE -- assumed /16, see header comment */
#define WDT_TC 188u       /* 188 * 128 / (32000/16) = 12.0 s nominal */

/* Reset-retained accounting and operation breadcrumbs: watchdog_record.c. */
extern uint32_t __ram0_end__; /* for the DFU magic in bootloader_jump() */
#define WDT_RAM_TOP ((uint32_t)&__ram0_end__)

#define WDT_DEGRADED_THRESHOLD 3u

static bool    wdg_running   = false;
static bool    fired_last    = false;
static bool    degraded_mode = false;
static uint8_t consec_resets = 0;

static const WDGConfig wdg_cfg = {
    .pr = WDT_PRESC_SEL,
    .tc = WDT_TC,
};

/* Raw RSTST at boot: bit0 SWRSTF, bit1 WDTRSTF, bit2 LVDRSTF (brownout),
 * bit3 EXTRSTF, bit4 PORRSTF. Captured before any clearing so the slider
 * power-switchover resets can be NAMED instead of guessed at. */
static uint8_t boot_rstst = 0;
uint8_t watchdog_boot_rstst(void) { return boot_rstst; }

void watchdog_boot_check(void) {
    boot_rstst = (uint8_t)(SN_SYS0->RSTST & 0x1F);
    fired_last = (boot_rstst & (1u << 1)) != 0;
    /* RSTST flags are STICKY and write-0-to-clear; clear ALL of them after
     * the capture so each boot's snapshot names THIS boot's cause instead of
     * a union of history. (The first readout proved the point: it showed
     * SW+LVD+POR at once -- the LVD being the residue of the slider's
     * BT->cable brownout resets.) */
    SN_SYS0->RSTST = 0;

    consec_resets = watchdog_record_boot(boot_rstst);

    /* A deterministic boot-time failure plus an armed watchdog is an endless
     * reset loop, possibly on battery. Past the threshold the WDT stays off:
     * the board still boots and types, it just loses the auto-recovery. */
    degraded_mode = (consec_resets >= WDT_DEGRADED_THRESHOLD);

#ifdef CONSOLE_ENABLE
    if (fired_last) {
        printf("[wdt] reset recovery: consecutive=%u%s\n", consec_resets,
               degraded_mode ? " -- DEGRADED, watchdog left off" : "");
    }
#endif
}

void watchdog_start(void) {
    if (degraded_mode || wdg_running) return;

    /* The datasheet requires a WDT peripheral reset after changing the
     * prescale value (SYS1_APBCP1 note), which the LLD skips. Set the
     * prescaler first, pulse the reset, then let wdgStart() re-assert the
     * same prescaler (its sys1SelectAPB1 is an OR -- a no-op by then). */
    sys1EnableWDT();
    SN_SYS1->APBCP1_b.WDTPRE = WDT_PRESC_SEL;
    SN_SYS1->PRST_b.WDTRST   = 1;
    SN_SYS1->PRST_b.WDTRST   = 0;

    wdgStart(&WDGD1, &wdg_cfg);
    wdg_running = true;
}

void watchdog_kick(void) {
    if (wdg_running) wdgReset(&WDGD1);
}

void watchdog_stop(void) {
    if (wdg_running) {
        wdgStop(&WDGD1);
        wdg_running = false;
    }
}

uint8_t watchdog_reset_count(void)   { return consec_resets; }
bool    watchdog_fired_last_boot(void) { return fired_last; }
bool    watchdog_degraded(void)      { return degraded_mode; }
bool    watchdog_running(void)       { return wdg_running && !degraded_mode; }

/* Stop the watchdog before entering the bootloader. The weak default in
 * platforms/chibios/bootloaders/sn32_dfu.c writes the RAM magic and calls
 * NVIC_SystemReset() with the WDT still armed; if the WDT survives a system
 * reset, it would then reset out of the bootloader's wait loop mid-flash.
 * Whether it survives is unverified on this part -- stopping it first is
 * correct either way. Magic/address must stay in sync with sn32_dfu.c. */
void bootloader_jump(void) {
    watchdog_stop();
    *(volatile uint32_t *)(WDT_RAM_TOP - 4u) = 0xDEADBEEFu;
    wait_us(1);
    NVIC_SystemReset();
}

#else /* !HAL_USE_WDG */

void    watchdog_boot_check(void) {}
void    watchdog_start(void) {}
void    watchdog_kick(void) {}
void    watchdog_stop(void) {}
uint8_t watchdog_boot_rstst(void) { return 0; }
uint8_t watchdog_reset_count(void) { return 0; }
bool    watchdog_fired_last_boot(void) { return false; }
bool    watchdog_degraded(void) { return false; }
bool    watchdog_running(void) { return false; }

#endif
