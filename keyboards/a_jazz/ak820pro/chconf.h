// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include_next <chconf.h>

/* chSysHalt() disables interrupts and spins; without this it never reaches
 * HardFault and the watchdog's record would name the interrupted operation as
 * if it had hung. The common chconf.h defines the hook unconditionally, so it
 * is replaced here. __builtin_return_address(0) expands inside chSysHalt():
 * its caller, the code that halted. See fault.c. */
#undef CH_CFG_SYSTEM_HALT_HOOK
#define CH_CFG_SYSTEM_HALT_HOOK(reason) do {                                \
    (void)(reason);                                                         \
    extern void watchdog_record_halt(uint32_t caller);                      \
    watchdog_record_halt((uint32_t)__builtin_return_address(0));            \
} while (0)

#ifdef AK820_DEV_HOOKS
/* Sleep experiments (sleep.c, plans/SLEEP-PLAN.md). The board's idle hooks
 * write PMU_CTRL = 4 (Sleep) on every idle entry, harmless while no WFI runs
 * (config.h keeps CORTEX_ENABLE_WFI_IDLE off); with sleep.c's WFI that would
 * enter Sleep, which turns the USB transceiver off (manual p. 245). So the
 * mode is chosen at each WFI instead: 4 only when sleep.c asks for PMU Sleep,
 * else 0. */
#    undef CH_CFG_IDLE_ENTER_HOOK
#    define CH_CFG_IDLE_ENTER_HOOK() do {                                   \
    } while (0)
#    undef CH_CFG_IDLE_LEAVE_HOOK
#    define CH_CFG_IDLE_LEAVE_HOOK() do {                                   \
        SN_PMU->CTRL = 0;                                                   \
    } while (0)
#    undef CH_CFG_IDLE_LOOP_HOOK
#    define CH_CFG_IDLE_LOOP_HOOK() do {                                    \
        extern volatile bool     ak820_wfi, ak820_pmu_sleep;                \
        extern volatile uint32_t ak820_wfi_count;                           \
        if (ak820_wfi) {                                                    \
            SN_PMU->CTRL = ak820_pmu_sleep ? 4u : 0u;                       \
            ak820_wfi_count++;                                              \
            __WFI();                                                        \
            SN_PMU->CTRL = 0;                                               \
        }                                                                   \
    } while (0)
#endif
