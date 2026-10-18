// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <stdint.h>

/* Stable wire IDs; append only. Main-loop context ONLY: ISR writers would
 * overwrite the operation they interrupted and require synchronization. */
enum watchdog_site {
    WDT_SITE_NONE = 0, WDT_SITE_MAIN_LOOP = 1, WDT_SITE_HOUSEKEEPING = 2,
    WDT_SITE_KEY_EVENT = 3, WDT_SITE_WIRELESS = 4, WDT_SITE_RAW_HID = 5,
    WDT_SITE_PARAM_REPEAT = 6, WDT_SITE_RTC_FAST = 7, WDT_SITE_SECOND_EDGE = 8,
    WDT_SITE_DISPLAY_PUMP = 9, WDT_SITE_DISPLAY_HK = 10, WDT_SITE_USER_HK = 11,
    WDT_SITE_LCD_WAIT = 12, WDT_SITE_LCD_TRANSFER = 13,
    WDT_SITE_FLASH_READ = 14, WDT_SITE_FLASH_ERASE = 15,
    WDT_SITE_FLASH_PROGRAM = 16, WDT_SITE_I2C_READ = 17, WDT_SITE_I2C_WRITE = 18,
    WDT_SITE_EFL_WRITE = 19, WDT_SITE_EFL_ERASE = 20, WDT_SITE_EFL_DRAIN = 21,
    WDT_SITE_TEST_STALL = 22,
    /* Terminal sites, written by fault.c instead of a scope: the CPU faulted,
     * took a vector nobody handles, or ChibiOS halted. Their parent is the
     * operation that was running. */
    WDT_SITE_HARD_FAULT = 23, WDT_SITE_UNHANDLED_EXCEPTION = 24, WDT_SITE_HALT = 25,
    WDT_SITE_TEST_FAULT = 26,
    WDT_SITE_DEEP_SLEEP = 27,   /* sleep.c's experiment: the reset happened asleep */
    /* 32 + enum loop_site in ak820pro.h; keep that enum append-only too. */
    WDT_SITE_LOOP_BASE = 32
};

/* Capture BEFORE instrumented operations start. Returns consecutive resets.
 * A WDT-only warm reset may trust RAM; POR/LVD invalidate it even if magic
 * happens to survive. Old firmware's magic is deliberately not accepted. */
uint8_t watchdog_record_boot(uint8_t rstst);
void watchdog_record_uptime(uint32_t pass_uptime_ms);
uint32_t watchdog_record_enter(uint8_t site);
void watchdog_record_leave(uint32_t *previous);
/* GCC cleanup restores the parent on EVERY return, including early returns.
 * No timers, allocation, I/O or interrupt masking on these paths. */
#define WDT_SCOPE(site) uint32_t wdt_saved_scope __attribute__((cleanup(watchdog_record_leave), unused)) = watchdog_record_enter(site)
#define WDT_CALL(site, call) do { WDT_SCOPE(site); call; } while (0)

/* HC_GET5, protocol >=6, 28 bytes, little endian:
 * u8 format, flags (bit0 valid), site, parent;
 * u32 last_pass_uptime_ms; u8 boot_rstst, consecutive_resets;
 * format 1 (an interrupted operation): 18 zero bytes.
 * format 2 (a terminal site: fault, unhandled vector, halt; protocol >=7):
 *   u32 pc, u8 exception, then 13 zero bytes. last_pass_uptime_ms is 0 --
 *   the retained word held the PC instead. pc 0xFFFFFFFF: the fault's stack
 *   frame was outside RAM or misaligned. pc 0: not recoverable here (an
 *   unhandled vector). exception: the IPSR of the interrupted context (0 =
 *   thread, 16+n = IRQ n); 0xFF = unknown.
 * Old decoders reject format 2 rather than misread a PC as an uptime.
 * Invalid records have zero site/parent/uptime. Boot facts remain available.
 * Frozen for this boot; health_reset() does not touch it. */
void watchdog_record_fill(uint8_t *out28);

/* Terminal records, from fault.c. Write nothing before boot capture (the
 * previous boot's evidence is not yet read, and the watchdog is not armed).
 * Only a small, bounded stack is used: fault.c switches to a private one. */
/* HardFault: frame_addr is the stacked exception frame the stub selected
 * from EXC_RETURN. Validated before any load (a load from a bad address
 * inside HardFault locks the core up). */
void watchdog_record_fault_frame(uintptr_t frame_addr);
/* Anything else terminal: site, the IPSR to report, and a PC if known. */
void watchdog_record_stop(uint8_t site, uint8_t exception, uint32_t pc);

#ifdef WDT_TEST_HOOKS
/* Instrumented builds only: boot copies the four raw retained words here
 * before rewriting them, so a record that decodes as invalid can still be
 * read (HC_PEEK, 0x20007F80..8F). Unused RAM at the top of the heap --
 * nothing allocates from the top on this build -- and clear of the DFU magic
 * at __ram0_end__ - 4. Not cleared by crt0; survives a watchdog reset. This
 * is how the lost final write in commit_terminal was found (2026-09-23).
 * Nothing in the fault path writes here: probes in the handler hid that bug
 * by adding a later write. */
#define WDT_BOOT_RAW ((volatile uint32_t *)0x20007F80u)
#endif

/* The consecutive-reset count carried into the NEXT boot clears once a boot
 * has run this long: the degraded-mode guard exists to stop a boot loop, and
 * without this, three crashes weeks apart would switch the watchdog off for
 * good (crash-hunt review, finding 3). This boot's reported count is a boot
 * fact and stays. */
#define WATCHDOG_COUNT_AGE_OUT_MS (10u * 60u * 1000u)
