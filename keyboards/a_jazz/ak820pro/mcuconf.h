// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include_next <mcuconf.h>

#undef SN32_SPI_USE_SPI0
#define SN32_SPI_USE_SPI0 TRUE
#define SN32_SPI0_FLASH_DMA   /* flash->LCD DMA driver extension (hal_spi_v2_lld) */

/* SPI1 (external flash) under the ChibiOS SPI driver, like SPI0. All flash
 * I/O goes through spiSend/spiExchange(&SPID1) (graphics/lcd_bus.c); only the
 * DMA blit's command phase polls SPI1 directly, while the flash->LCD DMA
 * extension has SPID1's vector disabled. */
#undef SN32_SPI_USE_SPI1
#define SN32_SPI_USE_SPI1 TRUE

#undef SN32_SERIAL_USE_UART2
#define SN32_SERIAL_USE_UART2 TRUE

// RGB matrix hardware PWM spreads the 15 columns over three CT16 timers
// (CT16B0/B1/B2) because CT16B1 alone has only 12 PWM channels. See
// docs/HARDWARE_PWM.md for the column->(timer,channel) map and PFPA constants.
// (No SN32_PWM_NO_RESET: hardware PWM wants the period match to auto-reset the
// counter; only the old software-PWM path re-armed the counter manually.)
// The ChibiOS OS-tick free-running counter defaults to CT16B0 -- but hardware PWM
// needs CT16B0 (column C14 can ONLY route to CT16B0.3). Move the tick counter to
// the otherwise-unused CT16B5 so CT16B0 is free for PWM. (The tick interrupt runs
// on the ARM Cortex SysTick regardless; only the free-running counter uses a CT16.)
#undef SN32_ST_USE_TIMER
#define SN32_ST_USE_TIMER SN32_TIM_CT16B5

#undef SN32_PWM_USE_CT16B0
#define SN32_PWM_USE_CT16B0 TRUE
#undef SN32_PWM_USE_CT16B1
#define SN32_PWM_USE_CT16B1 TRUE
#undef SN32_PWM_USE_CT16B2
#define SN32_PWM_USE_CT16B2 TRUE

/* CT16B3 (GPTD4) drives the backlight/indicator PWM tick. B0/B1/B2 are RGB
 * hardware PWM and B5 is the OS-tick counter; B3 and B4 are the only free
 * timers, and this needs just a counter and an interrupt. */
#undef SN32_GPT_USE_CT16B3
#define SN32_GPT_USE_CT16B3 TRUE

/* Hardware watchdog on the SN32 WDT block (watchdog.c). */
#undef SN32_WDG_USE_WDT
#define SN32_WDG_USE_WDT TRUE

/* Outrank the RGB row-scan ISR (SN32_PWM_CT16B*_IRQ_PRIORITY = 2). The row ISR
 * is long -- it reloads 15 PWM channels -- and at priority 3 our 50us PWM tick
 * was being blocked by it: measured 15385 Hz against 20000 configured, i.e. 23%
 * of ticks lost. Lost ticks make the pulse train irregular, which reads as
 * backlight flicker regardless of the nominal switching rate.
 *
 * Safe to put above it: this handler is a counter, a compare and a few GPIO
 * writes, on the order of a microsecond, so it cannot meaningfully delay
 * anything else -- including the CH582F UART, whose byte time is ~87us. */
/* Interrupt priority ordering (lower number = higher priority). Getting this
 * wrong is what caused both the Bluetooth throughput collapse and the backlight
 * flicker, so the reasoning is spelled out:
 *
 *   0  UART2 (CH582F)  -- byte-loss critical (1 until 10-19; see below). A missed ACK stalls a TX frame for
 *                         CH582_TX_ACK_TIMEOUT_MS; at 3.7 retries per frame that
 *                         is ~7 characters/second and you can out-type the link.
 *   2  GPT CT16B3      -- the backlight/indicator PWM tick. Tiny (a counter, a
 *                         compare, a few GPIO writes) but must be REGULAR:
 *                         dropped ticks make the pulse train uneven, which reads
 *                         as flicker. At the stock priority 3 it sat below the
 *                         row scan and lost 23% of its ticks (measured 15385 Hz
 *                         against 20000 configured).
 *   3  RGB row scan    -- long (reloads 15 PWM channels) but latency-tolerant;
 *                         a few microseconds of delay is invisible on an LED.
 *
 * The defaults had this inverted: UART 3, PWM 2, GPT 3. */
#undef SN32_SERIAL_UART2_PRIORITY
#define SN32_SERIAL_UART2_PRIORITY 0
/* 0, above USB (1), since 10-19: UART2's receiver holds about one byte, so it
 * must be served within ~87 us, and the USB handler (up to ~0.085 ms) shared
 * its priority (plans/UART-LATENCY-PLAN.md, fix 2). This port allows OS calls
 * at 0 (ARMv6-M: PRIMASK locking, the context switch through the NMI). The
 * system timer is 0 too, stated below: the two cannot preempt each other.
 * Priority 0 still cannot preempt a PRIMASK section. */
#undef SN32_ST_IRQ_PRIORITY
#define SN32_ST_IRQ_PRIORITY 0      /* was the default 8, which the 2-bit field truncated to 0 */

#undef SN32_GPT_CT16B3_IRQ_PRIORITY
#define SN32_GPT_CT16B3_IRQ_PRIORITY 2

#undef SN32_PWM_CT16B0_IRQ_PRIORITY
#define SN32_PWM_CT16B0_IRQ_PRIORITY 3
#undef SN32_PWM_CT16B1_IRQ_PRIORITY
#define SN32_PWM_CT16B1_IRQ_PRIORITY 3
#undef SN32_PWM_CT16B2_IRQ_PRIORITY
#define SN32_PWM_CT16B2_IRQ_PRIORITY 3
