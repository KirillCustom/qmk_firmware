// Copyright 2026 Fernando Birra
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#define HAL_USE_PAL TRUE
#define HAL_USE_SPI TRUE   /* unified experiment: dashboard CPU pushes via the driver */
#define HAL_USE_SERIAL TRUE
#define HAL_USE_RTC TRUE

// RGB matrix: hardware PWM across the SN32 CT16B0/B1/B2 timers.
// See drivers/led/sn32f2xx.c F290 block + hardware_pwm.diff.
#define HAL_USE_PWM TRUE

// External PCF8563 RTC on P0.14/P0.15 via the ChibiOS software (bit-banged) I2C
// fallback LLD (USE_HAL_I2C_FALLBACK=yes in rules.mk). Requires the i2c_fallback.diff patch
// to the fallback driver (see rtc.c / readme). The SN32 HW I2C peripheral cannot
// reach those pins.
#define HAL_USE_I2C TRUE
#define SW_I2C_USE_I2C1 TRUE        // provides the I2CD1 instance
#define SW_I2C_USE_OPENDRAIN FALSE  // emulate open-drain by input/output switching
#define SW_I2C_USE_OSAL_DELAY FALSE // use rtc.c's busy-wait delay (non-yielding)
/* HAL holds the system lock for a whole transaction and the busy-wait never
 * yields, so every interrupt waits for the whole transfer: 2.6 ms for the
 * calendar read. The fallback driver can release it at each byte boundary
 * (SW_I2C_BYTE_PREEMPT, chibios-contrib ak820pro-patches), but measured on
 * 10-19 that is no fix: UART2's receiver holds far less than 16 bytes, so its
 * 0.3 ms stretches overran ~8x more often, and the calendar read took 5.4 ms,
 * past the deep sleep's 5 ms deadline (history/overrun-ab-20261019/). Off in
 * released builds; dev builds compile it behind the run-time switch below,
 * which boots per transaction.
 *
 * Dev builds: the overrun measurement's hooks (overrun_diag.c) and that
 * switch. */
#if defined(AK820_DEV_HOOKS) && !defined(_FROM_ASM_) && !defined(__ASSEMBLER__)
#define SW_I2C_BYTE_PREEMPT TRUE
extern volatile unsigned char od_i2c_byte_mode;
void od_mask_begin(void);
void od_mask_end(void);
void od_uart_oe(unsigned char in_drain);
void od_uart_rxdrop(void);
void od_usb_enter(void);
void od_usb_exit(void);
void od_st_enter(void);
void od_st_exit(void);
#define SW_I2C_BYTE_PREEMPT_ENABLED()       (od_i2c_byte_mode != 0)
#define SW_I2C_MASK_BEGIN()                 od_mask_begin()
#define SW_I2C_MASK_END()                   od_mask_end()
#define SN32_SERIAL_OE_HOOK(sdp, in_drain)  od_uart_oe(in_drain)
#define SN32_SERIAL_RXDROP_HOOK(sdp)        od_uart_rxdrop()
#define SN32_USB_IRQ_ENTER_HOOK()           od_usb_enter()
#define SN32_USB_IRQ_EXIT_HOOK()            od_usb_exit()
#define SN32_ST_IRQ_ENTER_HOOK()            od_st_enter()
#define SN32_ST_IRQ_EXIT_HOOK()             od_st_exit()
#endif

/* Dedicated timer for the LCD-backlight and indicator software PWM (CT16B3 /
 * GPTD4). It used to ride the RGB row-scan ISR, which welded the PWM switching
 * rate to RGB_MATRIX_SPD_STEP -- so tuning the LED field rate for the rainbow
 * artifact silently retuned the backlight into or out of flicker.
 *
 * NOTE: an earlier version of this comment said the row ISR "has to stay SLOW or
 * the CH582F UART starves". That was WRONG, and is now exactly the wrong lesson
 * to carry: the UART starved because SN32_SERIAL_UART2 sat at the LOWEST
 * interrupt priority, not because the row ISR was fast. With the ordering in
 * mcuconf.h (UART 1, GPT 2, row scan 3) the row ISR runs at ~18800/s and
 * Bluetooth measures BETTER than it ever did at the stock rate. */
#define HAL_USE_GPT TRUE

/* Hardware watchdog (SN32 WDT). Armed at the end of post-init, kicked at the
 * end of every housekeeping pass; see watchdog.c for the 12 s rationale. */
#define HAL_USE_WDG TRUE

#include_next <halconf.h>