// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

/* The radio link's UART overruns, measured (plans/OVERRUN-CAPS-PLAN.md, step
 * 1). Dev builds only: every entry point below is a no-op elsewhere, and the
 * hooks the drivers call (halconf.h) exist only in a dev build.
 *
 * What it records:
 *   - each LS snapshot with OE set (the ISR's first read, and the drain's
 *     re-reads, which raise no flag), and each byte the input queue refused;
 *   - for each OE, the context it was seen in: the main loop's activity bits
 *     (a PCF transaction, a flash program or erase, a deep sleep) as the UART
 *     ISR finds them, the last masked span of the I2C, and the longest USB
 *     and system-timer handlers since the entry before. Context, not cause;
 *   - for each PCF transaction kind: count, failures, the longest masked span
 *     and the longest whole call, and how many spans passed 1.0 and 1.4 ms;
 *   - the deep sleep's receiver-off intervals, counted apart.
 * Times are system ticks (CH_CFG_ST_FREQUENCY, 5.33 us), taken from the
 * free-running counter, which counts while interrupts are masked but stops in
 * deep sleep; spans are 16-bit modular, so nothing longer than ~349 ms. */

#include <stdbool.h>
#include <stdint.h>

/* Activity bits, set by the main loop around each candidate. */
#define OD_ACT_PCF         0x01u
#define OD_ACT_FLASH_PROG  0x02u
#define OD_ACT_FLASH_ERASE 0x04u
#define OD_ACT_SLEEP       0x08u
#define OD_ACT_MASK_PROBE  0x10u   /* the receiver's depth probe: a masked span on demand */

/* The PCF transaction's lock (od_i2c_lock_mode, halconf.h). */
enum { OD_LOCK_TRANSACTION = 0,   /* the system lock throughout (before 10-19) */
       OD_LOCK_BYTE        = 1,   /* released at each byte boundary: no fix */
       OD_LOCK_UNLOCKED    = 2 }; /* unlocked, the row ISR held off: every build's way */

/* PCF transaction kinds and their wire bytes. */
enum { OD_PCF_R1 = 0,   /* a 1-byte register read: 4 */
       OD_PCF_R7,       /* the 7-byte calendar read: 10 */
       OD_PCF_W1,       /* a 1-byte register write: 3 */
       OD_PCF_W8,       /* the 8-byte time write: 9 */
       OD_PCF_KINDS };

#ifdef AK820_DEV_HOOKS
void od_activity_set(uint8_t bits);
void od_activity_clear(uint8_t bits);
void od_pcf_begin(uint8_t kind);
void od_pcf_end(uint8_t kind, bool ok);
void od_rx_off(void);          /* I-class: the deep sleep turned the receiver off */
void od_reset(void);
void od_mask_probe(uint16_t ticks);   /* the depth probe: interrupts masked this long */
uint8_t od_lock_mode(void);
bool    od_set_lock_mode(uint8_t mode);          /* false for a value past OD_LOCK_UNLOCKED */
uint8_t od_set_uart_priority(uint8_t prio);      /* 0 or 1; returns NVIC_GetPriority's readback */
uint8_t od_uart_priority(void);
void    od_mask_begin(void);
void    od_mask_end(void);
/* The interrupt-service-gap monitor on the 20 kHz CT16B3 tick (indicators.c). */
void od_tick(void);                              /* from the tick's handler */
void od_tick_reprime(void);                      /* whenever the tick stops, starts, or the monitor turns on */
void od_mon_set(uint8_t sub);                    /* 1 on, 2 off, 3 zero */
void od_fill_mon(uint8_t *out);                  /* op 25 */
bool od_fill_mon_ring(uint8_t page, uint8_t *out);   /* op 26 */
/* The observed context: the ACCT() scope running now, or ACCT_UNACCOUNTED. */
extern volatile uint8_t od_ctx;
/* HC_DEVHOOK ops 20-22's replies, 28 bytes at out. */
void od_fill_summary(uint8_t *out);
bool od_fill_ring(uint8_t page, uint8_t *out);
bool od_fill_kind(uint8_t kind, uint8_t *out);
#    define OD_ACTIVITY_SET(b)   od_activity_set(b)
#    define OD_ACTIVITY_CLEAR(b) od_activity_clear(b)
#    define OD_PCF_BEGIN(k)      od_pcf_begin(k)
#    define OD_PCF_END(k, ok)    od_pcf_end((k), (ok))
#    define OD_RX_OFF()          od_rx_off()
#else
#    define OD_ACTIVITY_SET(b)   ((void)0)
#    define OD_ACTIVITY_CLEAR(b) ((void)0)
#    define OD_PCF_BEGIN(k)      ((void)0)
#    define OD_PCF_END(k, ok)    ((void)0)
#    define OD_RX_OFF()          ((void)0)
#endif
