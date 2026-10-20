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
bool od_byte_mode(void);
void od_set_byte_mode(bool on);
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
