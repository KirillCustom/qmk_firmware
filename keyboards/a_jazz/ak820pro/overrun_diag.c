// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The radio link's UART overruns, measured -- see overrun_diag.h. Dev builds
 * only; the file is empty otherwise. */

#include "quantum.h"
#include "overrun_diag.h"
#include "rtc/rtc.h"   /* rtc_seconds_since_boot */

#ifdef AK820_DEV_HOOKS

/* 1.0 and 1.4 ms in system ticks (187,500 Hz): 1.4 ms is a 16-byte FIFO's
 * span at 115200 8N1, if the FIFO is 16 bytes (its depth is unstated). */
#define OD_T_1MS0 ((uint16_t)(CH_CFG_ST_FREQUENCY / 1000u))
#define OD_T_1MS4 ((uint16_t)(CH_CFG_ST_FREQUENCY * 14u / 10000u))
#define OD_RING   32u

static inline uint16_t od_now(void) { return (uint16_t)st_lld_get_counter(); }

/* Read by the drivers' hooks (halconf.h): the lock's span between bytes, as
 * the board boots (0, per transaction) until a dev command sets it. */
volatile uint8_t od_i2c_byte_mode = 0;

static volatile uint8_t  activity;        /* OD_ACT_* */
static volatile uint8_t  pcf_kind;        /* the transaction in flight */
static volatile uint16_t mask_t0;         /* the current locked stretch's start */
static volatile uint16_t mask_last;       /* the last finished stretch's length */
static volatile uint16_t mask_max_cur;    /* the transaction's longest stretch */
static volatile uint16_t pcf_t0;          /* the transaction's start */

typedef struct {
    uint32_t count, n_ge1, n_ge14, spans;
    uint16_t fails, max_span, max_whole;
} od_kind_t;
static od_kind_t kinds[OD_PCF_KINDS];

static volatile uint32_t oe_obs, oe_drain, rx_drop, rx_off;
static volatile uint16_t ctx_pcf, ctx_flash, ctx_sleep, ctx_none;
static volatile uint16_t usb_t0, st_t0, usb_max, st_max, usb_max_since, st_max_since;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t secs;       /* the RTC's seconds since boot (never timer_read32: a
                          * 64-bit software divide, ~250 us under the row ISR) */
    uint8_t  act;
    uint8_t  flags;      /* bits 0-1 the PCF kind, 6 in the drain, 7 byte mode */
    uint16_t span;       /* the last masked I2C stretch, ticks */
    uint16_t usb_max;    /* the longest USB handler since the entry before */
    uint16_t st_max;     /* the longest system-timer handler since then */
} od_entry_t;
_Static_assert(sizeof(od_entry_t) == 14, "two entries fill a 28-byte reply");
static od_entry_t ring[OD_RING];
static volatile uint16_t ring_seq;     /* entries written; entry n is in slot n % OD_RING */

/* ---- The drivers' hooks (halconf.h), ISR or locked contexts ------------- */

void od_mask_begin(void) { mask_t0 = od_now(); }

/* Before the unlock, so the UART ISR that runs at it finds this stretch. */
void od_mask_end(void) {
    uint16_t d = (uint16_t)(od_now() - mask_t0);
    mask_last = d;
    if (d > mask_max_cur) mask_max_cur = d;
    od_kind_t *k = &kinds[pcf_kind < OD_PCF_KINDS ? pcf_kind : 0];
    k->spans++;
    if (d >= OD_T_1MS0) k->n_ge1++;
    if (d >= OD_T_1MS4) k->n_ge14++;
}

static void oe_seen(uint8_t in_drain) {
    if (in_drain) oe_drain++;
    else oe_obs++;
    uint8_t act = activity;
    if (act & OD_ACT_PCF) ctx_pcf++;
    if (act & (OD_ACT_FLASH_PROG | OD_ACT_FLASH_ERASE)) ctx_flash++;
    if (act & OD_ACT_SLEEP) ctx_sleep++;
    if (!act) ctx_none++;
    od_entry_t *e = &ring[ring_seq % OD_RING];
    e->seq     = ring_seq;
    e->secs    = rtc_seconds_since_boot();
    e->act     = act;
    e->flags   = (uint8_t)((pcf_kind & 3u) | (in_drain ? 0x40u : 0u) | (od_i2c_byte_mode ? 0x80u : 0u));
    e->span    = mask_last;
    e->usb_max = usb_max_since;
    e->st_max  = st_max_since;
    usb_max_since = st_max_since = 0;
    ring_seq++;
}
void od_uart_oe(uint8_t in_drain) { oe_seen(in_drain); }
void od_uart_rxdrop(void) { rx_drop++; }

void od_usb_enter(void) { usb_t0 = od_now(); }
void od_usb_exit(void) {
    uint16_t d = (uint16_t)(od_now() - usb_t0);
    if (d > usb_max) usb_max = d;
    if (d > usb_max_since) usb_max_since = d;
}
void od_st_enter(void) { st_t0 = od_now(); }
void od_st_exit(void) {
    uint16_t d = (uint16_t)(od_now() - st_t0);
    if (d > st_max) st_max = d;
    if (d > st_max_since) st_max_since = d;
}

void od_rx_off(void) { rx_off++; }

/* ---- The main loop's side ------------------------------------------------ */

void od_activity_set(uint8_t bits) {
    chSysLock();
    activity |= bits;
    chSysUnlock();
}
void od_activity_clear(uint8_t bits) {
    chSysLock();
    activity &= (uint8_t)~bits;
    chSysUnlock();
}

void od_pcf_begin(uint8_t kind) {
    chSysLock();
    pcf_kind     = kind < OD_PCF_KINDS ? kind : 0;
    mask_max_cur = 0;
    pcf_t0       = od_now();
    activity |= OD_ACT_PCF;
    chSysUnlock();
}

void od_pcf_end(uint8_t kind, bool ok) {
    chSysLock();
    uint16_t whole = (uint16_t)(od_now() - pcf_t0);
    activity &= (uint8_t)~OD_ACT_PCF;
    od_kind_t *k = &kinds[kind < OD_PCF_KINDS ? kind : 0];
    k->count++;
    if (!ok && k->fails < 0xFFFFu) k->fails++;
    if (mask_max_cur > k->max_span) k->max_span = mask_max_cur;
    if (whole > k->max_whole) k->max_whole = whole;
    chSysUnlock();
}

bool od_byte_mode(void) { return od_i2c_byte_mode != 0; }
void od_set_byte_mode(bool on) { od_i2c_byte_mode = on ? 1u : 0u; }

void od_reset(void) {
    chSysLock();
    memset(kinds, 0, sizeof(kinds));
    memset(ring, 0, sizeof(ring));
    ring_seq = 0;
    oe_obs = oe_drain = rx_drop = rx_off = 0;
    ctx_pcf = ctx_flash = ctx_sleep = ctx_none = 0;
    usb_max = st_max = usb_max_since = st_max_since = 0;
    chSysUnlock();
}

static void put16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

/* Op 20: [0..3] OE observations (first reads), [4..7] OE in the drain's
 * re-reads, [8..11] refused bytes, [12..13] ring entries written, [14] byte
 * mode, [15] 0, [16..17] the longest USB handler, [18..19] the longest
 * system-timer handler (ticks), [20..21] OEs seen with a PCF transaction,
 * [22..23] with a flash program or erase, [24..25] with neither nor a
 * sleep, [26..27] the deep sleep's receiver-off intervals (u16). The sleep's
 * own OE count is page 1 of op 22 (kind 0xFF). */
void od_fill_summary(uint8_t *out) {
    memset(out, 0, 28);
    chSysLock();
    put32(&out[0], oe_obs);
    put32(&out[4], oe_drain);
    put32(&out[8], rx_drop);
    put16(&out[12], ring_seq);
    out[14] = od_i2c_byte_mode;
    put16(&out[16], usb_max);
    put16(&out[18], st_max);
    put16(&out[20], ctx_pcf);
    put16(&out[22], ctx_flash);
    put16(&out[24], ctx_none);
    put16(&out[26], (uint16_t)rx_off);
    chSysUnlock();
}

/* Op 21, page 0..15: slots 2*page and 2*page+1, 14 bytes each. */
bool od_fill_ring(uint8_t page, uint8_t *out) {
    if (page >= OD_RING / 2u) return false;
    chSysLock();
    memcpy(out, &ring[2u * page], 2u * sizeof(od_entry_t));
    chSysUnlock();
    return true;
}

/* Op 22, kind 0..3: [0..3] transactions, [4..5] failures, [6..7] the longest
 * masked stretch, [8..9] the longest whole call (ticks), [10..13] stretches
 * of 1.0 ms or more, [14..17] of 1.4 ms or more, [18..21] stretches in all.
 * Kind 0xFF: [0..1] OEs seen during a deep sleep's teardown or wake, the
 * tick rate u32 at [2..5], and the RTC's seconds since boot now at [6..9]
 * (the ring's entries are stamped in the same seconds). */
bool od_fill_kind(uint8_t kind, uint8_t *out) {
    memset(out, 0, 28);
    if (kind == 0xFFu) {
        chSysLock();
        put16(&out[0], ctx_sleep);
        chSysUnlock();
        put32(&out[2], CH_CFG_ST_FREQUENCY);
        put32(&out[6], rtc_seconds_since_boot());
        return true;
    }
    if (kind >= OD_PCF_KINDS) return false;
    chSysLock();
    od_kind_t k = kinds[kind];
    chSysUnlock();
    put32(&out[0], k.count);
    put16(&out[4], k.fails);
    put16(&out[6], k.max_span);
    put16(&out[8], k.max_whole);
    put32(&out[10], k.n_ge1);
    put32(&out[14], k.n_ge14);
    put32(&out[18], k.spans);
    return true;
}

#endif /* AK820_DEV_HOOKS */
