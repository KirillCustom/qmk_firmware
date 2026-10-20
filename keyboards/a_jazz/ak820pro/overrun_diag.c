// Copyright 2026 JD Lien
// SPDX-License-Identifier: GPL-2.0-or-later
/* The radio link's UART overruns, measured -- see overrun_diag.h. Dev builds
 * only; the file is empty otherwise. */

#include "quantum.h"
#include "overrun_diag.h"
#include "rtc/rtc.h"   /* rtc_seconds_since_boot */
#include "loop_acct.h" /* ACCT_UNACCOUNTED, the observed context's sentinel */

#ifdef AK820_DEV_HOOKS

/* 1.0 and 1.4 ms in system ticks (187,500 Hz): 1.4 ms is a 16-byte FIFO's
 * span at 115200 8N1, if the FIFO is 16 bytes (its depth is unstated). */
#define OD_T_1MS0 ((uint16_t)(CH_CFG_ST_FREQUENCY / 1000u))
#define OD_T_1MS4 ((uint16_t)(CH_CFG_ST_FREQUENCY * 14u / 10000u))
#define OD_RING   32u

static inline uint16_t od_now(void) { return (uint16_t)st_lld_get_counter(); }

/* Read by the drivers' hooks (halconf.h): the PCF transaction's lock
 * (OD_LOCK_*), unlocked as every build boots; and the stuck-SCL injection. */
volatile uint8_t od_i2c_lock_mode = OD_LOCK_UNLOCKED;
volatile uint8_t od_i2c_stuck     = 0;
volatile uint8_t od_ctx           = ACCT_UNACCOUNTED;   /* loop_acct.h's ACCT() publishes it */

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
static volatile uint16_t ctx_pcf, ctx_flash, ctx_sleep, ctx_none, ctx_probe;
static volatile uint16_t usb_t0, st_t0, usb_max, st_max, usb_max_since, st_max_since;
static volatile uint16_t uart_t0, uart_max;

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

/* In the UART ISR. Under the ISR lock: SysTick outranks UART2 and could
 * publish a new maximum between the copy and the clear below. */
static void oe_seen(uint8_t in_drain) {
    chSysLockFromISR();
    if (in_drain) oe_drain++;
    else oe_obs++;
    uint8_t act = activity;
    if (act & OD_ACT_PCF) ctx_pcf++;
    if (act & (OD_ACT_FLASH_PROG | OD_ACT_FLASH_ERASE)) ctx_flash++;
    if (act & OD_ACT_SLEEP) ctx_sleep++;
    if (act & OD_ACT_MASK_PROBE) ctx_probe++;
    if (!act) ctx_none++;
    od_entry_t *e = &ring[ring_seq % OD_RING];
    e->seq     = ring_seq;
    e->secs    = rtc_seconds_since_boot();
    e->act     = act;
    e->flags   = (uint8_t)((pcf_kind & 3u) | ((od_i2c_lock_mode & 3u) << 4) | (in_drain ? 0x40u : 0u));
    e->span    = mask_last;
    e->usb_max = usb_max_since;
    e->st_max  = st_max_since;
    usb_max_since = st_max_since = 0;
    ring_seq++;
    chSysUnlockFromISR();
}
void od_uart_oe(uint8_t in_drain) { oe_seen(in_drain); }
void od_uart_rxdrop(void) { rx_drop++; }

/* USB's span includes any UART2 handler that preempted it (UART2 is above it
 * since 10-19), and both hooks sit inside the port's prologue and epilogue.
 * The maxima under the ISR lock: UART2's OE entry clears usb_max_since. */
void od_usb_enter(void) { usb_t0 = od_now(); }
void od_usb_exit(void) {
    uint16_t d = (uint16_t)(od_now() - usb_t0);
    chSysLockFromISR();
    if (d > usb_max) usb_max = d;
    if (d > usb_max_since) usb_max_since = d;
    chSysUnlockFromISR();
}
void od_uart_enter(void) { uart_t0 = od_now(); }
void od_uart_exit(void) {
    uint16_t d = (uint16_t)(od_now() - uart_t0);
    if (d > uart_max) uart_max = d;   /* UART2 and the system timer, both at 0, cannot preempt it */
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

/* The receiver's depth probe (rtc.c's burst, op 23 sub 3): interrupts masked
 * for `ticks`, timed on the free-running counter, the span published as an
 * I2C stretch's is, so the ring's entries carry it. Main loop, unlocked. */
void od_mask_probe(uint16_t ticks) {
    od_activity_set(OD_ACT_MASK_PROBE);
    chSysLock();
    uint16_t t0 = od_now();
    while ((uint16_t)(od_now() - t0) < ticks) {}
    mask_last = (uint16_t)(od_now() - t0);
    chSysUnlock();
    od_activity_clear(OD_ACT_MASK_PROBE);
}

uint8_t od_lock_mode(void) { return od_i2c_lock_mode; }
/* The main thread only, between transactions: never during one. */
bool od_set_lock_mode(uint8_t mode) {
    if (mode > OD_LOCK_UNLOCKED) return false;
    od_i2c_lock_mode = mode;
    return true;
}

/* UART2's priority at run time, for the A/B (0 every build, 1 before 10-19):
 * under the lock, its line disabled across the write (CMSIS's disable has its
 * barriers; the pending state is kept), the enable put back as found. Not
 * ChibiOS's vector helpers, which clear the pending state; a serial restart
 * would put mcuconf's value back. */
uint8_t od_set_uart_priority(uint8_t prio) {
    if (prio > 1u) prio = 1u;
    chSysLock();
    bool on = (NVIC->ISER[0] & (1u << UART2_IRQn)) != 0u;
    NVIC_DisableIRQ(UART2_IRQn);
    NVIC_SetPriority(UART2_IRQn, prio);
    if (on) NVIC_EnableIRQ(UART2_IRQn);
    chSysUnlock();
    return (uint8_t)NVIC_GetPriority(UART2_IRQn);
}
uint8_t od_uart_priority(void) { return (uint8_t)NVIC_GetPriority(UART2_IRQn); }

/* ---- The interrupt-service-gap monitor (plans/UART-LATENCY-PLAN.md, fix 3) --
 * The 20 kHz tick's entries, a nominal 50 us apart: a longer gap means its
 * handler was held off, by a masked stretch or by a handler at 0-1 (UART2,
 * the system timer, USB). A gap, not UART2's latency; and what the main loop
 * was in (od_ctx, the activity bits) is the observed context, not a cause.
 * The fast path is a counter read, a subtract and a compare. */
/* In whole ticks of 5.33 us: 24, 37 and 65, i.e. gaps of 128, 197 and 347 us
 * (lateness ~80, ~150, ~300 against the nominal 50). */
#define MON_T130 ((uint16_t)(CH_CFG_ST_FREQUENCY * 130u / 1000000u))
#define MON_T200 ((uint16_t)(CH_CFG_ST_FREQUENCY * 200u / 1000000u))
#define MON_T350 ((uint16_t)(CH_CFG_ST_FREQUENCY * 350u / 1000000u))
#define MON_RING 16u
static volatile uint8_t  mon_on, mon_primed;
static uint16_t          mon_last;
static volatile uint16_t mon_max, mon_seq;
static volatile uint8_t  mon_max_ctx, mon_max_act;
static volatile uint32_t mon_n130, mon_n200, mon_n350, mon_ticks;
typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t secs;
    uint16_t gap;      /* ticks */
    uint8_t  ctx, act;
} mon_entry_t;
static mon_entry_t mon_ring[MON_RING];

void od_tick_reprime(void) { mon_primed = 0; }

void od_tick(void) {
    if (!mon_on) return;
    uint16_t now = od_now();
    uint16_t gap = (uint16_t)(now - mon_last);
    mon_last = now;
    if (!mon_primed) { mon_primed = 1; return; }
    mon_ticks++;
    if (gap < MON_T130) return;
    /* The rare path. */
    chSysLockFromISR();
    mon_n130++;
    if (gap >= MON_T200) mon_n200++;
    if (gap >= MON_T350) mon_n350++;
    uint8_t ctx = od_ctx, act = activity;
    if (gap > mon_max) { mon_max = gap; mon_max_ctx = ctx; mon_max_act = act; }
    if (gap >= MON_T200) {
        mon_entry_t *e = &mon_ring[mon_seq % MON_RING];
        e->seq = mon_seq++; e->secs = rtc_seconds_since_boot(); e->gap = gap; e->ctx = ctx; e->act = act;
    }
    chSysUnlockFromISR();
}

void od_mon_set(uint8_t sub) {
    chSysLock();
    if (sub == 1) { mon_on = 1; mon_primed = 0; }
    if (sub == 2) mon_on = 0;
    if (sub == 3) {
        mon_max = mon_seq = 0; mon_max_ctx = mon_max_act = 0;
        mon_n130 = mon_n200 = mon_n350 = mon_ticks = 0;
        memset(mon_ring, 0, sizeof(mon_ring));
        mon_primed = 0;
    }
    chSysUnlock();
}

void od_reset(void) {
    chSysLock();
    memset(kinds, 0, sizeof(kinds));
    memset(ring, 0, sizeof(ring));
    ring_seq = 0;
    oe_obs = oe_drain = rx_drop = rx_off = 0;
    ctx_pcf = ctx_flash = ctx_sleep = ctx_none = ctx_probe = 0;
    usb_max = st_max = usb_max_since = st_max_since = 0;
    uart_max = 0;
    chSysUnlock();
}

static void put16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

/* Op 20: [0..3] OE observations (first reads), [4..7] OE in the drain's
 * re-reads, [8..11] refused bytes, [12..13] ring entries written, [14] byte
 * mode, [15] 0, [16..17] the longest USB handler, [18..19] the longest
 * system-timer handler (ticks), [20..21] OEs seen with a PCF transaction,
 * [22..23] with a flash program or erase, [24..25] with neither nor a
 * sleep, [26..27] the deep sleep's receiver-off intervals (u16); [14] is the
 * lock mode (OD_LOCK_*) and [15] UART2's priority as the NVIC reads it. The
 * sleep's own OE count is op 22's kind 0xFF. */
void od_fill_summary(uint8_t *out) {
    memset(out, 0, 28);
    chSysLock();
    put32(&out[0], oe_obs);
    put32(&out[4], oe_drain);
    put32(&out[8], rx_drop);
    put16(&out[12], ring_seq);
    out[14] = od_i2c_lock_mode;
    out[15] = (uint8_t)NVIC_GetPriority(UART2_IRQn);
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
 * tick rate u32 at [2..5], the RTC's seconds since boot now at [6..9] (the
 * ring's entries are stamped in the same seconds: coarse context only, since
 * a phase set or a wake's correction can move them), [10..11] OEs seen with
 * the depth probe's masked span, [12..13] the UART2 handler's longest run
 * (ticks, prologue to epilogue excluded). */
bool od_fill_kind(uint8_t kind, uint8_t *out) {
    memset(out, 0, 28);
    if (kind == 0xFFu) {
        chSysLock();
        put16(&out[0], ctx_sleep);
        chSysUnlock();
        put32(&out[2], CH_CFG_ST_FREQUENCY);
        put32(&out[6], rtc_seconds_since_boot());
        put16(&out[10], ctx_probe);
        put16(&out[12], uart_max);
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

/* Op 25: [0] on, [1] primed, [2..3] the longest gap that crossed the first
 * threshold (ticks; 0 if none did), [4..7] gaps of 24 ticks (128 us) or more,
 * [8..11] of 37 (197 us), [12..15] of 65 (347 us), [16..19] gaps measured,
 * [20] the longest's observed context (an ACCT_* scope; ACCT_UNACCOUNTED,
 * 12 today, between scopes), [21] its activity bits, [22..23] the ring's
 * entries written (gaps of 37 ticks or more). */
void od_fill_mon(uint8_t *out) {
    memset(out, 0, 28);
    chSysLock();
    out[0] = mon_on; out[1] = mon_primed;
    put16(&out[2], mon_max);
    put32(&out[4], mon_n130); put32(&out[8], mon_n200); put32(&out[12], mon_n350);
    put32(&out[16], mon_ticks);
    out[20] = mon_max_ctx; out[21] = mon_max_act;
    put16(&out[22], mon_seq);
    chSysUnlock();
}

/* Op 26, page 0..7: slots 2*page and 2*page+1, 10 bytes each at [0..9] and
 * [10..19]: u16 sequence, u32 RTC seconds, u16 the gap (ticks), the
 * observed context, the activity bits. */
bool od_fill_mon_ring(uint8_t page, uint8_t *out) {
    if (page >= MON_RING / 2u) return false;
    memset(out, 0, 28);
    chSysLock();
    memcpy(out, &mon_ring[2u * page], 2u * sizeof(mon_entry_t));
    chSysUnlock();
    return true;
}

#endif /* AK820_DEV_HOOKS */
