#include "watchdog_record.h"
#include "ch582f_ajazz.h"
#include "battery.h"   /* battery_5c_report */
#include "bluetooth.h"
#include "quantum.h"
#include "hal.h"
#include "health.h"   /* health_note_rx_malformed() -- main-loop context only */
#include "loop_acct.h" /* D1: this task's share of each main-loop pass */
#ifdef AK820_DEV_HOOKS
#include "lockchan.h"  /* the lock-LED channel's receiver, fed from 5A */
#endif

#ifndef CH582_SERIAL_DRIVER
#    define CH582_SERIAL_DRIVER SD2
#endif

#define TX_MAX_PAYLOAD 16

/* Mirror the stock firmware's ACK behaviour: after receiving a 5B (connection
 * state) or 5C (battery) frame, the stock MCU replies with the 61 0D 0A ("a\r\n")
 * ACK token ~1.3 ms later. It does NOT ack 5A (LED) frames. Logic-analyzer-
 * confirmed against a stock BT boot. Experimental: enable to test whether a
 * "well-behaved" peer yields cleaner cold-boot connects or new frame types.
 * Define CH582_ACK_FRAMES=0 in config.h to disable. */
#ifndef CH582_ACK_FRAMES
#    define CH582_ACK_FRAMES 1
#endif

/* How often to re-issue the channel-select command while a connection is
 * requested but not yet established. At cold boot the dip-switch callback fires
 * a one-shot 0xA6 microseconds after sdStart(), before the CH582F has finished
 * its own power-up, so that first command is dropped. Retrying until the module
 * answers with a connect event makes boot-in-BT work. */
#ifndef CH582_CONNECT_RETRY_MS
#    define CH582_CONNECT_RETRY_MS 500
#endif

/* How often to poll the module for battery level. The stock firmware periodically
 * sends `A6 53` and the module answers with `5C <pct>` frames -- 2-3 per 5 s poll
 * cycle, bunched, by the battery log's counts; whether those are all replies, or
 * replies plus an unprompted stream, is unconfirmed (docs/battery.md). (The LCD clock is NOT on this wire: the set-time utility
 * talks to the MCU directly, so time never crosses the CH582 serial link.) */
/* Retry cadence for the FIRST reading only, until the module has answered once.
 * Short because a boot with no battery shown is the visible symptom, and each
 * attempt is one ~10-byte frame. */
#ifndef CH582_BATTERY_FIRST_POLL_MS
#    define CH582_BATTERY_FIRST_POLL_MS 250
#endif
#ifndef CH582_BATTERY_POLL_MS
#    define CH582_BATTERY_POLL_MS 5000
#endif

/* Fallback for a MISSED `5B 32`. That frame is the only "connected" signal the
 * module ever sends, it is sent once at link-up, and the RX line is silent at
 * idle -- so dropping it strands conn_state in LINKING forever while the link is
 * actually live (observed: channel digit blinks indefinitely after a successful
 * pair, cleared only by toggling the mode slider).
 *
 * A `5A` host-LED frame is decent evidence of a live link, since a host only
 * sends LED state to a device it is connected to. But `5A` is exactly what the
 * power-up / 2.4G link-up burst forges (see the 5A case below), so promoting on
 * it naively would trade a stuck-blinking digit for a false "connected".
 * Requiring the LINKING state to have persisted this long first steps past that
 * burst: a genuine `5B 32` arrives at link-up and would already have won, so
 * anything still LINKING after this window is a frame we missed, not one in
 * flight. Delay is cosmetic-only -- the digit is the sole consumer. */
/* Pairing is CONFIRMED by a `5B 31`, not by having sent `A6 51`. The module
 * appears to IGNORE the pair command while a connect attempt it started is still
 * in flight (`5B 33`/`34`), and only accepts it once that attempt is abandoned
 * (`5B 36`) or otherwise settles. Pressing a slot key issues a connect, so a
 * hold-to-pair fires straight into exactly that window and the single `A6 51`
 * is silently dropped.
 *
 * Symptom this caused: hold 1 s -> nothing, hold 2 s -> nothing, hold 3 s ->
 * works. It reads as a timing threshold and is not one; by the third attempt the
 * module has simply given up on connecting and is finally listening. A one-shot
 * capture happened to catch it already-abandoned and "confirmed" 1 s, which is
 * how this hid.
 *
 * So retry `A6 51` until the module actually says `5B 31`. */
/* A select (`A6 <slot>`) issued while the module is ADVERTISING is DECLINED.
 * Captured on the wire 2026-08-29:
 *
 *   [tx] A6 51 pair      -> [rx] 5B 31   module advertising
 *   [tx] A6 32 select    -> [rx] 5B 23   idle. No 33 (attempting), no 32.
 *   [tx] A6 33 select    -> [rx] 5B 36
 *   [tx] A6 32 select    -> [rx] 5B 32   works after the back-and-forth
 *
 * Note it is NOT a lost frame: the module answers, with `5B 23`. It stops
 * advertising and simply never starts connecting. Symptom: after entering
 * pairing and going back to a paired device, it will not reconnect until you
 * switch slots back and forth.
 *
 * The select is otherwise NEVER re-issued once module_alive is set (see the
 * cold-boot retry below), so a declined select is unrecoverable by design.
 *
 * Retry ONLY while the module has not started an attempt. A working select is
 * answered with 33/34/32 inside a second (measured), so absence of any of those
 * is a reliable "it declined". This deliberately does NOT re-issue once 33/34
 * has arrived -- that is exactly the slow macOS directed-advertising case the
 * cold-boot guard protects, where re-selecting restarts advertising and starves
 * a reconnect that was progressing fine.
 *
 * SECOND MEASUREMENT (the first fix was too impatient and did NOT work):
 *
 *   5B 31            advertising starts
 *   A6 32   +2s      declined
 *   reselect +3s     declined
 *   reselect +4s     declined      <- gave up here
 *   5B 23    +6s     module goes IDLE, one second too late
 *
 * The module ignores a select for as long as it is advertising, and only becomes
 * receptive once it emits `5B 23` (idle/finalize). Three tries over ~3.6 s all
 * landed inside the advertising window. So the retry is now driven by that
 * EVENT, not only by the clock: a `5B 23` while a select is pending re-issues it
 * immediately, which is the exact moment the module is free. The timed retry
 * remains as a backstop and the try budget is wider (8). */
#ifndef CH582_SELECT_CONFIRM_MS
#    define CH582_SELECT_CONFIRM_MS 1500
#endif
#ifndef CH582_SELECT_MAX_TRIES
#    define CH582_SELECT_MAX_TRIES 8
#endif

#ifndef CH582_PAIR_RETRY_MS
#    define CH582_PAIR_RETRY_MS 400
#endif
#ifndef CH582_PAIR_MAX_TRIES
#    define CH582_PAIR_MAX_TRIES 12   /* ~4.8 s of trying, then give up */
#endif

#ifndef CH582_5A_PROMOTE_MS
#    define CH582_5A_PROMOTE_MS 3000
#endif

static volatile bool    is_module_connected = false;
/* Active BT slot 1-3. Derived from the selected 0xA6 profile, NOT from the 5B
 * stream: a logic-analyzer capture proved the 5B second byte is a handshake-STAGE
 * code (0x23/0x31..0x34 seen regardless of slot), not the slot number. 0 = none. */
static volatile uint8_t connected_slot = 0;

/* True while the module is advertising / waiting to bond (5B 31). */
static volatile bool    is_pairing = false;
/* Detailed connection state for the LCD indicator (superset of the two bools). */
static volatile ch582_conn_state_t conn_state = CH582_CONN_IDLE;
/* When conn_state last became CH582_CONN_LINKING, for the 5A promotion window
 * above. Deliberately NOT last_attempt_time, which the connect retry resets
 * every CH582_CONNECT_RETRY_MS and so never accumulates. 32-bit so a long stall
 * in LINKING cannot wrap back under the window. */
static volatile uint32_t linking_since = 0;
/* Battery percentage (0-100) from 5C frames; 0xFF until the module first reports. */
static volatile uint8_t battery_level = 0xFF;
/* Host keyboard LED bitmap from 5A frames (USB LED bits; bit1 = caps lock). */
static volatile uint8_t host_leds = 0;

/* Pending connection request: the profile the user asked for, retried until the
 * module reports connected (or the request is cleared by leaving BT mode). */
static volatile bool    connect_requested = false;
static volatile uint8_t requested_profile = 0;
/* True while the mode slider is in the USB position (wireless link not in use).
 * Set from the dip-switch handler via ch582_cancel_connect / ch582_set_profile. */
static volatile bool    usb_mode = false;
/* True once the module has ACKed anything, i.e. its UART is up and receiving.
 * Cold boot is the only time it is false (the boot-time slot select can fire
 * before the CH582F finishes powering up). Used to bound the select retry: we
 * re-issue the select only until the module is alive, NOT until it connects --
 * re-selecting a live module restarts its advertising and starves slow (macOS)
 * reconnects, which need the peripheral to keep advertising uninterrupted. */
static volatile bool    module_alive = false;

#ifndef CH582_BOUNCE_MS
#    define CH582_BOUNCE_MS 700
#endif

/* THE pending control action -- at most one exists at a time, and
 * supersession is total. This used to be three parallel flag/timer/counter
 * triplets (bounce_pending/select_pending/pairing_pending), each added by a
 * separate incident; the next BT bug was going to be an interaction between
 * two of them. The transition spec, wire captures and do-not-break
 * invariants live in hardening-plan/findings-ch582-states.md (workspace
 * repo) -- behaviour here is byte-identical to the triplet version.
 *
 *   PA_SELECT: an A6 <slot> sent but the module has not acted on it
 *              (5B 33/34/32). Re-sent on a 1.5 s clock AND immediately on a
 *              5B 23 (the moment an advertising module becomes receptive --
 *              clock-only retries measurably missed that window). MUST stop
 *              the moment 33/34 arrives: re-selecting an attempting module
 *              restarts advertising and starves slow macOS reconnects.
 *   PA_PAIR:   an A6 51 sent but no 5B 31 confirmation -- the module drops
 *              the pair command while a connect attempt is in flight, so it
 *              is re-sent every 400 ms until confirmed (or connected, or
 *              out of budget).
 *   PA_BOUNCE: cancel-pairing bounce (see ch582_set_profile): a DIFFERENT
 *              slot was sent to force the advertising module out of its
 *              same-slot-select-is-a-no-op state; after CH582_BOUNCE_MS the
 *              action becomes PA_SELECT on the real target.
 *
 * The cold-boot select retry below is deliberately NOT a kind here: its
 * lifecycle (uncapped, until the first ACK proves the module's UART is up,
 * fires regardless of what else is pending) is genuinely different. */
typedef enum { PA_NONE = 0, PA_BOUNCE, PA_SELECT, PA_PAIR } pa_kind_t;
static struct {
    pa_kind_t kind;
    uint8_t   target;     /* PA_SELECT/PA_BOUNCE: the REAL slot wanted */
    uint16_t  last_send;
    uint8_t   tries;
} pa = { PA_NONE, 0, 0, 0 };

static uint16_t         last_attempt_time = 0;
/* Last time a battery poll (A6 53) was sent. */
static uint16_t         last_battery_poll = 0;
/* Battery observability (debug page). battery_level alone cannot distinguish
 * "the module says 100%" from "the module has never answered", because the
 * 0xFF initial value is masked by the d <= 100 guard on the way in. */
/* 32-bit: a 16-bit timestamp read through timer_elapsed() wraps every 65.5 s,
 * so a module that had gone silent looked fresh again once a minute. */
static uint32_t         battery_last_recv = 0;
static uint16_t         battery_reports   = 0;   /* 5C frames received, wrapping */
static bool             battery_seen      = false;
static bool             battery_stale     = false;   /* the last report predates a deep sleep */
static bool             battery_poll_held = false;   /* a deep sleep's wake: until sleep.c lets go */
static uint8_t          battery_min       = 100;

/* Request a battery-level report from the module. Logic-analyzer-decoded from the
 * stock firmware: the MCU sends `A6 53` and the module replies with a `5C <pct>`
 * frame (parsed in ch582_task). The module is otherwise silent, so battery only
 * updates if we poll. */
#define CH582_BATTERY_REQ_PARAM 0x53
void ch582_poll_status(void) {
    uint8_t param = CH582_BATTERY_REQ_PARAM;
    ch582_send_command(0xA6, &param, 1);
}

typedef enum {
    STATE_WAIT_TYPE = 0,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM
} parse_state_t;

#ifdef WDT_TEST_HOOKS
/* Fault-injection ring (instrumented builds; see the parser loop). Main-loop
 * only: filled from raw HID (main loop), drained from ch582_task (main loop). */
#define CH582_INJECT_MAX 64
#define CH582_A6_TRACE   16
static uint8_t inj_buf[CH582_INJECT_MAX];
static uint8_t inj_head = 0, inj_tail = 0;

/* While muted, REAL UART bytes are drained and discarded so the parser sees
 * only injected traffic -- the live module answers A6 commands even in wired
 * mode, and its interleaved 5B frames made the pending-action tests
 * nondeterministic (Codex phase-4 review #1). */
static bool ch582_rx_muted = false;
void ch582_rx_mute(bool on) { ch582_rx_muted = on; }

void ch582_inject(const uint8_t *bytes, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t nxt = (uint8_t)((inj_tail + 1) % CH582_INJECT_MAX);
        if (nxt == inj_head) return;   /* full: drop the rest, tests keep packets small */
        inj_buf[inj_tail] = bytes[i];
        inj_tail          = nxt;
    }
}

bool ch582_inject_pop(uint8_t *out) {
    if (inj_head == inj_tail) return false;
    *out     = inj_buf[inj_head];
    inj_head = (uint8_t)((inj_head + 1) % CH582_INJECT_MAX);
    return true;
}

/* Outbound A6 trace: the last CH582_A6_TRACE params sent, newest last, plus a
 * running count. This is what lets a host test OBSERVE the pending-action
 * machinery (supersession, the bounce's two ordered selects, retry cadence)
 * instead of only the rx-driven state -- the Codex phase-3 review's gap.
 * Recorded at enqueue, which equals send order: 0xA6 is never coalesced. */
static uint8_t  a6_trace[CH582_A6_TRACE];
static uint8_t  a6_trace_n = 0;    /* valid entries, saturates at the ring size */
static uint16_t a6_count   = 0;

static void a6_trace_note(uint8_t param) {
    if (a6_trace_n == CH582_A6_TRACE) {
        memmove(a6_trace, a6_trace + 1, CH582_A6_TRACE - 1);
        a6_trace_n--;
    }
    a6_trace[a6_trace_n++] = param;
    a6_count++;
}

uint8_t ch582_a6_trace(uint8_t *out, uint16_t *count) {
    memcpy(out, a6_trace, a6_trace_n);
    *count = a6_count;
    return a6_trace_n;
}
#endif

#ifdef AK820_DEV_HOOKS
/* Inbound 5A trace (dev builds; HC_DEVHOOK ops 7-9): every checksum-valid 5A
 * frame, accepted or not. This is what the module passes on from the host's LED
 * report, seen with no display in the path: which of the five bits arrive,
 * whether a repeated value does, and how close together frames can come
 * (plans/BACKLOG.md, the lock-LED channel). Main-loop context only, like the
 * parser that feeds it and the raw-HID handler that reads it.
 *
 * Frame n lands in slot n % CH582_5A_RING, so a host that knows the count knows
 * which frame each slot holds. The gap is the parse time's, which is within one
 * main-loop pass of the frame's arrival; frames that queue behind a long pass
 * show gaps near 0. */
#define CH582_5A_RING 52   /* four 13-entry pages */
static uint32_t led5a_frames   = 0;
static uint32_t led5a_accepted = 0;
static uint32_t led5a_last_at  = 0;
static uint8_t  led5a_last     = 0;
static uint8_t  led5a_ring[CH582_5A_RING][2];   /* {byte, ms since the previous frame, 255 = longer or none} */

static void led5a_note(uint8_t d, bool accepted) {
    uint32_t now = timer_read32();
    uint32_t gap = led5a_frames ? now - led5a_last_at : UINT32_MAX;
    uint8_t *e   = led5a_ring[led5a_frames % CH582_5A_RING];
    e[0]         = d;
    e[1]         = gap > 254u ? 255u : (uint8_t)gap;
    led5a_frames++;
    if (accepted) led5a_accepted++;
    led5a_last_at = now;
    led5a_last    = d;
}

void ch582_led5a_fill(uint8_t *out) {
    uint32_t age = led5a_frames ? timer_elapsed32(led5a_last_at) : UINT32_MAX;
    memcpy(&out[0], &led5a_frames, 4);
    memcpy(&out[4], &led5a_accepted, 4);
    out[8] = host_leds;
    out[9] = led5a_last;
    memcpy(&out[10], &age, 4);
    out[14] = CH582_5A_RING;
}

/* The reply echoes its page, so a host can reject a stale one (codex, flash 2g). */
bool ch582_led5a_page(uint8_t page, uint8_t *out) {
    if (page >= CH582_5A_RING / 13) return false;
    out[0] = page;
    out[1] = 0;
    memcpy(&out[2], led5a_ring[page * 13], 26);
    return true;
}

void ch582_led5a_reset(void) {
    led5a_frames = led5a_accepted = 0;
    led5a_last_at = 0;
    led5a_last    = 0;
    memset(led5a_ring, 0, sizeof(led5a_ring));
}

/* The RX tap (dev builds; HC_DEVHOOK ops 15-16): every raw byte the module
 * sends, while it is armed. The parser's window drops a byte that is not part
 * of a 5A/5B/5C frame or an ACK without a trace, so a frame type nobody has
 * documented would be invisible; this is what shows one -- for the 2.4G
 * dongle's raw-HID-usage interface first (plans/GREMLIN-RESULTS.md). Off by
 * default and off after a reset, so it costs one test per byte; armed, one
 * store. Main-loop context only, like the parser that feeds it.
 *
 * Byte n lands in slot n % CH582_RXTAP_RING; the count is the total since
 * arming, so a host reading the count first knows which slots it has. The
 * slot is its own index, not count % ring: 2^32 is not a multiple of 260, so
 * a wrapped count would skip slots 256-259 (codex rx-tap should-fix 1), and
 * the M0 has no divide instruction to spend per byte. */
#define CH582_RXTAP_RING 260   /* ten 26-byte pages */
static bool     rxtap_armed = false;
static uint32_t rxtap_count = 0;
static uint16_t rxtap_slot  = 0;
static uint8_t  rxtap_ring[CH582_RXTAP_RING];

static inline void rxtap_note(uint8_t c) {
    if (!rxtap_armed) return;
    rxtap_ring[rxtap_slot] = c;
    if (++rxtap_slot == CH582_RXTAP_RING) rxtap_slot = 0;
    rxtap_count++;
}

void ch582_rxtap_arm(bool armed) {
    if (armed) {
        rxtap_count = 0;
        rxtap_slot  = 0;
        memset(rxtap_ring, 0, sizeof(rxtap_ring));
    }
    rxtap_armed = armed;
}

void ch582_rxtap_fill(uint8_t *out) {
    out[0] = rxtap_armed ? 1 : 0;
    memcpy(&out[1], &rxtap_count, 4);
    uint16_t ring = CH582_RXTAP_RING;
    memcpy(&out[5], &ring, 2);
}

/* The reply echoes its page, as the 5A trace's does. */
bool ch582_rxtap_page(uint8_t page, uint8_t *out) {
    if (page >= CH582_RXTAP_RING / 26) return false;
    out[0] = page;
    out[1] = 0;
    memcpy(&out[2], &rxtap_ring[page * 26], 26);
    return true;
}

/* The lock-LED channel's receiver (lockchan.c; plans/LOCK-LED-CHANNEL-PLAN.md).
 * Dev builds only while it is an experiment: frames are decoded and counted,
 * and HC_DEVHOOK ops 10-13 read them; nothing acts on them yet. Fed every 5A
 * byte while linked, raw (before the d <= 0x1F guard: P3 carries data in the
 * padding bits 5-7, which the module passes and the guard keeps from QMK).
 * Zero-initialised, which is lockchan_init's state. */
static lockchan_t lockchan;

const lockchan_t *ch582_lockchan(void) { return &lockchan; }
void ch582_lockchan_clear(void) { lockchan_clear_stats(&lockchan); }
/* Only while the radio is the active host: in cable mode the LED state the
 * band shows is USB's, which a radio frame must not hold (codex, flash 2h). */
bool ch582_lockchan_holds_scroll(void) { return !usb_mode && lockchan_holds_scroll(&lockchan); }
/* Every site where the link may move also drops stage 1's pending echo, so a
 * reply cannot leave on a link other than its request's (r81 below; codex
 * stage-1 review, should-fix 4). */
static void r81_link_lost(void);
#    define LOCKCHAN_LINK_LOST() do { lockchan_link_lost(&lockchan); r81_link_lost(); } while (0)
#else
#    define LOCKCHAN_LINK_LOST() ((void)0)
#endif

static const SerialConfig serial_cfg = {
    .speed               = 115200,
    .UART_WordLength     = 3,  // 8 data bits
    .UART_StopBits       = 0,  // 1 stop bit
    .UART_Parity         = 0,  // No parity
    .UART_FIFOControl    = 0,
    .UART_AutoBaudControl = 0,
    .UART_Oversampling   = 16,
    .UART_HalfDuplexMode = 0,
};

void ch582_send_keyboard_report(report_keyboard_t *report) {
    /* Boot keyboard report = [mods][reserved][key1..key6]; this matches the
     * stock A1 frame byte-for-byte.
     *
     * ⚠️ report->keys is valid here ONLY because bluetooth_can_send_nkro()
     * returns false, so QMK always hands this function a genuine
     * report_keyboard_t. An earlier comment claimed keys[] was safe
     * "regardless of NKRO (first member of the report union)" -- WRONG in
     * this tree (caught by Rachel, 2026-09-01): report_keyboard_t and
     * report_nkro_t are two SEPARATE packed structs in report.h, the NKRO
     * one has no keys[] slot array at all (it is a mods+bitmap layout), and
     * their leading bytes differ (report_id is unconditional in NKRO,
     * KEYBOARD_SHARED_EP-gated -- i.e. absent here -- in the boot report).
     * If anyone ever flips can_send_nkro to true, this function must grow a
     * bitmap->slots conversion; reading keys[] off an NKRO report would be
     * silent garbage. */
    uint8_t buf[8];
    buf[0] = report->mods;
    buf[1] = 0; /* reserved */
    for (uint8_t i = 0; i < 6; i++) {
        buf[2 + i] = report->keys[i];
    }
    ch582_send_command(0xA1, buf, sizeof(buf));
}

/* --- QMK Bluetooth driver API ------------------------------------------------
 * With BLUETOOTH_DRIVER = custom, QMK's host.c routes key reports to bt_driver
 * (these bluetooth_* functions) whenever the active connection host is
 * Bluetooth, and to the USB driver otherwise. The mode slider drives the active
 * host (see dip_switch_update_user). No host_set_driver hacks, no clobber, and
 * no manual USB/wireless routing -- core handles it. */
void bluetooth_init(void) {
    is_module_connected = false;
    sdStart(&CH582_SERIAL_DRIVER, &serial_cfg);
}

void bluetooth_task(void) {
    ACCT(ACCT_CH582, ch582_task());
}

bool bluetooth_is_connected(void) {
    return is_module_connected;
}

/* The module's wireless input is a 6KRO boot report (A1). Reporting NKRO as
 * unsupported makes host_can_send_nkro() return false while Bluetooth is the
 * active host, so QMK sends a plain boot report_keyboard_t via
 * bluetooth_send_keyboard() -- no NKRO->boot conversion needed here. USB keeps
 * NKRO (keyboard.json "nkro": true) when USB is the active host. */
bool bluetooth_can_send_nkro(void) {
    return false;
}

uint8_t bluetooth_keyboard_leds(void) {
    return host_leds;
}

void bluetooth_send_keyboard(report_keyboard_t *report) {
    ch582_send_keyboard_report(report);
}

void bluetooth_send_nkro(report_nkro_t *report) {
    /* Not reached: bluetooth_can_send_nkro() is false. Stubbed for safety. */
}

void bluetooth_send_mouse(report_mouse_t *report) {}

/* Consumer control (volume/media, e.g. the encoder) over the wireless link.
 * The encoder emits HID consumer usages; without forwarding them here they only
 * work over USB. Frame is 0xA3 + 2-byte little-endian usage, mirroring the A1
 * keyboard framing -- confirmed working on hardware over both BT and 2.4G.
 *
 * Gated on connect_requested ONLY -- deliberately NOT on is_module_connected.
 * `5B 32` is the single, never-repeated "connected" announcement, so one lost
 * frame strands conn_state in LINKING forever (see CH582_5A_PROMOTE_MS). While
 * stranded, is_module_connected is false and this used to silently drop every
 * media keypress -- volume dead over BT while typing kept working, because
 * bluetooth_send_keyboard() has no such gate. That asymmetry was the bug: the
 * keyboard path already sends regardless, and a consumer frame sent to a link
 * that is genuinely down simply goes nowhere. Match the keyboard path. */
void bluetooth_send_consumer(uint16_t usage) {
    if (!connect_requested) return;
    uint8_t buf[2];
    buf[0] = usage & 0xFF;
    buf[1] = (usage >> 8) & 0xFF;
    ch582_send_command(0xA3, buf, sizeof(buf));
}

void bluetooth_send_system(uint16_t usage) {}

#ifdef RAW_ENABLE
/* Raw-HID replies go back over USB even in wireless mode.
 *
 * The default is a WEAK NO-OP in drivers/bluetooth/bluetooth.c, and
 * tmk_core/protocol/host.c wires it in as the Bluetooth driver's .send_raw_hid.
 * So in BT/2.4G mode host_raw_hid_send() hands every reply to an empty function
 * and they are SILENTLY DISCARDED -- the CH582F protocol has no concept of raw
 * HID and there is nowhere for them to go.
 *
 * Consequences, all of which read as "the tool is broken":
 *   - VIA cannot complete its handshake, so it will not connect at all
 *   - every round-trip ak820ctl command fails (info, flash write, flash crc)
 *   - and it fails EVEN WITH THE USB CABLE PLUGGED IN, because the dip switch,
 *     not the cable, is what selects the active host driver
 *
 * Inbound reports are unaffected: they arrive on the USB OUT endpoint whatever
 * the dip switch says, which is why the request lands and only the answer
 * disappears. That asymmetry is what makes it look like a tooling bug.
 *
 * Raw HID is a HOST-TOOL channel, not a typing channel. There is no reason for
 * it to follow the keyboard's output route -- send it back the way it came.
 *
 * Safe when USB is down: send_report() is bounded at 100 ms, and a reply can
 * only exist in response to a request that arrived over USB in the first place,
 * so on battery this is never reached. */
extern void send_raw_hid(uint8_t *data, uint8_t length);  /* chibios/usb_main.c */

void bluetooth_send_raw_hid(uint8_t *data, uint8_t length) {
    send_raw_hid(data, length);
}
#endif

/* --- reliable (ACK'd, retrying) TX queue --------------------------------------
 * The module ACKs every frame we send with `61 0D 0A`. Sending once and ignoring
 * that ACK means a dropped/checksum-rejected frame is simply lost -- for a key
 * RELEASE that strands the key and the host auto-repeats it (the sporadic BT
 * stuck-key). Stock (and the open @isuua reference in aliou/keebs,
 * vendor/edthu-wireless -- the same module protocol) instead queue each frame and
 * retransmit until the module ACKs. This is a minimal port of that: one frame
 * in flight at a time, retried on ACK-timeout, popped on ACK. Frames are
 * idempotent (they carry STATE, not events), so a retransmit is always safe. */
#define CH582_TXQ_LEN       24
#define CH582_TX_FRAME_MAX  (TX_MAX_PAYLOAD + 2)
#ifndef CH582_TX_ACK_TIMEOUT_MS
#    define CH582_TX_ACK_TIMEOUT_MS 10
#endif
#ifndef CH582_TX_MAX_RETRIES
#    define CH582_TX_MAX_RETRIES 8
#endif

typedef struct {
    uint8_t data[CH582_TX_FRAME_MAX];
    uint8_t len;
} ch582_tx_frame_t;

static ch582_tx_frame_t tx_q[CH582_TXQ_LEN];
static uint8_t          tx_head = 0, tx_tail = 0;   /* head = frame in flight / next to send */
static bool             tx_in_flight = false;
static uint16_t         tx_sent_time = 0;
static uint8_t          tx_retries   = 0;

/* TX health counters. NOTE (corrected 2026-09-01, audit BW-5): an earlier
 * comment here claimed the RX FIFO was disabled and a byte had to be serviced
 * within ~87 us. WRONG -- the SN32 serial LLD ALWAYS enables the 16550-style
 * 16-byte FIFO; `UART_FIFOControl = 0` in serial_cfg merely selects an RX
 * threshold of 1 (IRQ per byte), so the real overrun tolerance is ~16 bytes
 * ~= 1.4 ms at 115200. The historical byte loss came from the interrupt
 * priority inversion starving the ISR past even that, not from a missing
 * FIFO. A lost ACK stalls the in-flight frame for CH582_TX_ACK_TIMEOUT_MS,
 * and a keystroke is two frames -- so ACK loss shows up as throughput
 * collapse (you can out-type the link), not as mild latency. */
static uint32_t tx_stat_sent = 0, tx_stat_timeout = 0, tx_stat_drop = 0;
/* The other two ways a frame is lost without any long main-loop stall
 * (Phase 1b, D1). tx_stat_drop counts only a full queue's rejection, so these
 * read as zero drops until counted: a frame abandoned after its retransmits,
 * and a queued state frame overwritten by a newer one (which can collapse a
 * double-tap into one press). Plain counters, every build. */
static uint32_t tx_stat_giveup = 0, tx_stat_replaced = 0;

/* UART receive errors, from the serial driver's event flags. The driver now
 * reports a hardware overrun (a byte lost with the FIFO full) as
 * SD_OVERRUN_ERROR, the flag it already raised for a full input queue.
 * Counted per task pass that saw each, not per byte: a trend. In every build
 * since Phase 1b (D1) -- they were console-only, so the daily build that the
 * stall soaks run on could not see a byte lost on the link. */
static event_listener_t uart_err_listener;
static uint32_t         uart_err_overrun, uart_err_framing, uart_err_parity;

/* Read and count the UART's error flags since the last call; returns them so a
 * caller can also act on them (the raw-HID collector below). Main-loop only. */
static eventflags_t uart_err_poll(void) {
    static bool listening = false;
    if (!listening) {   /* lazily: the listener binds to this (main) thread */
        chEvtRegisterMaskWithFlags(chnGetEventSource(&CH582_SERIAL_DRIVER),
                                   &uart_err_listener, EVENT_MASK(0),
                                   SD_OVERRUN_ERROR | SD_FRAMING_ERROR | SD_PARITY_ERROR);
        listening = true;
    }
    eventflags_t f = chEvtGetAndClearFlags(&uart_err_listener);
    if (f & SD_OVERRUN_ERROR) uart_err_overrun++;
    if (f & SD_FRAMING_ERROR) uart_err_framing++;
    if (f & SD_PARITY_ERROR)  uart_err_parity++;
    return f;
}

#ifdef AK820_DEV_HOOKS
/* Raw HID over the 2.4G dongle, stage 1: an echo (plans/RAWHID-24G-PLAN.md,
 * "Codex's plan review and stage 1 as specified"). A 32-byte report the host
 * writes to the dongle's FF60 interface arrives from the module as
 *     81 <32 bytes> <sum = (0x81 + the 32) & 0xFF>
 * and the stock firmware answers with 91 <32 bytes> <sum> the same way. Stage 1
 * answers each committed 81 with a 91 carrying the same 32 bytes, and nothing
 * else: no dispatch, so a false commit costs a stray reply and no more.
 *
 * Receive, beside the 3-byte window:
 *   - A collection starts only on a 0x81 at a BOUNDARY, decided before the
 *     byte is consumed: after the window was cleared by a matched frame or
 *     ACK, or after measured silence. Any byte the window takes without a
 *     match clears it, so `5C 25 | 81` (37%) completes the 5C and never
 *     starts, nor does P3's `5A 81 ..` (codex plan review, must-fix 1).
 *   - Silence: a drain that finds the input queue EMPTY at least
 *     RAW81_SILENCE_MS after the last byte was read. The UART ISR queues each
 *     byte as it arrives, so an empty queue then means none came in that span
 *     (give or take the FIFO trigger and a masked ISR, both well under it).
 *   - Collecting, bytes go to the buffer, not the window (payloads can hold a
 *     valid 5x frame or 61 0D 0A). A UART error flag seen meanwhile, a bad
 *     sum, or a collection older than RAW81_LIFETIME_MS aborts it; an abort
 *     QUARANTINES the line: every byte is dropped until it has been silent
 *     RAW81_SILENCE_MS, so no tail reaches the window (must-fix 2).
 *   - Collected in every mode, so a 81's payload never reaches the window;
 *     answered only in 2.4G with the link up.
 * Transmit: one 34-byte slot, sent only when the keyboard queue is empty and
 * nothing is in flight (keys first), never retried or coalesced, dropped and
 * counted if a reply is still waiting when the next 81 commits. After a 91
 * the A-frame pump waits RAW91_ACK_WINDOW_MS: an ACK in that window is counted
 * as the 91's and pops nothing, so a late one cannot release an A1 (61 0D 0A
 * carries no sequence). That count is what says whether the module ACKs 91
 * (must-fix 5). */
#define RAW81_LEN           34u
#define RAW81_SILENCE_MS    5u
/* A collection may span several passes (RAW81_DRAIN_MS), and the drain was
 * measured taking 8-9 ms over one 81's 34 bytes (the Mac session, 10-19;
 * whether that is the module's pacing or the cost per byte is open, and op
 * 17's pass count is what tells): 15 ms would leave too little margin once
 * the collection spans passes. */
#define RAW81_LIFETIME_MS   40u
/* While a collection is in progress, a drain stops after this long, so one
 * pass never pays for a whole frame: before it, every 81 cost one pass of
 * 13-15 ms, 8-9 of them in this drain (history/raw81-echo-20261019-mac/). */
#define RAW81_DRAIN_MS      2u
#define RAW91_ACK_WINDOW_MS 15u
typedef enum { R81_IDLE, R81_COLLECT, R81_QUARANTINE } r81_state_t;
static r81_state_t r81_state       = R81_IDLE;
static bool        r81_boundary    = true;
static uint8_t     r81_buf[RAW81_LEN];
static uint8_t     r81_len;
static uint16_t    r81_start;
static uint16_t    rx_last_byte_at;
static bool        r81_tainted;   /* a UART error since the line was last silent */
static bool        r91_pending, r91_ack_window;
static uint8_t     r91_buf[RAW81_LEN];
static uint16_t    r91_sent_at;
static uint32_t    r81_ok, r81_badsum, r81_lifetime, r81_taint, r81_qbytes;
static uint8_t     r81_passes;                       /* drains the current collection has spanned */
static uint16_t    r81_last_ms;                      /* the last commit: ms from its 81 to its sum */
static uint8_t     r81_last_passes, r81_max_passes;  /* ... and the passes it spanned; the most */
static uint32_t    r91_sent, r91_dropped, r91_acks;

static void r81_abort(uint32_t *why) {
    (*why)++;
    r81_state    = R81_QUARANTINE;
    r81_boundary = false;
}

/* The line starts over (a resync, a sleep's wake): nothing half-collected or
 * waiting survives it. The ACK window is left to run out on its own. */
static void r81_reset(void) {
    r81_state    = R81_IDLE;
    r81_boundary = true;
    r81_tainted  = false;
    r91_pending  = false;
}

/* The link moved (LOCKCHAN_LINK_LOST): the reply waiting for the old one is
 * dropped. A collection in progress is not cut loose, since its tail would
 * reach the window; it goes to quarantine, which ends at the next silence. */
static void r81_link_lost(void) {
    r91_pending = false;
    if (r81_state == R81_COLLECT) {
        r81_state    = R81_QUARANTINE;
        r81_boundary = false;
    }
}

static void r81_commit(void) {
    if (uart_err_poll() || r81_tainted) {   /* an error during this drain, before the commit */
        r81_abort(&r81_taint);
        return;
    }
    if (timer_elapsed(r81_start) >= RAW81_LIFETIME_MS) {   /* crossed it mid-drain (should-fix 3) */
        r81_abort(&r81_lifetime);
        return;
    }
    uint8_t s = 0;
    for (uint8_t i = 0; i < RAW81_LEN - 1u; i++) s += r81_buf[i];
    if (s != r81_buf[RAW81_LEN - 1u]) {
        r81_abort(&r81_badsum);
        return;
    }
    r81_ok++;
    r81_last_ms     = timer_elapsed(r81_start);
    r81_last_passes = r81_passes;
    if (r81_passes > r81_max_passes) r81_max_passes = r81_passes;
    r81_state    = R81_IDLE;
    r81_boundary = true;
    if (!is_module_connected || usb_mode || requested_profile != CH582_PROFILE_PEER_24G) return;
    if (r91_pending) {
        r91_dropped++;
        return;
    }
    s          = 0x91;
    r91_buf[0] = 0x91;
    for (uint8_t i = 1; i < RAW81_LEN - 1u; i++) {
        r91_buf[i] = r81_buf[i];
        s += r81_buf[i];
    }
    r91_buf[RAW81_LEN - 1u] = s;
    r91_pending             = true;
}

/* HC_DEVHOOK op 17 (hid_protocol.c). page 0: commits, bad sums, lifetime
 * aborts, UART-taint aborts, 91s sent, replies dropped, ACKs in the window
 * (u32 each, 28 bytes). page 1: [state][boundary][pending][ack window]
 * [quarantined bytes u32]. */
void ch582_raw81_fill(uint8_t page, uint8_t *out28) {
    memset(out28, 0, 28);
    if (page == 0) {
        const uint32_t v[7] = {r81_ok, r81_badsum, r81_lifetime, r81_taint, r91_sent, r91_dropped, r91_acks};
        memcpy(out28, v, sizeof(v));
    } else {
        out28[0] = (uint8_t)r81_state;
        out28[1] = r81_boundary ? 1 : 0;
        out28[2] = r91_pending ? 1 : 0;
        out28[3] = r91_ack_window ? 1 : 0;
        memcpy(&out28[4], &r81_qbytes, 4);
        memcpy(&out28[8], &r81_last_ms, 2);
        out28[10] = r81_last_passes;
        out28[11] = r81_max_passes;
    }
}

void ch582_raw81_clear_counters(void) {
    r81_ok = r81_badsum = r81_lifetime = r81_taint = r81_qbytes = 0;
    r91_sent = r91_dropped = r91_acks = 0;
    r81_last_ms = 0;
    r81_last_passes = r81_max_passes = 0;
}
#endif

#ifdef CONSOLE_ENABLE
/* ACK turnaround, instrumented builds (2026-09-23): docs/wireless.md asks for
 * this before CH582_TX_ACK_TIMEOUT_MS is touched -- ~half of all frames time
 * out at 10 ms and are ACKed after one retransmit. Measured from the FIRST
 * send of a frame, in 1 ms buckets (the last pools >= 15 ms), plus the worst
 * case and the number of retransmits the frame needed. `orphan` counts ACKs
 * that arrive with nothing in flight: if `61 0D 0A` were also a periodic
 * heartbeat (the other reading of the parser's comments), it would show there. */
#define ACK_BUCKETS 16u
static systime_t tx_first_sent_st;
static uint32_t  ack_hist[ACK_BUCKETS], ack_orphan, ack_after_retry, ack_max_us;
#endif

/* Health-counter readout (health.c). Main-loop only, like everything here. */
void ch582_tx_stats(uint32_t *sent, uint32_t *timeouts, uint32_t *drops) {
    *sent     = tx_stat_sent;
    *timeouts = tx_stat_timeout;
    *drops    = tx_stat_drop;
}

void ch582_link_stats(ch582_link_stats_t *out) {
    out->sent         = tx_stat_sent;
    out->timeouts     = tx_stat_timeout;
    out->queue_full   = tx_stat_drop;
    out->giveups      = tx_stat_giveup;
    out->replaced     = tx_stat_replaced;
    out->uart_overrun = uart_err_overrun;
    out->uart_framing = uart_err_framing;
    out->uart_parity  = uart_err_parity;
}

/* Phase 1b, D2 round 2 (flash 1c): WHERE a slow ch582_task call goes. The
 * first battery soak (2026-10-05/06) put the battery-only stalls here --
 * ch582 the largest scope in 4720 of 5494 slow passes, ~7-8 ms each -- while
 * the task's average cost per pass was the same on USB and on battery. So it
 * is an occasional long call, and this splits each call >= PROF_SLOW_TICKS
 * into: the control block (selects, pending actions, the poll, UART flags),
 * the RX drain and parse, the 5C hook (battery_5c_report, its trimmed-mean
 * sort), the 5B/5C acknowledgement writes, and the TX pump. Shares D1's
 * runtime flag (loop_acct_on) and its inline tick (ACCT_NOW); a call longer
 * than the 16-bit tick's 349.5 ms would alias, and is not expected here. */
#define PROF_SLOW_TICKS ((uint16_t)(CH_CFG_ST_FREQUENCY / 250u))   /* 4 ms */
#define PROF_RING       8u
enum { PROF_CTRL, PROF_RX, PROF_C5, PROF_ACK, PROF_TX, PROF_N };
typedef struct {
    uint32_t uptime_ms;
    uint16_t total;
    uint16_t sec[PROF_N];
    uint8_t  bytes;   /* RX bytes this call drained */
    uint8_t  n5c;     /* 5C reports this call parsed */
} prof_entry_t;
static prof_entry_t prof_ring[PROF_RING];
static uint8_t      prof_next;
static uint32_t     prof_slow_calls, prof_slow_sec[PROF_N];   /* ticks, slow calls only */
static uint32_t     prof_c5_reports, prof_c5_ticks;           /* every call */
static uint16_t     prof_c5_max;                              /* the longest one 5C hook */
static uint16_t     prof_call_c5, prof_call_ack;              /* this call */
static uint8_t      prof_call_n5c;

__attribute__((noinline)) static void prof_end(uint16_t p0, uint16_t p1, uint16_t p2, uint8_t bytes) {
    uint16_t now   = ACCT_NOW();
    uint16_t total = (uint16_t)(now - p0);
    if (total < PROF_SLOW_TICKS) return;
    uint16_t rx_all = (uint16_t)(p2 - p1);
    uint16_t hooks  = (uint16_t)(prof_call_c5 + prof_call_ack);
    uint16_t sec[PROF_N];
    sec[PROF_CTRL] = (uint16_t)(p1 - p0);
    sec[PROF_RX]   = rx_all > hooks ? (uint16_t)(rx_all - hooks) : 0u;
    sec[PROF_C5]   = prof_call_c5;
    sec[PROF_ACK]  = prof_call_ack;
    sec[PROF_TX]   = (uint16_t)(now - p2);
    prof_slow_calls++;
    prof_entry_t *e = &prof_ring[prof_next];
    e->uptime_ms = timer_read32();
    if (e->uptime_ms == 0) e->uptime_ms = 1;
    e->total = total;
    for (uint8_t i = 0; i < PROF_N; i++) {
        prof_slow_sec[i] += sec[i];
        e->sec[i] = sec[i];
    }
    e->bytes  = bytes;
    e->n5c    = prof_call_n5c;
    prof_next = (uint8_t)((prof_next + 1u) % PROF_RING);
}

/* HC_LINK pages 0x21-0x2A (hid_protocol.c), 28 bytes, little-endian, ticks at
 * CH_CFG_ST_FREQUENCY. Numbered from 0x21 so no page can equal a protocol
 * version: firmware without the profile echoes the version (8) in that byte.
 *   0x21: u32 slow calls (>= 4 ms), u32 their ticks per section: control,
 *         RX, 5C hook, ack writes, TX pump
 *   0x22: u32 5C reports (every call), u32 their hook ticks, u16 the longest
 *         one, u8 next ring slot, u8 ring length, u16 PROF_SLOW_TICKS
 *   0x23-0x2A: ring slot 0-7: u32 uptime_ms (0: empty), u16 total, u16 x 5
 *         sections, u8 RX bytes, u8 5C reports */
#define PROF_PAGE0 0x21u
bool ch582_prof_fill(uint8_t page, uint8_t *out28) {
    uint8_t *p = out28;
    memset(out28, 0, 28);
#define P32(x) do { uint32_t v_ = (x); *p++ = v_ & 0xFF; *p++ = (v_ >> 8) & 0xFF; *p++ = (v_ >> 16) & 0xFF; *p++ = (uint8_t)(v_ >> 24); } while (0)
#define P16(x) do { uint16_t v_ = (x); *p++ = v_ & 0xFF; *p++ = (uint8_t)(v_ >> 8); } while (0)
    if (page == PROF_PAGE0) {
        P32(prof_slow_calls);
        for (uint8_t i = 0; i < PROF_N; i++) P32(prof_slow_sec[i]);
    } else if (page == PROF_PAGE0 + 1u) {
        P32(prof_c5_reports); P32(prof_c5_ticks); P16(prof_c5_max);
        *p++ = prof_next; *p++ = PROF_RING; P16(PROF_SLOW_TICKS);
    } else if (page >= PROF_PAGE0 + 2u && page < PROF_PAGE0 + 2u + PROF_RING) {
        const prof_entry_t *e = &prof_ring[page - (PROF_PAGE0 + 2u)];
        P32(e->uptime_ms); P16(e->total);
        for (uint8_t i = 0; i < PROF_N; i++) P16(e->sec[i]);
        *p++ = e->bytes; *p++ = e->n5c;
    } else {
        return false;
    }
#undef P16
#undef P32
    return true;
}

static inline uint8_t tx_next(uint8_t i) { return (uint8_t)((i + 1) % CH582_TXQ_LEN); }

/* Drop the head frame (ACKed or given up) and stop waiting on it. */
static void ch582_tx_pop(void) {
    if (tx_head != tx_tail) tx_head = tx_next(tx_head);
    tx_in_flight = false;
    tx_retries   = 0;
}

/* A select or pair (any A6 but the battery poll) is where the link may move:
 * the channel's receiver starts over when one is queued (ch582_send_command)
 * and again whenever it goes out on the wire, first send or retransmit, since
 * old-link LED traffic can arrive while it waits behind other frames (codex,
 * flash 2h verify 2). */
static inline void tx_link_change_check(const ch582_tx_frame_t *f) {
    if (f->len >= 3 && f->data[0] == 0xA6 && f->data[1] != CH582_BATTERY_REQ_PARAM) LOCKCHAN_LINK_LOST();
}

/* Advance the queue: send the head frame, time out and retransmit, or give up.
 * Called every ch582_task() (main-loop cadence, sub-ms), so the 10 ms timeout is
 * honoured with fine granularity. */
static void ch582_tx_pump(void) {
#ifdef AK820_DEV_HOOKS
    /* After a 91, hold the A-frames until its ACK, if any, has had time to
     * come (r81 above): one arriving now must not pop an A1. Nothing is in
     * flight here, since a 91 only goes when the queue is idle. */
    if (r91_ack_window) {
        if (timer_elapsed(r91_sent_at) < RAW91_ACK_WINDOW_MS) return;
        r91_ack_window = false;
    }
#endif
    if (tx_in_flight) {
        if (timer_elapsed(tx_sent_time) < CH582_TX_ACK_TIMEOUT_MS) return;   /* still waiting for ACK */
        tx_stat_timeout++;
        if (++tx_retries > CH582_TX_MAX_RETRIES) {                          /* give up, move on */
            tx_stat_giveup++;
            ch582_tx_pop();
            return;
        }
        tx_link_change_check(&tx_q[tx_head]);
        sdWrite(&CH582_SERIAL_DRIVER, tx_q[tx_head].data, tx_q[tx_head].len); /* retransmit */
        tx_sent_time = timer_read();
        return;
    }
    if (tx_head == tx_tail) {                                                /* queue empty */
#ifdef AK820_DEV_HOOKS
        /* The reply slot goes only now, behind every keyboard frame. The link
         * is checked again: it may have moved since the 81 committed. The same
         * blocking write as every frame; the output queue holds 128 bytes
         * (SERIAL_BUFFERS_SIZE, the common halconf.h), so 34 fit without a
         * wait unless earlier writes still fill it. */
        if (r91_pending) {
            r91_pending = false;
            if (is_module_connected && !usb_mode && requested_profile == CH582_PROFILE_PEER_24G) {
                sdWrite(&CH582_SERIAL_DRIVER, r91_buf, RAW81_LEN);
                r91_sent++;
                r91_sent_at    = timer_read();
                r91_ack_window = true;
            }
        }
#endif
        return;
    }
    tx_link_change_check(&tx_q[tx_head]);
    sdWrite(&CH582_SERIAL_DRIVER, tx_q[tx_head].data, tx_q[tx_head].len);
    tx_sent_time = timer_read();
#ifdef CONSOLE_ENABLE
    tx_first_sent_st = chVTGetSystemTimeX();
#endif
    tx_in_flight = true;
    tx_stat_sent++;
}

/* The parser's 3-byte window starts over at the next ch582_task(): bytes from
 * before a gap in the line (sleep.c's deep sleep, its receiver off) must not
 * pair with bytes after it into a phantom frame. */
static volatile bool rx_resync_req = false;

void ch582_rx_resync(void) {
    rx_resync_req = true;
}

bool ch582_tx_idle(void) {
#ifdef AK820_DEV_HOOKS
    if (r91_pending || r91_ack_window) return false;   /* sleep admission waits for a reply too */
#endif
    return !tx_in_flight && tx_head == tx_tail;
}

/* Deep sleep (sleep.c; plans/DEEP-SLEEP-BUILD-PLAN.md, sections 4-5). At the
 * wake: the parser's window starts over; the last battery report is from
 * before the sleep, so its age reads as none until a new one comes (QMK's
 * timer stood still asleep and would call it fresh); the lock-LED receiver's
 * frame in progress is dropped; and the periodic battery poll waits for
 * sleep.c's input gate, so the waking key's report goes first. Then one poll
 * at once. */
void ch582_sleep_wake(void) {
    rx_resync_req     = true;
    battery_stale     = true;
    battery_poll_held = true;
    LOCKCHAN_LINK_LOST();
}

void ch582_battery_poll_release(void) {
    if (!battery_poll_held) return;
    battery_poll_held = false;
    last_battery_poll = timer_read();
    ch582_poll_status();
}

bool ch582_tx_drain(uint16_t timeout_ms) {
    /* A reset that beats the main loop's pump strands whatever is queued: for
     * the jump, the key releases just queued, which would leave a key down at
     * the BT host for the whole flash (codex, flash 2b finding 1). A give-up
     * is not a delivery, and a full queue's drop never got queued, so either
     * fails the drain. */
    uint32_t giveups = tx_stat_giveup, drops = tx_stat_drop;
    uint16_t t0      = timer_read();
    while (tx_in_flight || tx_head != tx_tail) {
        if (timer_elapsed(t0) >= timeout_ms) return false;
        ch582_task();
    }
    return tx_stat_giveup == giveups && tx_stat_drop == drops;
}

bool ch582_send_releases(void) {
    /* Straight onto the queue, not through clear_keyboard(): QMK skips a
     * report equal to the last one it sent, so after a release that was given
     * up, a second clear_keyboard() queues nothing and an empty drain would
     * pass with the key still down at the host (codex, flash 2b verification).
     * A full queue with no frame of the type to overwrite drops the release
     * here, before any drain could see it (the second verification). */
    static const uint8_t none[8] = {0};
    uint32_t drops = tx_stat_drop;
    ch582_send_command(0xA1, none, 8);   /* keyboard: no modifiers, no keys */
    ch582_send_command(0xA3, none, 2);   /* consumer: usage 0 */
    return tx_stat_drop == drops;
}

/* The module ACKed the in-flight frame -> drop it and let the next one go. Also
 * the first proof the module's UART is up (see module_alive). */
static void ch582_tx_ack(void) {
    module_alive = true;
#ifdef CONSOLE_ENABLE
    if (tx_in_flight) {
        uint32_t us = TIME_I2US(chVTTimeElapsedSinceX(tx_first_sent_st));
        uint32_t ms = us / 1000u;
        ack_hist[ms < ACK_BUCKETS ? ms : ACK_BUCKETS - 1u]++;
        if (us > ack_max_us) ack_max_us = us;
        if (tx_retries) ack_after_retry++;
    } else {
        ack_orphan++;
    }
#endif
#ifdef AK820_DEV_HOOKS
    if (r91_ack_window && !tx_in_flight) r91_acks++;   /* the 91's, if it has one */
#endif
    if (tx_in_flight) ch582_tx_pop();
}

void ch582_send_command(uint8_t cmd, const uint8_t *params, uint8_t param_len) {
    if (param_len > TX_MAX_PAYLOAD) return;
    /* Any select or pair may move the link to another host, so the channel's
     * receiver starts over: covers the deferred ones too (the bounce's real
     * select, select retries, the 5B 23 re-issue, the connect retry), which
     * a reset at the call sites missed (codex, flash 2h verify). The battery
     * poll (A6 53) is not a link change. */
    if (cmd == 0xA6 && param_len >= 1 && params[0] != CH582_BATTERY_REQ_PARAM) LOCKCHAN_LINK_LOST();

    /* NEWEST-SUPERSEDES for STATE frames (audit BW-6). 0xA1 (keyboard) and
     * 0xA3 (consumer) carry absolute state, not events -- the frame contract
     * above says so -- so if one of the same type is already QUEUED (not the
     * in-flight head), the newer state can simply overwrite it in place.
     * Under a typing burst against a stalled link the queue then converges to
     * the true final state instead of filling and DROPPING a release, which
     * was a stuck key over BT (queue-full needed ~23 frames: ACK stall x
     * burst).
     *
     * Gated on the queue being NEARLY FULL, not always-on: coalescing eats
     * intermediate edges (a double-tap queued during a stall collapses into
     * a continuous hold), so while there is room, every edge still ships in
     * order -- delayed-but-complete beats collapsed. Only when the ring is
     * about to overflow does convergence-to-final-state take over, because
     * the alternative at that point is the stuck-key drop.
     *
     * ⚠️ NEVER coalesce 0xA6 (or anything else): the cancel-pairing bounce
     * depends on ORDERED, DISTINCT selects (bounce slot, then target) --
     * collapsing them re-derives the documented same-slot-select no-op trap. */
    uint8_t used = (uint8_t)((tx_tail - tx_head + CH582_TXQ_LEN) % CH582_TXQ_LEN);
    if ((cmd == 0xA1 || cmd == 0xA3) && used >= CH582_TXQ_LEN - 4) {
        /* Overwrite the LAST queued frame of this type, never an earlier one:
         * frames behind an overwritten earlier slot would deliver OLDER state
         * after the newest and the host would end wrong. */
        ch582_tx_frame_t *last = NULL;
        uint8_t i = tx_in_flight ? tx_next(tx_head) : tx_head;
        for (; i != tx_tail; i = tx_next(i)) {
            if (tx_q[i].data[0] == cmd) last = &tx_q[i];
        }
        if (last) {
            uint16_t s = cmd;
            for (uint8_t j = 0; j < param_len; j++) {
                last->data[1 + j] = params[j];
                s += params[j];
            }
            last->data[param_len + 1] = (uint8_t)(s & 0xFF);
            last->len                 = param_len + 2;
            tx_stat_replaced++;
            return;
        }
    }

    uint8_t nxt = tx_next(tx_tail);
    if (nxt == tx_head) { tx_stat_drop++; return; }  /* queue full -> DROPPED KEYSTROKE */

    ch582_tx_frame_t *f = &tx_q[tx_tail];
    uint16_t sum = cmd;
    f->data[0]   = cmd;
    for (uint8_t i = 0; i < param_len; i++) {
        f->data[1 + i] = params[i];
        sum += params[i];
    }
    f->data[param_len + 1] = (uint8_t)(sum & 0xFF);
    f->len                 = param_len + 2;
    tx_tail                = nxt;
#ifdef WDT_TEST_HOOKS
    /* Battery polls (A6 53) are periodic noise; excluding them makes the
     * count an EXACT observable for the pending-action tests. */
    if (cmd == 0xA6 && param_len == 1 && params[0] != 0x53) a6_trace_note(params[0]);
#endif
}

#if CH582_ACK_FRAMES
/* Raw 61 0D 0A ACK token (not a checksummed command, so it bypasses
 * ch582_send_command). Stock replies with this to 5B/5C frames. */
static void ch582_send_ack(void) {
    static const uint8_t ack[3] = {0x61, 0x0D, 0x0A};
    sdWrite(&CH582_SERIAL_DRIVER, ack, sizeof(ack));
}
#endif

void ch582_set_profile(ch582_profile_t profile) {
    uint8_t param        = (uint8_t)profile;
    bool    was_pairing  = (conn_state == CH582_CONN_PAIRING);

    pa.kind             = PA_NONE;  /* a new selection supersedes any pending action */
    requested_profile   = param;
    connect_requested   = true;
    usb_mode            = false;
    is_module_connected = false;
    LOCKCHAN_LINK_LOST();
    conn_state          = CH582_CONN_LINKING;   /* attempting until 5B says otherwise */
    linking_since       = timer_read32();
    last_attempt_time   = timer_read();

    /* CANCEL-PAIRING BOUNCE.
     *
     * Selecting the slot that is CURRENTLY ADVERTISING is a flat no-op -- the
     * module appears to read it as "you are already on that slot". MEASURED
     * 2026-08-29: eight `A6 32` sent over 10 s while advertising on slot 2 drew
     * ZERO responses; the module is entirely silent until it finally emits one
     * `5B 23`. Retrying, on a timer or on that idle event, cannot fix it -- both
     * were tried and both failed.
     *
     * Naming a DIFFERENT slot does force a state change, which is exactly why
     * the manual workaround (Fn+Q then Fn+W) works. So do that: send another
     * slot, wait CH582_BOUNCE_MS, then send the real target.
     *
     * The bounce slot may itself be bonded, so this can briefly attempt a link
     * to the wrong device. It is bounded -- the real select follows 700 ms later
     * and, being a DIFFERENT slot from the bounce, is honoured normally. Only
     * reachable when cancelling pairing, never on an ordinary slot change. */
    if (was_pairing && param >= CH582_PROFILE_BT_1 && param <= CH582_PROFILE_BT_3) {
        uint8_t other = (param == CH582_PROFILE_BT_1)
                            ? (uint8_t)CH582_PROFILE_BT_2
                            : (uint8_t)CH582_PROFILE_BT_1;
        pa.kind      = PA_BOUNCE;    /* no select retries until the real one goes out */
        pa.target    = param;
        pa.last_send = timer_read();
        ch582_send_command(0xA6, &other, 1);
        return;
    }

    pa.kind      = PA_SELECT;
    pa.target    = param;
    pa.tries     = 1;
    pa.last_send = timer_read();
    ch582_send_command(0xA6, &param, 1);
}

/* Pairing command, decoded from stock TX captures: a CONSTANT `0xA6 0x51`, sent
 * twice, which puts the CURRENTLY-SELECTED slot into pairing/advertising. It
 * carries no slot of its own (slot 2 and slot 3 pairing BOTH emitted `A6 51`), so
 * the target slot must already be selected via ch582_set_profile() first. This is
 * why the stock long-press only pairs the active slot. The old 0xA9/0xA1 guess was
 * wrong (0xA1 is the keystroke opcode). */
#define CH582_PAIR_PARAM 0x51

void ch582_enter_pairing(void) {
    uint8_t param       = CH582_PAIR_PARAM;
    is_module_connected = false;
    LOCKCHAN_LINK_LOST();
    is_pairing          = true;
    /* Optimistic: show pairing immediately for feedback, and let the retry below
     * make it true. Waiting for 5B 31 to update the display would leave the band
     * showing "Link failed" for seconds after the user did the right thing. */
    conn_state          = CH582_CONN_PAIRING;
    pa.kind             = PA_PAIR;   /* supersedes a pending select/bounce */
    pa.tries            = 1;
    pa.last_send        = timer_read();
    /* Sent ONCE, matching the @isuua/edthu reference. Stock repeated it, but the
     * ACK/retry queue now guarantees delivery, so one is enough. */
    ch582_send_command(0xA6, &param, 1);
}

void ch582_pair(ch582_profile_t profile) {
    /* Select the slot, then put it into pairing. (On stock, select is the short
     * press and pairing the long press on the same slot; this convenience helper
     * does both for callers that just want "pair this slot now".) */
    ch582_set_profile(profile);
    ch582_enter_pairing();
}

void ch582_cancel_connect(void) {
    connect_requested = false;
    usb_mode          = true;
    conn_state        = CH582_CONN_IDLE;
    LOCKCHAN_LINK_LOST();
}

bool ch582_is_connected(void) {
    return is_module_connected;
}

uint8_t ch582_get_slot(void) {
    return connected_slot;
}

bool ch582_is_24g(void) {
    return !usb_mode && requested_profile == CH582_PROFILE_PEER_24G;
}

bool ch582_is_usb(void) {
    return usb_mode;
}

bool ch582_is_pairing(void) {
    return is_pairing;
}

uint8_t ch582_get_battery(void) {
    return battery_level;
}

/* Lowest percent ever reported since boot, or 0xFF if the module has never
 * answered at all. Still 100 after weeks of use means either the cell has never
 * meaningfully discharged or the number is not tracking anything. */
uint8_t ch582_battery_min(void) {
    return battery_seen ? battery_min : 0xFF;
}

/* Milliseconds since the last 5C frame. UINT32_MAX if the module has never sent
 * one -- the caller must not render that as an age. */
uint32_t ch582_battery_age_ms(void) {
    if (!battery_seen || battery_stale) return UINT32_MAX;
    return timer_elapsed32(battery_last_recv);
}

/* Count of 5C frames received since boot, wrapping. A change means a NEW
 * report, even when it repeats the last value -- which is what a filter needs:
 * re-reading ch582_get_battery() only duplicates the one it already has. */
uint16_t ch582_battery_reports(void) {
    return battery_reports;
}

uint8_t ch582_get_host_leds(void) {
    return host_leds;
}

/* Map a selected 0xA6 profile byte (0x31..0x33) to a 1..3 slot; 0 for 2.4G/none. */
static uint8_t profile_to_slot(uint8_t profile) {
    if (profile >= CH582_PROFILE_BT_1 && profile <= CH582_PROFILE_BT_3) {
        return profile - 0x30;
    }
    return 0;
}

ch582_conn_state_t ch582_get_conn_state(void) {
    return conn_state;
}

/* Slot to display: the slot we are aiming for, valid while linking/pairing (not
 * just when connected). 2.4G shows as slot 1; USB/none is 0. */
uint8_t ch582_get_target_slot(void) {
    if (usb_mode) return 0;
    if (requested_profile == CH582_PROFILE_PEER_24G) return 1;
    return profile_to_slot(requested_profile);
}

void ch582_task(void) {
    WDT_SCOPE(WDT_SITE_WIRELESS);
    const bool prof = loop_acct_on;   /* D2's profile: see prof_end() */
    uint16_t   p0   = prof ? ACCT_NOW() : 0u;
    /* Rolling 3-byte window over the RX stream. The module interleaves two frame
     * formats with no length prefix, and the stream can drop bytes on a burst, so
     * a fixed [type,data,cksum] state machine desyncs permanently. Matching on a
     * sliding window instead self-resynchronises and tolerates dropped bytes.
     *
     * Logic-analyzer-decoded against stock firmware. The ONLY trustworthy
     * connection signals are the 5B state transitions; everything else (61 0D 0A,
     * 5C battery, 5B 23) is periodic and streams regardless of link state:
     *   - 61 0D 0A   -> "a\r\n": the per-frame ACK (see the parser below). An
     *                   older reading called it a periodic idle heartbeat; the
     *                   2026-09-23 measurement says ACK -- one ACK arrived with
     *                   nothing in flight in ~2,200 frames (docs/wireless.md).
     *                   Either way, ignore it for connection state.
     *   - 5A <led>   -> host keyboard LED bitmap (USB LED bits; bit1 = caps lock)
     *   - 5B <code>  -> connection state machine (code is a STATE, not a slot):
     *        32 = link established (connected);  31 = advertising/pairing;
     *        33/34 = connect ATTEMPT (link down, retrying);  23 = idle (ignore,
     *        it appears both connected and disconnected).
     *   - 5C <pct>   -> battery percent in decimal (0x64=100). PERIODIC and link-
     *                   independent (streams even while disconnected) -> never use
     *                   it as a "connected" signal. */
    static uint8_t b2 = 0, b1 = 0, b0 = 0;
    /* In every build: a deep sleep's wake starts the window over (codex,
     * deep-sleep review 24: draining the UART alone left the history stale). */
    if (rx_resync_req) {
        rx_resync_req = false;
        b2 = b1 = b0 = 0;
#ifdef AK820_DEV_HOOKS
        r81_reset();
#endif
    }

    /* Cold-boot-only select retry. The mode slider is wired directly to the
     * CH582F, so once the module is up it handles advertising/reconnect itself
     * (like the @isuua/edthu reference, which sends the select ONCE per switch).
     * We must NOT keep re-issuing the select on a live module: re-selecting
     * restarts its advertising and starves slow reconnects -- a phone reconnects
     * in <500 ms and beats the retry, but macOS directed-advertising takes longer
     * and never completes, so it needed a manual pairing entry. The only case
     * that genuinely needs a retry is cold boot, where the boot-time select fires
     * before the CH582F's UART is up and is dropped. So retry ONLY until the
     * module is alive (has ACKed anything), then stop and let it reconnect at its
     * own pace. Transient (non-boot) select drops are covered by the ACK/retry
     * queue instead. */
    if (connect_requested && !module_alive &&
        timer_elapsed(last_attempt_time) >= CH582_CONNECT_RETRY_MS) {
        last_attempt_time = timer_read();
        uint8_t param = requested_profile;
        ch582_send_command(0xA6, &param, 1);
    }

    /* Drive the pending control action -- one switch instead of three
     * independent timer blocks. Cadences, budgets and clearing rules are the
     * table in findings-ch582-states.md. */
    switch (pa.kind) {
        case PA_BOUNCE:
            /* Second half of the cancel-pairing bounce: the real select. */
            if (timer_elapsed(pa.last_send) >= CH582_BOUNCE_MS) {
                pa.kind      = PA_SELECT;
                pa.tries     = 1;
                pa.last_send = timer_read();
                uint8_t p = pa.target;
                ch582_send_command(0xA6, &p, 1);
            }
            break;
        case PA_SELECT:
            if (pa.tries >= CH582_SELECT_MAX_TRIES) {
                pa.kind = PA_NONE;
            } else if (timer_elapsed(pa.last_send) >= CH582_SELECT_CONFIRM_MS) {
                pa.last_send = timer_read();
                pa.tries++;
                uint8_t param = requested_profile;
                ch582_send_command(0xA6, &param, 1);
            }
            break;
        case PA_PAIR:
            /* Resend A6 51 until the module confirms with 5B 31 -- it drops
             * the pair command while its own connect attempt is in flight.
             * Bounded so a module that never answers does not retry forever. */
            if (pa.tries >= CH582_PAIR_MAX_TRIES) {
                pa.kind = PA_NONE;
            } else if (timer_elapsed(pa.last_send) >= CH582_PAIR_RETRY_MS) {
                pa.last_send = timer_read();
                pa.tries++;
                uint8_t param = CH582_PAIR_PARAM;
                ch582_send_command(0xA6, &param, 1);
            }
            break;
        case PA_NONE:
        default:
            break;
    }

    /* Periodically poll the module for battery level, as the stock firmware does.
     * Whether the module would keep sending 5C without the poll is untested, so
     * the poll stays. Poll regardless of connection mode: the gauge is valid in BT,
     * 2.4G and USB-charging states, and A6 53 is a status query that doesn't touch
     * the link. */
    /* Report TX health alongside the battery poll (every 5 s), and only when
     * something went wrong -- silence means the link is clean. */
    {
        static uint32_t seen_timeout = 0, seen_drop = 0;
        static uint16_t stat_timer = 0;
        if (timer_elapsed(stat_timer) >= 5000) {
            stat_timer = timer_read();
            if (tx_stat_timeout != seen_timeout || tx_stat_drop != seen_drop) {
                printf("[ch582] sent=%lu ack-timeouts=%lu dropped=%lu\n",
                       (unsigned long)tx_stat_sent,
                       (unsigned long)tx_stat_timeout,
                       (unsigned long)tx_stat_drop);
                seen_timeout = tx_stat_timeout;
                seen_drop    = tx_stat_drop;
            }
#ifdef CONSOLE_ENABLE
            static uint8_t ack_report = 0;
            if (++ack_report >= 12) {                      /* once a minute */
                ack_report = 0;
                printf("[ch582] ack ms:");
                for (uint8_t i = 0; i < ACK_BUCKETS; i++) printf(" %lu", (unsigned long)ack_hist[i]);
                printf(" | max=%luus retried=%lu orphan=%lu | uart ovr=%lu fe=%lu pe=%lu\n",
                       (unsigned long)ack_max_us, (unsigned long)ack_after_retry,
                       (unsigned long)ack_orphan, (unsigned long)uart_err_overrun,
                       (unsigned long)uart_err_framing, (unsigned long)uart_err_parity);
            }
#endif
        }
    }

    /* First poll fires as soon as the module answers ANYTHING, not a full
     * CH582_BATTERY_POLL_MS after boot.
     *
     * last_battery_poll starts at 0 and timer_elapsed() is measured from boot,
     * so the old form waited out the whole 5 s interval before asking even
     * once -- which is exactly the "battery appears a few seconds late on every
     * boot" JD noticed. Asking earlier than module_alive is pointless:
     * at cold boot the module's UART is not up yet and the frame is dropped
     * (the same reason the connect select needs its own cold-boot retry). */
    if (battery_poll_held) {
        /* a deep sleep's wake: sleep.c releases it */
    } else if (module_alive && !battery_seen) {
        if (timer_elapsed(last_battery_poll) >= CH582_BATTERY_FIRST_POLL_MS) {
            last_battery_poll = timer_read();
            ch582_poll_status();
        }
    } else if (timer_elapsed(last_battery_poll) >= CH582_BATTERY_POLL_MS) {
        last_battery_poll = timer_read();
        ch582_poll_status();
    }

    {
        eventflags_t f = uart_err_poll();
#ifdef AK820_DEV_HOOKS
        /* A byte lost under a collection, or one older than its lifetime:
         * abort before another byte is taken (r81 above). */
        /* Remembered in every state until the line is next silent: an error
         * from a burst whose 81 is still in the queue must taint the
         * collection it starts (codex stage-1 review, should-fix 2). */
        if (f) r81_tainted = true;
        if (r81_state == R81_COLLECT) {
            if (f) r81_abort(&r81_taint);
            else if (timer_elapsed(r81_start) >= RAW81_LIFETIME_MS) r81_abort(&r81_lifetime);
        }
#else
        (void)f;
#endif
    }

    uint8_t c;
    uint8_t bytes_processed = 0;
#ifdef AK820_DEV_HOOKS
    const uint16_t drain_t0 = timer_read();
    bool           rx_left  = false;   /* the drain stopped on its budget, bytes still queued */
    if (r81_state == R81_COLLECT && r81_passes < 255u) r81_passes++;
#endif
    uint16_t p1 = prof ? ACCT_NOW() : 0u;
    prof_call_c5 = prof_call_ack = 0;
    prof_call_n5c = 0;

    /* Drain the queue fully (capped to bound worst-case latency) so a connect
     * burst doesn't overflow the serial input buffer and shed bytes. */
    while (bytes_processed < 64) {
#ifdef AK820_DEV_HOOKS
        /* Mid-collection, the drain gives the pass back after RAW81_DRAIN_MS:
         * draining one 81 took 8-9 ms and held one pass for all of it. The
         * rest waits in the input queue (128 bytes) for the next pass; the
         * collection's lifetime spans the passes. ⚠️ Bytes are left queued,
         * and an ACK among them must be consumed before the TX pump judges
         * anything, so the pump skips this pass (rx_left, below). */
        if (r81_state == R81_COLLECT && timer_elapsed(drain_t0) >= RAW81_DRAIN_MS) {
            rx_left = true;
            break;
        }
#endif
#ifdef WDT_TEST_HOOKS
        /* Fault injection (instrumented builds): bytes queued via
         * ch582_inject() are consumed BEFORE real UART bytes, as if received
         * from the module -- so a host script can replay every wire capture
         * in findings-ch582-states.md (missed 5B 32, the advertising
         * decline, byte soup) against the REAL parser and assert the
         * resulting state over raw HID. */
        extern bool ch582_inject_pop(uint8_t *out);
        if (ch582_inject_pop(&c)) {
            bytes_processed++;
        } else
#endif
        if (chnReadTimeout(&CH582_SERIAL_DRIVER, &c, 1, TIME_IMMEDIATE) == 0) {
#ifdef AK820_DEV_HOOKS
            /* The queue is empty: if nothing has come for RAW81_SILENCE_MS, the
             * line was silent that long, so a quarantine ends and the next byte
             * starts at a boundary (r81 above). A collection is left to its
             * lifetime. */
            if (r81_state != R81_COLLECT && timer_elapsed(rx_last_byte_at) >= RAW81_SILENCE_MS) {
                /* Leaving quarantine, the window starts over too: bytes from
                 * before the abort must not pair with bytes after it into a
                 * phantom ACK or 5x (codex stage-1 review, must-fix 1). */
                if (r81_state == R81_QUARANTINE) b2 = b1 = b0 = 0;
                r81_state    = R81_IDLE;
                r81_boundary = true;
                r81_tainted  = false;
            }
#endif
            break;
        }
#ifdef WDT_TEST_HOOKS
        else if (ch582_rx_muted) { bytes_processed++; continue; } /* discard real byte */
#endif
        else {
            bytes_processed++;
#ifdef AK820_DEV_HOOKS
            /* Here, not after the if-chain: only a real, unmuted byte from the
             * module (codex rx-tap should-fix 2: an injected byte falls through
             * to below in a build with WDT_TEST_HOOKS too). */
            rxtap_note(c);
#endif
        }
#ifdef AK820_DEV_HOOKS
        /* The 81 collector (r81 above), before the window sees the byte. */
        rx_last_byte_at = timer_read();
        if (r81_state == R81_QUARANTINE) {
            r81_qbytes++;
            continue;
        }
        if (r81_state == R81_COLLECT) {
            r81_buf[r81_len++] = c;
            if (r81_len == RAW81_LEN) r81_commit();
            continue;
        }
        if (r81_boundary && c == 0x81) {
            /* The window starts over at a collection: a pending `61 0D` (after
             * a silence) must not meet a `0A` that follows the whole 81 and
             * forge an ACK, which could release an A1 (codex stage-1 review,
             * must-fix 1). */
            b2 = b1 = b0 = 0;
            r81_buf[0] = c;
            r81_len    = 1;
            r81_start  = timer_read();
            r81_passes = 1;
            if (r81_tainted) {
                r81_abort(&r81_taint);   /* from a burst already known corrupt */
                continue;
            }
            r81_state = R81_COLLECT;
            continue;
        }
#endif

        b2 = b1;
        b1 = b0;
        b0 = c;

        bool matched = false;

        if (b2 == 0x61 && b1 == 0x0D && b0 == 0x0A) {
            /* `61 0D 0A` ("a\r\n") is the per-frame ACK the module returns for what
             * we send -- NOT a connection signal (it is silent at idle). Use it to
             * release the in-flight TX frame so the next queued one can go. */
            ch582_tx_ack();
            matched             = true;
        } else if ((b2 == 0x5A || b2 == 0x5B || b2 == 0x5C) &&
                   b0 == ((uint8_t)(b2 + b1))) {
            uint8_t type       = b2;
            uint8_t d          = b1;
            switch (type) {
                case 0x5A: /* host LED bitmap (caps lock = bit1) */
                    /* The 1-byte additive checksum is weak, so the mixed RX stream
                     * (notably the CH582F power-up / 2.4G link-up burst) can throw a
                     * bogus 5A frame that spuriously lights Caps. Guard twofold:
                     *   - only honor LED frames once a link exists, and
                     *   - require a plausible HID LED bitmap (low 5 bits only:
                     *     num/caps/scroll/compose/kana). Random garbage that happens
                     *     to pass the checksum usually has high bits set. */
#ifdef AK820_DEV_HOOKS
                    led5a_note(d, is_module_connected && (d & ~0x1F) == 0);
                    if (is_module_connected && !usb_mode) lockchan_feed(&lockchan, d, timer_read32());
#endif
                    if (is_module_connected && (d & ~0x1F) == 0) {
                        host_leds = d;
                    } else if ((d & ~0x1F) == 0 && conn_state == CH582_CONN_LINKING &&
                               timer_elapsed32(linking_since) >= CH582_5A_PROMOTE_MS) {
                        /* Recovery for a missed `5B 32` -- see CH582_5A_PROMOTE_MS.
                         * Adopt the link, but deliberately NOT this frame's LED bits:
                         * the reason 5A is distrusted here at all is that a forged
                         * frame can pass the weak checksum, and lighting Caps off the
                         * very frame that establishes the link is the failure the
                         * guard above exists to prevent. A real host repeats its LED
                         * state, so the next 5A takes the normal path.
                         * Pending actions as `5B 32` leaves them: a select retry
                         * is retired, a bounce survives (codex, flash 2h verify 2:
                         * a retry left pending could reselect a live link). */
                        if (pa.kind != PA_BOUNCE) pa.kind = PA_NONE;
                        is_module_connected = true;
                        is_pairing          = false;
                        conn_state          = CH582_CONN_CONNECTED;
                        connected_slot      = profile_to_slot(requested_profile);
                    }
                    break;
                case 0x5B: /* connection state code (NOT a slot number) */
                    switch (d) {
                        case 0x32: /* link established - the only "connected" signal */
                            /* A PA_BOUNCE survives: this 5B 32 may be the BOUNCE
                             * slot linking up mid-window, and the real target's
                             * select must still follow at the 700 ms mark --
                             * clearing it would strand the user on the wrong
                             * slot. (The triplet version's 5B 32 case never
                             * touched bounce_pending either.) */
                            if (pa.kind != PA_BOUNCE) pa.kind = PA_NONE;
                            is_module_connected = true;
                            is_pairing          = false;
                            conn_state          = CH582_CONN_CONNECTED;
                            connected_slot      = profile_to_slot(requested_profile);
                            /* A link-up is a new link even if its down state was
                             * lost: the channel starts over. Sent once at link-up
                             * (docs/wireless.md), so it never cuts a live frame
                             * short (codex, flash 2h verify 3). */
                            LOCKCHAN_LINK_LOST();
                            break;
                        case 0x31: /* advertising / pairing, waiting for a device */
                            /* PA_BOUNCE survives here too, same reason as 32. */
                            if (pa.kind != PA_BOUNCE) pa.kind = PA_NONE;
                            is_pairing          = true;
                            is_module_connected = false;
                            conn_state          = CH582_CONN_PAIRING;
                            connected_slot      = 0;
                            host_leds           = 0; /* no link -> drop stale LED state */
                            LOCKCHAN_LINK_LOST();
                            break;
                        case 0x33: /* connect ATTEMPT - link is down and retrying */
                        case 0x34:
                            /* The module IS attempting: never re-issue the select
                             * now, or a slow (macOS directed-advertising)
                             * reconnect gets restarted and never completes. A
                             * pending PAIR survives -- the module ignores A6 51
                             * mid-attempt, which is why that retry exists. */
                            if (pa.kind == PA_SELECT) pa.kind = PA_NONE;
                            is_module_connected = false;
                            is_pairing          = false;
                            if (conn_state != CH582_CONN_LINKING) linking_since = timer_read32();
                            conn_state          = CH582_CONN_LINKING;
                            connected_slot      = 0;
                            host_leds           = 0; /* no link -> drop stale LED state */
                            LOCKCHAN_LINK_LOST();
                            break;
                        /* 0x36: connect attempt ABANDONED. Absent from
                         * CH582F_PROTOCOL.md's state table; the disassembly's
                         * "host refused" reading is a guess. CAPTURED ON THE WIRE
                         * 2026-08-29 selecting an unreachable slot:
                         *   5B 33, 5B 33, 5B 36, then 5B 23 (idle, ignored)
                         * so it follows failed attempts and then PERSISTS -- the
                         * module never retracts it. Note this is "the attempt
                         * failed", NOT "not paired": a bonded but powered-off
                         * host would look identical. Do not label it as bonding. */
                        case 0x36:
                            is_module_connected = false;
                            is_pairing          = false;
                            conn_state          = CH582_CONN_REJECTED;
                            connected_slot      = 0;
                            host_leds           = 0;
                            LOCKCHAN_LINK_LOST();
                            break;
                        default:
                            /* 0x23 idle/finalize: periodic, appears both
                             * connected and disconnected -> leave state.
                             *
                             * But it IS the moment the module stops advertising
                             * and becomes receptive again, so a select it
                             * declined can be re-issued right here. Measured: a
                             * select sent while advertising is ignored, and the
                             * 5B 23 lands ~6 s later. Retrying on the clock
                             * alone kept missing this window. */
                            if (pa.kind == PA_SELECT && d == 0x23 &&
                                pa.tries < CH582_SELECT_MAX_TRIES) {
                                pa.last_send = timer_read();
                                pa.tries++;
                                uint8_t p = requested_profile;
                                ch582_send_command(0xA6, &p, 1);
                            }
                            break;
                    }
                    break;
                case 0x5C: /* battery: a linear voltmeter on the pack (battery.c), 2-3
                            * a poll cycle in any link state -> do NOT touch
                            * connection state */
                    if (d <= 100) {
                        battery_level     = d;
                        battery_last_recv = timer_read32();
                        battery_stale     = false;
                        battery_reports++;
                        if (prof) {
                            uint16_t h0 = ACCT_NOW();
                            battery_5c_report(d);   /* every report, not the last of a burst */
                            uint16_t dh = (uint16_t)(ACCT_NOW() - h0);
                            prof_call_c5 = (uint16_t)(prof_call_c5 + dh);
                            prof_c5_ticks += dh;
                            prof_c5_reports++;
                            if (dh > prof_c5_max) prof_c5_max = dh;
                            prof_call_n5c++;
                        } else {
                            battery_5c_report(d);
                        }
                        battery_seen      = true;
                        /* Lowest ever seen, for the debug page. The module is the
                         * only thing that can read the cell, and we have no idea
                         * how good its estimate is -- almost certainly a voltage
                         * reading, and lithium voltage is flat through the middle
                         * of the curve. A board that spends its life on the cable
                         * may report 100 forever, and a min that never moves off
                         * 100 is the cheapest possible evidence of that. */
                        if (d < battery_min) battery_min = d;
                    }
                    break;
            }
#if CH582_ACK_FRAMES
            /* Mirror stock: ack 5B (connection state) and 5C (battery), never 5A. */
            if (type == 0x5B || type == 0x5C) {
                if (prof) {
                    uint16_t a0 = ACCT_NOW();
                    ch582_send_ack();
                    prof_call_ack = (uint16_t)(prof_call_ack + (uint16_t)(ACCT_NOW() - a0));
                } else {
                    ch582_send_ack();
                }
            }
#endif
            matched = true;
        }

        if (matched) {
            /* Clear the window so the consumed bytes can't form a phantom overlap
             * match with the next byte. */
            b2 = b1 = b0 = 0;
        } else if ((b2 == 0x5A || b2 == 0x5B || b2 == 0x5C) &&
                   b0 != (uint8_t)(b2 + b1) &&
                   b0 != 0x5A && b0 != 0x5B && b0 != 0x5C && b0 != 0x61) {
            /* A frame-shaped window with a WRONG checksum, whose tail byte
             * cannot itself begin a new frame -- the signature of corrupted
             * UART traffic (what the 2026-08-29 priority inversion produced).
             * Count it: the counter is a TREND instrument, not an exact frame
             * count -- payload bytes can pose as type bytes, and a VALID frame
             * preceded by one type-looking noise byte also counts once before
             * parsing correctly (the window only protects a candidate starting
             * at b0, not b1). Expect a small false-positive rate; read trends,
             * not values. Main-loop context only (audit C-4).
             * (Audit finding IV-2, findings-input-validation.md.) */
            health_note_rx_malformed();
        }
#ifdef AK820_DEV_HOOKS
        /* A boundary only right after a frame or ACK the window completed. */
        r81_boundary = matched;
#endif
    }

    /* Drive the reliable TX queue: send/retransmit/drop the in-flight frame.
     * AFTER the RX drain, not before it (codex review, 2026-09-23): an ACK
     * already sitting in the input queue must release the in-flight frame
     * before its timeout is judged. Pumped first, a buffered ACK could not
     * stop a needless retransmit, whose own ACK then arrived with nothing in
     * flight (an orphan) -- or, worse, popped the NEXT frame, since `61 0D 0A`
     * carries no sequence number. */
    uint16_t p2 = prof ? ACCT_NOW() : 0u;
#ifdef AK820_DEV_HOOKS
    /* An ACK may still be queued behind the collection's tail: judging the
     * in-flight frame now could retransmit it, and that copy's later ACK would
     * release the NEXT frame unsent (wireless.md; codex raw81 drain review,
     * must-fix 1). Likewise the 91's window must not close with its ACK
     * unread. So the pump waits for a drain that reached the queue's end:
     * the next pass, ~3 ms on. */
    if (!rx_left)
#endif
    ch582_tx_pump();
#ifdef AK820_DEV_HOOKS
    lockchan_poll(&lockchan, timer_read32(), is_module_connected && !usb_mode);
#endif
    if (prof) prof_end(p0, p1, p2, bytes_processed);
}
