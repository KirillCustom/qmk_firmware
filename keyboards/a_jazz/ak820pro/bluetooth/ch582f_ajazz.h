#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "report.h"

typedef enum {
    CH582_PROFILE_PEER_24G = 0x30, /* '0' */
    CH582_PROFILE_BT_1     = 0x31, /* '1' */
    CH582_PROFILE_BT_2     = 0x32, /* '2' */
    CH582_PROFILE_BT_3     = 0x33, /* '3' */
    CH582_PROFILE_PAIR_24G = 0x35  /* '5' */
} ch582_profile_t;

/* Wireless connection state, driven by the module's 5B frames. */
typedef enum {
    CH582_CONN_IDLE = 0,   /* not attempting (USB, or nothing requested) */
    CH582_CONN_LINKING,    /* attempting to (re)connect: 5B 33/34 */
    CH582_CONN_PAIRING,    /* advertising for a new device: 5B 31 */
    CH582_CONN_CONNECTED,  /* linked: 5B 32 */
    CH582_CONN_REJECTED,   /* host refused the connection: 5B 36 */
} ch582_conn_state_t;

/* Init/task are provided through QMK's Bluetooth driver API (bluetooth_init /
 * bluetooth_task in ch582f_ajazz.c); ch582_task() is the internal RX/poll pump
 * called from bluetooth_task(). */
void ch582_task(void);
void ch582_set_profile(ch582_profile_t profile);
void ch582_pair(ch582_profile_t profile);
/* Put the CURRENTLY-SELECTED slot into pairing (stock `A6 51`, slotless). Call
 * ch582_set_profile() first to choose the slot. */
void ch582_enter_pairing(void);
void ch582_cancel_connect(void);
void ch582_poll_status(void);
void ch582_send_command(uint8_t cmd, const uint8_t *params, uint8_t param_len);
void ch582_send_keyboard_report(report_keyboard_t *report);
/* Pump the TX queue until it is empty, for at most timeout_ms. False on the
 * timeout, or if any frame was given up rather than ACKed meanwhile. For the
 * raw-HID bootloader jump (hid_protocol.c). */
bool ch582_tx_drain(uint16_t timeout_ms);
/* Nothing queued and nothing awaiting an ACK (a sleep's precondition). */
bool ch582_tx_idle(void);
/* Start the RX parser's window over (after a gap in the line). */
void ch582_rx_resync(void);
/* A deep sleep's wake (sleep.c): the parser resynced, the last battery report
 * marked stale, the lock-LED receiver's frame dropped, the periodic battery
 * poll held until ch582_battery_poll_release() (which sends one at once). */
void ch582_sleep_wake(void);
void ch582_battery_poll_release(void);
/* Queue fresh all-released keyboard (A1) and consumer (A3) frames, whatever
 * QMK last sent. False if a full queue dropped either. For the same jump. */
bool ch582_send_releases(void);
bool ch582_is_connected(void);
bool ch582_is_pairing(void);
/* True when the selected profile is the 2.4GHz dongle (not a BT slot). */
bool ch582_is_24g(void);
/* True when the mode slider is in the USB position (wireless not in use). */
bool ch582_is_usb(void);
uint8_t ch582_get_slot(void);
/* Detailed connection state (5B-driven), for the LCD status indicator. */
ch582_conn_state_t ch582_get_conn_state(void);
/* Slot number we are on / aiming for (1-3 BT, 1 for 2.4G, 0 = none), valid even
 * while linking/pairing (unlike ch582_get_slot, which is 0 until connected). */
uint8_t ch582_get_target_slot(void);

/* Battery level reported by the module (0-100, 0xFF = unknown). From 5C frames. */
uint8_t ch582_get_battery(void);
/* Lowest percent ever reported since boot; 0xFF if the module has never
 * answered. Debug-page only -- see the note at the 0x5C parse site. */
uint8_t  ch582_battery_min(void);
/* ms since the last 5C frame; UINT32_MAX if there has never been one. */
uint32_t ch582_battery_age_ms(void);
/* 5C frames received since boot, wrapping: a change is a new report. */
uint16_t ch582_battery_reports(void);
/* Host LED bitmap last reported by the module (USB LED bits; bit1 = caps). From 5A frames. */
uint8_t ch582_get_host_leds(void);
#ifdef AK820_DEV_HOOKS
/* The inbound 5A trace (HC_DEVHOOK ops 7-9). fill: [frames u32][accepted u32]
 * [host_leds][last byte][ms since the last frame u32, UINT32_MAX if none]
 * [ring size] at out[0..14]. page: [page][0], then slots page*13 .. page*13+12
 * as {byte, gap ms} pairs, 28 bytes; false past the last page. */
void ch582_led5a_fill(uint8_t *out);
bool ch582_led5a_page(uint8_t page, uint8_t *out);
void ch582_led5a_reset(void);
/* The RX tap (HC_DEVHOOK ops 15-16): every raw byte from the module while
 * armed. arm(true) clears and starts it, arm(false) stops it. fill: [armed]
 * [bytes since arming u32][ring size u16] at out[0..6]. page: [page][0], then
 * slots page*26 .. page*26+25, 28 bytes; false past the last page. */
void ch582_rxtap_arm(bool armed);
void ch582_rxtap_fill(uint8_t *out);
bool ch582_rxtap_page(uint8_t page, uint8_t *out);
/* The lock-LED channel's receiver (HC_DEVHOOK ops 10-13). */
#include "lockchan.h"
const lockchan_t *ch582_lockchan(void);
void ch582_lockchan_clear(void);
bool ch582_lockchan_holds_scroll(void);
#else
static inline bool ch582_lockchan_holds_scroll(void) { return false; }
#endif

/* Cumulative TX health counters (sent / ACK timeouts / queue-full drops). */
void ch582_tx_stats(uint32_t *sent, uint32_t *timeouts, uint32_t *drops);

/* Every way the link can lose input, since boot (Phase 1b, D1). Each can lose
 * a keystroke with no >= 25 ms main-loop gap, so they are read beside the
 * stall counters, never inferred from them. Plain counters, every build. */
typedef struct {
    uint32_t sent;          /* frames first sent */
    uint32_t timeouts;      /* ACK waits that expired (each retransmits or gives up) */
    uint32_t queue_full;    /* frames refused by a full TX queue (tx_stat_drop) */
    uint32_t giveups;       /* frames abandoned after CH582_TX_MAX_RETRIES */
    uint32_t replaced;      /* queued 0xA1/0xA3 state frames overwritten by a newer one */
    uint32_t uart_overrun;  /* task passes that saw each UART error flag */
    uint32_t uart_framing;
    uint32_t uart_parity;
} ch582_link_stats_t;
void ch582_link_stats(ch582_link_stats_t *out);
/* D2 round 2 (flash 1c): the profile of slow ch582_task calls, HC_LINK pages
 * 0x21-0x2A (layout at ch582_prof_fill in the .c). False for an unknown page. */
bool ch582_prof_fill(uint8_t page, uint8_t *out28);

#ifdef WDT_TEST_HOOKS
/* Fault injection (instrumented builds): queue bytes the parser consumes as
 * if received from the module. See findings-ch582-states.md. */
void ch582_inject(const uint8_t *bytes, uint8_t len);
/* Outbound A6 trace: copies the last (<=16) A6 params into out, newest last;
 * returns the entry count and writes the running total to *count. Lets host
 * tests observe the pending-action machinery (supersession, bounce order,
 * retry cadence), not just the rx-driven state. */
uint8_t ch582_a6_trace(uint8_t *out, uint16_t *count);
/* Discard real UART RX while true, so tests see only injected bytes. */
void ch582_rx_mute(bool on);
#endif
