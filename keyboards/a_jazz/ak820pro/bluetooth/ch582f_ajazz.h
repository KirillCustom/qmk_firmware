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

/* Cumulative TX health counters (sent / ACK timeouts / queue-full drops). */
void ch582_tx_stats(uint32_t *sent, uint32_t *timeouts, uint32_t *drops);

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
