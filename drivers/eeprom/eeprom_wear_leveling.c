// Copyright 2022 Nick Brassel (@tzarc)
// SPDX-License-Identifier: GPL-2.0-or-later
#include <stdint.h>
#include <string.h>

#include "eeprom_driver.h"
#include "wear_leveling.h"

void eeprom_driver_init(void) {
    wear_leveling_init();
}

void eeprom_driver_format(bool erase) {
    /* wear leveling requires the write log data structures to be erased before use. */
    (void)erase;
    eeprom_driver_erase();
}

void eeprom_driver_erase(void) {
    wear_leveling_erase();
}

void eeprom_read_block(void *buf, const void *addr, size_t len) {
    wear_leveling_read((uint32_t)addr, buf, len);
}

/* Board hook, called before every write into the emulated EEPROM with its
 * logical address -- before wear_leveling_write() decides whether anything
 * changed. A board can attribute the physical writes that follow to the
 * eeconfig region being written (the AK820 Pro counts which writer blinks its
 * LEDs). Default is an empty no-op, so nothing changes for other boards. */
__attribute__((weak)) void eeprom_wear_leveling_write_hook(uint32_t addr, const void *buf, size_t len) {
    (void)addr;
    (void)buf;
    (void)len;
}

void eeprom_write_block(const void *buf, void *addr, size_t len) {
    eeprom_wear_leveling_write_hook((uint32_t)addr, buf, len);
    wear_leveling_write((uint32_t)addr, buf, len);
}
