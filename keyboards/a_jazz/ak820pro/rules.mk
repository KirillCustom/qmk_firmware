# The GC9107 panel + dashboard are driven entirely bare-metal (graphics/lcd_bus.c):
# we own SPI0 (+ Vector58) for the interrupt-driven flash->LCD DMA, and decode QP's
# qgf/qff asset blobs ourselves. No Quantum Painter runtime -- its concurrent main-loop
# activity corrupted the background DMA. See docs/LCD_FLASH_LAYER.md.
SRC   += graphics/lcd_bus.c

# CH582F wireless module exposed through QMK's official Bluetooth driver API.
# BLUETOOTH_DRIVER = custom defines BLUETOOTH_ENABLE, CONNECTION_ENABLE and
# NO_USB_STARTUP_CHECK and compiles bluetooth.c (weak bluetooth_* defaults);
# our ch582f_ajazz.c (below in SRC) provides the strong overrides.
BLUETOOTH_ENABLE = yes
BLUETOOTH_DRIVER = custom

# External PCF8563 RTC over the ChibiOS software (bit-banged) I2C fallback LLD.
# Swaps the SN32 HW I2C driver for the SW fallback; rtc.c drives it via the I2C HAL
# API. Working on hardware since 2026-08 (docs/clock.md). The bit-banged
# transaction runs with interrupts masked, ~1 ms per byte-sized transfer.
USE_HAL_I2C_FALLBACK = yes

SRC += bluetooth/ch582f_ajazz.c

# Dashboard graphics now live in EXTERNAL FLASH, not firmware. Generate the blob
# with `python3 graphics/res/mkraw.py --flash` and upload it once per keyboard:
#   ak820ctl flash write 0x0CE0000 graphics/res/flash_assets.bin
# The firmware reads the index at boot (flash_assets_init) and DMA-draws by id.

SRC += graphics/display.c
SRC += rtc/rtc.c
SRC += watchdog.c   # hardware WDT: arm/kick/boot accounting/bootloader stop
SRC += watchdog_record.c # reset-retained operation breadcrumbs
SRC += fault.c      # HardFault / unhandled-vector / halt: terminal records
SRC += health.c     # unified health counters; raw HID channel 0x13
SRC += kb_eeconfig.c # owner of the persisted kb datablock
SRC += bt_ui.c      # wireless slider, BT slot keys, hold-to-pair
SRC += consumer_mod.c # modified-consumer endpoint-ordering sequencer
SRC += param_overlay.c # RGB hold-to-repeat + parameter readout overlay
SRC += indicators.c  # Caps/WinLock/Charging PWM, 20 kHz tick, lock states
SRC += battery.c     # VDD from the SN32 ADC (AIN16 against VDD)
SRC += power.c       # power caps: the low-battery lights cut; the idle ladder only with POWER_LADDER_ENABLE
SRC += hid_protocol.c # raw-HID channels 0x10-0x13 + VIA/non-VIA dispatch
VPATH += bluetooth
VPATH += graphics
VPATH += rtc

# Board-local RGB effects (rgb_matrix_kb.inc): RAINFALL
RGB_MATRIX_CUSTOM_KB = yes

# Debounce: per-key deferred. NOT the QMK default (sym_defer_g), NOT eager.
#
# sym_defer_g's real defect is that its timer is GLOBAL: any key changing
# restarts it, and the commit compares FINAL matrix state, so a key that goes
# down and back up before the WHOLE matrix is quiet for DEBOUNCE ms leaves
# raw == cooked and the press is discarded. QMK tests exactly this case
# (quantum/debounce/tests/sym_defer_g_tests.cpp:63). It is invisible to
# health.c's key_presses, which lives in process_record_kb, AFTER debounce.
#
# ⚠️ BUT the rate of that swallow was OVERSTATED here on 2026-09-03 and the
# claim is retracted: the global timer restarts on a raw STATE CHANGE, not on
# every scan. Sustaining it needs ~185 raw transitions/s; real typing at
# 10-15 keys/s produces 20-30 edges/s, tens of ms apart. It is an edge case,
# not the demonstrated cause of the dropped characters. See
# plans/review-codex-sol-2026-09-03.md.
#
# sym_defer_pk removes the cross-key coupling -- each key has its own counter,
# so one key's activity can no longer extend or batch another's -- while still
# requiring a stable press per key. Per-key also removes the batched commit
# that discarded key ORDER (matrix_task emits in row/col order, not press
# order), which is the transposition mechanism.
#
# asym_eager_defer_pk was tried and REVERTED. Eager reports the first closure
# immediately, so on a worn switch an isolated noise closure becomes a phantom
# tap or a false hold -- a worse failure than a drop on a board where JD has
# replaced corroded switches before. It also delays release ~5-10 ms, which can
# flip tap-hold decisions near TAPPING_TERM.
#
# Do NOT shorten DEBOUNCE below 5 without switch traces.
DEBOUNCE_TYPE = sym_defer_pk
