# ChibiOS patches — the submodule branch is now authoritative

As of 2026-09-01 the hand-applied ChibiOS patches are **committed** in
`lib/chibios-contrib` on the local branch **`ak820pro-patches`**, on top of
the upstream pin `5bed8690`, and the superproject gitlink pins that branch's
tip. The branch carries:

- the seven patches below;
- the `SN32F290.ld` `.ram7` reservation for the watchdog record;
- `spi0_dispatch` (`2a17a73b48`) and its revert (`bf9310ca84`), see below;
- three fixes from 2026-09-23 (details in
  `plans/FIRMWARE-FINDINGS-2026-09-23.md`):
  - `a3fdffe26d`: the serial LLD's `load()` no longer unlocks inside its
    caller's lock (it could swap two bytes of a CH582F frame);
  - `c57623d0d2`: the SPI0 DMA handler no longer erases a completion that
    races its flag clear. That is every "never started" blit timeout on
    record.
  - `c212e20dd2`: the USB LLD's `usb_lld_start_in()` has the same
    lock-nesting fix as the serial LLD.
- two from 2026-09-24:
  - `f247ebc639`: the EFL driver refuses to program a line that is not
    erased;
  - `a4f8412134`: the serial LLD reports a hardware receive overrun (`LS.OE`,
    a byte lost with the FIFO full) as `SD_OVERRUN_ERROR`, which it raised
    before only for a full input queue (HC_LINK's `uart_overrun`);
- one from 2026-10-18: `27e852d25a`, `rtc_lld_add_secondsI()`, which the
  deep sleep calls under the lock to hand the RTC driver's calendar the
  seconds it counted asleep with their interrupts cleared unserved
  (`sleep.c` through `rtc_account_sleepI`).

`git submodule update` can no longer silently destroy them — it will detach
to the pinned (patched) commit instead.

The `.diff` files in this directory are kept as documentation and as the
recovery path. There are **seven**: `spi_dma_abort.diff` was added
2026-08-31 with the blit-timeout work. The `.ram7` linker commit and every
commit from 2026-09-23 on have no `.diff` here; take them from the branch
(pushed, below). The bundle predates the 2026-09-24 and 2026-10-18 commits.
Apply order (each `git apply` from the chibios-contrib root):

```
hardware_pwm -> i2c_fallback -> rtc_lld -> spi_fifo_pump
             -> spi_flash_dma -> spi_dma_abort -> efl_ramtext
             -> (SN32F290.ld .ram7 commit)
             -> (2026-09-23: serial load(), SPI0 lost completion, USB start_in)
             -> (2026-09-24: EFL erased check, serial hardware overrun)
             -> (2026-10-18: RTC add_secondsI)
```

`spi_fifo_pump` / `spi_flash_dma` / `spi_dma_abort` touch the same SPI LLD
file and must stay in that order; `efl_ramtext` is required for VIA.

⚠️ **`spi0_dispatch` was tried and reverted (2026-09-23).** It routed the
SPI0 handler on `sn32_dma_busy` instead of the raw `RIS & 0x30`, and cleared
only the DMA flags it read, to close a lost-completion race and a FIFO
interrupt a stale DMA flag could hijack. Its first hunt, on the daily build,
produced six "unknown" blit timeouts in 21 minutes, a class the old
dispatch never produced. Each was a DMA that started and never delivered its
completion, plus a 1.95 s main-loop stall. The hazards it aimed at are
still open; the patch is in the branch history (`2a17a73b48`).

Recovery if the branch is ever lost: a git bundle of it lives at
`ak820pro-builds/chibios-ak820pro-patches.bundle`
(`git fetch <bundle> ak820pro-patches`), and the flattened applied state at
`ak820pro-builds/chibios-applied-state-backup-2026-09-01.diff`.

The branch is pushed to https://github.com/jdlien/ChibiOS-Contrib
(`ak820pro-patches`), which `.gitmodules` now points at, so a fresh clone's
`git submodule update` fetches the pinned patched commit directly.
