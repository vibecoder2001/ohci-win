# ohci-win — Open-source OHCI UCX miniport for Windows

A UCX-based OHCI (USB 1.1) host controller driver for Windows, primarily
targeting Rockchip RK3588 on Windows on ARM, with an x86 QEMU test vehicle.
Replaces the in-box `USBOHCI.SYS` to fix the EHCI → OHCI companion handoff
on WoA.

See `docs/superpowers/specs/2026-04-22-ohci-ucx-driver-design.md` for the
design document.

## Status

Under active development. v1 scope: Control / Bulk / Interrupt transfers;
Low and Full speed devices; no iso, no remote wakeup.

## Build prerequisites

- Windows 10/11 x64 development machine
- EWDK (Enterprise WDK) — https://aka.ms/ewdk
- CMake 3.22+ (for the usermode test harness)
- QEMU for Windows (for Tier 2 tests)

## Layout

- `core/` — `OhciCore.lib`, buildable as both kernel static lib and usermode
- `func/shared/` — bus-agnostic OHCI/UCX glue (DMA, MMIO, ISR/DPC, UCX
  controller/root-hub/usb-device/endpoint callbacks, isoc) shared by
  every bus-specific function driver below
- `func/acpi/` — ACPI function driver (ARM64 / RK3588)
- `func/pci/` — PCI function driver (x86 QEMU)
- `inf/` — driver INFs and catalog
- `test/harness/` — Tier 1 usermode harness
- `test/devfuzz/` — AFL++-compatible fuzzer
- `test/e2e/` — Tier 2 / 3 end-to-end tests
- `tools/` — build scripts, QEMU launcher, VM automation

## License

MIT. See `LICENSE`.
