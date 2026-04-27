# OhciPci — PCI function driver

KMDF driver that binds to QEMU's `pci-ohci` and drives it through the
`OhciCore` library.

## Build

```
pwsh ..\..\tools\build_driver.ps1 -Project OhciPci.vcxproj -Arch x64 -Config Debug
```

Produces `OhciPci.sys`, `OhciPci.inf`, `OhciPci.cat`, `OhciPci.cer` under
`x64\Debug\`.

## Install in a Win11 x64 QEMU VM

See `docs\superpowers\runbooks\plan4-vm-smoke.md` for the full flow. Short
version: `bcdedit /set testsigning on`, install the test cert, then
`pnputil /add-driver OhciPci.inf /install`.

## Architecture

This driver is a thin kernel adapter. The bus-agnostic OHCI/UCX glue
lives in `func/shared/` (compiled directly into this vcxproj via
`..\shared\*.c`); the transfer logic lives in `core/`. The driver:

1. Parses PCI BAR + interrupt resources into a `DEVICE_CONTEXT`
2. Allocates a DMA-coherent buffer via `WDFCOMMONBUFFER`
3. Wires both into the core lib's `ohci_mmio_ops` and `ohci_dma_region`
4. Calls `ohci_hc_init` to drive the controller into Operational state
5. Connects an ISR/DPC; the DPC calls `ohci_drain_done`
6. Calls `UcxControllerCreate` so UCX recognises the device

USB device enumeration is Plan 5 (UcxRootHub + endpoint wiring).
