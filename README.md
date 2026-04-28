# 🚀✨ ohci-win — Open-source OHCI UCX Miniport for Windows ✨🚀

Welcome to **ohci-win** — a UCX-based OHCI host controller driver for Windows,
created because **OHCI is not optional on EHCI companion-controller platforms**.
Yes, it is USB 1.1. Yes, it is old. And yes — it is still required when EHCI
hands low-speed and full-speed devices off to its companion controller. 🔌⚡

This project primarily targets **Rockchip RK3588 running Windows on ARM**, with
an **x86 QEMU test vehicle** for faster iteration, debugging, and validation.

On platforms where EHCI owns the high-speed path, low-speed and full-speed USB
devices are handled by the companion OHCI controller. That means mice,
keyboards, hubs, USB audio devices, and full-speed storage can depend on a
working OHCI stack. So this is not just “retro USB archaeology” — this is the
missing piece that makes the EHCI companion handoff path usable on Windows on
ARM. 😤🧠🔌

In practical terms, this driver replaces the in-box `USBOHCI.SYS` in order to
fix the **EHCI → OHCI companion handoff** path on WoA platforms where the stock
driver does not properly handle the hardware configuration.

The original design spec lives outside the committed tree, because apparently
even USB host controller drivers can have lore. The public repo is therefore
structured so the source layout, comments, tests, and driver components explain
the architecture directly. 🤖📚✨

## 📊 Status

This project is under **active development** — meaning the driver is rapidly
evolving from “why is USB doing that?” to “wait, this actually works?” 🚀

### ✅ Validated on RK3588 real ARM hardware

The driver has been validated on actual RK3588 hardware with real USB devices,
not just a suspiciously cooperative emulator.

Currently validated on **OHCI**:

- ✅ Composite USB mouse
- ✅ Composite USB keyboard
- ✅ USB headset playback through Windows inbox USB audio
- ✅ USB headset microphone capture through Windows inbox USB audio
- ✅ USB storage through EHCI → OHCI companion handoff

The USB audio path is handled by the **Windows inbox USB audio driver** on top
of this OHCI miniport, which means the host-controller behavior is being tested
through the normal Windows USB stack — including isochronous IN and OUT
transfers. 🎧🎙️

USB storage was already working on EHCI, but is now also validated on **OHCI**
thanks to an EHCI stub that properly hands low/full-speed devices off to the
OHCI companion controller. 💾🔌

### ✅ Validated in QEMU

The x86 QEMU test vehicle currently validates:

- ✅ USB mouse
- ✅ USB keyboard
- ✅ USB storage
- ✅ USB audio
- ✅ USB hub

QEMU remains useful for fast iteration, regression testing, and generally
finding out whether the driver is broken before asking real hardware to suffer.

### 🚧 Current driver scope

Current scope includes:

- ✅ Control transfers
- ✅ Bulk transfers
- ✅ Interrupt transfers
- ✅ Isochronous transfers
- ✅ Low-speed devices
- ✅ Full-speed devices
- ✅ Composite devices
- ✅ EHCI companion handoff support
- 🚧 Remote wakeup support

## 🛠️ Build Prerequisites

To build this very serious, extremely necessary, definitely-not-pointless USB
host controller adventure, you will need:

- **Windows 10/11 x64 development machine**
- **EWDK / Enterprise WDK** — https://aka.ms/ewdk
- **CMake 3.22+** — for the usermode test harness
- **QEMU for Windows** — for Tier 2 tests and emulator-based validation

## 🧱 Repository Layout

The repository is organized into clean, purposeful, highly intentional layers —
because even an EHCI companion controller deserves a proper software stack. 🤖

- `core/`  
  Contains `OhciCore.lib`, buildable as both a kernel static library and a
  usermode library for testing. This is where the core OHCI logic lives.

- `func/shared/`  
  Bus-agnostic OHCI/UCX glue shared by all function drivers, including DMA,
  MMIO, ISR/DPC, UCX controller callbacks, root-hub callbacks, USB-device
  callbacks, endpoint callbacks, and isochronous transfer plumbing.

- `func/acpi/`  
  ACPI function driver targeting ARM64 platforms, especially RK3588.

- `func/pci/`  
  PCI function driver used by the x86 QEMU test vehicle.

- `inf/`  
  Driver INFs and catalog files — because Windows driver installation requires
  the correct ceremony.

- `test/harness/`  
  Tier 1 usermode harness for fast validation of core logic without immediately
  entering kernel-mode chaos.

- `test/devfuzz/`  
  AFL++-compatible fuzzing support for catching USB weirdness before USB
  weirdness catches you.

- `test/e2e/`  
  Tier 2 and Tier 3 end-to-end tests for QEMU and real-hardware validation.

- `tools/`  
  Build scripts, QEMU launcher helpers, VM automation, and other useful tools
  for keeping the development loop moving.

## 🧪 Testing Philosophy

The testing strategy is intentionally layered, because USB drivers are where
“works once” goes to become “why did unplugging a hub at the wrong time summon
a bug from another dimension?” 🧪🔌

The project uses:

- Usermode core tests for fast logic validation
- Fuzzing for descriptor and state-machine abuse
- QEMU end-to-end tests for repeatable VM validation
- Real RK3588 hardware tests for actual controller behavior
- Windows inbox class drivers wherever possible, so the miniport has to behave
  like a real host controller instead of relying on custom shortcuts

The goal is not merely to make one keyboard blink. The goal is to make Windows’
normal USB stack believe this is a real, usable OHCI controller — because it is.

## ⚠️ Notes

This driver is still under active development. Expect sharp edges, changing
interfaces, and the occasional USB device that decides to personally disagree
with the implementation.

That said, the major transfer types are now functional across QEMU and/or real
RK3588 hardware, including isochronous audio playback and microphone capture
through Windows’ inbox USB audio stack.

## 📜 License

MIT. See `LICENSE`.

Because low-speed and full-speed USB still need to go somewhere when EHCI says:
“not my problem.” ✨