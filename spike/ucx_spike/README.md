# OhciSpike — Phase 0 UCX spike driver

Purpose: answer the Phase 0 gate question — **will `Ucx01000.sys` accept a
non-xHCI client at `UcxControllerCreate`?**

This is a throwaway probe driver. It attaches to a contrived PCI ID
(`PCI\VEN_106B&DEV_003F&CC_0C0310` — an Apple OHCI device class), calls
`UcxInitializeDeviceInit` + `UcxControllerCreate`, logs the resulting
`NTSTATUS` via `DbgPrintEx`, and otherwise does nothing. It does not drive
hardware, does not create a root hub, and does not register endpoints.

**NOT FOR PRODUCTION. Test-signed only. Kernel debugger must be attached
to observe the log line — that log line IS the experimental result.**

## Header research (WDK 10.0.26100.0, UCX **1.6**)

Read against the Microsoft headers under
`D:\Windows Kits\10\Include\10.0.26100.0\km\ucx\1.6\`.
No Microsoft source is reproduced here — only the extracted API shape.

### 1. `UCX_CONTROLLER_CONFIG` (ucxcontroller.h)

Fields, in order:

| Field                                                    | Type                                                            |
|----------------------------------------------------------|-----------------------------------------------------------------|
| `Size`                                                   | `ULONG` (self-size)                                             |
| `NumberOfPresentedDeviceMgmtEvtCallbacks`                | `ULONG` (initialized to `(ULONG)-1` by `_INIT`)                 |
| `EvtControllerQueryUsbCapability`                        | `PFN_UCX_CONTROLLER_QUERY_USB_CAPABILITY`                       |
| `Reserved1`                                              | `HANDLE`                                                        |
| `EvtControllerGetCurrentFrameNumber`                     | `PFN_UCX_CONTROLLER_GET_CURRENT_FRAMENUMBER`                    |
| `EvtControllerUsbDeviceAdd`                              | `PFN_UCX_CONTROLLER_USBDEVICE_ADD` (**required, `__notnull`**)  |
| `EvtControllerReset`                                     | `PFN_UCX_CONTROLLER_RESET`                                      |
| `Reserved2` / `Reserved3` / `Reserved4`                  | `HANDLE`                                                        |
| `ParentBusType`                                          | `UCX_CONTROLLER_PARENT_BUS_TYPE` enum                           |
| `PciDeviceInfo`                                          | `UCX_CONTROLLER_PCI_INFORMATION`                                |
| `AcpiDeviceInfo`                                         | `UCX_CONTROLLER_ACPI_INFORMATION`                               |
| `DeviceDescription[MAX_GENERIC_USB_CONTROLLER_NAME_SIZE]`| `UCHAR[40]`                                                     |
| `ManufacturerNameString` / `ModelNameString` / `ModelNumberString` | `UNICODE_STRING`                                      |
| `EvtControllerGetTransportCharacteristics`               | `PFN_UCX_CONTROLLER_GET_TRANSPORT_CHARACTERISTICS`              |
| `EvtControllerSetTransportCharacteristicsChangeNotification` | `PFN_UCX_CONTROLLER_SET_TRANSPORT_CHARACTERISTICS_CHANGE_NOTIFICATION` |
| `Reserved5` / `Reserved6` / `Reserved7`                  | `HANDLE`                                                        |

**Finding 6 up-front — critical to the gate question:**
`UCX_CONTROLLER_CONFIG` has **no controller-type field**. There is no
`ControllerType`, no `InterfaceVersion` discriminator, no class GUID, and
no HC-type enum. The only type-adjacent field is `ParentBusType`
(`UcxControllerParentBusTypeCustom/Pci/Acpi/MaUsb`), which describes the
*bus* the controller sits on, not the *HC class* (xHCI/EHCI/OHCI/UHCI).

### 2. `UCX_CONTROLLER_CONFIG_INIT`

```c
VOID FORCEINLINE
UCX_CONTROLLER_CONFIG_INIT(
    __out PUCX_CONTROLLER_CONFIG Config,
    __in  LPCSTR                 DeviceDescription
);
```

Behaviour: zeros the struct, sets `Size`, sets
`NumberOfPresentedDeviceMgmtEvtCallbacks = (ULONG)-1`, sets
`ParentBusType = UcxControllerParentBusTypeCustom`, clears PCI/ACPI IDs,
copies `DeviceDescription` into the 40-byte `DeviceDescription` field.

There is no HC-type argument. The initializer cannot refuse a non-xHCI
caller because it has no way to know who's calling.

There are also two companion helpers:
- `UCX_CONTROLLER_CONFIG_SET_PCI_INFO(Config, Vid, Did, Rev, Bus, Dev, Fn)` — sets `ParentBusType = Pci` and populates `PciDeviceInfo`.
- `UCX_CONTROLLER_CONFIG_SET_ACPI_INFO(Config, Vid, Did, Rev)` — sets `ParentBusType = Acpi` and populates `AcpiDeviceInfo`.

### 3. `UcxControllerCreate`

```c
NTSTATUS FORCEINLINE
UcxControllerCreate(
    __in     WDFDEVICE                Device,
    __in     PUCX_CONTROLLER_CONFIG   Config,
    __in_opt PWDF_OBJECT_ATTRIBUTES   Attributes,
    __out    UCXCONTROLLER*           Controller
);
```

Implemented as a thunk through `UcxFunctions[UcxControllerCreateTableIndex]`
(the global dispatch table populated by the UCX class extension when the
driver binds). SAL-only requirement: `Config->EvtControllerUsbDeviceAdd`
must be non-NULL, and `Config->Size` must be non-zero. No type parameter.

### 4. `UcxInitializeDeviceInit`

```c
NTSTATUS FORCEINLINE
UcxInitializeDeviceInit(
    __inout PWDFDEVICE_INIT DeviceInit
);
```

Must be called before `WdfDeviceCreate`, at `PASSIVE_LEVEL`. Wires the
driver up as a UCX class-extension client on this device. Again: no HC
type parameter.

### 5. Event callback typedefs referenced by `UCX_CONTROLLER_CONFIG`

All are stubbed in the spike:

| Callback typedef                                          | Return |
|-----------------------------------------------------------|--------|
| `EVT_UCX_CONTROLLER_QUERY_USB_CAPABILITY`                 | NTSTATUS |
| `EVT_UCX_CONTROLLER_GET_CURRENT_FRAMENUMBER`              | NTSTATUS |
| `EVT_UCX_CONTROLLER_USBDEVICE_ADD`                        | NTSTATUS (**required by SAL**) |
| `EVT_UCX_CONTROLLER_RESET`                                | VOID |
| `EVT_UCX_CONTROLLER_GET_TRANSPORT_CHARACTERISTICS`        | NTSTATUS |
| `EVT_UCX_CONTROLLER_SET_TRANSPORT_CHARACTERISTICS_CHANGE_NOTIFICATION` | VOID |

### 6. Controller-type enum / GUID survey

There is exactly one `CONTROLLER_TYPE` enum in the UCX 1.6 public headers,
defined in `ucxroothub.h`:

```c
typedef enum _CONTROLLER_TYPE {
    ControllerTypeXhci = 0,
    ControllerTypeSoftXhci,
} CONTROLLER_TYPE;
```

**Values: only xHCI and SoftXhci.** No OHCI, no EHCI, no UHCI, no "other",
no reserved escape hatch.

It lives on `ROOTHUB_INFO`, which is returned **out** from the client's
`EvtRootHubGetInfo` callback when UCX asks for root-hub metadata. It is
not set on the controller-create path; it's set when UCX asks the *client*
to describe its root hub. So strictly speaking, `UcxControllerCreate`
itself never sees the type.

**Phase 0 implications:**
- `UcxControllerCreate` has no type argument and accepts no type identifier
  on its config struct. There is no compile-time rejection of a
  "non-xHCI client."
- Whether `Ucx01000.sys` *runtime-rejects* a client that never declares
  itself xHCI, or one that would later declare `ROOTHUB_INFO.ControllerType
  = <anything but Xhci/SoftXhci>`, is the empirical question this spike
  was built to answer.
- We set up only the controller (no root hub). The first test point is:
  does `UcxControllerCreate` itself return `STATUS_SUCCESS`? If yes, UCX
  accepts clients without ever knowing or caring about HC class. If no,
  the returned NTSTATUS will tell us *why* — and that's still a useful
  result, because we learn whether UCX gates on anything it can check at
  this call site (caller context, driver binding metadata, I/O target,
  etc.).
- **Known unknown — post-`UcxControllerCreate` callbacks:** if
  `UcxControllerCreate` succeeds, UCX may immediately (or shortly after
  the device enters the started state) invoke subsequent event callbacks
  such as `EvtControllerUsbDeviceAdd` or `EvtControllerQueryUsbCapability`.
  The spike stubs for those return `STATUS_NOT_IMPLEMENTED`. Whether UCX
  treats that as a fatal condition (bugcheck / device removal) or as a
  graceful per-callback failure is a secondary unknown. Task 7's runbook
  should attach WinDbg *before* unloading the driver so that any
  `DRIVER_VERIFIER_DETECTED_VIOLATION` or unexpected callback invocation
  is captured in the live debugger rather than post-mortem.

## Build notes

- UCX 1.6 headers: `D:\Windows Kits\10\Include\10.0.26100.0\km\ucx\1.6`
- UCX import lib: `D:\Windows Kits\10\Lib\10.0.26100.0\km\x64\ucx\1.6\ucxstub.lib` (**not** `ucx01000.lib` as the plan text anticipated; WDK 10.0.26100.0 ships it as `ucxstub.lib`).
- WDK 10.0.26100.0 does not define a `$(UcxDriver)=true` switch in
  `WindowsDriver.KernelMode.CX.Default.props` (only Mbb/Net/Spb/Ucm/UcmTcpci/Urs/Ude/Wifi are listed). The vcxproj therefore hard-codes the UCX include path and lib.

Deviations from the plan's vcxproj template, encountered during the build:

1. Removed `<TimeStamp>$(INF_TIMESTAMP)</TimeStamp>` on the `<Inf>` item —
   with `INF_TIMESTAMP` unset, stampinf gets `-v` with no argument and
   returns error 87. Leaving the item default (no explicit `<TimeStamp>`)
   lets stampinf use `-v "*"` (current time) and that works.
2. Replaced `DriverVer = ; stampinf will fill` in the INX with a concrete
   `01/01/2026,1.0.0.0` placeholder — stampinf rewrites it anyway.
3. Added `TargetOSVersion` decorations (`.10.0...16299`) to the INX's
   `[Manufacturer]`/`[Standard.NTamd64]` sections. Required because
   `DestinationDirs = 13` (DIRID 13 = driver package directory) needs the
   install to target 10.0.16299 or later — InfVerif error 1199 without it.
4. Added `<DisableSpecificWarnings>4201</DisableSpecificWarnings>` to
   `<ClCompile>`. The UCX 1.6 headers use nameless structs/unions, which
   the default `/W4 /WX` would reject.
5. Do **not** add `_KERNEL_MODE` to `<PreprocessorDefinitions>` — the
   `WindowsKernelModeDriver10.0` toolset already defines it, and adding
   it again trips C4117 (reserved macro name), which `/WX` escalates.
6. Added a small `AddDriverBinaryToPackage` target that injects
   `$(TargetPath)` into `@(FilesToPackage)`. The default
   `GetPackageFiles` only auto-adds the INF; without this, inf2cat runs
   against a package dir that contains only the `.inf` and emits
   "22.9.1: ohcispike.sys in [ohcispike_copyfiles] ... is missing".

## Files

- `driver.c` — the probe itself.
- `OhciSpike.inx` — INF template. Minimal class=USB install for a contrived PCI match.
- `OhciSpike.vcxproj` — KMDF driver project, x64+ARM64, WDK 10.0.26100.0.

## Running the probe

1. Build: `pwsh .\tools\build_driver.ps1 -Project spike\ucx_spike\OhciSpike.vcxproj -Arch x64 -Config Debug`
2. Test-sign the `.sys` on the target VM (out of scope for this driver).
3. Create a virtual PCI device matching `VEN_106B&DEV_003F&CC_0C0310`, or
   adjust the INX to match something present.
4. Attach WinDbg with `DbgPrintEx` filter for `IHVDRIVER` and look for the
   line:
   ```
   OhciSpike: UcxControllerCreate -> 0x%08X  (THIS IS THE GATE)
   ```
   The hex value is the verdict.
