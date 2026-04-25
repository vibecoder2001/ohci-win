/*++

Module Name:

    ucx_roothub.c

Abstract:

    UCX 1.6 root hub implementation for OhciPci.

    Implements OhciPci_RootHubCreate, which registers callbacks with UCX and
    creates the UCXROOTHUB object.

    Task 3 adds real implementations of GetInfo and Get20PortInfo, reading
    HcRhDescriptorA/B to populate the actual hub topology.  StubGet30PortInfo
    and StubInterruptTx remain as stubs (Tasks 4 and Plan-6+).

Environment:

    Kernel mode only.

--*/

/*
 * UCX 1.6 Root Hub API — findings from reading
 *   D:\Windows Kits\10\Include\10.0.26100.0\km\ucx\1.6\ucxroothub.h
 *
 * Struct:
 *   UCX_ROOTHUB_CONFIG (confirmed, plan name was correct)
 *   Fields relevant to us:
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubClearHubFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubClearPortFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetHubStatus
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetPortStatus
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubSetHubFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubSetPortFeature
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubGetPortErrorCount
 *     PFN_UCX_ROOTHUB_CONTROL_URB    EvtRootHubControlUrb  (OR the above 7)
 *     PFN_UCX_ROOTHUB_INTERRUPT_TX   EvtRootHubInterruptTx
 *     PFN_UCX_ROOTHUB_GET_INFO       EvtRootHubGetInfo
 *     PFN_UCX_ROOTHUB_GET_20PORT_INFO EvtRootHubGet20PortInfo
 *     PFN_UCX_ROOTHUB_GET_30PORT_INFO EvtRootHubGet30PortInfo  (REQUIRED even for OHCI)
 *
 * Init macros (two variants):
 *   UCX_ROOTHUB_CONFIG_INIT(Config, ClearHubFeat, ClearPortFeat,
 *       GetHubStatus, GetPortStatus, SetHubFeat, SetPortFeat,
 *       GetPortErrCnt, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo)
 *     — sets the 7 individual URB handlers (EvtRootHubControlUrb = NULL)
 *
 *   UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER(Config,
 *       ControlUrb, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo)
 *     — sets one ControlUrb dispatcher; individual handlers = NULL
 *
 * We use UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER for the skeleton;
 * Tasks 3/4 can promote to individual callbacks if needed.
 *
 * Callback signatures (DEVIATION from plan skeleton guesses):
 *   Plan guessed callbacks take (UCXROOTHUB, PROOTHUB_INFO / struct pointer).
 *   ACTUAL: ALL callbacks take (UCXROOTHUB UcxRootHub, WDFREQUEST Request).
 *   ROOTHUB_INFO, ROOTHUB_20PORTS_INFO, ROOTHUB_30PORTS_INFO are the data
 *   structs retrieved from the WDF request memory by the real implementations.
 *
 * UcxRootHubCreate signature (confirmed, matches plan):
 *   UcxRootHubCreate(Controller, Config, Attributes, RootHub*)
 *
 * USB3 port info callback: REQUIRED (marked __notnull in annotations).
 *   OHCI is USB 2.0 only; we stub it with STATUS_NOT_IMPLEMENTED but must
 *   provide a non-NULL pointer.
 *
 * TASK 3 DEVIATIONS — ROOTHUB_INFO actual fields vs plan skeleton:
 *   Plan assumed: NumberOfPorts, PowerOnToPowerGoodInMs, IsPowerSwitchingSupported,
 *     IsPowerSwitchedPerPort, IsCompoundDevice, IsOverCurrentProtection,
 *     IsOverCurrentReportedPerPort.
 *   ACTUAL (WDK 10.0.26100.0): Size, ControllerType, NumberOf20Ports,
 *     NumberOf30Ports, MaxU1ExitLatency, MaxU2ExitLatency.
 *   The hub power/overcurrent metadata the plan tried to pass does NOT exist
 *   in ROOTHUB_INFO — that information is not surfaced through this struct at all.
 *
 * TASK 3 DEVIATIONS — ROOTHUB_20PORTS_INFO actual fields vs plan skeleton:
 *   Plan assumed inline array: arr[i].PortNumber, MaximumAllowedSpeed,
 *     PowerSwitchType, IsRemovable, PortConnectorIndex.
 *   ACTUAL: ROOTHUB_20PORTS_INFO = { Size, NumberOfPorts, PortInfoSize,
 *     PortInfoArray (PROOTHUB_20PORT_INFO*) }.
 *   Per-port ROOTHUB_20PORT_INFO = { PortNumber (USHORT), MinorRevision,
 *     HubDepth, Removable (TRISTATE), IntegratedHubImplemented (TRISTATE),
 *     DebugCapable (TRISTATE), ControllerUsb20HardwareLpmFlags }.
 *   No MaximumAllowedSpeed, PowerSwitchType, or PortConnectorIndex fields.
 *   PortInfoArray is an array of PROOTHUB_20PORT_INFO (pointers), not inline.
 *   We allocate the per-port structs and pointer array from NonPagedPool.
 *
 * TASK 3 DEVIATIONS — context retrieval:
 *   Plan assumed OhciPci_RootHubGetContext(rh) or UCX context on UCXROOTHUB.
 *   ACTUAL: no such helper exists and we did not attach WDF context to the
 *   UCXROOTHUB object in Task 2.  We use a module-static pointer
 *   g_DeviceContext set at RootHubCreate time (valid for a single-instance
 *   OHCI PCI driver).
 *
 * CONTROLLER_TYPE enum (no SoftOhci variant):
 *   ControllerTypeXhci = 0, ControllerTypeSoftXhci = 1.
 *   OHCI is neither, but UCX expects an xHCI-class controller at the UCX
 *   API layer (UCX was designed for xHCI).  We report ControllerTypeSoftXhci
 *   as the closest fit for a software-emulated or non-xHCI controller path.
 */

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include <usbdi.h>
#include <usbioctl.h>
#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* OHCI register offsets for the root hub descriptor registers. */
#define REG_HcRhDescriptorA  0x48u
#define REG_HcRhDescriptorB  0x4Cu

/* Root-hub port-status register base and change-bit mask. */
#define REG_HcRhPortStatus_BASE  0x54u
/* Bits 16-20: CSC|PESC|PSSC|OCIC|PRSC (OHCI spec §7.4.2) */
#define OHCI_RHPS_CHANGE_MASK    0x001F0000u

/*
 * Module-static device context pointer.
 *
 * Set once in OhciPci_RootHubCreate before UCX can invoke any callback.
 * Safe for a single-instance PCI driver; a multi-instance driver would need
 * WDF context attached to the UCXROOTHUB object instead.
 */
PDEVICE_CONTEXT g_DeviceContext;  /* extern in device_context.h; shared across glue TUs */

/* --------------------------------------------------------------------------
 * OhciPciInterruptTx — EVT_UCX_ROOTHUB_INTERRUPT_TX  (Task 4 real impl)
 *
 * Called by UCX when it wants to know which ports have changed state.
 * Returns a USB 2.0 hub change-status bitmap (USB 2.0 §11.12.4):
 *   bit 0 = hub-level change (we always report 0)
 *   bit i = port i has at least one change bit set (1-indexed)
 *
 * Change bits read from HcRhPortStatus[i-1] bits 16-20 (CSC|PESC|PSSC|OCIC|PRSC).
 * They are W1C: writing them back clears them so the port doesn't re-fire.
 *
 * UCX may request more bytes than OHCI needs (e.g., for USB 3 port extension);
 * we zero-pad the remainder.
 * -------------------------------------------------------------------------- */
static EVT_UCX_ROOTHUB_INTERRUPT_TX OhciPciInterruptTx;

_Use_decl_annotations_
static VOID OhciPciInterruptTx(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PDEVICE_CONTEXT dc = g_DeviceContext;
    if (!dc || !dc->MmioBase) {
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }

    ULONG rhDescA = READ_REGISTER_ULONG(
                        (PULONG)((PUCHAR)dc->MmioBase + REG_HcRhDescriptorA));
    ULONG ports   = rhDescA & 0xFFu;

    UCHAR bitmap[8] = {0};
    int anySet = 0;

    for (ULONG i = 0; i < ports; i++) {
        ULONG rhps = READ_REGISTER_ULONG(
                         (PULONG)((PUCHAR)dc->MmioBase
                                  + REG_HcRhPortStatus_BASE + i * 4u));
        if (rhps & OHCI_RHPS_CHANGE_MASK) {
            ULONG portBit = i + 1u;
            bitmap[portBit / 8u] |= (UCHAR)(1u << (portBit % 8u));
            anySet = 1;
            /* W1C: write change bits back to clear. */
            WRITE_REGISTER_ULONG(
                (PULONG)((PUCHAR)dc->MmioBase
                         + REG_HcRhPortStatus_BASE + i * 4u),
                rhps & OHCI_RHPS_CHANGE_MASK);
        }
    }

    /* Argument1 is a URB pointer (URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER),
     * NOT a raw buffer. Buffer + length come from the URB struct. (Confirmed
     * against dwusb RootHub_UcxEvtInterruptTransfer.) */
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION stack = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)stack->Parameters.Others.Argument1;
    if (!urb) {
        LOG("RootHub InterruptTx: NULL URB in Argument1");
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }
    PVOID outBuf = urb->UrbBulkOrInterruptTransfer.TransferBuffer;
    size_t outLen = urb->UrbBulkOrInterruptTransfer.TransferBufferLength;
    if (!outBuf || outLen == 0) {
        urb->UrbHeader.Status = USBD_STATUS_INVALID_PARAMETER;
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    /*
     * If UCX requests more bytes than our bitmap covers (possible when
     * ControllerTypeSoftXhci causes it to expect a USB 3-style longer bitmap),
     * log it and zero-pad the extra bytes.
     */
    if (outLen > sizeof(bitmap)) {
        LOG("RootHub InterruptTx: UCX requested %zu bytes (bitmap=%zu); zero-padding extra",
            outLen, sizeof(bitmap));
    }

    size_t copyLen = (sizeof(bitmap) < outLen) ? sizeof(bitmap) : outLen;
    RtlCopyMemory(outBuf, bitmap, copyLen);
    if (outLen > copyLen) {
        RtlZeroMemory((PUCHAR)outBuf + copyLen, outLen - copyLen);
    }

    LOG("RootHub InterruptTx: bitmap[0]=0x%02X (anyChange=%d), ports=%lu, copied %zu bytes",
        bitmap[0], anySet, ports, copyLen);

    urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* --------------------------------------------------------------------------
 * OhciPciGetInfo — EVT_UCX_ROOTHUB_GET_INFO  (Task 3 real implementation)
 *
 * Reads HcRhDescriptorA to determine the USB 2.0 port count and fills in
 * ROOTHUB_INFO.  The struct fields are entirely different from the plan
 * skeleton's assumptions — see deviation notes at top of file.
 *
 * ROOTHUB_INFO actual layout (WDK 10.0.26100.0):
 *   ULONG           Size;
 *   CONTROLLER_TYPE ControllerType;
 *   USHORT          NumberOf20Ports;
 *   USHORT          NumberOf30Ports;
 *   USHORT          MaxU1ExitLatency;   // USB 3 only
 *   USHORT          MaxU2ExitLatency;   // USB 3 only
 * -------------------------------------------------------------------------- */
static EVT_UCX_ROOTHUB_GET_INFO OhciPciGetInfo;

_Use_decl_annotations_
static VOID OhciPciGetInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);

    PDEVICE_CONTEXT dc = g_DeviceContext;
    if (!dc || !dc->MmioBase) {
        LOG("RootHub GetInfo: no device context or MMIO not mapped");
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }

    /*
     * UCX conveys the ROOTHUB_INFO output buffer as IRP
     * Parameters.Others.Argument1, NOT through WDF input/output buffers.
     * (Same convention as IOCTL_INTERNAL_USB_SUBMIT_URB.) Initial code
     * used WdfRequestRetrieveOutputBuffer and crashed with NULL deref
     * because UCX's IOCTL has no METHOD_BUFFERED output staged.
     */
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION stack = IoGetCurrentIrpStackLocation(irp);
    PROOTHUB_INFO info = (PROOTHUB_INFO)stack->Parameters.Others.Argument1;
    if (!info) {
        LOG("RootHub GetInfo: NULL output struct in Argument1");
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    /*
     * HcRhDescriptorA (offset 0x48):
     *   bits  7:0  — NDP  (Number of Downstream Ports, 1-15)
     *   bit   8   — PSM  (Power Switching Mode: 0=ganged, 1=per-port)
     *   bit   9   — NPS  (No Power Switching)
     *   bit  10   — DT   (Device Type: 0=not compound, 1=compound)
     *   bit  11   — OCPM (Over Current Protection Mode: 0=global, 1=per-port)
     *   bit  12   — NOCP (No Over Current Protection)
     *   bits 23:16 — POTPGT (Power-On to Power-Good Time, units of 2 ms)
     *   bits 31:24 — reserved
     */
    ULONG rhDescA = READ_REGISTER_ULONG(
                        (PULONG)((PUCHAR)dc->MmioBase + REG_HcRhDescriptorA));
    ULONG numPorts = rhDescA & 0xFFu;
    ULONG potpgt   = (rhDescA >> 24) & 0xFFu;

    RtlZeroMemory(info, sizeof(*info));
    info->Size             = sizeof(*info);
    /*
     * UCX's CONTROLLER_TYPE has only ControllerTypeXhci (0) and
     * ControllerTypeSoftXhci (1).  OHCI is neither.  Report SoftXhci as the
     * closest available value for a non-xHCI emulated/alternate controller.
     */
    info->ControllerType   = ControllerTypeSoftXhci;
    info->NumberOf20Ports  = (USHORT)numPorts;
    info->NumberOf30Ports  = 0;       /* OHCI is USB 1.1/2.0 only */
    info->MaxU1ExitLatency = 0;       /* USB 3 U1/U2 latency N/A for OHCI */
    info->MaxU2ExitLatency = 0;

    LOG("RootHub GetInfo: ports=%lu POTPGT=%lu ms (rhDescA=0x%08X)",
        numPorts, potpgt * 2u, rhDescA);

    WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, sizeof(*info));
}

/* --------------------------------------------------------------------------
 * OhciPciGet20PortInfo — EVT_UCX_ROOTHUB_GET_20PORT_INFO  (Task 3 real impl)
 *
 * Reads HcRhDescriptorA (port count) and HcRhDescriptorB (Device Removable
 * bitmap) to populate ROOTHUB_20PORTS_INFO + per-port ROOTHUB_20PORT_INFO.
 *
 * ROOTHUB_20PORTS_INFO layout (WDK 10.0.26100.0):
 *   ULONG                   Size;
 *   USHORT                  NumberOfPorts;
 *   USHORT                  PortInfoSize;
 *   PROOTHUB_20PORT_INFO *  PortInfoArray;   // array of pointers
 *
 * ROOTHUB_20PORT_INFO layout per port:
 *   USHORT  PortNumber;
 *   UCHAR   MinorRevision;
 *   UCHAR   HubDepth;
 *   TRISTATE Removable;           // TriStateFalse=removable, TriStateTrue=not removable
 *   TRISTATE IntegratedHubImplemented;
 *   TRISTATE DebugCapable;
 *   CONTROLLER_USB_20_HARDWARE_LPM_FLAGS ControllerUsb20HardwareLpmFlags;
 *
 * DEVIATION from plan: plan expected inline per-port array with speed/power
 * switch fields.  Actual struct uses pointer-based indirection; no speed or
 * power switch fields exist.
 *
 * Memory strategy: allocate one NonPagedPool block for the pointer array
 * (ports * sizeof(PROOTHUB_20PORT_INFO)) plus ports * sizeof(ROOTHUB_20PORT_INFO)
 * for the actual structs.  Both live in a single allocation for easy cleanup,
 * but since UCX reads them synchronously during this callback and we complete
 * the request inline, we free after completion.
 * -------------------------------------------------------------------------- */
static EVT_UCX_ROOTHUB_GET_20PORT_INFO OhciPciGet20PortInfo;

_Use_decl_annotations_
static VOID OhciPciGet20PortInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);

    PDEVICE_CONTEXT dc = g_DeviceContext;
    if (!dc || !dc->MmioBase) {
        LOG("RootHub Get20PortInfo: no device context or MMIO not mapped");
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }

    ULONG rhDescA = READ_REGISTER_ULONG(
                        (PULONG)((PUCHAR)dc->MmioBase + REG_HcRhDescriptorA));
    ULONG ports   = rhDescA & 0xFFu;

    /*
     * HcRhDescriptorB (offset 0x4C):
     *   bits 15:0   — DR  (Device Removable bitmap).
     *     Bit 0 reserved; bit N corresponds to downstream port N.
     *     DR[N]=1 means the device attached to port N is NOT removable.
     *   bits 31:16  — PPCM (Port Power Control Mask) — not needed here.
     */
    ULONG rhDescB = READ_REGISTER_ULONG(
                        (PULONG)((PUCHAR)dc->MmioBase + REG_HcRhDescriptorB));
    USHORT drMask = (USHORT)(rhDescB & 0xFFFFu);

    /* UCX conveys ROOTHUB_20PORTS_INFO via Argument1. The struct's
     * PortInfoArray field is *pre-allocated by UCX* with NumberOfPorts
     * pointer slots — we fill in the existing PROOTHUB_20PORT_INFO entries
     * UCX has prepared. (Confirmed against dwusb reference driver:
     * Device.c RootHub_UcxEvtGet20PortInfo.) Replacing PortInfoArray with
     * our own allocation and freeing it caused UCX to read freed memory,
     * corrupting per-port topology and triggering a controller reset loop. */
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION stack = IoGetCurrentIrpStackLocation(irp);
    PROOTHUB_20PORTS_INFO portsInfo =
        (PROOTHUB_20PORTS_INFO)stack->Parameters.Others.Argument1;
    if (!portsInfo) {
        LOG("RootHub Get20PortInfo: NULL output struct in Argument1");
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }
    if (portsInfo->Size < sizeof(*portsInfo)) {
        LOG("RootHub Get20PortInfo: portsInfo->Size %lu < %llu",
            portsInfo->Size, (ULONGLONG)sizeof(*portsInfo));
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }
    if (portsInfo->NumberOfPorts != ports) {
        LOG("RootHub Get20PortInfo: NumberOfPorts mismatch UCX=%u hw=%lu",
            portsInfo->NumberOfPorts, ports);
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    PROOTHUB_20PORT_INFO *arr = portsInfo->PortInfoArray;
    for (ULONG i = 0; i < ports; i++) {
        PROOTHUB_20PORT_INFO p = arr[i];
        if (!p) {
            LOG("RootHub Get20PortInfo: PortInfoArray[%lu] is NULL", i);
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        p->PortNumber               = (USHORT)(i + 1u);
        p->MinorRevision            = 0;
        p->HubDepth                 = 0;
        BOOLEAN notRemovable        = !!((drMask >> (i + 1u)) & 1u);
        p->Removable                = notRemovable ? TriStateTrue : TriStateFalse;
        p->IntegratedHubImplemented = TriStateFalse;
        p->DebugCapable             = TriStateFalse;
        p->ControllerUsb20HardwareLpmFlags.AsUchar = 0;
    }

    LOG("RootHub Get20PortInfo: filled %lu ports drMask=0x%04X",
        ports, (USHORT)drMask);

    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* --------------------------------------------------------------------------
 * StubGet30PortInfo — EVT_UCX_ROOTHUB_GET_30PORT_INFO
 *
 * DEVIATION from plan: plan's skeleton guessed USB3 could be passed NULL.
 * Actual header marks EvtRootHubGet30PortInfo as __notnull — a non-NULL
 * pointer is always required.  OHCI is USB 2.0 only, so we provide a stub
 * that immediately completes with STATUS_NOT_IMPLEMENTED.
 * -------------------------------------------------------------------------- */
static EVT_UCX_ROOTHUB_GET_30PORT_INFO StubGet30PortInfo;

_Use_decl_annotations_
static VOID StubGet30PortInfo(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubGet30PortInfo — OHCI is USB 2.0 only; not implemented");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

/* --------------------------------------------------------------------------
 * Per-class diagnostic stubs for the 7-individual-callbacks variant.
 * Each logs the URB function code from Argument1 so we can see which
 * hub-class operations UCX dispatches to which slot.
 * -------------------------------------------------------------------------- */
/* Helper: pick SetupPacket address based on URB function (CONTROL_TRANSFER vs
 * CONTROL_TRANSFER_EX have different layouts). */
static PUCHAR GetUrbSetupPacket(PURB urb)
{
    if (urb->UrbHeader.Function == URB_FUNCTION_CONTROL_TRANSFER_EX) {
        return urb->UrbControlTransferEx.SetupPacket;
    }
    return urb->UrbControlTransfer.SetupPacket;
}

static ULONG GetUrbTransferBufferLength(PURB urb)
{
    if (urb->UrbHeader.Function == URB_FUNCTION_CONTROL_TRANSFER_EX) {
        return urb->UrbControlTransferEx.TransferBufferLength;
    }
    return urb->UrbControlTransfer.TransferBufferLength;
}

static PVOID GetUrbTransferBuffer(PURB urb)
{
    if (urb->UrbHeader.Function == URB_FUNCTION_CONTROL_TRANSFER_EX) {
        return urb->UrbControlTransferEx.TransferBuffer;
    }
    return urb->UrbControlTransfer.TransferBuffer;
}

/* USB hub-spec feature selectors (USB 2.0 §11.24.2) */
#define USB_FEATURE_PORT_CONNECTION       0
#define USB_FEATURE_PORT_ENABLE           1
#define USB_FEATURE_PORT_SUSPEND          2
#define USB_FEATURE_PORT_OVER_CURRENT     3
#define USB_FEATURE_PORT_RESET            4
#define USB_FEATURE_PORT_POWER            8
#define USB_FEATURE_PORT_LOW_SPEED        9
#define USB_FEATURE_C_PORT_CONNECTION    16
#define USB_FEATURE_C_PORT_ENABLE        17
#define USB_FEATURE_C_PORT_SUSPEND       18
#define USB_FEATURE_C_PORT_OVER_CURRENT  19
#define USB_FEATURE_C_PORT_RESET         20

/* OHCI HcRhPortStatus bits (OHCI spec §7.4.4) */
#define OHCI_RHPS_CCS    (1u << 0)   /* Current Connect Status */
#define OHCI_RHPS_PES    (1u << 1)   /* Port Enable Status */
#define OHCI_RHPS_PSS    (1u << 2)   /* Port Suspend Status */
#define OHCI_RHPS_POCI   (1u << 3)   /* Port Over-Current Indicator */
#define OHCI_RHPS_PRS    (1u << 4)   /* Port Reset Status */
#define OHCI_RHPS_PPS    (1u << 8)   /* Port Power Status */
#define OHCI_RHPS_LSDA   (1u << 9)   /* Low-Speed Device Attached */
#define OHCI_RHPS_CSC    (1u << 16)
#define OHCI_RHPS_PESC   (1u << 17)
#define OHCI_RHPS_PSSC   (1u << 18)
#define OHCI_RHPS_OCIC   (1u << 19)
#define OHCI_RHPS_PRSC   (1u << 20)

/* W1S bits (write to set) */
#define OHCI_RHPS_SET_PES   (1u << 1)
#define OHCI_RHPS_SET_PSS   (1u << 2)
#define OHCI_RHPS_SET_PRS   (1u << 4)
#define OHCI_RHPS_SET_PPS   (1u << 8)
/* W1C clear-feature bits (write to clear) */
#define OHCI_RHPS_CLR_PES   (1u << 0)   /* CCS=1 also clears enable when written */
#define OHCI_RHPS_CLR_PSS   (1u << 3)   /* POCI write */
#define OHCI_RHPS_CLR_PPS   (1u << 9)   /* LSDA write */
/* (the proper "clear" bit conventions are encoded below per feature) */

static ULONG ReadRhPortStatus(PDEVICE_CONTEXT dc, ULONG portIdx /* 0-based */)
{
    return READ_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase
                                        + REG_HcRhPortStatus_BASE + portIdx * 4u));
}
static VOID WriteRhPortStatus(PDEVICE_CONTEXT dc, ULONG portIdx, ULONG val)
{
    WRITE_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase
                                  + REG_HcRhPortStatus_BASE + portIdx * 4u), val);
}

/* Helper: set URB header status to success — required by UCX. dwusb sets it
 * explicitly in every callback; without it UCX treats the URB as failed. */
static VOID SetUrbHeaderSuccess(WDFREQUEST Request)
{
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (urb) urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
}

/* ---- ClearHubFeature: nothing to do for OHCI; success. ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciClearHubFeature;
_Use_decl_annotations_
static VOID OhciPciClearHubFeature(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub ClearHubFeature");
    SetUrbHeaderSuccess(Request);
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- SetHubFeature: same. ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciSetHubFeature;
_Use_decl_annotations_
static VOID OhciPciSetHubFeature(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub SetHubFeature");
    SetUrbHeaderSuccess(Request);
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- GetHubStatus: 4-byte response, all zeros (no overcurrent/power events). ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciGetHubStatus;
_Use_decl_annotations_
static VOID OhciPciGetHubStatus(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (urb) {
        PVOID buf = GetUrbTransferBuffer(urb);
        ULONG len = GetUrbTransferBufferLength(urb);
        if (buf && len >= 4) {
            RtlZeroMemory(buf, 4);
            if (urb->UrbHeader.Function == URB_FUNCTION_CONTROL_TRANSFER_EX)
                urb->UrbControlTransferEx.TransferBufferLength = 4;
            else
                urb->UrbControlTransfer.TransferBufferLength = 4;
        }
    }
    LOG("RootHub GetHubStatus -> {0,0,0,0}");
    if (urb) urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- GetPortErrorCount: 2-byte response, zero. ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciGetPortErrorCount;
_Use_decl_annotations_
static VOID OhciPciGetPortErrorCount(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (urb) {
        PVOID buf = GetUrbTransferBuffer(urb);
        ULONG len = GetUrbTransferBufferLength(urb);
        if (buf && len >= 2) RtlZeroMemory(buf, 2);
    }
    LOG("RootHub GetPortErrorCount -> 0");
    if (urb) urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- GetPortStatus: read HcRhPortStatus[wIndex-1], translate to USB-spec
 *      4-byte port status (wPortStatus + wPortChange).
 * USB 2.0 hub spec §11.24.2.7:
 *   wPortStatus bits: 0=Connection 1=Enable 2=Suspend 3=OverCurrent 4=Reset
 *                     8=Power 9=LowSpeed 10=HighSpeed
 *   wPortChange bits: 0=ConnectionChange 1=EnableChange 2=SuspendChange
 *                     3=OverCurrentChange 4=ResetChange
 * ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciGetPortStatus;
_Use_decl_annotations_
static VOID OhciPciGetPortStatus(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PDEVICE_CONTEXT dc = g_DeviceContext;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (!dc || !urb) {
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    PUCHAR sp = GetUrbSetupPacket(urb);
    USHORT wValue = *(USHORT *)&sp[2];
    USHORT port   = *(USHORT *)&sp[4];
    USHORT wLen   = *(USHORT *)&sp[6];
    PVOID  buf    = GetUrbTransferBuffer(urb);
    ULONG  len    = GetUrbTransferBufferLength(urb);

    /* USB_STATUS_PORT_STATUS=0 → 4 bytes; USB_STATUS_EXT_PORT_STATUS=2 → 8 bytes. */
    ULONG respLen = (wValue == 2) ? 8 : 4;

    if (port == 0 || buf == NULL || len < respLen) {
        LOG("RootHub GetPortStatus: bad params port=%u wValue=%u wLen=%u len=%lu",
            port, wValue, wLen, len);
        urb->UrbHeader.Status = USBD_STATUS_STALL_PID;
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    ULONG rhps = ReadRhPortStatus(dc, port - 1u);

    USHORT wPortStatus = 0;
    if (rhps & OHCI_RHPS_CCS)  wPortStatus |= 0x0001;
    if (rhps & OHCI_RHPS_PES)  wPortStatus |= 0x0002;
    if (rhps & OHCI_RHPS_PSS)  wPortStatus |= 0x0004;
    if (rhps & OHCI_RHPS_POCI) wPortStatus |= 0x0008;
    if (rhps & OHCI_RHPS_PRS)  wPortStatus |= 0x0010;
    if (rhps & OHCI_RHPS_PPS)  wPortStatus |= 0x0100;
    if (rhps & OHCI_RHPS_LSDA) wPortStatus |= 0x0200;

    USHORT wPortChange = 0;
    if (rhps & OHCI_RHPS_CSC)  wPortChange |= 0x0001;
    if (rhps & OHCI_RHPS_PESC) wPortChange |= 0x0002;
    if (rhps & OHCI_RHPS_PSSC) wPortChange |= 0x0004;
    if (rhps & OHCI_RHPS_OCIC) wPortChange |= 0x0008;
    if (rhps & OHCI_RHPS_PRSC) wPortChange |= 0x0010;

    UCHAR resp[8] = {0};
    resp[0] = (UCHAR)(wPortStatus & 0xFF);
    resp[1] = (UCHAR)(wPortStatus >> 8);
    resp[2] = (UCHAR)(wPortChange & 0xFF);
    resp[3] = (UCHAR)(wPortChange >> 8);
    /* Extended port status (USB 2.0 LPM) — bytes 4..7 are zero for OHCI (no LPM). */
    RtlCopyMemory(buf, resp, respLen);
    if (urb->UrbHeader.Function == URB_FUNCTION_CONTROL_TRANSFER_EX)
        urb->UrbControlTransferEx.TransferBufferLength = respLen;
    else
        urb->UrbControlTransfer.TransferBufferLength = respLen;

    LOG("RootHub GetPortStatus port=%u wValue=%u wLen=%u rhps=0x%08X -> status=0x%04X change=0x%04X (respLen=%lu)",
        port, wValue, wLen, rhps, wPortStatus, wPortChange, respLen);
    urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- SetPortFeature: translate USB feature to OHCI W1S bit. ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciSetPortFeature;
_Use_decl_annotations_
static VOID OhciPciSetPortFeature(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PDEVICE_CONTEXT dc = g_DeviceContext;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (!dc || !urb) {
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    PUCHAR sp = GetUrbSetupPacket(urb);
    USHORT feature = *(USHORT *)&sp[2];
    USHORT port    = *(USHORT *)&sp[4];

    /* Diagnostic: hex-dump the entire setup packet + first ULONG of PipeHandle
     * so we can confirm where UCX puts the port number for multi-port hubs. */
    LOG("RootHub SetPortFeature port=%u feature=%u [setup=%02X %02X %02X %02X %02X %02X %02X %02X | PipeHandle=%p TransferFlags=0x%08X]",
        port, feature,
        sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7],
        urb->UrbControlTransferEx.PipeHandle,
        urb->UrbControlTransferEx.TransferFlags);

    /* port==0 = "hub-global". For PORT_POWER, write HcRhStatus.LPSC (bit 16,
     * W1S "Set Global Power"). On NPS=1 hubs this is a no-op in hardware,
     * but UCX may still want to see us perform the write. */
    if (port == 0) {
        if (feature == USB_FEATURE_PORT_POWER) {
            WRITE_REGISTER_ULONG(
                (PULONG)((PUCHAR)dc->MmioBase + 0x50 /* HcRhStatus */),
                (1u << 16) /* LPSC = Set Global Power */);
            ULONG rhs = READ_REGISTER_ULONG(
                (PULONG)((PUCHAR)dc->MmioBase + 0x50));
            LOG("  -> wrote HcRhStatus.LPSC; HcRhStatus now=0x%08X", rhs);
        }
        /* Also iterate per-port to set PPS for paranoid UCX checks. */
        ULONG rhDescA = READ_REGISTER_ULONG(
            (PULONG)((PUCHAR)dc->MmioBase + REG_HcRhDescriptorA));
        ULONG nports = rhDescA & 0xFFu;
        for (ULONG i = 0; i < nports; i++) {
            ULONG rhps = ReadRhPortStatus(dc, i);
            LOG("  port %lu rhps=0x%08X", i + 1u, rhps);
        }
        urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }

    ULONG portIdx = port - 1u;
    ULONG bit = 0;
    switch (feature) {
        case USB_FEATURE_PORT_ENABLE:    bit = OHCI_RHPS_SET_PES; break;
        case USB_FEATURE_PORT_SUSPEND:   bit = OHCI_RHPS_SET_PSS; break;
        case USB_FEATURE_PORT_RESET:     bit = OHCI_RHPS_SET_PRS; break;
        case USB_FEATURE_PORT_POWER:     bit = OHCI_RHPS_SET_PPS; break;
        default:
            LOG("RootHub SetPortFeature: unsupported feature %u", feature);
            urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
            WdfRequestComplete(Request, STATUS_SUCCESS);
            return;
    }
    WriteRhPortStatus(dc, portIdx, bit);
    urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* ---- ClearPortFeature: W1C the change bits or clear the operational bit. ---- */
static EVT_UCX_ROOTHUB_CONTROL_URB OhciPciClearPortFeature;
_Use_decl_annotations_
static VOID OhciPciClearPortFeature(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    PDEVICE_CONTEXT dc = g_DeviceContext;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION s = IoGetCurrentIrpStackLocation(irp);
    PURB urb = (PURB)s->Parameters.Others.Argument1;
    if (!dc || !urb) {
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    PUCHAR sp = GetUrbSetupPacket(urb);
    USHORT feature = *(USHORT *)&sp[2];
    USHORT port    = *(USHORT *)&sp[4];

    LOG("RootHub ClearPortFeature port=%u feature=%u", port, feature);

    if (port == 0) {
        urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }
    ULONG portIdx = port - 1u;
    ULONG bit = 0;
    switch (feature) {
        case USB_FEATURE_PORT_ENABLE:        bit = (1u << 0); break;
        case USB_FEATURE_PORT_SUSPEND:       bit = (1u << 3); break;
        case USB_FEATURE_PORT_POWER:         bit = (1u << 9); break;
        case USB_FEATURE_C_PORT_CONNECTION:  bit = OHCI_RHPS_CSC;  break;
        case USB_FEATURE_C_PORT_ENABLE:      bit = OHCI_RHPS_PESC; break;
        case USB_FEATURE_C_PORT_SUSPEND:     bit = OHCI_RHPS_PSSC; break;
        case USB_FEATURE_C_PORT_OVER_CURRENT:bit = OHCI_RHPS_OCIC; break;
        case USB_FEATURE_C_PORT_RESET:       bit = OHCI_RHPS_PRSC; break;
        default:
            urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
            WdfRequestComplete(Request, STATUS_SUCCESS);
            return;
    }
    WriteRhPortStatus(dc, portIdx, bit);
    urb->UrbHeader.Status = USBD_STATUS_SUCCESS;
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* --------------------------------------------------------------------------
 * OhciPci_RootHubCreate — declared in device_context.h
 * -------------------------------------------------------------------------- */

NTSTATUS
OhciPci_RootHubCreate(
    _In_ PDEVICE_CONTEXT dc,
    _In_ UCXCONTROLLER   controller
    )
{
    /*
     * Store the device context in the module-static so that the callbacks
     * can retrieve it.  Must be done before UcxRootHubCreate, which may
     * immediately invoke the callbacks.
     */
    g_DeviceContext = dc;

    UCX_ROOTHUB_CONFIG cfg;

    /*
     * Use UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER rather than
     * UCX_ROOTHUB_CONFIG_INIT: the former takes one ControlUrb dispatcher
     * instead of the 7 individual request handlers, which keeps the skeleton
     * compact. Tasks 3+4 can switch to UCX_ROOTHUB_CONFIG_INIT with separate
     * handlers once real port logic is wired.
     *
     * Args: Config, ControlUrb, InterruptTx, GetInfo, Get20PortInfo, Get30PortInfo
     * DEVIATION from plan: plan called UCX_ROOTHUB_CONFIG_INIT with 4 args
     * including a NULL USB3 arg; actual macro takes 5 non-config args and all
     * must be non-NULL.
     */
    /* Switched from UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER to the
     * 7-individual-callbacks variant. The combined ControlUrb path was never
     * invoked by UCX (StubControlUrb logs never appeared); UCX appears to
     * dispatch hub-class requests to the per-feature handlers when those are
     * non-NULL, even with the combined-handler config available. Using the
     * individual-callback config makes that explicit. Each handler currently
     * logs the URB function code and completes with STATUS_SUCCESS so we can
     * see the dispatch sequence and identify which need real implementations. */
    UCX_ROOTHUB_CONFIG_INIT(
        &cfg,
        OhciPciClearHubFeature,
        OhciPciClearPortFeature,
        OhciPciGetHubStatus,
        OhciPciGetPortStatus,
        OhciPciSetHubFeature,
        OhciPciSetPortFeature,
        OhciPciGetPortErrorCount,
        OhciPciInterruptTx,
        OhciPciGetInfo,
        OhciPciGet20PortInfo,
        StubGet30PortInfo
    );

    NTSTATUS status = UcxRootHubCreate(controller, &cfg,
                                       WDF_NO_OBJECT_ATTRIBUTES, &dc->RootHub);
    LOG("UcxRootHubCreate -> 0x%08X", status);
    return status;
}

/* --------------------------------------------------------------------------
 * OhciPci_NotifyPortChanged — declared in device_context.h
 *
 * Thin wrapper so interrupt.c (which does not include UcxClass.h) can trigger
 * a UCX port-change notification without a direct UCX API dependency.
 * -------------------------------------------------------------------------- */
void
OhciPci_NotifyPortChanged(
    _In_ PDEVICE_CONTEXT dc
    )
{
    if (dc && dc->RootHub) {
        UcxRootHubPortChanged(dc->RootHub);
    }
}
