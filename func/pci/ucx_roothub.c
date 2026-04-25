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
 * StubControlUrb — EVT_UCX_ROOTHUB_CONTROL_URB
 *
 * Dispatched for all standard root hub control requests (GetHubStatus,
 * GetPortStatus, SetPortFeature, etc.) when using the combined handler path.
 * Task 4 decomposes into individual callbacks.
 * -------------------------------------------------------------------------- */
static EVT_UCX_ROOTHUB_CONTROL_URB StubControlUrb;

_Use_decl_annotations_
static VOID StubControlUrb(UCXROOTHUB UcxRootHub, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxRootHub);
    LOG("RootHub StubControlUrb (Task 4 will implement)");
    WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);
}

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

    /*
     * USB 2.0 hub change-status bitmap: bit 0 = hub, bit i = port i (1-indexed).
     * Allow up to 8 bytes — covers up to 63 ports, far more than OHCI's max 15.
     */
    UCHAR bitmap[8] = {0};
    int anySet = 0;

    for (ULONG i = 0; i < ports; i++) {
        ULONG rhps = READ_REGISTER_ULONG(
                         (PULONG)((PUCHAR)dc->MmioBase
                                  + REG_HcRhPortStatus_BASE + i * 4u));
        if (rhps & OHCI_RHPS_CHANGE_MASK) {
            ULONG portBit = i + 1u;          /* 1-indexed */
            bitmap[portBit / 8u] |= (UCHAR)(1u << (portBit % 8u));
            anySet = 1;
            /* W1C: write the change bits back to clear them. */
            WRITE_REGISTER_ULONG(
                (PULONG)((PUCHAR)dc->MmioBase
                         + REG_HcRhPortStatus_BASE + i * 4u),
                rhps & OHCI_RHPS_CHANGE_MASK);
        }
    }

    PVOID outBuf;
    size_t outLen;
    NTSTATUS status = WdfRequestRetrieveOutputBuffer(Request, 1, &outBuf, &outLen);
    if (!NT_SUCCESS(status)) {
        WdfRequestComplete(Request, status);
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

    WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, outLen);
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

    PROOTHUB_INFO info;
    NTSTATUS status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*info),
                                                     (PVOID *)&info, NULL);
    if (!NT_SUCCESS(status)) {
        LOG("RootHub GetInfo: WdfRequestRetrieveOutputBuffer -> 0x%08X", status);
        WdfRequestComplete(Request, status);
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

    PROOTHUB_20PORTS_INFO portsInfo;
    NTSTATUS status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*portsInfo),
                                                     (PVOID *)&portsInfo, NULL);
    if (!NT_SUCCESS(status)) {
        LOG("RootHub Get20PortInfo: WdfRequestRetrieveOutputBuffer -> 0x%08X", status);
        WdfRequestComplete(Request, status);
        return;
    }

    /*
     * Allocate a single block:
     *   [0 .. ports*sizeof(PROOTHUB_20PORT_INFO))  — pointer array
     *   [ports*sizeof(PROOTHUB_20PORT_INFO) .. end) — ROOTHUB_20PORT_INFO structs
     *
     * This block is freed after WdfRequestCompleteWithInformation because UCX
     * reads the data synchronously before returning from the callback.
     *
     * If ports == 0 (pathological hardware) skip allocation and return empty.
     */
    PROOTHUB_20PORT_INFO  *ptrArray  = NULL;
    PROOTHUB_20PORT_INFO   portStructs = NULL;
    SIZE_T ptrArrayBytes  = 0;
    SIZE_T structArrayBytes = 0;

    if (ports > 0) {
        ptrArrayBytes    = (SIZE_T)ports * sizeof(PROOTHUB_20PORT_INFO);
        structArrayBytes = (SIZE_T)ports * sizeof(ROOTHUB_20PORT_INFO);
        SIZE_T totalBytes = ptrArrayBytes + structArrayBytes;

        PVOID block = ExAllocatePool2(POOL_FLAG_NON_PAGED, totalBytes,
                                      'hrPO');   /* OhciPci port-info tag */
        if (!block) {
            LOG("RootHub Get20PortInfo: ExAllocatePool2 failed (ports=%lu)", ports);
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        RtlZeroMemory(block, totalBytes);

        ptrArray   = (PROOTHUB_20PORT_INFO *)block;
        portStructs = (PROOTHUB_20PORT_INFO)((PUCHAR)block + ptrArrayBytes);

        for (ULONG i = 0; i < ports; i++) {
            PROOTHUB_20PORT_INFO p = &portStructs[i];
            ptrArray[i] = p;

            p->PortNumber   = (USHORT)(i + 1u);  /* 1-indexed */
            p->MinorRevision = 0;                 /* OHCI 1.0a, minor = 0 */
            p->HubDepth      = 0;                 /* root hub is at depth 0 */

            /*
             * DR bit (i+1) set means device NOT removable (integrated);
             * DR bit clear means removable.  Map to TRISTATE:
             *   TriStateFalse = 'f' = removable (false = "not fixed")
             *   TriStateTrue  = 't' = not removable (true = "integrated/fixed")
             */
            BOOLEAN notRemovable = !!((drMask >> (i + 1u)) & 1u);
            p->Removable = notRemovable ? TriStateTrue : TriStateFalse;

            /* OHCI does not implement integrated hub; no debug capability. */
            p->IntegratedHubImplemented = TriStateFalse;
            p->DebugCapable             = TriStateFalse;

            /* LPM not supported by OHCI (USB 1.1 / USB 2.0 without LPM). */
            p->ControllerUsb20HardwareLpmFlags.AsUchar = 0;
        }
    }

    RtlZeroMemory(portsInfo, sizeof(*portsInfo));
    portsInfo->Size          = sizeof(*portsInfo);
    portsInfo->NumberOfPorts = (USHORT)ports;
    portsInfo->PortInfoSize  = sizeof(ROOTHUB_20PORT_INFO);
    portsInfo->PortInfoArray = ptrArray;

    LOG("RootHub Get20PortInfo: count=%lu drMask=0x%04X (rhDescA=0x%08X rhDescB=0x%08X)",
        ports, (USHORT)drMask, rhDescA, rhDescB);

    WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, sizeof(*portsInfo));

    /* Free the per-port allocation after completing the request. */
    if (ptrArray) {
        ExFreePoolWithTag(ptrArray, 'hrPO');
    }
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
    UCX_ROOTHUB_CONFIG_INIT_WITH_CONTROL_URB_HANDLER(
        &cfg,
        StubControlUrb,
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
