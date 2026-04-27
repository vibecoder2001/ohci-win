/*++

Module Name:

    ucx_glue.c

Abstract:

    UCX 1.6 controller registration for OhciPci.

    Implements OhciPci_UcxInitDeviceInit (called before WdfDeviceCreate in
    EvtDriverDeviceAdd) and OhciPci_UcxControllerCreate (called from
    EvtDevicePrepareHardware after ohci_hc_init succeeds).

    All UCX event callbacks below are stubs returning STATUS_NOT_IMPLEMENTED
    or VOID; the per-USB-device callbacks live in ucx_usbdevice.c.

    Callback signatures match the WDK 10.0.26100.0 / UCX 1.6 typedefs.

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#include <wdmguid.h>

/*
 * UCX 1.6 headers: UcxClass.h pulls in UcxGlobals, UcxFuncEnum, UcxObjects,
 * UcxController, UcxRootHub, UcxUsbDevice, UcxEndpoint, UcxSStreams. The
 * older <ucx.h> + <ucxcontroller.h> split is not how WDK 10.0.26100.0
 * exposes UCX 1.6 — UcxClass.h is the correct single-header include.
 */
#include <UcxClass.h>

#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * UCX controller event callbacks.
 *
 * Signatures match the EVT_UCX_CONTROLLER_* typedefs in WDK 10.0.26100.0.
 * -------------------------------------------------------------------------- */

/*
 * EvtControllerReset — matches EVT_UCX_CONTROLLER_RESET (VOID return).
 *
 * Defers UcxControllerResetComplete to a workitem so the callback returns
 * before ResetComplete fires. Synchronous completion appears to leave UCX
 * in a state that immediately re-issues Reset (observed: 9-retry loop).
 */
typedef struct _OHCIPCI_RESET_WORK_CTX {
    UCXCONTROLLER UcxController;
} OHCIPCI_RESET_WORK_CTX, *POHCIPCI_RESET_WORK_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_RESET_WORK_CTX, OhciPci_ResetWorkCtxGet)

static EVT_WDF_WORKITEM OhciPci_ResetCompleteWork;

_Use_decl_annotations_
static VOID OhciPci_ResetCompleteWork(WDFWORKITEM WorkItem)
{
    POHCIPCI_RESET_WORK_CTX ctx = OhciPci_ResetWorkCtxGet(WorkItem);
    UCX_CONTROLLER_RESET_COMPLETE_INFO rci;
    UCX_CONTROLLER_RESET_COMPLETE_INFO_INIT(&rci, UcxControllerStatePreserved, FALSE);
    UcxControllerResetComplete(ctx->UcxController, &rci);
    LOG("ResetCompleteWork: ResetComplete called (Preserved/FALSE)");

    /* Prime UCX with a port-change notification so it queries InterruptTx
     * and learns the current port-status bitmap. Without this, UCX may
     * be in a "waiting for first port event" watchdog and reset again. */
    extern PDEVICE_CONTEXT g_DeviceContext;
    if (g_DeviceContext && g_DeviceContext->RootHub) {
        UcxRootHubPortChanged(g_DeviceContext->RootHub);
        LOG("ResetCompleteWork: UcxRootHubPortChanged kicked");
    }

    WdfObjectDelete(WorkItem);
}

static VOID
EvtControllerReset(
    _In_ UCXCONTROLLER UcxController
    )
{
    LOG("EvtControllerReset (deferring completion to workitem)");

    WDF_WORKITEM_CONFIG wic;
    WDF_WORKITEM_CONFIG_INIT(&wic, OhciPci_ResetCompleteWork);

    WDF_OBJECT_ATTRIBUTES attrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attrs, OHCIPCI_RESET_WORK_CTX);
    attrs.ParentObject = UcxController;

    WDFWORKITEM wi;
    NTSTATUS st = WdfWorkItemCreate(&wic, &attrs, &wi);
    if (!NT_SUCCESS(st)) {
        LOG("EvtControllerReset: WdfWorkItemCreate failed 0x%08X — falling back to sync", st);
        UCX_CONTROLLER_RESET_COMPLETE_INFO rci;
        UCX_CONTROLLER_RESET_COMPLETE_INFO_INIT(&rci, UcxControllerStateLost, TRUE);
        UcxControllerResetComplete(UcxController, &rci);
        return;
    }
    POHCIPCI_RESET_WORK_CTX ctx = OhciPci_ResetWorkCtxGet(wi);
    ctx->UcxController = UcxController;
    WdfWorkItemEnqueue(wi);
}

/*
 * EvtControllerQueryUsbCapability — matches EVT_UCX_CONTROLLER_QUERY_USB_CAPABILITY.
 * Parameter order per the WDK typedef is OutputBufferLength, OutputBuffer,
 * ResultLength — and *ResultLength = 0 must be assigned (it's an out-param,
 * not UNREFERENCED).
 */
static NTSTATUS
EvtControllerQueryUsbCapability(
    _In_                                         UCXCONTROLLER UcxController,
    _In_                                         PGUID         CapabilityType,
    _In_                                         ULONG         OutputBufferLength,
    _Out_writes_to_opt_(OutputBufferLength, *ResultLength)
                                                 PVOID         OutputBuffer,
    _Out_                                        PULONG        ResultLength
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(OutputBuffer);
    if (CapabilityType) {
        LOG("QueryUsbCapability GUID={%08lX-%04hX-%04hX-...} outLen=%lu",
            CapabilityType->Data1, CapabilityType->Data2, CapabilityType->Data3,
            OutputBufferLength);
    } else {
        LOG("QueryUsbCapability NULL GUID outLen=%lu", OutputBufferLength);
    }
    *ResultLength = 0;
    return STATUS_NOT_SUPPORTED;
}

/*
 * EvtControllerGetCurrentFrameNumber — matches EVT_UCX_CONTROLLER_GET_CURRENT_FRAMENUMBER.
 */
static NTSTATUS
EvtControllerGetCurrentFrameNumber(
    _In_  UCXCONTROLLER UcxController,
    _Out_ PULONG        FrameNumber
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    /* Read the live OHCI HcFmNumber (offset 0x3C, 16-bit running counter
     * that increments every 1ms). usbaudio uses this to pick StartFrame
     * for the next isoch URB; returning a constant 0 makes every URB
     * land in the past → HC silently skips frames → no audio. */
    extern PDEVICE_CONTEXT g_DeviceContext;
    if (g_DeviceContext && g_DeviceContext->MmioOps.read32) {
        *FrameNumber = g_DeviceContext->MmioOps.read32(
                            g_DeviceContext->MmioOps.context, 0x3C) & 0xFFFFu;
    } else {
        *FrameNumber = 0;
    }
    return STATUS_SUCCESS;
}

/*
 * EvtControllerGetTransportCharacteristics — matches
 *   EVT_UCX_CONTROLLER_GET_TRANSPORT_CHARACTERISTICS.
 * Out-param is PUCX_CONTROLLER_TRANSPORT_CHARACTERISTICS (pointer); zero
 * the struct via RtlZeroMemory.
 */
static NTSTATUS
EvtControllerGetTransportCharacteristics(
    _In_  UCXCONTROLLER                          UcxController,
    _Out_ PUCX_CONTROLLER_TRANSPORT_CHARACTERISTICS UcxControllerTransportCharacteristics
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    RtlZeroMemory(UcxControllerTransportCharacteristics,
                  sizeof(*UcxControllerTransportCharacteristics));
    LOG("GetTransportCharacteristics called");
    return STATUS_SUCCESS;
}

/*
 * EvtControllerSetTransportCharsChange — matches
 *   EVT_UCX_CONTROLLER_SET_TRANSPORT_CHARACTERISTICS_CHANGE_NOTIFICATION.
 * The second parameter is passed BY VALUE per the WDK typedef:
 *   UCX_CONTROLLER_TRANSPORT_CHARACTERISTICS_CHANGE_FLAGS ChangeNotificationFlags
 */
static VOID
EvtControllerSetTransportCharsChange(
    _In_ UCXCONTROLLER                                        UcxController,
    _In_ UCX_CONTROLLER_TRANSPORT_CHARACTERISTICS_CHANGE_FLAGS ChangeNotificationFlags
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(ChangeNotificationFlags);
}

/* --------------------------------------------------------------------------
 * Default-queue dispatcher: forwards IOCTLs (in particular
 * IOCTL_INTERNAL_USB_SUBMIT_URB issued by UsbHub3 against the root hub)
 * into UCX via UcxIoDeviceControl, which routes them to the appropriate
 * registered callback (e.g. EvtRootHubControlUrb). Without this, URB
 * IOCTLs land on our WDFDEVICE unhandled and UsbHub3 retries → UCX
 * resets in a loop.
 * -------------------------------------------------------------------------- */
static EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL OhciPci_EvtDefaultIoctl;

_Use_decl_annotations_
static VOID
OhciPci_EvtDefaultIoctl(
    WDFQUEUE   Queue,
    WDFREQUEST Request,
    size_t     OutputBufferLength,
    size_t     InputBufferLength,
    ULONG      IoControlCode
    )
{
    WDFDEVICE device = WdfIoQueueGetDevice(Queue);
    BOOLEAN handled = UcxIoDeviceControl(device, Request,
                                         OutputBufferLength, InputBufferLength,
                                         IoControlCode);
    if (!handled) {
        LOG("DefaultIoctl: UCX did not handle IoControlCode=0x%08X", IoControlCode);
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_REQUEST);
    }
}

NTSTATUS
OhciPci_CreateDefaultQueue(
    _In_ PDEVICE_CONTEXT dc
    )
{
    WDF_IO_QUEUE_CONFIG cfg;
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&cfg, WdfIoQueueDispatchParallel);
    cfg.EvtIoInternalDeviceControl = OhciPci_EvtDefaultIoctl;
    cfg.EvtIoDeviceControl         = OhciPci_EvtDefaultIoctl;
    cfg.PowerManaged = WdfFalse;

    WDFQUEUE queue;
    NTSTATUS status = WdfIoQueueCreate(dc->Device, &cfg,
                                       WDF_NO_OBJECT_ATTRIBUTES, &queue);
    LOG("OhciPci_CreateDefaultQueue -> 0x%08X", status);
    return status;
}

/* --------------------------------------------------------------------------
 * Public helpers declared in device_context.h
 * -------------------------------------------------------------------------- */

/*
 * OhciPci_UcxInitDeviceInit
 *
 * Must be called BEFORE WdfDeviceCreate. Binds this driver as a UCX
 * class-extension client on the device being added.
 */
NTSTATUS
OhciPci_UcxInitDeviceInit(
    _Inout_ PWDFDEVICE_INIT DeviceInit
    )
{
    NTSTATUS status = UcxInitializeDeviceInit(DeviceInit);
    LOG("UcxInitializeDeviceInit -> 0x%08X", status);
    return status;
}

/*
 * OhciPci_UcxControllerCreate
 *
 * Called from EvtDevicePrepareHardware after ohci_hc_init succeeds.
 * Creates the UCX controller object and, on success, marks it ready.
 */
NTSTATUS
OhciPci_UcxControllerCreate(
    _In_ PDEVICE_CONTEXT dc
    )
{
    UCX_CONTROLLER_CONFIG cfg;
    UCX_CONTROLLER_CONFIG_INIT(&cfg, "OhciPci");

    /* Bus-specific identification (PCI VID/DID/REV, ACPI _HID, ...) comes
     * from the bus-glue function driver (func/pci/, func/acpi/, ...).
     * Without this UCX defaults to ParentBusTypeCustom + bogus VID/DID
     * (LONG_MAX) which makes UCX's bring-up sequence loop on Reset. */
    Ohci_FillUcxControllerIdent(dc, &cfg);

    cfg.EvtControllerUsbDeviceAdd                                      = OhciPci_UsbDeviceAdd;
    cfg.EvtControllerQueryUsbCapability                                = EvtControllerQueryUsbCapability;
    cfg.EvtControllerGetCurrentFrameNumber                             = EvtControllerGetCurrentFrameNumber;
    cfg.EvtControllerReset                                             = EvtControllerReset;
    cfg.EvtControllerGetTransportCharacteristics                       = EvtControllerGetTransportCharacteristics;
    cfg.EvtControllerSetTransportCharacteristicsChangeNotification     = EvtControllerSetTransportCharsChange;

    UCXCONTROLLER controller;
    NTSTATUS status = UcxControllerCreate(dc->Device, &cfg,
                                          WDF_NO_OBJECT_ATTRIBUTES, &controller);
    LOG("UcxControllerCreate -> 0x%08X  (THIS IS THE GATE)", status);

    /*
     * Deviation from task plan: UcxControllerSetReady does not exist in
     * WDK 10.0.26100.0 UCX 1.6 headers (not in ucxcontroller.h, not in
     * ucxfuncenum.h). The plan's skeleton called it; this WDK version has no
     * such export. Omitted — the call would not compile or link.
     */

    if (!NT_SUCCESS(status)) return status;

    /* Save controller handle so OhciPci_RootHubCreate (ucx_roothub.c) can
     * use it. Previously the handle was a local variable and was discarded. */
    dc->Controller = controller;
    return STATUS_SUCCESS;
}
