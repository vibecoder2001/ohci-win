/*++

Module Name:

    ucx_glue.c

Abstract:

    UCX 1.6 controller registration for OhciPci.

    Implements OhciPci_UcxInitDeviceInit (called before WdfDeviceCreate in
    EvtDriverDeviceAdd) and OhciPci_UcxControllerCreate (called from
    EvtDevicePrepareHardware after ohci_hc_init succeeds).

    All UCX event callbacks are stubs returning STATUS_NOT_IMPLEMENTED or VOID.
    Plan 5 fills in the real implementations.

    Callback signatures are taken from the confirmed-working spike at
    spike/ucx_spike/driver.c (WDK 10.0.26100.0, UCX 1.6).

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#include <wdmguid.h>

/*
 * UCX 1.6 headers: UcxClass.h pulls in UcxGlobals, UcxFuncEnum, UcxObjects,
 * UcxController, UcxRootHub, UcxUsbDevice, UcxEndpoint, UcxSStreams.
 * Deviation from plan skeleton: plan used <ucx.h> + <ucxcontroller.h>;
 * spike confirmed the correct single-header include is <UcxClass.h>.
 */
#include <UcxClass.h>

#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * Stub UCX controller event callbacks.
 *
 * Signatures are matched to the EVT_UCX_CONTROLLER_* typedefs confirmed in
 * spike/ucx_spike/driver.c. Deviations from the plan skeleton are noted.
 * -------------------------------------------------------------------------- */

/*
 * StubReset — matches EVT_UCX_CONTROLLER_RESET (VOID return).
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
StubReset(
    _In_ UCXCONTROLLER UcxController
    )
{
    LOG("StubReset (deferring completion to workitem)");

    WDF_WORKITEM_CONFIG wic;
    WDF_WORKITEM_CONFIG_INIT(&wic, OhciPci_ResetCompleteWork);

    WDF_OBJECT_ATTRIBUTES attrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attrs, OHCIPCI_RESET_WORK_CTX);
    attrs.ParentObject = UcxController;

    WDFWORKITEM wi;
    NTSTATUS st = WdfWorkItemCreate(&wic, &attrs, &wi);
    if (!NT_SUCCESS(st)) {
        LOG("StubReset: WdfWorkItemCreate failed 0x%08X — falling back to sync", st);
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
 * StubQueryUsbCapability — matches EVT_UCX_CONTROLLER_QUERY_USB_CAPABILITY.
 *
 * Deviation from plan skeleton: the plan listed parameters as
 *   (c, g, l, ll, b) i.e. ResultLength before OutputBuffer.
 * Spike (and the WDK typedef) use:
 *   OutputBufferLength, OutputBuffer, ResultLength
 * i.e. OutputBuffer before ResultLength. Corrected here.
 * Also: spike sets *ResultLength = 0, not UNREFERENCED on the out-param.
 */
static NTSTATUS
StubQueryUsbCapability(
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
 * StubGetCurrentFrameNumber — matches EVT_UCX_CONTROLLER_GET_CURRENT_FRAMENUMBER.
 * Signature identical to plan skeleton; spike confirms it.
 */
static NTSTATUS
StubGetCurrentFrameNumber(
    _In_  UCXCONTROLLER UcxController,
    _Out_ PULONG        FrameNumber
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    *FrameNumber = 0;
    LOG("GetCurrentFrameNumber called");
    return STATUS_SUCCESS;
}

/*
 * StubGetTransportCharacteristics — matches
 *   EVT_UCX_CONTROLLER_GET_TRANSPORT_CHARACTERISTICS.
 * Spike confirms the out-param is PUCX_CONTROLLER_TRANSPORT_CHARACTERISTICS
 * (pointer); plan skeleton matched. Zero the struct via RtlZeroMemory.
 */
static NTSTATUS
StubGetTransportCharacteristics(
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
 * StubSetTransportCharsChange — matches
 *   EVT_UCX_CONTROLLER_SET_TRANSPORT_CHARACTERISTICS_CHANGE_NOTIFICATION.
 *
 * Deviation from plan skeleton: the plan declared the second parameter as
 *   PUCX_CONTROLLER_TRANSPORT_CHARACTERISTICS_CHANGE_FLAGS (pointer).
 * Spike and WDK typedef pass it by VALUE:
 *   UCX_CONTROLLER_TRANSPORT_CHARACTERISTICS_CHANGE_FLAGS ChangeNotificationFlags
 * Corrected here to match spike.
 */
static VOID
StubSetTransportCharsChange(
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

    /*
     * Read this device's PCI IDs and B/D/F via the BUS_INTERFACE_STANDARD
     * the PCI bus driver exposes, then set ParentBusType=Pci. Without this
     * UCX defaults to ParentBusTypeCustom + bogus VID/DID (LONG_MAX) which
     * makes UCX's PCI-controller bring-up sequence fail and loop on Reset.
     */
    BUS_INTERFACE_STANDARD bus = {0};
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(dc->Device);
    if (pdo) {
        KEVENT ev;
        IO_STATUS_BLOCK iosb;
        KeInitializeEvent(&ev, NotificationEvent, FALSE);
        PIRP irp = IoBuildSynchronousFsdRequest(IRP_MJ_PNP, pdo,
                                                NULL, 0, NULL, &ev, &iosb);
        if (irp) {
            PIO_STACK_LOCATION s = IoGetNextIrpStackLocation(irp);
            s->MajorFunction = IRP_MJ_PNP;
            s->MinorFunction = IRP_MN_QUERY_INTERFACE;
            s->Parameters.QueryInterface.InterfaceType    = &GUID_BUS_INTERFACE_STANDARD;
            s->Parameters.QueryInterface.Size             = sizeof(bus);
            s->Parameters.QueryInterface.Version          = 1;
            s->Parameters.QueryInterface.Interface        = (PINTERFACE)&bus;
            s->Parameters.QueryInterface.InterfaceSpecificData = NULL;
            irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
            NTSTATUS qiSt = IoCallDriver(pdo, irp);
            if (qiSt == STATUS_PENDING) {
                KeWaitForSingleObject(&ev, Executive, KernelMode, FALSE, NULL);
                qiSt = iosb.Status;
            }
            if (NT_SUCCESS(qiSt) && bus.GetBusData) {
                ULONG vidDid = 0;
                bus.GetBusData(bus.Context, PCI_WHICHSPACE_CONFIG, &vidDid, 0, sizeof(vidDid));
                USHORT vid = (USHORT)(vidDid & 0xFFFF);
                USHORT did = (USHORT)(vidDid >> 16);
                UCHAR rev = 0;
                bus.GetBusData(bus.Context, PCI_WHICHSPACE_CONFIG, &rev, 0x08, sizeof(rev));
                LOG("PCI VID=0x%04X DID=0x%04X REV=0x%02X", vid, did, rev);
                UCX_CONTROLLER_CONFIG_SET_PCI_INFO(&cfg, vid, did, rev, 0, 0, 0);
                if (bus.InterfaceDereference) bus.InterfaceDereference(bus.Context);
            } else {
                LOG("BUS_INTERFACE_STANDARD query failed: 0x%08X", qiSt);
            }
        }
    }

    cfg.EvtControllerUsbDeviceAdd                                      = OhciPci_UsbDeviceAdd;
    cfg.EvtControllerQueryUsbCapability                                = StubQueryUsbCapability;
    cfg.EvtControllerGetCurrentFrameNumber                             = StubGetCurrentFrameNumber;
    cfg.EvtControllerReset                                             = StubReset;
    cfg.EvtControllerGetTransportCharacteristics                       = StubGetTransportCharacteristics;
    cfg.EvtControllerSetTransportCharacteristicsChangeNotification     = StubSetTransportCharsChange;

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
