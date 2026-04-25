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
 * StubUsbDeviceAdd — matches EVT_UCX_CONTROLLER_USBDEVICE_ADD.
 * Signature is identical to plan skeleton; spike confirms it.
 */
static NTSTATUS
StubUsbDeviceAdd(
    _In_ UCXCONTROLLER      UcxController,
    _In_ PUCXUSBDEVICE_INFO UcxUsbDeviceInfo,
    _In_ PUCXUSBDEVICE_INIT UsbDeviceInit
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(UcxUsbDeviceInfo);
    UNREFERENCED_PARAMETER(UsbDeviceInit);
    LOG("StubUsbDeviceAdd called (Plan 5 will implement)");
    return STATUS_NOT_IMPLEMENTED;
}

/*
 * StubReset — matches EVT_UCX_CONTROLLER_RESET (VOID return).
 * Signature identical to plan skeleton; spike confirms it.
 */
static VOID
StubReset(
    _In_ UCXCONTROLLER UcxController
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("StubReset");
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
    UNREFERENCED_PARAMETER(CapabilityType);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(OutputBuffer);
    *ResultLength = 0;
    return STATUS_NOT_IMPLEMENTED;
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

    cfg.EvtControllerUsbDeviceAdd                                      = StubUsbDeviceAdd;
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
