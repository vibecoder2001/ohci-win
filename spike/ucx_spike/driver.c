/*++

Module Name:

    driver.c

Abstract:

    Phase 0 spike: minimal KMDF driver that binds as a UCX class-extension
    client and calls UcxControllerCreate on a contrived non-xHCI device,
    then logs the resulting NTSTATUS. Does not drive hardware, does not
    create a root hub, does nothing else.

    The experimental question is: does Ucx01000.sys reject a client that
    never declares itself as xHCI? Per Phase 0 header research, the
    UCX_CONTROLLER_CONFIG struct has no controller-type identifier, so the
    only way to answer is at runtime via this spike.

Environment:

    Kernel mode only. Driver must be test-signed. Observe output with a
    kernel debugger using a DbgPrintEx filter for DPFLTR_IHVDRIVER_ID.

--*/

#include <ntddk.h>
#include <wdf.h>

//
// UCX 1.6 headers. UcxClass.h pulls in UcxGlobals, UcxFuncEnum, UcxObjects,
// UcxController, UcxRootHub, UcxUsbDevice, UcxEndpoint, UcxSStreams.
//
#include <UcxClass.h>

//
// Forward declarations.
//
DRIVER_INITIALIZE                                       DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                               EvtDriverDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE                         EvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE                         EvtDeviceReleaseHardware;

EVT_UCX_CONTROLLER_USBDEVICE_ADD                        EvtControllerUsbDeviceAdd;
EVT_UCX_CONTROLLER_QUERY_USB_CAPABILITY                 EvtControllerQueryUsbCapability;
EVT_UCX_CONTROLLER_GET_CURRENT_FRAMENUMBER              EvtControllerGetCurrentFrameNumber;
EVT_UCX_CONTROLLER_RESET                                EvtControllerReset;
EVT_UCX_CONTROLLER_GET_TRANSPORT_CHARACTERISTICS        EvtControllerGetTransportCharacteristics;
EVT_UCX_CONTROLLER_SET_TRANSPORT_CHARACTERISTICS_CHANGE_NOTIFICATION
                                                        EvtControllerSetTransportCharacteristicsChangeNotification;

//
// Logging helper. Uses DPFLTR_IHVDRIVER_ID so the caller can filter the
// output in WinDbg with `ed nt!Kd_IHVDRIVER_Mask 8` (DPFLTR_ERROR_LEVEL).
//
#define OHCI_SPIKE_LOG(fmt, ...)                                    \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,             \
               "OhciSpike: " fmt "\n", ##__VA_ARGS__)

//----------------------------------------------------------------------------
// DriverEntry
//----------------------------------------------------------------------------
NTSTATUS
DriverEntry(
    _In_ PDRIVER_OBJECT  DriverObject,
    _In_ PUNICODE_STRING RegistryPath
    )
{
    NTSTATUS              status;
    WDF_DRIVER_CONFIG     config;

    OHCI_SPIKE_LOG("DriverEntry entered");

    WDF_DRIVER_CONFIG_INIT(&config, EvtDriverDeviceAdd);

    status = WdfDriverCreate(DriverObject,
                             RegistryPath,
                             WDF_NO_OBJECT_ATTRIBUTES,
                             &config,
                             WDF_NO_HANDLE);

    OHCI_SPIKE_LOG("WdfDriverCreate -> 0x%08X", status);
    return status;
}

//----------------------------------------------------------------------------
// EvtDriverDeviceAdd
//
// Wire up UCX, create the WDFDEVICE, call UcxControllerCreate. Returns
// failure status if UcxInitializeDeviceInit or WdfDeviceCreate fail (WDF
// contract: EvtDriverDeviceAdd must not return STATUS_SUCCESS unless a
// WDFDEVICE was created). Returns STATUS_SUCCESS after UcxControllerCreate
// regardless of its outcome so the device object persists for inspection.
//----------------------------------------------------------------------------
NTSTATUS
EvtDriverDeviceAdd(
    _In_    WDFDRIVER       Driver,
    _Inout_ PWDFDEVICE_INIT DeviceInit
    )
{
    NTSTATUS                        status;
    WDFDEVICE                       device;
    WDF_PNPPOWER_EVENT_CALLBACKS    pnpCallbacks;
    UCX_CONTROLLER_CONFIG           ucxConfig;
    UCXCONTROLLER                   ucxController;

    UNREFERENCED_PARAMETER(Driver);

    OHCI_SPIKE_LOG("EvtDriverDeviceAdd entered");

    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);
    pnpCallbacks.EvtDevicePrepareHardware = EvtDevicePrepareHardware;
    pnpCallbacks.EvtDeviceReleaseHardware = EvtDeviceReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);

    //
    // Step A: bind as a UCX client.
    //
    status = UcxInitializeDeviceInit(DeviceInit);
    OHCI_SPIKE_LOG("UcxInitializeDeviceInit -> 0x%08X", status);
    if (!NT_SUCCESS(status)) {
        //
        // UCX refused us before we even had a device. No WDFDEVICE exists,
        // so WDF's contract requires we return failure here. The log line
        // above preserves the diagnostic.
        //
        return status;
    }

    //
    // Step B: create the WDFDEVICE.
    //
    status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES, &device);
    OHCI_SPIKE_LOG("WdfDeviceCreate -> 0x%08X", status);
    if (!NT_SUCCESS(status)) {
        //
        // No WDFDEVICE was created; returning STATUS_SUCCESS here would
        // violate WDF's contract and bugcheck under Driver Verifier. The
        // log line above preserves the diagnostic.
        //
        return status;
    }

    //
    // Step C: build UCX_CONTROLLER_CONFIG. The struct has no HC-type
    // discriminator -- only a "DeviceDescription" human-readable string.
    // We deliberately use "OhciSpike" so that if Ucx01000.sys ever logs
    // the rejected device it is obvious which probe it is.
    //
    UCX_CONTROLLER_CONFIG_INIT(&ucxConfig, "OhciSpike (Phase 0 probe)");

    ucxConfig.EvtControllerUsbDeviceAdd                               = EvtControllerUsbDeviceAdd;
    ucxConfig.EvtControllerQueryUsbCapability                         = EvtControllerQueryUsbCapability;
    ucxConfig.EvtControllerGetCurrentFrameNumber                      = EvtControllerGetCurrentFrameNumber;
    ucxConfig.EvtControllerReset                                      = EvtControllerReset;
    ucxConfig.EvtControllerGetTransportCharacteristics                = EvtControllerGetTransportCharacteristics;
    ucxConfig.EvtControllerSetTransportCharacteristicsChangeNotification =
                                                                        EvtControllerSetTransportCharacteristicsChangeNotification;

    //
    // Step D: the gate. UcxControllerCreate has no type argument.
    // Whatever it returns is the Phase 0 verdict.
    //
    status = UcxControllerCreate(device,
                                 &ucxConfig,
                                 WDF_NO_OBJECT_ATTRIBUTES,
                                 &ucxController);
    OHCI_SPIKE_LOG("UcxControllerCreate -> 0x%08X  (THIS IS THE GATE)",
                   status);

    //
    // Regardless of UcxControllerCreate outcome, return SUCCESS -- we want
    // the device object to stay around so the log is visible and so PNP
    // does not attempt to retry or failover.
    //
    return STATUS_SUCCESS;
}

//----------------------------------------------------------------------------
// PnP stubs. Required so the device doesn't bounce out of started state.
//----------------------------------------------------------------------------
NTSTATUS
EvtDevicePrepareHardware(
    _In_ WDFDEVICE   Device,
    _In_ WDFCMRESLIST ResourcesRaw,
    _In_ WDFCMRESLIST ResourcesTranslated
    )
{
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesRaw);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    OHCI_SPIKE_LOG("EvtDevicePrepareHardware (stub)");
    return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceReleaseHardware(
    _In_ WDFDEVICE   Device,
    _In_ WDFCMRESLIST ResourcesTranslated
    )
{
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    OHCI_SPIKE_LOG("EvtDeviceReleaseHardware (stub)");
    return STATUS_SUCCESS;
}

//----------------------------------------------------------------------------
// UCX controller event callback stubs.
//
// Every callback logs that it was hit and returns a non-success/void. We
// never expect any of these to fire in the spike because we never create a
// root hub (UCX has no device to drive). If any of them *does* fire that is
// also useful data.
//----------------------------------------------------------------------------
NTSTATUS
EvtControllerUsbDeviceAdd(
    _In_ UCXCONTROLLER       UcxController,
    _In_ PUCXUSBDEVICE_INFO  UcxUsbDeviceInfo,
    _In_ PUCXUSBDEVICE_INIT  UsbDeviceInit
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(UcxUsbDeviceInfo);
    UNREFERENCED_PARAMETER(UsbDeviceInit);
    OHCI_SPIKE_LOG("EvtControllerUsbDeviceAdd (stub) -> STATUS_NOT_IMPLEMENTED");
    return STATUS_NOT_IMPLEMENTED;
}

NTSTATUS
EvtControllerQueryUsbCapability(
    _In_                              UCXCONTROLLER UcxController,
    _In_                              PGUID         CapabilityType,
    _In_                              ULONG         OutputBufferLength,
    _Out_writes_to_opt_(OutputBufferLength, *ResultLength)
                                      PVOID         OutputBuffer,
    _Out_                             PULONG        ResultLength
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(CapabilityType);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(OutputBuffer);
    *ResultLength = 0;
    OHCI_SPIKE_LOG("EvtControllerQueryUsbCapability (stub) -> STATUS_NOT_IMPLEMENTED");
    return STATUS_NOT_IMPLEMENTED;
}

NTSTATUS
EvtControllerGetCurrentFrameNumber(
    _In_  UCXCONTROLLER UcxController,
    _Out_ PULONG        FrameNumber
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    *FrameNumber = 0;
    OHCI_SPIKE_LOG("EvtControllerGetCurrentFrameNumber (stub) -> STATUS_NOT_IMPLEMENTED");
    return STATUS_NOT_IMPLEMENTED;
}

VOID
EvtControllerReset(
    _In_ UCXCONTROLLER UcxController
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCI_SPIKE_LOG("EvtControllerReset (stub)");
}

NTSTATUS
EvtControllerGetTransportCharacteristics(
    _In_  UCXCONTROLLER UcxController,
    _Out_ PUCX_CONTROLLER_TRANSPORT_CHARACTERISTICS UcxControllerTransportCharacteristics
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    RtlZeroMemory(UcxControllerTransportCharacteristics,
                  sizeof(*UcxControllerTransportCharacteristics));
    OHCI_SPIKE_LOG("EvtControllerGetTransportCharacteristics (stub) -> STATUS_NOT_IMPLEMENTED");
    return STATUS_NOT_IMPLEMENTED;
}

VOID
EvtControllerSetTransportCharacteristicsChangeNotification(
    _In_ UCXCONTROLLER UcxController,
    _In_ UCX_CONTROLLER_TRANSPORT_CHARACTERISTICS_CHANGE_FLAGS ChangeNotificationFlags
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(ChangeNotificationFlags);
    OHCI_SPIKE_LOG("EvtControllerSetTransportCharacteristicsChangeNotification (stub)");
}
