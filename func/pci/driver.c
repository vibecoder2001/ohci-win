#include <ntddk.h>
#include <wdf.h>

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

DRIVER_INITIALIZE                  DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD          EvtDriverDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE    EvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE    EvtDeviceReleaseHardware;

_Use_decl_annotations_
NTSTATUS DriverEntry(PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath) {
    WDF_DRIVER_CONFIG config;
    WDF_DRIVER_CONFIG_INIT(&config, EvtDriverDeviceAdd);
    LOG("DriverEntry");
    return WdfDriverCreate(DriverObject, RegistryPath,
                           WDF_NO_OBJECT_ATTRIBUTES, &config, WDF_NO_HANDLE);
}

_Use_decl_annotations_
NTSTATUS EvtDriverDeviceAdd(WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit) {
    UNREFERENCED_PARAMETER(Driver);
    NTSTATUS status;
    LOG("EvtDriverDeviceAdd");

    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = EvtDevicePrepareHardware;
    pnp.EvtDeviceReleaseHardware = EvtDeviceReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDFDEVICE device;
    status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES, &device);
    LOG("WdfDeviceCreate -> 0x%08X", status);
    return status;
}

_Use_decl_annotations_
NTSTATUS EvtDevicePrepareHardware(WDFDEVICE Device,
                                  WDFCMRESLIST ResourcesRaw,
                                  WDFCMRESLIST ResourcesTranslated) {
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesRaw);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    LOG("EvtDevicePrepareHardware (skeleton)");
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS EvtDeviceReleaseHardware(WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated) {
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    LOG("EvtDeviceReleaseHardware");
    return STATUS_SUCCESS;
}
