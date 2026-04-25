#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"

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

    WDF_OBJECT_ATTRIBUTES attrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attrs, DEVICE_CONTEXT);

    WDFDEVICE device;
    status = WdfDeviceCreate(&DeviceInit, &attrs, &device);
    LOG("WdfDeviceCreate -> 0x%08X", status);
    if (!NT_SUCCESS(status)) return status;

    PDEVICE_CONTEXT dc = DeviceContextGet(device);
    RtlZeroMemory(dc, sizeof(*dc));
    dc->Device = device;
    OhciPci_InitMmioOps(dc);
    LOG("Context attached and mmio_ops wired");
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS EvtDevicePrepareHardware(WDFDEVICE Device,
                                  WDFCMRESLIST ResourcesRaw,
                                  WDFCMRESLIST ResourcesTranslated) {
    UNREFERENCED_PARAMETER(ResourcesRaw);
    PDEVICE_CONTEXT dc = DeviceContextGet(Device);
    LOG("EvtDevicePrepareHardware entered");

    BOOLEAN haveMemory = FALSE;
    BOOLEAN haveInterrupt = FALSE;
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);

    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR desc =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (!desc) continue;

        if (desc->Type == CmResourceTypeMemory && !haveMemory) {
            dc->MmioLength = desc->u.Memory.Length;
            dc->MmioBase   = MmMapIoSpaceEx(desc->u.Memory.Start,
                                            dc->MmioLength,
                                            PAGE_READWRITE | PAGE_NOCACHE);
            if (!dc->MmioBase) {
                LOG("MmMapIoSpaceEx failed for BAR at 0x%llX (len 0x%lX)",
                    desc->u.Memory.Start.QuadPart, (ULONG)dc->MmioLength);
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            LOG("MMIO base 0x%p, length 0x%lX",
                dc->MmioBase, (ULONG)dc->MmioLength);
            haveMemory = TRUE;
        } else if (desc->Type == CmResourceTypeInterrupt && !haveInterrupt) {
            dc->InterruptVector = desc->u.Interrupt.Vector;
            dc->InterruptIrql   = (KIRQL)desc->u.Interrupt.Level;
            dc->InterruptMode   = (desc->Flags & CM_RESOURCE_INTERRUPT_LATCHED)
                                  ? Latched : LevelSensitive;
            LOG("Interrupt vector %u, IRQL %u, mode %d",
                dc->InterruptVector, dc->InterruptIrql, dc->InterruptMode);
            haveInterrupt = TRUE;
        }
    }

    if (!haveMemory) {
        LOG("No memory BAR found");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }
    if (!haveInterrupt) {
        LOG("No interrupt resource found");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    /* Read HcRevision to confirm we can talk to the controller. OHCI 1.0a
     * registers reads return 0x10 in the low byte. */
    ULONG revision = READ_REGISTER_ULONG((PULONG)dc->MmioBase);
    LOG("HcRevision = 0x%08X (low byte 0x10 means OHCI 1.0)", revision);

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS EvtDeviceReleaseHardware(WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated) {
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PDEVICE_CONTEXT dc = DeviceContextGet(Device);
    LOG("EvtDeviceReleaseHardware");

    if (dc->MmioBase) {
        MmUnmapIoSpace(dc->MmioBase, dc->MmioLength);
        dc->MmioBase = NULL;
        dc->MmioLength = 0;
    }
    return STATUS_SUCCESS;
}
