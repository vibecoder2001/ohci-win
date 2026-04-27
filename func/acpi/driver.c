/*++

Module Name:

    driver.c (ACPI variant)

Abstract:

    KMDF entry points for the ACPI-enumerated OHCI function driver.
    Mirrors func/pci/driver.c — the PnP resource walk is identical
    because WDF translates ACPI _CRS into the same
    CmResourceType{Memory,Interrupt} descriptors that the PCI bus
    driver produces. The only meaningful difference vs. the PCI
    variant is the bus-glue ident hook (acpi_ident.c instead of
    pci_ident.c).

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include "device_context.h"
#include "ohci_regs.h"
#include "ohci_log.h"

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

    NTSTATUS ucxInitStatus = OhciPci_UcxInitDeviceInit(DeviceInit);
    if (!NT_SUCCESS(ucxInitStatus)) return ucxInitStatus;

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

    /* Mark the device as user-disable-able in Device Manager. ACPI-enumerated
     * USB host controllers default to non-removable, which leaves the
     * "Disable Device" affordance greyed out. Setting Removable + SurpriseOK
     * lets the user (or pnputil /disable-device) stop the controller
     * cleanly without uninstalling the driver. */
    {
        WDF_DEVICE_PNP_CAPABILITIES pnpCaps;
        WDF_DEVICE_PNP_CAPABILITIES_INIT(&pnpCaps);
        pnpCaps.Removable         = WdfTrue;
        pnpCaps.SurpriseRemovalOK = WdfTrue;
        WdfDeviceSetPnpCapabilities(device, &pnpCaps);
    }

    {
        WDF_OBJECT_ATTRIBUTES lockAttrs;
        WDF_OBJECT_ATTRIBUTES_INIT(&lockAttrs);
        lockAttrs.ParentObject = device;
        NTSTATUS lockStatus = WdfSpinLockCreate(&lockAttrs, &dc->CoreLock);
        LOG("WdfSpinLockCreate (CoreLock) -> 0x%08X", lockStatus);
        if (!NT_SUCCESS(lockStatus)) return lockStatus;
    }
    InitializeListHead(&dc->DeferredCompletions);
    dc->NextUsbAddress = 0;

    InitializeListHead(&dc->IsocEps);
    {
        WDF_OBJECT_ATTRIBUTES iolAttrs;
        WDF_OBJECT_ATTRIBUTES_INIT(&iolAttrs);
        iolAttrs.ParentObject = device;
        NTSTATUS iolSt = WdfSpinLockCreate(&iolAttrs, &dc->IsocEpsLock);
        if (!NT_SUCCESS(iolSt)) {
            LOG("WdfSpinLockCreate (IsocEpsLock) failed 0x%08X — refill disabled",
                iolSt);
            dc->IsocEpsLock = NULL;
        }
    }
    {
        WDF_TIMER_CONFIG tcfg;
        WDF_TIMER_CONFIG_INIT_PERIODIC(&tcfg, OhciPci_EvtIsocBackstopTimer,
                                        OHCIPCI_ISOC_BACKSTOP_TIMER_MS);
        WDF_OBJECT_ATTRIBUTES tattrs;
        WDF_OBJECT_ATTRIBUTES_INIT(&tattrs);
        tattrs.ParentObject  = device;
        tattrs.ExecutionLevel = WdfExecutionLevelDispatch;
        NTSTATUS ts = WdfTimerCreate(&tcfg, &tattrs, &dc->IsocRefillTimer);
        if (!NT_SUCCESS(ts)) {
            LOG("WdfTimerCreate (isoc backstop) failed 0x%08X — refill"
                " runs only from EvtDpc", ts);
            dc->IsocRefillTimer = NULL;
        }
    }

    NTSTATUS qStatus = OhciPci_CreateDefaultQueue(dc);
    if (!NT_SUCCESS(qStatus)) {
        LOG("Default queue setup failed; refusing to start");
        return qStatus;
    }

    NTSTATUS isStatus = OhciPci_CreateInterrupt(dc);
    if (!NT_SUCCESS(isStatus)) {
        LOG("Interrupt setup failed; refusing to start");
        return isStatus;
    }
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
                LOG("MmMapIoSpaceEx failed for region at 0x%llX (len 0x%lX)",
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
        LOG("No memory resource found");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }
    if (!haveInterrupt) {
        LOG("No interrupt resource found");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    ULONG revision = READ_REGISTER_ULONG((PULONG)dc->MmioBase);
    LOG("HcRevision = 0x%08X (low byte 0x10 means OHCI 1.0)", revision);

    NTSTATUS status = OhciPci_AllocateDma(dc);
    if (!NT_SUCCESS(status)) {
        LOG("OhciPci_AllocateDma -> 0x%08X", status);
        return status;
    }
    LOG("DMA region 0x%lX bytes at virt %p / phys 0x%X",
        (ULONG)dc->DmaRegion.size, dc->DmaRegion.base, dc->DmaRegion.phys_base);

    struct ohci_hc_config hccfg = {
        .td_pool_size       = 256,
        .control_ed_count   = 16,
        .bulk_ed_count      = 16,
        .interrupt_ed_count = 16,
        .isoc_ed_count      = 4,
        .itd_pool_size      = 32,
    };
    int rc = ohci_hc_init(&dc->Hc, &dc->MmioOps, &dc->DmaRegion, &hccfg);
    LOG("ohci_hc_init -> %d", rc);
    if (rc != 0) {
        return STATUS_UNSUCCESSFUL;
    }
    dc->HcInitialized = TRUE;

    ULONG ctrl = READ_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + 0x04));
    LOG("HcControl after init = 0x%08X (expect HCFS=10 + CLE+BLE+PLE+IE)", ctrl);

    WRITE_REGISTER_ULONG((PULONG)((PUCHAR)dc->MmioBase + 0x10),
                         OHCI_INT_RHSC | OHCI_INT_UE | OHCI_INT_SO);
    LOG("RHSC + UE + SO interrupts enabled (WDH/MIE were set by hc_init)");

    NTSTATUS bcStatus = OhciPci_BounceInit(dc);
    LOG("OhciPci_BounceInit -> 0x%08X", bcStatus);
    if (!NT_SUCCESS(bcStatus)) return bcStatus;

    NTSTATUS ucxCtrlStatus = OhciPci_UcxControllerCreate(dc);
    if (!NT_SUCCESS(ucxCtrlStatus)) {
        LOG("UcxControllerCreate failed; returning failure to refuse start");
        return ucxCtrlStatus;
    }

    NTSTATUS rhStatus = OhciPci_RootHubCreate(dc, dc->Controller);
    if (!NT_SUCCESS(rhStatus)) {
        LOG("UcxRootHubCreate failed; refusing to start");
        return rhStatus;
    }

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
