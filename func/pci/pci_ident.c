/*++

Module Name:

    pci_ident.c

Abstract:

    PCI-bus implementation of Ohci_FillUcxControllerIdent (declared in
    func/shared/device_context.h, called by shared/ucx_glue.c during
    UcxControllerCreate).

    Reads VID/DID/REV from the device's PCI config space via the
    BUS_INTERFACE_STANDARD the PCI bus driver exposes, then populates
    the UCX_CONTROLLER_CONFIG with ParentBusType=Pci. Without this UCX
    defaults to ParentBusTypeCustom + bogus VID/DID (LONG_MAX) and the
    bring-up sequence loops on EvtControllerReset.

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#include <wdmguid.h>
#include <UcxClass.h>

#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

VOID
Ohci_FillUcxControllerIdent(
    _In_ PDEVICE_CONTEXT          dc,
    _In_ PUCX_CONTROLLER_CONFIG   cfg
    )
{
    BUS_INTERFACE_STANDARD bus = {0};
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(dc->Device);
    if (!pdo) {
        LOG("FillUcxControllerIdent: no PDO");
        return;
    }

    KEVENT ev;
    IO_STATUS_BLOCK iosb;
    KeInitializeEvent(&ev, NotificationEvent, FALSE);
    PIRP irp = IoBuildSynchronousFsdRequest(IRP_MJ_PNP, pdo,
                                            NULL, 0, NULL, &ev, &iosb);
    if (!irp) {
        LOG("FillUcxControllerIdent: IoBuildSynchronousFsdRequest failed");
        return;
    }

    PIO_STACK_LOCATION s = IoGetNextIrpStackLocation(irp);
    s->MajorFunction = IRP_MJ_PNP;
    s->MinorFunction = IRP_MN_QUERY_INTERFACE;
    s->Parameters.QueryInterface.InterfaceType         = &GUID_BUS_INTERFACE_STANDARD;
    s->Parameters.QueryInterface.Size                  = sizeof(bus);
    s->Parameters.QueryInterface.Version               = 1;
    s->Parameters.QueryInterface.Interface             = (PINTERFACE)&bus;
    s->Parameters.QueryInterface.InterfaceSpecificData = NULL;
    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;

    NTSTATUS qiSt = IoCallDriver(pdo, irp);
    if (qiSt == STATUS_PENDING) {
        KeWaitForSingleObject(&ev, Executive, KernelMode, FALSE, NULL);
        qiSt = iosb.Status;
    }
    if (!NT_SUCCESS(qiSt) || !bus.GetBusData) {
        LOG("BUS_INTERFACE_STANDARD query failed: 0x%08X", qiSt);
        return;
    }

    ULONG vidDid = 0;
    bus.GetBusData(bus.Context, PCI_WHICHSPACE_CONFIG, &vidDid, 0, sizeof(vidDid));
    USHORT vid = (USHORT)(vidDid & 0xFFFF);
    USHORT did = (USHORT)(vidDid >> 16);
    UCHAR rev = 0;
    bus.GetBusData(bus.Context, PCI_WHICHSPACE_CONFIG, &rev, 0x08, sizeof(rev));
    LOG("PCI VID=0x%04X DID=0x%04X REV=0x%02X", vid, did, rev);
    UCX_CONTROLLER_CONFIG_SET_PCI_INFO(cfg, vid, did, rev, 0, 0, 0);
    if (bus.InterfaceDereference) {
        bus.InterfaceDereference(bus.Context);
    }
}
