/*++

Module Name:

    acpi_ident.c

Abstract:

    ACPI-bus implementation of Ohci_FillUcxControllerIdent (declared in
    func/shared/device_context.h).

    Stamps the UCX_CONTROLLER_CONFIG with ParentBusType=Acpi and an
    ACPI _HID-style identification triple (VendorId / DeviceId /
    RevisionId). The strings are set at compile time — ACPI doesn't
    expose a per-device interface for reading _HID at runtime the way
    PCI exposes BUS_INTERFACE_STANDARD, and UCX only needs them as
    opaque labels for its bring-up logic and ETW.

    The values below are placeholders for RK3588 USB1 (OHCI). If the
    target platform's ACPI tables advertise a different _HID, update
    the literals here — they don't need to match the .inx HardwareID
    exactly, but a recognisable value helps when reading UCX traces.

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>

#include "device_context.h"

VOID
Ohci_FillUcxControllerIdent(
    _In_ PDEVICE_CONTEXT          dc,
    _In_ PUCX_CONTROLLER_CONFIG   cfg
    )
{
    UNREFERENCED_PARAMETER(dc);
    UCX_CONTROLLER_CONFIG_SET_ACPI_INFO(cfg,
                                         "RKCP",       /* VendorId  */
                                         "0001",       /* DeviceId  */
                                         "00");        /* RevisionId */
}
