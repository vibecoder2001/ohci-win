/*++

Module Name:

    ucx_usbdevice.c

Abstract:

    UCX 1.6 USB device object creation for OhciPci.

    Implements OhciPci_UsbDeviceAdd, the EvtControllerUsbDeviceAdd callback
    registered in ucx_glue.c. When UCX wants to enumerate a USB device it
    calls this; we allocate a UCXUSBDEVICE and supply per-device callbacks.

    OhciPci_DefaultEndpointAdd and OhciPci_EndpointAdd are implemented in
    ucx_endpoint.c (added in Task 6). This file contains only UsbDeviceAdd.

=== UCX 1.6 USB-device API surface (WDK 10.0.26100.0, ucxusbdevice.h) ===

  Input info struct:
      UCXUSBDEVICE_INFO  (NOT UCX_USBDEVICE_INFO)
          .Size           — sizeof sentinel
          .DeviceSpeed    — USB_DEVICE_SPEED enum
          .TtHub          — UCXUSBDEVICE transaction-translator hub (or NULL)
          .PortPath       — USB_DEVICE_PORT_PATH (PortPathDepth, PortPath[])
      No UsbDeviceAddress field — address is not in the info struct.

  Init / create:
      UcxUsbDeviceInitSetEventCallbacks(PUCXUSBDEVICE_INIT, PUCX_USBDEVICE_EVENT_CALLBACKS)
      UcxUsbDeviceCreate(UCXCONTROLLER, PUCXUSBDEVICE_INIT*, PWDF_OBJECT_ATTRIBUTES, UCXUSBDEVICE*)
          Note: second arg is double-pointer (PUCXUSBDEVICE_INIT*), matching
          the deref-inout annotation in the header.

  Callbacks struct:
      UCX_USBDEVICE_EVENT_CALLBACKS  (13 fields, all required except last 3)
      UCX_USBDEVICE_EVENT_CALLBACKS_INIT(Callbacks,
          EndpointsConfigure,   // PFN_UCX_USBDEVICE_ENDPOINTS_CONFIGURE
          Enable,               // PFN_UCX_USBDEVICE_ENABLE
          Disable,              // PFN_UCX_USBDEVICE_DISABLE
          Reset,                // PFN_UCX_USBDEVICE_RESET
          Address,              // PFN_UCX_USBDEVICE_ADDRESS
          Update,               // PFN_UCX_USBDEVICE_UPDATE
          HubInfo,              // PFN_UCX_USBDEVICE_HUB_INFO
          DefaultEndpointAdd,   // PFN_UCX_USBDEVICE_DEFAULT_ENDPOINT_ADD
          EndpointAdd)          // PFN_UCX_USBDEVICE_ENDPOINT_ADD
      Note: INIT macro only sets the first 9 callbacks. The remaining three
      (Suspend, Resume, GetCharacteristic) default to NULL from RtlZeroMemory.

  Per-device callback signatures:
      EVT_UCX_USBDEVICE_ENABLE(UCXCONTROLLER, WDFREQUEST)         VOID
      EVT_UCX_USBDEVICE_DISABLE(UCXCONTROLLER, WDFREQUEST)        VOID
      EVT_UCX_USBDEVICE_RESET(UCXCONTROLLER, WDFREQUEST)          VOID
      EVT_UCX_USBDEVICE_ADDRESS(UCXCONTROLLER, WDFREQUEST)        VOID
      EVT_UCX_USBDEVICE_UPDATE(UCXCONTROLLER, WDFREQUEST)         VOID
      EVT_UCX_USBDEVICE_HUB_INFO(UCXCONTROLLER, WDFREQUEST)       VOID
      EVT_UCX_USBDEVICE_ENDPOINTS_CONFIGURE(UCXCONTROLLER, WDFREQUEST) VOID
      EVT_UCX_USBDEVICE_DEFAULT_ENDPOINT_ADD(UCXCONTROLLER, UCXUSBDEVICE,
          ULONG MaxPacketSize, PUCXENDPOINT_INIT)                 NTSTATUS
      EVT_UCX_USBDEVICE_ENDPOINT_ADD(UCXCONTROLLER, UCXUSBDEVICE,
          PUSB_ENDPOINT_DESCRIPTOR, ULONG BufLength,
          PUSB_SUPERSPEED_ENDPOINT_COMPANION_DESCRIPTOR,
          PUCXENDPOINT_INIT)                                       NTSTATUS

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include "device_context.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * Forward declarations for endpoint callbacks.
 *
 * OhciPci_DefaultEndpointAdd is declared extern here; a temporary stub
 * implementation lives at the bottom of this file. Task 6 deletes that stub
 * and provides the real body in ucx_endpoint.c.
 *
 * OhciPci_EndpointAdd (non-default endpoints) is Plan 6; same pattern.
 * -------------------------------------------------------------------------- */
extern EVT_UCX_USBDEVICE_DEFAULT_ENDPOINT_ADD  OhciPci_DefaultEndpointAdd;
extern EVT_UCX_USBDEVICE_ENDPOINT_ADD          OhciPci_EndpointAdd;

/* --------------------------------------------------------------------------
 * Stubs for WDFREQUEST-based per-device callbacks.
 *
 * All VOID return; they complete the request with STATUS_SUCCESS.
 * Data exchange is through WdfRequestRetrieveOutputBuffer on the request
 * (same pattern as root-hub callbacks confirmed in Plan 5 Tasks 2-4).
 * -------------------------------------------------------------------------- */

static EVT_UCX_USBDEVICE_ENABLE               StubUsbDeviceEnable;
static EVT_UCX_USBDEVICE_DISABLE              StubUsbDeviceDisable;
static EVT_UCX_USBDEVICE_RESET                StubUsbDeviceReset;
static EVT_UCX_USBDEVICE_ADDRESS              StubUsbDeviceAddress;
static EVT_UCX_USBDEVICE_UPDATE               StubUsbDeviceUpdate;
static EVT_UCX_USBDEVICE_HUB_INFO             StubUsbDeviceHubInfo;
static EVT_UCX_USBDEVICE_ENDPOINTS_CONFIGURE  StubUsbDeviceEndpointsConfigure;

_Use_decl_annotations_
static VOID
StubUsbDeviceEnable(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceEnable (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceDisable(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceDisable (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceReset(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceReset (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceAddress(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceAddress (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceUpdate(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceUpdate (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceHubInfo(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceHubInfo (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
StubUsbDeviceEndpointsConfigure(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("UsbDeviceEndpointsConfigure (stub)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* --------------------------------------------------------------------------
 * OhciPci_UsbDeviceAdd
 *
 * EvtControllerUsbDeviceAdd — called by UCX when it wants to enumerate a new
 * USB device on one of our ports.
 *
 * Matches EVT_UCX_CONTROLLER_USBDEVICE_ADD:
 *   (UCXCONTROLLER, PUCXUSBDEVICE_INFO, PUCXUSBDEVICE_INIT)  NTSTATUS
 *
 * Note: info struct is UCXUSBDEVICE_INFO (confirmed from header), not the
 * plan's approximate name UCX_USBDEVICE_INFO.
 *
 * Note: UcxUsbDeviceCreate takes PUCXUSBDEVICE_INIT* (double-pointer).
 * We pass &UsbDeviceInit which already is PUCXUSBDEVICE_INIT, so &init
 * gives PUCXUSBDEVICE_INIT* as required.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS
OhciPci_UsbDeviceAdd(
    _In_ UCXCONTROLLER      Controller,
    _In_ PUCXUSBDEVICE_INFO UsbDeviceInfo,
    _In_ PUCXUSBDEVICE_INIT UsbDeviceInit
    )
{
    LOG("UsbDeviceAdd: speed=%d portDepth=%lu",
        (int)UsbDeviceInfo->DeviceSpeed,
        UsbDeviceInfo->PortPath.PortPathDepth);

    /*
     * Build the per-device event callbacks struct.
     *
     * UCX_USBDEVICE_EVENT_CALLBACKS_INIT requires exactly 9 named callbacks
     * (EndpointsConfigure through EndpointAdd). The three optional callbacks
     * (Suspend, Resume, GetCharacteristic) are zeroed by RtlZeroMemory inside
     * the macro and left NULL — UCX treats NULL as "not implemented."
     */
    UCX_USBDEVICE_EVENT_CALLBACKS cbs;
    UCX_USBDEVICE_EVENT_CALLBACKS_INIT(
        &cbs,
        StubUsbDeviceEndpointsConfigure,  /* EvtUsbDeviceEndpointsConfigure */
        StubUsbDeviceEnable,              /* EvtUsbDeviceEnable             */
        StubUsbDeviceDisable,             /* EvtUsbDeviceDisable            */
        StubUsbDeviceReset,               /* EvtUsbDeviceReset              */
        StubUsbDeviceAddress,             /* EvtUsbDeviceAddress            */
        StubUsbDeviceUpdate,              /* EvtUsbDeviceUpdate             */
        StubUsbDeviceHubInfo,             /* EvtUsbDeviceHubInfo            */
        OhciPci_DefaultEndpointAdd,       /* EvtUsbDeviceDefaultEndpointAdd */
        OhciPci_EndpointAdd               /* EvtUsbDeviceEndpointAdd        */
    );

    UcxUsbDeviceInitSetEventCallbacks(UsbDeviceInit, &cbs);

    UCXUSBDEVICE usbDevice;
    NTSTATUS status = UcxUsbDeviceCreate(Controller,
                                         &UsbDeviceInit,
                                         WDF_NO_OBJECT_ATTRIBUTES,
                                         &usbDevice);
    LOG("UcxUsbDeviceCreate -> 0x%08X", status);
    return status;
}

/* OhciPci_DefaultEndpointAdd and OhciPci_EndpointAdd were temporary stubs
 * here in Task 5. Task 6 deleted them; real implementations are in
 * ucx_endpoint.c. */
