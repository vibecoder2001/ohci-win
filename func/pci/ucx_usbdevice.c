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
#include <usbioctl.h>
#include "device_context.h"
#include "ohci_control.h"
#include "ohci_urb.h"
#include "ohci_dma.h"
#include "ohci_ed.h"
#include "ohci_regs.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * Synchronous SETUP-only Control transfer helper (Plan 7).
 *
 * Used by EvtUsbDeviceAddress to issue a real SET_ADDRESS on the wire.
 * EvtUsbDeviceAddress is __drv_maxIRQL(DISPATCH_LEVEL) per ucxusbdevice.h
 * — in practice during enumeration UCX dispatches at PASSIVE so KeWait is
 * legal. Asserts PASSIVE; if a future Windows build delivers Address at
 * DISPATCH this will trip and we'll switch to a WDFWORKITEM.
 * -------------------------------------------------------------------------- */
typedef struct _OHCIPCI_SYNC_URB {
    struct ohci_urb urb;
    KEVENT          done;
} OHCIPCI_SYNC_URB;

static VOID OhciPci_SyncUrbComplete(struct ohci_urb *u) {
    OHCIPCI_SYNC_URB *s = CONTAINING_RECORD(u, OHCIPCI_SYNC_URB, urb);
    KeSetEvent(&s->done, IO_NO_INCREMENT, FALSE);
}

static NTSTATUS
OhciPci_SyncSetupOnly(PDEVICE_CONTEXT dc,
                      OHCIPCI_EP_CONTEXT *ep0,
                      const UCHAR setup[8])
{
    NT_ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);

    uint32_t setup_phys;
    void *setupBounce;

    WdfSpinLockAcquire(dc->CoreLock);
    setupBounce = OhciPci_BounceAlloc(dc, &setup_phys);
    WdfSpinLockRelease(dc->CoreLock);
    if (setupBounce == NULL) return STATUS_INSUFFICIENT_RESOURCES;
    RtlCopyMemory(setupBounce, setup, 8);

    OHCIPCI_SYNC_URB s;
    RtlZeroMemory(&s, sizeof(s));
    KeInitializeEvent(&s.done, NotificationEvent, FALSE);
    RtlCopyMemory(s.urb.setup, setup, 8);
    s.urb.setup_phys = setup_phys;
    s.urb.buffer     = NULL;
    s.urb.length     = 0;
    s.urb.direction  = OHCI_URB_DIR_IN;
    s.urb.complete   = OhciPci_SyncUrbComplete;

    WdfSpinLockAcquire(dc->CoreLock);
    int rc = ohci_control_submit(&dc->Hc, &ep0->Core.Control, &s.urb);
    WdfSpinLockRelease(dc->CoreLock);
    if (rc != 0) {
        WdfSpinLockAcquire(dc->CoreLock);
        OhciPci_BounceFree(dc, setupBounce);
        WdfSpinLockRelease(dc->CoreLock);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    LARGE_INTEGER timeout;
    timeout.QuadPart = -50LL * 1000LL * 10LL; /* 50 ms */
    NTSTATUS w = KeWaitForSingleObject(&s.done, Executive, KernelMode, FALSE, &timeout);

    WdfSpinLockAcquire(dc->CoreLock);
    OhciPci_BounceFree(dc, setupBounce);
    WdfSpinLockRelease(dc->CoreLock);

    if (w != STATUS_SUCCESS) return STATUS_IO_TIMEOUT;
    return (s.urb.status == OHCI_URB_STATUS_OK)
            ? STATUS_SUCCESS
            : STATUS_DEVICE_DATA_ERROR;
}

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
    extern PDEVICE_CONTEXT g_DeviceContext;

    WDF_REQUEST_PARAMETERS rp;
    WDF_REQUEST_PARAMETERS_INIT(&rp);
    WdfRequestGetParameters(Request, &rp);
    PUSBDEVICE_ADDRESS addrPkt = (PUSBDEVICE_ADDRESS)rp.Parameters.Others.Arg1;
    if (addrPkt == NULL) {
        LOG("UsbDeviceAddress: missing addr packet");
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    /* USBDEVICE_MGMT_HEADER carries UCXUSBDEVICE UsbDevice via an anon
     * struct member; reach it through the typed pointer cast. */
    UCXUSBDEVICE usbDev = addrPkt->UsbDevice;
    OHCIPCI_USBDEV_CTX *udc = OhciPci_UsbDevContextGet(usbDev);
    UCHAR newAddr = (UCHAR)(addrPkt->Address & 0x7F);

    if (udc == NULL || udc->Ep0 == NULL) {
        LOG("UsbDeviceAddress: per-device context or EP0 missing");
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }

    LOG("UsbDeviceAddress: requested addr=%u (current=%u)", newAddr, udc->FuncAddr);

    /* Build SET_ADDRESS SETUP packet (USB §9.4.6):
     *   bmRequestType=0x00 (host->dev, std, device)
     *   bRequest=0x05 (SET_ADDRESS)
     *   wValue=newAddr  wIndex=0  wLength=0  (no data stage) */
    UCHAR setup[8] = { 0x00, 0x05, newAddr, 0x00, 0x00, 0x00, 0x00, 0x00 };
    NTSTATUS s = OhciPci_SyncSetupOnly(g_DeviceContext, udc->Ep0, setup);
    LOG("UsbDeviceAddress: SyncSetupOnly -> 0x%08X", s);
    if (!NT_SUCCESS(s)) {
        WdfRequestComplete(Request, s);
        return;
    }

    /* Device is now at newAddr (USB §9.4.6 says 2 ms transition; we'll
     * naturally wait that out before the next URB). Rewrite the EP0 ED
     * func_addr so all subsequent transfers go to the right address.
     * Pause CLE around the rewrite per OHCI §6.2.1 to avoid racing the HC
     * mid-walk. */
    struct ohci_ed *ed = udc->Ep0->Core.Control.ed;
    PDEVICE_CONTEXT dc = g_DeviceContext;

    WdfSpinLockAcquire(dc->CoreLock);
    uint32_t hc_ctrl = dc->MmioOps.read32(dc->MmioOps.context, 0x04);
    int was_enabled = (hc_ctrl & OHCI_CTRL_CLE) != 0;
    if (was_enabled) {
        dc->MmioOps.write32(dc->MmioOps.context, 0x04, hc_ctrl & ~OHCI_CTRL_CLE);
        dc->MmioOps.barrier(dc->MmioOps.context);
        uint32_t f0 = dc->MmioOps.read32(dc->MmioOps.context, 0x3C);
        for (int i = 0; i < 10000; i++) {
            uint32_t f = dc->MmioOps.read32(dc->MmioOps.context, 0x3C);
            if (f != f0) break;
        }
    }
    /* OHCI §4.2.2: ED.Control bits [6:0] = function address. */
    ed->Control = (ed->Control & ~0x7Fu) | ((uint32_t)newAddr & 0x7F);
    dc->MmioOps.barrier(dc->MmioOps.context);
    if (was_enabled) {
        dc->MmioOps.write32(dc->MmioOps.context, 0x04, hc_ctrl | OHCI_CTRL_CLE);
    }
    udc->FuncAddr = newAddr;
    WdfSpinLockRelease(dc->CoreLock);

    LOG("UsbDeviceAddress: ED.func_addr rewritten to %u", newAddr);
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

    WDF_OBJECT_ATTRIBUTES devAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&devAttrs, OHCIPCI_USBDEV_CTX);

    UCXUSBDEVICE usbDevice;
    NTSTATUS status = UcxUsbDeviceCreate(Controller,
                                         &UsbDeviceInit,
                                         &devAttrs,
                                         &usbDevice);
    LOG("UcxUsbDeviceCreate -> 0x%08X", status);
    if (!NT_SUCCESS(status)) return status;

    OHCIPCI_USBDEV_CTX *udc = OhciPci_UsbDevContextGet(usbDevice);
    RtlZeroMemory(udc, sizeof(*udc));
    udc->Speed    = UsbDeviceInfo->DeviceSpeed;
    udc->FuncAddr = 0;
    LOG("UsbDevContext attached: speed=%d", (int)udc->Speed);
    return status;
}

/* OhciPci_DefaultEndpointAdd and OhciPci_EndpointAdd were temporary stubs
 * here in Task 5. Task 6 deleted them; real implementations are in
 * ucx_endpoint.c. */
