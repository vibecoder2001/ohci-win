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
#include "ohci_bulk.h"
#include "ohci_interrupt.h"
#include "ohci_urb.h"
#include "ohci_dma.h"
#include "ohci_ed.h"
#include "ohci_regs.h"
#include "ohci_drain.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* --------------------------------------------------------------------------
 * Synchronous SETUP-only Control transfer helper (Plan 7).
 *
 * Used by the SET_ADDRESS path to issue a real SET_ADDRESS on the wire.
 * This routine MUST run at PASSIVE_LEVEL because it KeWaits on the URB
 * completion event. EvtUsbDeviceAddress, however, can fire at DISPATCH
 * (UCX delivers it from a timer DPC) so the caller is responsible for
 * dispatching here via a WDF workitem (see OhciPci_SetAddressWorker).
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

    /* USB §9.2.6.3 gives the device 50 ms for SET_ADDRESS itself, but the
     * end-to-end wall clock here also has to absorb workitem scheduling and
     * the WDH DPC latency. 1 s is plenty and still bounded. */
    LARGE_INTEGER timeout;
    timeout.QuadPart = -1LL * 1000LL * 1000LL * 10LL; /* 1 s */
    NTSTATUS w = KeWaitForSingleObject(&s.done, Executive, KernelMode, FALSE, &timeout);

    WdfSpinLockAcquire(dc->CoreLock);
    /* Critical: if the URB never retired (timeout), it is still on
     * hc->in_flight pointing at our stack frame. Cancel for the EP's ED
     * pulls it off (and halts the ED so the HC stops touching the TDs).
     * Without this, the next walker of in_flight derefs freed stack memory. */
    if (w != STATUS_SUCCESS) {
        ohci_urb_cancel_for_ed(&dc->Hc, ep0->Core.Control.ed);
    }
    OhciPci_BounceFree(dc, setupBounce);
    WdfSpinLockRelease(dc->CoreLock);

    if (w != STATUS_SUCCESS) return STATUS_IO_TIMEOUT;
    return (s.urb.status == OHCI_URB_STATUS_OK)
            ? STATUS_SUCCESS
            : STATUS_DEVICE_DATA_ERROR;
}

/* Workitem context: carries the request + decoded args from
 * EvtUsbDeviceAddress (DISPATCH) down to the worker (PASSIVE). */
typedef struct _OHCIPCI_SETADDR_CTX {
    WDFREQUEST           Request;
    OHCIPCI_USBDEV_CTX  *Udc;
    OHCIPCI_EP_CONTEXT  *Ep0;
    UCHAR                NewAddr;
} OHCIPCI_SETADDR_CTX, *POHCIPCI_SETADDR_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(OHCIPCI_SETADDR_CTX, OhciPci_SetAddrCtxGet)

static EVT_WDF_WORKITEM OhciPci_SetAddressWorker;

_Use_decl_annotations_
static VOID
OhciPci_SetAddressWorker(WDFWORKITEM WorkItem)
{
    extern PDEVICE_CONTEXT g_DeviceContext;
    POHCIPCI_SETADDR_CTX ctx = OhciPci_SetAddrCtxGet(WorkItem);
    PDEVICE_CONTEXT dc = g_DeviceContext;

    UCHAR setup[8] = { 0x00, 0x05, ctx->NewAddr, 0x00, 0x00, 0x00, 0x00, 0x00 };
    NTSTATUS s = OhciPci_SyncSetupOnly(dc, ctx->Ep0, setup);
    LOG("UsbDeviceAddress[wi]: SyncSetupOnly addr=%u -> 0x%08X",
        ctx->NewAddr, s);
    if (!NT_SUCCESS(s)) {
        WdfRequestComplete(ctx->Request, s);
        WdfObjectDelete(WorkItem);
        return;
    }

    /* Rewrite the EP0 ED's func_addr field. Pause CLE around the edit
     * (OHCI §6.2.1) so the HC isn't mid-walk. */
    struct ohci_ed *ed = ctx->Ep0->Core.Control.ed;
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
    ed->Control = (ed->Control & ~0x7Fu) | ((uint32_t)ctx->NewAddr & 0x7F);
    dc->MmioOps.barrier(dc->MmioOps.context);
    if (was_enabled) {
        dc->MmioOps.write32(dc->MmioOps.context, 0x04, hc_ctrl | OHCI_CTRL_CLE);
    }
    ctx->Udc->FuncAddr = ctx->NewAddr;
    WdfSpinLockRelease(dc->CoreLock);

    LOG("UsbDeviceAddress[wi]: ED.func_addr rewritten to %u", ctx->NewAddr);
    WdfRequestComplete(ctx->Request, STATUS_SUCCESS);
    WdfObjectDelete(WorkItem);
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

    if (udc == NULL || udc->Ep0 == NULL) {
        LOG("UsbDeviceAddress: per-device context or EP0 missing");
        WdfRequestComplete(Request, STATUS_INVALID_DEVICE_STATE);
        return;
    }

    /* USBDEVICE_ADDRESS.Address is OUT, not IN — UCX expects the driver
     * to allocate a fresh USB address (1..127) and write it back into the
     * struct before completion. dwusb does the same via USBPORT_AllocateUsbAddress.
     * We use a simple monotonic allocator on the device context. */
    LONG allocated = InterlockedIncrement(&g_DeviceContext->NextUsbAddress);
    UCHAR newAddr = (UCHAR)(((allocated - 1) % 127) + 1);  /* 1..127 */
    addrPkt->Address = newAddr;

    LOG("UsbDeviceAddress: allocated addr=%u (current=%u) — deferring to workitem",
        newAddr, udc->FuncAddr);

    /* The actual SET_ADDRESS work (sync URB submit + KeWait + ED rewrite)
     * needs PASSIVE. UCX delivers this callback from a timer DPC at
     * DISPATCH, so we queue a workitem and return; the worker completes
     * the request when SET_ADDRESS lands on the wire. */
    WDF_WORKITEM_CONFIG wic;
    WDF_WORKITEM_CONFIG_INIT(&wic, OhciPci_SetAddressWorker);

    WDF_OBJECT_ATTRIBUTES wia;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&wia, OHCIPCI_SETADDR_CTX);
    wia.ParentObject = g_DeviceContext->Device;

    WDFWORKITEM wi;
    NTSTATUS ws = WdfWorkItemCreate(&wic, &wia, &wi);
    if (!NT_SUCCESS(ws)) {
        LOG("UsbDeviceAddress: WdfWorkItemCreate -> 0x%08X", ws);
        WdfRequestComplete(Request, ws);
        return;
    }
    POHCIPCI_SETADDR_CTX ctx = OhciPci_SetAddrCtxGet(wi);
    ctx->Request = Request;
    ctx->Udc     = udc;
    ctx->Ep0     = udc->Ep0;
    ctx->NewAddr = newAddr;
    WdfWorkItemEnqueue(wi);
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

    WDF_REQUEST_PARAMETERS params;
    WDF_REQUEST_PARAMETERS_INIT(&params);
    WdfRequestGetParameters(Request, &params);
    PENDPOINTS_CONFIGURE ec = (PENDPOINTS_CONFIGURE)params.Parameters.Others.Arg1;

    if (ec == NULL) {
        LOG("UsbDeviceEndpointsConfigure: NULL params");
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }

    LOG("UsbDeviceEndpointsConfigure: enable=%lu disable=%lu unchanged=%lu",
        ec->EndpointsToEnableCount,
        ec->EndpointsToDisableCount,
        ec->EndpointsEnabledAndUnchangedCount);

    /* Tear down EDs for endpoints UCX is removing from the active set.
     * Without this, EndpointAdd for the same logical pipe (which UCX calls
     * for the new altsetting before disabling the old one) leaves the old
     * ED on hc->bulk_head / interrupt skeleton — the device sees two EDs
     * polling its EP and qemu's MSD STALLs the resulting confusion. */
    PDEVICE_CONTEXT dc = g_DeviceContext;
    for (ULONG i = 0; i < ec->EndpointsToDisableCount; i++) {
        UCXENDPOINT ucxEp = ec->EndpointsToDisable[i];
        OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(ucxEp);
        if (ep == NULL) continue;
        WdfSpinLockAcquire(dc->CoreLock);
        switch (ep->Kind) {
        case OhciPciEpKindBulk:
            ohci_bulk_endpoint_destroy(&dc->Hc, &ep->Core.Bulk);
            break;
        case OhciPciEpKindInterrupt:
            ohci_interrupt_endpoint_destroy(&dc->Hc, &ep->Core.Interrupt);
            break;
        case OhciPciEpKindControl:
            ohci_control_endpoint_destroy(&dc->Hc, &ep->Core.Control);
            break;
        }
        WdfSpinLockRelease(dc->CoreLock);
        LOG("  disabled EP kind=%d", ep->Kind);
    }

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

    InitializeListHead(&udc->EndpointList);

    WDF_OBJECT_ATTRIBUTES lockAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT(&lockAttrs);
    lockAttrs.ParentObject = usbDevice;
    NTSTATUS lockSt = WdfSpinLockCreate(&lockAttrs, &udc->EndpointListLock);
    if (!NT_SUCCESS(lockSt)) {
        LOG("UsbDeviceAdd: WdfSpinLockCreate -> 0x%08X", lockSt);
        return lockSt;
    }

    LOG("UsbDevContext attached: speed=%d", (int)udc->Speed);
    return status;
}

/* OhciPci_DefaultEndpointAdd and OhciPci_EndpointAdd were temporary stubs
 * here in Task 5. Task 6 deleted them; real implementations are in
 * ucx_endpoint.c. */
