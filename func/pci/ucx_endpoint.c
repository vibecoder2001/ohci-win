/*++

Module Name:

    ucx_endpoint.c

Abstract:

    UCX 1.6 default Control endpoint (EP0) implementation for OhciPci.

    Implements OhciPci_DefaultEndpointAdd and OhciPci_EndpointAdd.
    The former is real (Task 6); the latter is a Plan 6 stub.

    Also contains EvtUrbIoctl — the WDFQUEUE EvtIoInternalDeviceControl
    callback that receives URBs from UCX and submits them to the OHCI core.

=== UCX 1.6 Endpoint API surface (WDK 10.0.26100.0, ucxendpoint.h) ===

  Key finding: the default EP uses a SEPARATE callbacks struct from
  regular (non-default) endpoints:

    UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS / UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS_INIT
      Fields: EvtEndpointPurge, EvtEndpointStart, EvtEndpointAbort,
              EvtEndpointOkToCancelTransfers, EvtDefaultEndpointUpdate

    UcxDefaultEndpointInitSetEventCallbacks(PUCXENDPOINT_INIT, &ecb)
      — used for the DEFAULT endpoint (NOT UcxEndpointInitSetEventCallbacks)

  Regular endpoints use UCX_ENDPOINT_EVENT_CALLBACKS_INIT with 8 args
  (Purge, Start, Abort, Reset, OkToCancel, StaticStreamsAdd,
   StaticStreamsEnable, StaticStreamsDisable).

  UcxEndpointCreate(usbDevice, &init, attrs, &ucxEp)
    — DOUBLE-POINTER for init (PUCXENDPOINT_INIT*), same as UcxUsbDeviceCreate.

  URB delivery mechanism:
    UCX does NOT have a custom submit callback. Instead the driver creates a
    WDFQUEUE (standard WDF IO queue) and registers it with UCX via:
      UcxEndpointSetWdfIoQueue(ucxEp, queue)
    UCX forwards URBs as IOCTL_INTERNAL_USB_SUBMIT_URB requests to that queue.
    The queue's EvtIoInternalDeviceControl callback handles them.

  Purge / abort lifecycle:
    EVT_UCX_ENDPOINT_PURGE(controller, endpoint) — called when UCX wants to
      drain the endpoint; driver must purge the queue and call
      UcxEndpointPurgeComplete(endpoint) when done.
    EVT_UCX_ENDPOINT_ABORT(controller, endpoint) — similar; call
      UcxEndpointAbortComplete(endpoint) when done.
    EVT_UCX_ENDPOINT_START(controller, endpoint) — UCX resumes the endpoint.
    EVT_UCX_ENDPOINT_OK_TO_CANCEL_TRANSFERS(endpoint) — signal from UCX that
      pending cancels can now be processed.

  Queue parent/context note:
    WDF does not have a WdfIoQueueGetParentObject API (only DPC, timer, and
    workitem have typed GetParentObject helpers). To bridge from EvtUrbIoctl
    (which receives only the WDFQUEUE) back to the per-EP context we attach a
    tiny ohcipci_queue_ctx to the WDFQUEUE object that holds a back-pointer to
    the owning ohcipci_ep_context.

  g_DeviceContext:
    Declared extern here; defined as non-static in ucx_roothub.c (Task 6
    changes it from static to extern-linkage). This is the established single-
    instance shortcut used by the existing root-hub callbacks.

Environment:

    Kernel mode only.

--*/

#include <ntddk.h>
#include <wdf.h>
#include <UcxClass.h>
#include <usbioctl.h>
#include <usbdi.h>
#include "device_context.h"
#include "ohci_control.h"
#include "ohci_urb.h"
#include "ohci_dma.h"

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* Declared in ucx_roothub.c; shared across glue modules for single-instance driver. */
extern PDEVICE_CONTEXT g_DeviceContext;

/* --------------------------------------------------------------------------
 * Forward declarations for UCX endpoint callbacks (default EP flavour).
 * -------------------------------------------------------------------------- */
static EVT_UCX_ENDPOINT_PURGE               EvtDefaultEpPurge;
static EVT_UCX_ENDPOINT_START               EvtDefaultEpStart;
static EVT_UCX_ENDPOINT_ABORT               EvtDefaultEpAbort;
static EVT_UCX_ENDPOINT_OK_TO_CANCEL_TRANSFERS EvtDefaultEpOkToCancel;
static EVT_UCX_DEFAULT_ENDPOINT_UPDATE      EvtDefaultEpUpdate;

/* Forward declaration for URB delivery handler.
 *
 * UCX 1.6 delivers per-endpoint transfers via EvtIoDefault (NOT
 * EvtIoInternalDeviceControl, which only the root-hub queue receives from
 * UsbHub3). The request's Parameters.Others.Arg1 is a TRANSFER_URB —
 * a UCX-internal layout that is NOT the legacy URB struct. (Confirmed in
 * dwusb reference: Driver.h::TRANSFER_URB, UsbDevice.c::Control_WdfEvtIoDefault.)
 */
static EVT_WDF_IO_QUEUE_IO_DEFAULT EvtUrbDefault;

/* TRANSFER_URB layout, copied from the dwusb reference (Driver.h). The struct
 * is a UCX-private contract not exported in any public WDK header; UCX puts a
 * pointer to it in Parameters.Others.Arg1 of every request enqueued to a
 * per-EP WDFQUEUE. SetupPacket[8] lives at u.SetupPacket for control transfers. */
typedef struct _OHCIPCI_UCX_URB_DATA {
    PVOID Reserved[8];
} OHCIPCI_UCX_URB_DATA;

typedef struct _OHCIPCI_TRANSFER_URB {
    struct _URB_HEADER Hdr;
    PVOID UsbdPipeHandle;
    ULONG TransferFlags;
    ULONG TransferBufferLength;
    PVOID TransferBuffer;
    PMDL  TransferBufferMDL;
    union {
        ULONG Timeout;
        PVOID ReservedMBNull;
    };
    OHCIPCI_UCX_URB_DATA UrbData;
    union {
        struct {
            ULONG StartFrame;
            ULONG NumberOfPackets;
            ULONG ErrorCount;
            USBD_ISO_PACKET_DESCRIPTOR IsoPacket[1];
        } Isoch;
        UCHAR SetupPacket[8];
    } u;
} OHCIPCI_TRANSFER_URB, *POHCIPCI_TRANSFER_URB;

/* --------------------------------------------------------------------------
 * OhciPci_UrbComplete
 *
 * Called from OHCI core (interrupt context or DPC) when a Control URB
 * finishes. Copies IN data back to the caller's buffer and completes the
 * WDFREQUEST.
 * -------------------------------------------------------------------------- */
static VOID
OhciPci_UrbComplete(struct ohci_urb *u)
{
    OHCIPCI_URB_CTX *uc =
        CONTAINING_RECORD(u, OHCIPCI_URB_CTX, CoreUrb);

    /* IN data copy-back. */
    if (uc->DataDirection == OHCI_URB_DIR_IN &&
        uc->DataLength > 0 &&
        uc->DataBounce != NULL)
    {
        PVOID dst = NULL;
        if (uc->UserMdl) {
            dst = MmGetSystemAddressForMdlSafe(uc->UserMdl, NormalPagePriority);
        } else if (uc->UserVa) {
            dst = uc->UserVa;
        }
        if (dst) {
            RtlCopyMemory(dst, uc->DataBounce, uc->DataLength);
        }
    }

    NTSTATUS status = (u->status == OHCI_URB_STATUS_OK)
                          ? STATUS_SUCCESS
                          : STATUS_DEVICE_DATA_ERROR;
    ULONG_PTR info = (ULONG_PTR)u->transferred;
    LOG("UrbComplete: core_status=%d transferred=%u dir=%u",
        u->status, u->transferred, uc->DataDirection);

    /* Write the result back into the TRANSFER_URB. UCX reads
     * Hdr.Status + TransferBufferLength to learn how the transfer went;
     * the WDF request status alone is not enough. */
    POHCIPCI_TRANSFER_URB turb = (POHCIPCI_TRANSFER_URB)uc->TransferUrb;
    if (turb != NULL) {
        turb->TransferBufferLength = u->transferred;
        switch (u->status) {
        case OHCI_URB_STATUS_OK:       turb->Hdr.Status = USBD_STATUS_SUCCESS;            break;
        case OHCI_URB_STATUS_STALL:    turb->Hdr.Status = USBD_STATUS_STALL_PID;          break;
        case OHCI_URB_STATUS_CRC:      turb->Hdr.Status = USBD_STATUS_CRC;                break;
        case OHCI_URB_STATUS_TIMEOUT:  turb->Hdr.Status = USBD_STATUS_TIMEOUT;            break;
        case OHCI_URB_STATUS_OVERRUN:  turb->Hdr.Status = USBD_STATUS_DATA_OVERRUN;       break;
        case OHCI_URB_STATUS_UNDERRUN: turb->Hdr.Status = USBD_STATUS_DATA_UNDERRUN;      break;
        default:                       turb->Hdr.Status = USBD_STATUS_INTERNAL_HC_ERROR;  break;
        }
    }

    /* Return bounce buffers to pool before completing the request. */
    if (uc->SetupBounce) {
        OhciPci_BounceFree(uc->EpCtx->Dc, uc->SetupBounce);
        uc->SetupBounce = NULL;
    }
    if (uc->DataBounce) {
        OhciPci_BounceFree(uc->EpCtx->Dc, uc->DataBounce);
        uc->DataBounce = NULL;
    }

    WdfRequestCompleteWithInformation(uc->Request, status, info);
}

/* --------------------------------------------------------------------------
 * EvtUrbDefault
 *
 * EvtIoDefault for the WDFQUEUE registered with UCX via
 * UcxEndpointSetWdfIoQueue. UCX delivers each per-endpoint transfer as a
 * plain WDF request whose Parameters.Others.Arg1 points to a TRANSFER_URB.
 *
 * We retrieve the per-EP context via the queue context (ohcipci_queue_ctx),
 * because WDF provides no WdfIoQueueGetParentObject API.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
static VOID
EvtUrbDefault(
    WDFQUEUE   Queue,
    WDFREQUEST Request
    )
{
    /* Retrieve per-EP context via queue context back-pointer. */
    OHCIPCI_QUEUE_CTX *qc = OhciPci_QueueCtxGet(Queue);
    OHCIPCI_EP_CONTEXT *ep = qc->EpCtx;
    PDEVICE_CONTEXT dc = ep->Dc;

    WDF_REQUEST_PARAMETERS params;
    WDF_REQUEST_PARAMETERS_INIT(&params);
    WdfRequestGetParameters(Request, &params);

    POHCIPCI_TRANSFER_URB urb =
        (POHCIPCI_TRANSFER_URB)params.Parameters.Others.Arg1;
    if (urb == NULL) {
        LOG("EvtUrbDefault: NULL TRANSFER_URB in Arg1");
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    USHORT fn = urb->Hdr.Function;
    /* Plan 5: control transfers only. UCX still uses URB_FUNCTION_CONTROL_TRANSFER_EX
     * inside the TRANSFER_URB header. */
    if (fn != URB_FUNCTION_CONTROL_TRANSFER &&
        fn != URB_FUNCTION_CONTROL_TRANSFER_EX)
    {
        LOG("EvtUrbDefault: non-control function 0x%X — Plan 5 only handles control", fn);
        WdfRequestComplete(Request, STATUS_NOT_SUPPORTED);
        return;
    }

    UCHAR  *setupBytes = urb->u.SetupPacket;
    ULONG   length     = urb->TransferBufferLength;
    PVOID   bufVa      = urb->TransferBuffer;
    PMDL    bufMdl     = urb->TransferBufferMDL;
    BOOLEAN isIn       = !!(urb->TransferFlags & USBD_TRANSFER_DIRECTION_IN);

    LOG("EvtUrbDefault: fn=0x%X len=%lu dir=%s setup=%02X %02X %02X %02X %02X %02X %02X %02X",
        fn, length, isIn ? "IN" : "OUT",
        setupBytes[0], setupBytes[1], setupBytes[2], setupBytes[3],
        setupBytes[4], setupBytes[5], setupBytes[6], setupBytes[7]);

    /* Allocate per-URB context on the WDFREQUEST object. */
    WDF_OBJECT_ATTRIBUTES reqAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&reqAttrs, OHCIPCI_URB_CTX);
    NTSTATUS status = WdfObjectAllocateContext(Request, &reqAttrs, NULL);
    if (!NT_SUCCESS(status)) {
        WdfRequestComplete(Request, status);
        return;
    }
    OHCIPCI_URB_CTX *uc = OhciPci_UrbCtxGet(Request);
    RtlZeroMemory(uc, sizeof(*uc));
    uc->Request     = Request;
    uc->EpCtx       = ep;
    uc->TransferUrb = urb;

    /* SETUP bounce: always 8 bytes. */
    uc->SetupBounce = OhciPci_BounceAlloc(dc, &uc->SetupBouncePhys);
    if (uc->SetupBounce == NULL) {
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    RtlCopyMemory(uc->SetupBounce, setupBytes, 8);

    /* DATA bounce (optional). */
    if (length > 0) {
        if (length > OHCIPCI_BOUNCE_SLAB_BYTES) {
            OhciPci_BounceFree(dc, uc->SetupBounce);
            WdfRequestComplete(Request, STATUS_BUFFER_TOO_SMALL);
            return;
        }
        uc->DataBounce = OhciPci_BounceAlloc(dc, &uc->DataBouncePhys);
        if (uc->DataBounce == NULL) {
            OhciPci_BounceFree(dc, uc->SetupBounce);
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        uc->DataLength    = length;
        uc->DataDirection = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
        uc->UserMdl       = bufMdl;
        uc->UserVa        = bufVa;

        /* For OUT transfers, copy caller data into bounce buffer now. */
        if (!isIn) {
            PVOID src = NULL;
            if (bufMdl) {
                src = MmGetSystemAddressForMdlSafe(bufMdl, NormalPagePriority);
            } else if (bufVa) {
                src = bufVa;
            }
            if (src) {
                RtlCopyMemory(uc->DataBounce, src, length);
            }
        }
    }

    /* Build and submit the core URB. */
    RtlCopyMemory(uc->CoreUrb.setup, setupBytes, 8);
    uc->CoreUrb.setup_phys  = uc->SetupBouncePhys;
    uc->CoreUrb.buffer      = uc->DataBounce;
    uc->CoreUrb.buffer_phys = uc->DataBouncePhys;
    uc->CoreUrb.length      = uc->DataLength;
    uc->CoreUrb.direction   = uc->DataDirection;
    uc->CoreUrb.complete    = OhciPci_UrbComplete;

    int rc = ohci_control_submit(&dc->Hc, &ep->Core, &uc->CoreUrb);
    if (rc != 0) {
        LOG("ohci_control_submit failed: %d", rc);
        if (uc->SetupBounce) OhciPci_BounceFree(dc, uc->SetupBounce);
        if (uc->DataBounce)  OhciPci_BounceFree(dc, uc->DataBounce);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    LOG("ohci_control_submit OK (setup_phys=0x%08X data_phys=0x%08X len=%u dir=%u)",
        uc->SetupBouncePhys, uc->DataBouncePhys, uc->DataLength, uc->DataDirection);
    /* Completion is asynchronous — OhciPci_UrbComplete will call
     * WdfRequestCompleteWithInformation when the OHCI hardware is done. */
}

/* --------------------------------------------------------------------------
 * Default endpoint lifecycle callbacks (required by UCX).
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
static VOID
EvtDefaultEpPurge(
    UCXCONTROLLER UcxController,
    UCXENDPOINT   UcxEndpoint
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("DefaultEp Purge");
    if (ep->UrbQueue != NULL) {
        WdfIoQueuePurge(ep->UrbQueue, WDF_NO_EVENT_CALLBACK, WDF_NO_CONTEXT);
    }
    UcxEndpointPurgeComplete(UcxEndpoint);
}

_Use_decl_annotations_
static VOID
EvtDefaultEpStart(
    UCXCONTROLLER UcxController,
    UCXENDPOINT   UcxEndpoint
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("DefaultEp Start");
    if (ep->UrbQueue != NULL) {
        WdfIoQueueStart(ep->UrbQueue);
    }
}

_Use_decl_annotations_
static VOID
EvtDefaultEpAbort(
    UCXCONTROLLER UcxController,
    UCXENDPOINT   UcxEndpoint
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("DefaultEp Abort");
    if (ep->UrbQueue != NULL) {
        WdfIoQueuePurge(ep->UrbQueue, WDF_NO_EVENT_CALLBACK, WDF_NO_CONTEXT);
    }
    UcxEndpointAbortComplete(UcxEndpoint);
}

_Use_decl_annotations_
static VOID
EvtDefaultEpOkToCancel(
    UCXENDPOINT UcxEndpoint
    )
{
    UNREFERENCED_PARAMETER(UcxEndpoint);
    LOG("DefaultEp OkToCancel (Plan 5: no-op)");
}

_Use_decl_annotations_
static VOID
EvtDefaultEpUpdate(
    UCXCONTROLLER UcxController,
    WDFREQUEST    Request
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    LOG("DefaultEp Update (MPS change stub — Plan 6 will update OHCI ED)");
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

/* --------------------------------------------------------------------------
 * OhciPci_DefaultEndpointAdd
 *
 * EVT_UCX_USBDEVICE_DEFAULT_ENDPOINT_ADD — called by UCX when the USB device
 * is being enumerated and needs EP0 created.
 *
 * Steps:
 *  1. Register UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS on the init object via
 *     UcxDefaultEndpointInitSetEventCallbacks (NOT the regular variant).
 *  2. Call UcxEndpointCreate (double-pointer for init, same as UsbDeviceCreate).
 *  3. Create a WDF IO queue (sequential). Store a back-pointer to ep in queue
 *     context (since WDF has no WdfIoQueueGetParentObject). Register queue with
 *     UCX via UcxEndpointSetWdfIoQueue.
 *  4. Initialise the OHCI core Control endpoint.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS
OhciPci_DefaultEndpointAdd(
    UCXCONTROLLER     UcxController,
    UCXUSBDEVICE      UcxUsbDevice,
    ULONG             MaxPacketSize,
    PUCXENDPOINT_INIT UcxEndpointInit
    )
{
    UNREFERENCED_PARAMETER(UcxController);

    LOG("DefaultEndpointAdd: MPS=%lu", MaxPacketSize);

    /* 1. Register default-EP callbacks on the init object.
     *    Note: use UcxDefaultEndpointInitSetEventCallbacks (not the regular
     *    UcxEndpointInitSetEventCallbacks) for default/EP0 endpoints. The two
     *    structs are distinct: UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS has 5 fields
     *    (no Reset, no StaticStreams) while UCX_ENDPOINT_EVENT_CALLBACKS has 11. */
    UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS ecb;
    UCX_DEFAULT_ENDPOINT_EVENT_CALLBACKS_INIT(
        &ecb,
        EvtDefaultEpPurge,
        EvtDefaultEpStart,
        EvtDefaultEpAbort,
        EvtDefaultEpOkToCancel,
        EvtDefaultEpUpdate
    );
    UcxDefaultEndpointInitSetEventCallbacks(UcxEndpointInit, &ecb);

    /* 2. Create the UCXENDPOINT.
     *    UcxEndpointCreate takes PUCXENDPOINT_INIT* (double-pointer). */
    WDF_OBJECT_ATTRIBUTES epAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&epAttrs, OHCIPCI_EP_CONTEXT);

    UCXENDPOINT ucxEp;
    NTSTATUS status = UcxEndpointCreate(UcxUsbDevice,
                                        &UcxEndpointInit,
                                        &epAttrs,
                                        &ucxEp);
    LOG("UcxEndpointCreate (default) -> 0x%08X", status);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    /* Initialise per-EP context. */
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(ucxEp);
    RtlZeroMemory(ep, sizeof(*ep));
    ep->Dc    = g_DeviceContext;   /* module-static, set during RootHubCreate */
    ep->UcxEp = ucxEp;

    /* 3. Create WDF IO queue for URB delivery.
     *
     *    The queue's parent is the UCXENDPOINT so its lifetime is tied to the
     *    endpoint. We cannot use WdfIoQueueGetParentObject (no such API in WDF)
     *    to recover the endpoint from inside EvtUrbIoctl, so we store a back-
     *    pointer in a small ohcipci_queue_ctx on the WDFQUEUE object. */
    WDF_IO_QUEUE_CONFIG qCfg;
    WDF_IO_QUEUE_CONFIG_INIT(&qCfg, WdfIoQueueDispatchSequential);
    qCfg.EvtIoDefault = EvtUrbDefault;
    /* UCX manages our power state for us; the queue must not be power-managed
     * or it will sit paused while UCX is trying to deliver URBs during
     * enumeration before the device finishes its D0 transition. */
    qCfg.PowerManaged = WdfFalse;

    WDF_OBJECT_ATTRIBUTES qAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&qAttrs, OHCIPCI_QUEUE_CTX);
    qAttrs.ParentObject = ucxEp;  /* Endpoint owns the queue. */

    WDFQUEUE urbQueue;
    status = WdfIoQueueCreate(ep->Dc->Device,
                               &qCfg,
                               &qAttrs,
                               &urbQueue);
    if (!NT_SUCCESS(status)) {
        LOG("WdfIoQueueCreate failed: 0x%08X", status);
        return status;
    }

    /* Store back-pointer in queue context. */
    OHCIPCI_QUEUE_CTX *qc = OhciPci_QueueCtxGet(urbQueue);
    qc->EpCtx = ep;

    ep->UrbQueue = urbQueue;

    /* Register the queue with UCX. After this call UCX will dispatch
     * IOCTL_INTERNAL_USB_SUBMIT_URB requests to urbQueue. */
    UcxEndpointSetWdfIoQueue(ucxEp, urbQueue);

    /* 4. Configure OHCI core Control endpoint for EP0 (func_addr=0, ep_num=0).
     *    OHCI requires the ED's S (low-speed) bit to match the device's
     *    actual port speed or the controller will not progress the transfer
     *    (no error reported, just nothing happens). UCX stores the speed in
     *    UCXUSBDEVICE_INFO at UsbDeviceAdd time; we stashed it on the device
     *    context for retrieval here (single-instance limitation). */
    struct ohci_control_endpoint_config cfg;
    cfg.func_addr       = 0;
    cfg.ep_num          = 0;
    cfg.max_packet_size = (uint16_t)MaxPacketSize;
    cfg.low_speed       = (ep->Dc->PendingDeviceSpeed == UsbLowSpeed) ? 1 : 0;
    LOG("DefaultEndpointAdd: low_speed=%u (PendingDeviceSpeed=%d)",
        cfg.low_speed, (int)ep->Dc->PendingDeviceSpeed);

    int rc = ohci_control_endpoint_create(&ep->Dc->Hc, &cfg, &ep->Core);
    if (rc != 0) {
        LOG("ohci_control_endpoint_create failed: %d", rc);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    LOG("DefaultEndpointAdd: EP0 ready (MPS=%lu)", MaxPacketSize);
    return STATUS_SUCCESS;
}

/* --------------------------------------------------------------------------
 * OhciPci_EndpointAdd
 *
 * EVT_UCX_USBDEVICE_ENDPOINT_ADD — non-default (Bulk/Interrupt/Isochronous)
 * endpoints. Implemented in Plan 6. Returns STATUS_NOT_IMPLEMENTED so UCX
 * fails gracefully during enumeration of multi-endpoint devices.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS
OhciPci_EndpointAdd(
    UCXCONTROLLER                                 UcxController,
    UCXUSBDEVICE                                  UcxUsbDevice,
    PUSB_ENDPOINT_DESCRIPTOR                      UsbEndpointDescriptor,
    ULONG                                         UsbEndpointDescriptorBufferLength,
    PUSB_SUPERSPEED_ENDPOINT_COMPANION_DESCRIPTOR SuperSpeedEndpointCompanionDescriptor,
    PUCXENDPOINT_INIT                             UcxEndpointInit
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(UcxUsbDevice);
    UNREFERENCED_PARAMETER(UsbEndpointDescriptor);
    UNREFERENCED_PARAMETER(UsbEndpointDescriptorBufferLength);
    UNREFERENCED_PARAMETER(SuperSpeedEndpointCompanionDescriptor);
    UNREFERENCED_PARAMETER(UcxEndpointInit);
    LOG("EndpointAdd (Plan 6 implements non-default endpoints)");
    return STATUS_NOT_IMPLEMENTED;
}
