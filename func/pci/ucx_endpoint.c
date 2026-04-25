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
#include "ohci_bulk.h"
#include "ohci_interrupt.h"
#include "ohci_urb.h"
#include "ohci_dma.h"
#include "ohci_drain.h"
#include "ohci_ed.h"

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

/* Non-default (Bulk/Interrupt) endpoint callbacks. The non-default callbacks
 * struct adds a Reset slot the default EP doesn't have, plus three streams
 * callbacks we leave NULL because OHCI doesn't support USB 3.0 streams. */
static EVT_UCX_ENDPOINT_PURGE                  EvtNonDefaultEpPurge;
static EVT_UCX_ENDPOINT_START                  EvtNonDefaultEpStart;
static EVT_UCX_ENDPOINT_ABORT                  EvtNonDefaultEpAbort;
static EVT_UCX_ENDPOINT_RESET                  EvtNonDefaultEpReset;
static EVT_UCX_ENDPOINT_OK_TO_CANCEL_TRANSFERS EvtNonDefaultEpOkToCancel;

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
    if (u->ed != NULL) {
        LOG("  ED-done: Ctrl=0x%08X HeadP=0x%08X TailP=0x%08X",
            u->ed->Control, u->ed->HeadP, u->ed->TailP);
    }
    if (u->head_td != NULL) {
        struct ohci_td *td = u->head_td;
        uint32_t ctrl = td->Control;
        uint32_t cc   = (ctrl >> 28) & 0xF;
        uint32_t ec   = (ctrl >> 26) & 0x3;
        uint32_t tval = (ctrl >> 24) & 0x3;
        LOG("  TD-done: Ctrl=0x%08X CC=%u EC=%u T=%u CBP=0x%08X BE=0x%08X",
            ctrl, cc, ec, tval, td->CBP, td->BE);
    }

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
        if (uc->DataBounceSlabs > 1) {
            OhciPci_BounceFreeBig(uc->EpCtx->Dc, uc->DataBounce, uc->DataBounceSlabs);
        } else {
            OhciPci_BounceFree(uc->EpCtx->Dc, uc->DataBounce);
        }
        uc->DataBounce = NULL;
    }

    /* Defer the actual WdfRequestComplete until CoreLock is released.
     * Calling it here would deadlock — UCX completion can synchronously
     * dispatch the next URB to our queue, which then re-acquires CoreLock,
     * but WDFSPINLOCK is non-recursive. EvtDpc drains the list. */
    uc->DeferredStatus = status;
    uc->DeferredInfo   = info;
    InsertTailList(&uc->EpCtx->Dc->DeferredCompletions, &uc->DeferredEntry);
}

/* --------------------------------------------------------------------------
 * OhciPci_HandleBulkOrInterruptUrb
 *
 * Handles URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER for non-default endpoints.
 * Allocates one bounce slab, copies OUT data in, submits via the matching
 * core entrypoint, completes asynchronously through OhciPci_UrbComplete.
 * -------------------------------------------------------------------------- */
static VOID
OhciPci_HandleBulkUrb(
    OHCIPCI_EP_CONTEXT   *ep,
    WDFREQUEST            Request,
    POHCIPCI_TRANSFER_URB urb)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    ULONG   length = urb->TransferBufferLength;
    PVOID   bufVa  = urb->TransferBuffer;
    PMDL    bufMdl = urb->TransferBufferMDL;
    BOOLEAN isIn   = !!(urb->TransferFlags & USBD_TRANSFER_DIRECTION_IN);

    LOG("EvtUrbBulk-SG: len=%lu dir=%s", length, isIn ? "IN" : "OUT");
    if (!isIn && length >= 8 && bufMdl != NULL) {
        PUCHAR sys = (PUCHAR)MmGetSystemAddressForMdlSafe(bufMdl, NormalPagePriority | MdlMappingNoExecute);
        if (sys) {
            LOG("  OUT-VA[0..7]=%02X %02X %02X %02X %02X %02X %02X %02X",
                sys[0], sys[1], sys[2], sys[3], sys[4], sys[5], sys[6], sys[7]);
        }
        /* Verify VA == phys content. If MDL has been re-pointed (partial
         * MDL with different PFNs), the HC will DMA garbage. */
        PPFN_NUMBER pfns0 = MmGetMdlPfnArray(bufMdl);
        ULONG bo = MmGetMdlByteOffset(bufMdl);
        PHYSICAL_ADDRESS pa;
        pa.QuadPart = ((ULONGLONG)pfns0[0] << PAGE_SHIFT) + bo;
        PUCHAR phys = (PUCHAR)MmMapIoSpaceEx(pa, 8, PAGE_READWRITE | PAGE_NOCACHE);
        if (phys) {
            LOG("  OUT-PHYS[0..7]=%02X %02X %02X %02X %02X %02X %02X %02X (pa=0x%llX)",
                phys[0], phys[1], phys[2], phys[3], phys[4], phys[5], phys[6], phys[7],
                pa.QuadPart);
            MmUnmapIoSpace(phys, 8);
        } else {
            LOG("  OUT-PHYS: MmMapIoSpaceEx failed (pa=0x%llX)", pa.QuadPart);
        }
    }

    if (length == 0) {
        urb->TransferBufferLength = 0;
        urb->Hdr.Status = USBD_STATUS_SUCCESS;
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }
    /* Some URBs come in with only a flat KVA buffer (no MDL). Bounce
     * those through a 32-bit-DMA slab — we can't walk PFNs without an
     * MDL anyway. Falls through to the no-MDL bounce branch below. */

    /* Decide whether to bounce. We bounce when:
     *   - There's no MDL (we can't walk PFNs from a flat KVA buffer)
     *   - Any PFN is above 4 GB (OHCI is 32-bit DMA only)
     * Otherwise we use the MDL's PFN array directly for true SG. */
    BOOLEAN     needsBounce = (bufMdl == NULL);
    ULONG       pageCount   = 0;
    ULONG       byteOffset  = 0;
    PPFN_NUMBER pfns        = NULL;

    if (bufMdl != NULL) {
        byteOffset = MmGetMdlByteOffset(bufMdl);
        ULONG byteCount = MmGetMdlByteCount(bufMdl);
        if (byteCount < length) length = byteCount;
        pageCount = ADDRESS_AND_SIZE_TO_SPAN_PAGES(
                        (PVOID)((ULONG_PTR)MmGetMdlVirtualAddress(bufMdl) + byteOffset),
                        length);
        if (pageCount == 0 || pageCount > OHCI_BULK_MAX_SG_PAGES) {
            LOG("  rejected: pageCount=%lu (cap=%u)", pageCount, OHCI_BULK_MAX_SG_PAGES);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        pfns = MmGetMdlPfnArray(bufMdl);
        for (ULONG i = 0; i < pageCount; i++) {
            if (((ULONGLONG)pfns[i] << PAGE_SHIFT) >= 0x100000000ULL) {
                needsBounce = TRUE;
                break;
            }
        }
    }

    struct ohci_bulk_sg_page sg[OHCI_BULK_MAX_SG_PAGES];
    void   *bounceVa   = NULL;
    uint32_t bouncePhys = 0;

    ULONG bounceSlabs = 0;
    if (needsBounce) {
        bounceSlabs = (length + OHCIPCI_BOUNCE_SLAB_BYTES - 1) / OHCIPCI_BOUNCE_SLAB_BYTES;
        if (bounceSlabs > OHCIPCI_BOUNCE_SLAB_COUNT) {
            LOG("  rejected: bounce needs %lu slabs > pool size %u",
                bounceSlabs, OHCIPCI_BOUNCE_SLAB_COUNT);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        WdfSpinLockAcquire(dc->CoreLock);
        bounceVa = OhciPci_BounceAllocBig(dc, bounceSlabs, &bouncePhys);
        WdfSpinLockRelease(dc->CoreLock);
        if (bounceVa == NULL) {
            LOG("  rejected: no contiguous bounce run for %lu slabs", bounceSlabs);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INSUFFICIENT_RESOURCES;
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        if (!isIn) {
            PUCHAR src = NULL;
            if (bufMdl != NULL) {
                src = (PUCHAR)MmGetSystemAddressForMdlSafe(bufMdl,
                    NormalPagePriority | MdlMappingNoExecute);
            } else {
                src = (PUCHAR)bufVa;
            }
            if (src == NULL) {
                WdfSpinLockAcquire(dc->CoreLock);
                OhciPci_BounceFreeBig(dc, bounceVa, bounceSlabs);
                WdfSpinLockRelease(dc->CoreLock);
                urb->TransferBufferLength = 0;
                urb->Hdr.Status = USBD_STATUS_INSUFFICIENT_RESOURCES;
                WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
                return;
            }
            RtlCopyMemory(bounceVa, src, length);
        }
        /* One SG entry per slab (4 KB). Slabs are physically contiguous
         * within the bounce pool, but breaking them up keeps each TD's
         * CBP→BE inside the OHCI single-page-boundary limit (§4.3.1.4)
         * without relying on the page-straddle case. */
        ULONG remaining = length;
        ULONG offset    = 0;
        ULONG idx       = 0;
        while (remaining > 0 && idx < OHCI_BULK_MAX_SG_PAGES) {
            ULONG chunk = OHCIPCI_BOUNCE_SLAB_BYTES;
            if (chunk > remaining) chunk = remaining;
            sg[idx].phys   = bouncePhys + offset;
            sg[idx].length = chunk;
            sg[idx].off    = offset;
            offset    += chunk;
            remaining -= chunk;
            idx++;
        }
        if (remaining > 0) {
            LOG("  rejected: bounce chunked into >%u SG entries", OHCI_BULK_MAX_SG_PAGES);
            WdfSpinLockAcquire(dc->CoreLock);
            OhciPci_BounceFreeBig(dc, bounceVa, bounceSlabs);
            WdfSpinLockRelease(dc->CoreLock);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        pageCount = idx;
    } else {
        ULONG offInPage = byteOffset & (PAGE_SIZE - 1);
        ULONG remaining = length;
        ULONG urbOff    = 0;
        for (ULONG i = 0; i < pageCount; i++) {
            ULONG bytesThisPage = PAGE_SIZE - offInPage;
            if (bytesThisPage > remaining) bytesThisPage = remaining;
            sg[i].phys   = (uint32_t)((ULONGLONG)pfns[i] << PAGE_SHIFT) + offInPage;
            sg[i].length = bytesThisPage;
            sg[i].off    = urbOff;
            urbOff      += bytesThisPage;
            remaining   -= bytesThisPage;
            offInPage    = 0;
        }
    }

    WDF_OBJECT_ATTRIBUTES reqAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&reqAttrs, OHCIPCI_URB_CTX);
    NTSTATUS status = WdfObjectAllocateContext(Request, &reqAttrs, NULL);
    if (!NT_SUCCESS(status)) {
        WdfRequestComplete(Request, status);
        return;
    }
    OHCIPCI_URB_CTX *uc = OhciPci_UrbCtxGet(Request);
    RtlZeroMemory(uc, sizeof(*uc));
    uc->Request       = Request;
    uc->EpCtx         = ep;
    uc->TransferUrb   = urb;
    uc->DataLength    = length;
    uc->DataDirection = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
    if (bounceVa != NULL) {
        uc->DataBounce      = bounceVa;
        uc->DataBouncePhys  = bouncePhys;
        uc->DataBounceSlabs = bounceSlabs;
        uc->UserMdl         = bufMdl;
        uc->UserVa          = (bufMdl == NULL) ? bufVa : NULL;
    }

    /* buffer_phys = first chunk's phys minus its off, so the drain's
     * "CBP - (buffer_phys + chunk_off)" arithmetic resolves correctly
     * for every TD record in data_tds[]. */
    uc->CoreUrb.buffer       = NULL;
    uc->CoreUrb.buffer_phys  = sg[0].phys - sg[0].off;
    uc->CoreUrb.length       = length;
    uc->CoreUrb.direction    = uc->DataDirection;
    uc->CoreUrb.complete     = OhciPci_UrbComplete;

    WdfSpinLockAcquire(dc->CoreLock);
    struct ohci_ed *bed = ep->Core.Bulk.ed;
    uint32_t hc_ctrl_pre   = dc->MmioOps.read32(dc->MmioOps.context, 0x04);
    uint32_t hc_bulk_head  = dc->MmioOps.read32(dc->MmioOps.context, 0x28);
    uint32_t hc_bulk_curr  = dc->MmioOps.read32(dc->MmioOps.context, 0x2C);
    uint32_t hc_cmd_status = dc->MmioOps.read32(dc->MmioOps.context, 0x08);
    LOG("  HC-pre:  HcControl=0x%08X (BLE=%u CLE=%u PLE=%u HCFS=%u) HcBulkHead=0x%08X HcBulkCurr=0x%08X HcCmdStat=0x%08X",
        hc_ctrl_pre,
        (hc_ctrl_pre >> 5) & 1, (hc_ctrl_pre >> 4) & 1, (hc_ctrl_pre >> 2) & 1,
        (hc_ctrl_pre >> 6) & 3,
        hc_bulk_head, hc_bulk_curr, hc_cmd_status);
    LOG("  ED-pre:  Ctrl=0x%08X HeadP=0x%08X TailP=0x%08X NextED=0x%08X (sg[0].phys=0x%08X len=%lu)",
        bed->Control, bed->HeadP, bed->TailP, bed->NextED,
        sg[0].phys, sg[0].length);
    int rc = ohci_bulk_submit_sg(&dc->Hc, &ep->Core.Bulk, &uc->CoreUrb,
                                  sg, pageCount);
    LOG("  ED-post: Ctrl=0x%08X HeadP=0x%08X TailP=0x%08X NextED=0x%08X",
        bed->Control, bed->HeadP, bed->TailP, bed->NextED);
    WdfSpinLockRelease(dc->CoreLock);
    if (rc != 0) {
        LOG("  bulk_submit_sg failed: rc=%d", rc);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    LOG("EvtUrbBulk-SG: submitted pages=%lu total=%lu", pageCount, length);
    /* Async completion via OhciPci_UrbComplete. */
}

static VOID
OhciPci_HandleInterruptUrb(
    OHCIPCI_EP_CONTEXT   *ep,
    WDFREQUEST            Request,
    POHCIPCI_TRANSFER_URB urb)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    ULONG   length = urb->TransferBufferLength;
    PVOID   bufVa  = urb->TransferBuffer;
    PMDL    bufMdl = urb->TransferBufferMDL;
    BOOLEAN isIn   = !!(urb->TransferFlags & USBD_TRANSFER_DIRECTION_IN);

    LOG("EvtUrbInt: len=%lu dir=%s", length, isIn ? "IN" : "OUT");

    if (length == 0 || length > OHCIPCI_BOUNCE_SLAB_BYTES) {
        urb->TransferBufferLength = 0;
        urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

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

    WdfSpinLockAcquire(dc->CoreLock);

    uc->DataBounce = OhciPci_BounceAlloc(dc, &uc->DataBouncePhys);
    if (uc->DataBounce == NULL) {
        WdfSpinLockRelease(dc->CoreLock);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    uc->DataLength    = length;
    uc->DataDirection = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
    uc->UserMdl       = bufMdl;
    uc->UserVa        = bufVa;

    if (!isIn) {
        PVOID src = bufMdl
            ? MmGetSystemAddressForMdlSafe(bufMdl, NormalPagePriority)
            : bufVa;
        if (src) RtlCopyMemory(uc->DataBounce, src, length);
    }

    uc->CoreUrb.buffer       = uc->DataBounce;
    uc->CoreUrb.buffer_phys  = uc->DataBouncePhys;
    uc->CoreUrb.length       = length;
    uc->CoreUrb.direction    = uc->DataDirection;
    uc->CoreUrb.complete     = OhciPci_UrbComplete;

    int rc = ohci_interrupt_submit(&dc->Hc, &ep->Core.Interrupt, &uc->CoreUrb);
    if (rc != 0) {
        OhciPci_BounceFree(dc, uc->DataBounce);
        uc->DataBounce = NULL;
        WdfSpinLockRelease(dc->CoreLock);
        LOG("  submit failed: rc=%d", rc);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    WdfSpinLockRelease(dc->CoreLock);
    /* Async completion via OhciPci_UrbComplete. */
}

static VOID
OhciPci_HandleBulkOrInterruptUrb(
    OHCIPCI_EP_CONTEXT   *ep,
    WDFREQUEST            Request,
    POHCIPCI_TRANSFER_URB urb)
{
    if (ep->Kind == OhciPciEpKindBulk) {
        OhciPci_HandleBulkUrb(ep, Request, urb);
    } else {
        OhciPci_HandleInterruptUrb(ep, Request, urb);
    }
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
    if (fn == URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER) {
        OhciPci_HandleBulkOrInterruptUrb(ep, Request, urb);
        return;
    }
    if (fn != URB_FUNCTION_CONTROL_TRANSFER &&
        fn != URB_FUNCTION_CONTROL_TRANSFER_EX)
    {
        LOG("EvtUrbDefault: unsupported function 0x%X", fn);
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

    /* Intercept CLEAR_FEATURE(ENDPOINT_HALT) on a non-default EP so we
     * can clear the matching OHCI ED's H+C bits in lock-step with the
     * device's wire-level halt clear. Without this, after a STALL the
     * ED stays halted and our toggle stays at whatever value it was at
     * STALL time — the resumed transfer drifts off the device's reset
     * DATA0 toggle and STALLs again. The Control transfer is still
     * forwarded so the device sees CLEAR_FEATURE on the wire too. */
    if (setupBytes[0] == 0x02 &&  /* OUT, standard, endpoint */
        setupBytes[1] == 0x01 &&  /* CLEAR_FEATURE */
        setupBytes[2] == 0x00 && setupBytes[3] == 0x00 &&  /* ENDPOINT_HALT */
        setupBytes[5] == 0x00)
    {
        UCHAR    targetEp  = setupBytes[4];
        UCHAR    targetEpN = targetEp & 0x0F;
        BOOLEAN  targetIn  = (targetEp & 0x80) != 0;
        UCHAR    funcAddr  = (UCHAR)(ep->Core.Control.ed->Control & 0x7F);
        uint32_t want_d    = targetIn ? OHCI_ED_D_IN : OHCI_ED_D_OUT;
        struct ohci_ed *target = NULL;
        WdfSpinLockAcquire(dc->CoreLock);
        for (struct ohci_bulk_endpoint *be = dc->Hc.bulk_head; be; be = be->next) {
            uint32_t c = be->ed->Control;
            if ((c & 0x7F) == funcAddr &&
                ((c >> OHCI_ED_EN_SHIFT) & 0x0F) == targetEpN &&
                (c & OHCI_ED_D_MASK) == want_d) {
                target = be->ed;
                break;
            }
        }
        if (target == NULL) {
            for (struct ohci_interrupt_endpoint *ie = dc->Hc.interrupt_head; ie; ie = ie->next) {
                uint32_t c = ie->ed->Control;
                if ((c & 0x7F) == funcAddr &&
                    ((c >> OHCI_ED_EN_SHIFT) & 0x0F) == targetEpN &&
                    (c & OHCI_ED_D_MASK) == want_d) {
                    target = ie->ed;
                    break;
                }
            }
        }
        if (target != NULL) {
            target->HeadP &= ~(uint32_t)(OHCI_ED_HEADP_H | OHCI_ED_HEADP_C);
            target->Control &= ~OHCI_ED_K;
            dc->MmioOps.barrier(dc->MmioOps.context);
            LOG("  CLEAR_FEATURE(HALT) ep=0x%02X addr=%u — cleared ED H+C+K",
                targetEp, funcAddr);
        } else {
            LOG("  CLEAR_FEATURE(HALT) ep=0x%02X addr=%u — no matching ED",
                targetEp, funcAddr);
        }
        WdfSpinLockRelease(dc->CoreLock);
    }

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

    /* CoreLock guards the bounce-pool bitmap RMW + ohci_control_submit's
     * mutation of hc->in_flight against the WDH DPC running concurrently. */
    WdfSpinLockAcquire(dc->CoreLock);

    /* SETUP bounce: always 8 bytes. */
    uc->SetupBounce = OhciPci_BounceAlloc(dc, &uc->SetupBouncePhys);
    if (uc->SetupBounce == NULL) {
        WdfSpinLockRelease(dc->CoreLock);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    RtlCopyMemory(uc->SetupBounce, setupBytes, 8);

    /* DATA bounce (optional). */
    if (length > 0) {
        if (length > OHCIPCI_BOUNCE_SLAB_BYTES) {
            OhciPci_BounceFree(dc, uc->SetupBounce);
            WdfSpinLockRelease(dc->CoreLock);
            WdfRequestComplete(Request, STATUS_BUFFER_TOO_SMALL);
            return;
        }
        uc->DataBounce = OhciPci_BounceAlloc(dc, &uc->DataBouncePhys);
        if (uc->DataBounce == NULL) {
            OhciPci_BounceFree(dc, uc->SetupBounce);
            WdfSpinLockRelease(dc->CoreLock);
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

    int rc = ohci_control_submit(&dc->Hc, &ep->Core.Control, &uc->CoreUrb);
    if (rc != 0) {
        if (uc->SetupBounce) OhciPci_BounceFree(dc, uc->SetupBounce);
        if (uc->DataBounce)  OhciPci_BounceFree(dc, uc->DataBounce);
        WdfSpinLockRelease(dc->CoreLock);
        LOG("ohci_control_submit failed: %d", rc);
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        return;
    }
    WdfSpinLockRelease(dc->CoreLock);
    LOG("ohci_control_submit OK (setup_phys=0x%08X data_phys=0x%08X len=%u dir=%u)",
        uc->SetupBouncePhys, uc->DataBouncePhys, uc->DataLength, uc->DataDirection);
    /* Completion is asynchronous — OhciPci_UrbComplete will call
     * WdfRequestCompleteWithInformation when the OHCI hardware is done. */
}

/* --------------------------------------------------------------------------
 * OhciPci_CancelEpInFlight
 *
 * Cancel any URBs sitting on the OHCI hc->in_flight list whose ED matches
 * this EP. Used by purge/abort so UCX doesn't wait forever for a poll that
 * will never come (Interrupt EP whose device just went away). The cancel
 * stages completions onto dc->DeferredCompletions via OhciPci_UrbComplete;
 * we drain the list under CoreLock then complete the WDFREQUESTs outside
 * the lock — same pattern as EvtDpc.
 * -------------------------------------------------------------------------- */
static struct ohci_ed *
OhciPci_EpEd(OHCIPCI_EP_CONTEXT *ep)
{
    switch (ep->Kind) {
    case OhciPciEpKindControl:   return ep->Core.Control.ed;
    case OhciPciEpKindBulk:      return ep->Core.Bulk.ed;
    case OhciPciEpKindInterrupt: return ep->Core.Interrupt.ed;
    }
    return NULL;
}

/* Re-enable an EP whose ED was halted by a prior purge/abort. UCX will
 * call Start after Purge — without clearing Skip, the HC ignores the
 * ED and the next URB on it sits forever (HC never raises WDH → timeout). */
static VOID
OhciPci_StartEp(OHCIPCI_EP_CONTEXT *ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    struct ohci_ed *ed = OhciPci_EpEd(ep);
    if (ed == NULL) return;
    WdfSpinLockAcquire(dc->CoreLock);
    ed->Control &= ~OHCI_ED_K;
    dc->MmioOps.barrier(dc->MmioOps.context);
    WdfSpinLockRelease(dc->CoreLock);
}

static VOID
OhciPci_CancelEpInFlight(OHCIPCI_EP_CONTEXT *ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    struct ohci_ed *ed = OhciPci_EpEd(ep);
    if (ed == NULL) return;

    LIST_ENTRY local;
    InitializeListHead(&local);

    WdfSpinLockAcquire(dc->CoreLock);
    ohci_urb_cancel_for_ed(&dc->Hc, ed);
    while (!IsListEmpty(&dc->DeferredCompletions)) {
        PLIST_ENTRY le = RemoveHeadList(&dc->DeferredCompletions);
        InsertTailList(&local, le);
    }
    WdfSpinLockRelease(dc->CoreLock);

    while (!IsListEmpty(&local)) {
        PLIST_ENTRY le = RemoveHeadList(&local);
        POHCIPCI_URB_CTX uc =
            CONTAINING_RECORD(le, OHCIPCI_URB_CTX, DeferredEntry);
        WdfRequestCompleteWithInformation(uc->Request,
                                          uc->DeferredStatus,
                                          uc->DeferredInfo);
    }
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
    OhciPci_CancelEpInFlight(ep);
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
    OhciPci_StartEp(ep);
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
    OhciPci_CancelEpInFlight(ep);
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
    OHCIPCI_USBDEV_CTX *udc = OhciPci_UsbDevContextGet(UcxUsbDevice);
    struct ohci_control_endpoint_config cfg;
    cfg.func_addr       = udc ? udc->FuncAddr : 0;
    cfg.ep_num          = 0;
    cfg.max_packet_size = (uint16_t)MaxPacketSize;
    cfg.low_speed       = (udc && udc->Speed == UsbLowSpeed) ? 1 : 0;
    LOG("DefaultEndpointAdd: low_speed=%u func_addr=%u (per-device ctx)",
        cfg.low_speed, cfg.func_addr);

    ep->Kind = OhciPciEpKindControl;
    int rc = ohci_control_endpoint_create(&ep->Dc->Hc, &cfg, &ep->Core.Control);
    /* Remember EP0 on the per-device context so EvtUsbDeviceAddress (Task 2)
     * can rewrite the func_addr field after SET_ADDRESS lands on the wire. */
    if (rc == 0 && udc) udc->Ep0 = ep;
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
/* --------------------------------------------------------------------------
 * Non-default endpoint lifecycle callbacks. Same shape as default-EP
 * variants but slot in to UCX_ENDPOINT_EVENT_CALLBACKS_INIT instead of
 * the default-EP-only struct.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
static VOID
EvtNonDefaultEpPurge(UCXCONTROLLER UcxController, UCXENDPOINT UcxEndpoint)
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("NonDefaultEp Purge (kind=%d)", ep->Kind);
    if (ep->UrbQueue) WdfIoQueuePurge(ep->UrbQueue, WDF_NO_EVENT_CALLBACK, WDF_NO_CONTEXT);
    OhciPci_CancelEpInFlight(ep);
    UcxEndpointPurgeComplete(UcxEndpoint);
}

_Use_decl_annotations_
static VOID
EvtNonDefaultEpStart(UCXCONTROLLER UcxController, UCXENDPOINT UcxEndpoint)
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("NonDefaultEp Start (kind=%d)", ep->Kind);
    OhciPci_StartEp(ep);
    if (ep->UrbQueue) WdfIoQueueStart(ep->UrbQueue);
}

_Use_decl_annotations_
static VOID
EvtNonDefaultEpAbort(UCXCONTROLLER UcxController, UCXENDPOINT UcxEndpoint)
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    LOG("NonDefaultEp Abort (kind=%d)", ep->Kind);
    if (ep->UrbQueue) WdfIoQueuePurge(ep->UrbQueue, WDF_NO_EVENT_CALLBACK, WDF_NO_CONTEXT);
    OhciPci_CancelEpInFlight(ep);
    UcxEndpointAbortComplete(UcxEndpoint);
}

_Use_decl_annotations_
static VOID
EvtNonDefaultEpReset(UCXCONTROLLER UcxController, UCXENDPOINT UcxEndpoint, WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(UcxController);
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(UcxEndpoint);
    PDEVICE_CONTEXT dc = ep->Dc;
    struct ohci_ed *ed = OhciPci_EpEd(ep);
    LOG("NonDefaultEp Reset (kind=%d) — clearing H/C toggle", ep->Kind);
    if (ed != NULL) {
        WdfSpinLockAcquire(dc->CoreLock);
        /* USB CLEAR_FEATURE(ENDPOINT_HALT) / SET_INTERFACE resets the
         * data toggle to DATA0 on this endpoint. Mirror that in the ED:
         * clear both H (halted) and C (toggle carry) bits in HeadP. */
        ed->HeadP &= ~(uint32_t)(OHCI_ED_HEADP_H | OHCI_ED_HEADP_C);
        ed->Control &= ~OHCI_ED_K;
        dc->MmioOps.barrier(dc->MmioOps.context);
        WdfSpinLockRelease(dc->CoreLock);
    }
    WdfRequestComplete(Request, STATUS_SUCCESS);
}

_Use_decl_annotations_
static VOID
EvtNonDefaultEpOkToCancel(UCXENDPOINT UcxEndpoint)
{
    UNREFERENCED_PARAMETER(UcxEndpoint);
}

/* --------------------------------------------------------------------------
 * OhciPci_EndpointAdd
 *
 * EVT_UCX_USBDEVICE_ENDPOINT_ADD — non-default (Bulk/Interrupt/Isoch)
 * endpoints. Routes by bmAttributes; isoch returns STATUS_NOT_SUPPORTED.
 * -------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS
OhciPci_EndpointAdd(
    UCXCONTROLLER                                 UcxController,
    UCXUSBDEVICE                                  UcxUsbDevice,
    PUSB_ENDPOINT_DESCRIPTOR                      Desc,
    ULONG                                         DescLen,
    PUSB_SUPERSPEED_ENDPOINT_COMPANION_DESCRIPTOR SsCompanion,
    PUCXENDPOINT_INIT                             EpInit
    )
{
    UNREFERENCED_PARAMETER(UcxController);
    UNREFERENCED_PARAMETER(UcxUsbDevice);
    UNREFERENCED_PARAMETER(DescLen);
    UNREFERENCED_PARAMETER(SsCompanion);

    if (Desc == NULL) return STATUS_INVALID_PARAMETER;

    UCHAR  attrs  = Desc->bmAttributes & 0x03; /* 0=Ctrl 1=Iso 2=Bulk 3=Int */
    UCHAR  epAddr = Desc->bEndpointAddress;
    UCHAR  epNum  = epAddr & 0x0F;
    BOOLEAN isIn  = (epAddr & 0x80) != 0;
    USHORT mps    = Desc->wMaxPacketSize & 0x07FF;

    LOG("EndpointAdd: addr=0x%02X attrs=0x%02X mps=%u", epAddr, attrs, mps);

    if (attrs == 0x01) {
        LOG("EndpointAdd: isochronous not supported (Plan 7+)");
        return STATUS_NOT_SUPPORTED;
    }
    if (attrs == 0x00) {
        LOG("EndpointAdd: non-default control endpoints not supported");
        return STATUS_NOT_SUPPORTED;
    }

    /* 1. Register non-default callbacks. */
    UCX_ENDPOINT_EVENT_CALLBACKS ecb;
    UCX_ENDPOINT_EVENT_CALLBACKS_INIT(
        &ecb,
        EvtNonDefaultEpPurge,
        EvtNonDefaultEpStart,
        EvtNonDefaultEpAbort,
        EvtNonDefaultEpReset,
        EvtNonDefaultEpOkToCancel,
        NULL,   /* StaticStreamsAdd     — OHCI has no streams */
        NULL,   /* StaticStreamsEnable  */
        NULL    /* StaticStreamsDisable */
    );
    UcxEndpointInitSetEventCallbacks(EpInit, &ecb);

    /* 2. Create UCXENDPOINT. */
    WDF_OBJECT_ATTRIBUTES epAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&epAttrs, OHCIPCI_EP_CONTEXT);
    UCXENDPOINT ucxEp;
    NTSTATUS status = UcxEndpointCreate(UcxUsbDevice, &EpInit, &epAttrs, &ucxEp);
    if (!NT_SUCCESS(status)) {
        LOG("UcxEndpointCreate (non-default) -> 0x%08X", status);
        return status;
    }
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(ucxEp);
    RtlZeroMemory(ep, sizeof(*ep));
    ep->Dc    = g_DeviceContext;
    ep->UcxEp = ucxEp;

    /* 3. Per-EP WDFQUEUE (same recipe as default EP). */
    WDF_IO_QUEUE_CONFIG qCfg;
    WDF_IO_QUEUE_CONFIG_INIT(&qCfg, WdfIoQueueDispatchSequential);
    qCfg.EvtIoDefault = EvtUrbDefault;
    qCfg.PowerManaged = WdfFalse;
    WDF_OBJECT_ATTRIBUTES qAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&qAttrs, OHCIPCI_QUEUE_CTX);
    qAttrs.ParentObject = ucxEp;
    WDFQUEUE q;
    status = WdfIoQueueCreate(ep->Dc->Device, &qCfg, &qAttrs, &q);
    if (!NT_SUCCESS(status)) {
        LOG("WdfIoQueueCreate (non-default) failed: 0x%08X", status);
        return status;
    }
    OhciPci_QueueCtxGet(q)->EpCtx = ep;
    ep->UrbQueue = q;
    UcxEndpointSetWdfIoQueue(ucxEp, q);

    /* 4. Configure the OHCI core endpoint. Read per-device state instead
     *    of the obsolete single-instance Pending* globals. */
    OHCIPCI_USBDEV_CTX *udc = OhciPci_UsbDevContextGet(UcxUsbDevice);
    UCHAR funcAddr = udc ? udc->FuncAddr : 0;
    UCHAR lowSpeed = (udc && udc->Speed == UsbLowSpeed) ? 1 : 0;
    int rc;
    if (attrs == 0x02) {
        ep->Kind = OhciPciEpKindBulk;
        struct ohci_bulk_endpoint_config bcfg;
        bcfg.func_addr       = funcAddr;
        bcfg.ep_num          = epNum;
        bcfg.max_packet_size = mps;
        bcfg.direction       = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
        bcfg.low_speed       = lowSpeed;
        WdfSpinLockAcquire(ep->Dc->CoreLock);
        rc = ohci_bulk_endpoint_create(&ep->Dc->Hc, &bcfg, &ep->Core.Bulk);
        WdfSpinLockRelease(ep->Dc->CoreLock);
        LOG("Bulk EP created: addr=%u ep=%u dir=%s mps=%u rc=%d",
            funcAddr, epNum, isIn ? "IN" : "OUT", mps, rc);
    } else { /* attrs == 0x03 — Interrupt */
        ep->Kind = OhciPciEpKindInterrupt;
        struct ohci_interrupt_endpoint_config icfg;
        icfg.func_addr            = funcAddr;
        icfg.ep_num               = epNum;
        icfg.max_packet_size      = mps;
        icfg.direction            = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
        icfg.low_speed            = lowSpeed;
        /* USB §9.6.6: for full/low-speed Interrupt, bInterval is the period
         * in frames (1..255). Plan 7 picker rounds down to nearest power of
         * two in [1, 32]. bInterval=0 is illegal but defensive-default to 32. */
        UCHAR bInterval = Desc->bInterval;
        if (bInterval == 0) bInterval = 32;
        icfg.poll_interval_frames = bInterval;
        WdfSpinLockAcquire(ep->Dc->CoreLock);
        rc = ohci_interrupt_endpoint_create(&ep->Dc->Hc, &icfg, &ep->Core.Interrupt);
        WdfSpinLockRelease(ep->Dc->CoreLock);
        LOG("Interrupt EP created: addr=%u ep=%u dir=%s mps=%u (32ms slot) rc=%d",
            funcAddr, epNum, isIn ? "IN" : "OUT", mps, rc);
    }
    if (rc != 0) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    return STATUS_SUCCESS;
}
