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
#include "ohci_isoc.h"
#include "isoc_mdl.h"
#include "ohci_log.h"

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

/* OHCIPCI_TRANSFER_URB now lives in device_context.h so isoc_mdl.c
 * and this TU share one definition. */

/* --------------------------------------------------------------------------
 * OhciPci_UrbComplete
 *
 * Called from OHCI core (interrupt context or DPC) when a Control URB
 * finishes. Copies IN data back to the caller's buffer and completes the
 * WDFREQUEST.
 * -------------------------------------------------------------------------- */
/* Map an OHCI condition code to a per-packet USBD_STATUS for isoch
 * IsoPacket[i].Status writeback. Used only by the isoch branch of
 * OhciPci_UrbComplete. */
static USBD_STATUS
OhciPci_CcToUsbd(uint8_t cc)
{
    switch (cc) {
    case OHCI_CC_NOERROR:             return USBD_STATUS_SUCCESS;
    case OHCI_CC_STALL:               return USBD_STATUS_STALL_PID;
    case OHCI_CC_CRC:                 return USBD_STATUS_CRC;
    case OHCI_CC_DEVICENOTRESPONDING: return USBD_STATUS_DEV_NOT_RESPONDING;
    case OHCI_CC_DATAOVERRUN:         return USBD_STATUS_DATA_OVERRUN;
    case OHCI_CC_DATAUNDERRUN:        return USBD_STATUS_DATA_UNDERRUN;
    case OHCI_CC_BUFFEROVERRUN:       return USBD_STATUS_BUFFER_OVERRUN;
    case OHCI_CC_BUFFERUNDERRUN:      return USBD_STATUS_BUFFER_UNDERRUN;
    case OHCI_CC_NOTACCESSED:         return USBD_STATUS_NOT_ACCESSED;
    default:                          return USBD_STATUS_INTERNAL_HC_ERROR;
    }
}

VOID
OhciPci_UrbComplete(struct ohci_urb *u)
{
    OHCIPCI_URB_CTX *uc =
        CONTAINING_RECORD(u, OHCIPCI_URB_CTX, CoreUrb);

    /* Isoch URBs (Plan 8 Task 8): no IN copy-back (DMA via
     * WdfDmaTransaction goes straight to/from the caller's MDL); write
     * per-packet IsoPacket[i].{Length,Status} from urb->isoc_pkts[],
     * tally ErrorCount, then short-circuit through the standard
     * txn/MDL teardown + deferred-completion path. Hdr.Status stays
     * USBD_STATUS_SUCCESS — per-packet detail lives in IsoPacket[]. */
    if (u->is_isoc) {
        POHCIPCI_TRANSFER_URB turb = (POHCIPCI_TRANSFER_URB)uc->TransferUrb;
        ULONG totalErr = 0;
        if (turb != NULL) {
            ULONG nPkts = turb->u.Isoch.NumberOfPackets;
            if (nPkts > u->isoc_pkt_count) nPkts = u->isoc_pkt_count;
            BOOLEAN isOut = (uc->DataDirection == OHCI_URB_DIR_OUT);
            ULONG urbLen = turb->TransferBufferLength;
            ULONG totalLen = 0;
            for (ULONG i = 0; i < nPkts; i++) {
                /* PSW.Size is only defined for IN per OHCI 1.0a §4.3.2.4
                 * — for OUT, qemu (and many real HCs) write garbage there.
                 * Reconstruct the per-packet bytes-transferred from the
                 * IsoPacket descriptor for OUT, trust the PSW for IN. */
                ULONG pktLen;
                if (isOut) {
                    ULONG pktOff = turb->u.Isoch.IsoPacket[i].Offset;
                    ULONG pktEnd = (i + 1 < nPkts)
                                    ? turb->u.Isoch.IsoPacket[i+1].Offset
                                    : urbLen;
                    pktLen = (pktEnd >= pktOff) ? (pktEnd - pktOff) : 0;
                    /* Don't zero pktLen on junk cc — PSW.CC is undefined
                     * for OUT (§4.3.2.4). usbaudio.sys advances its
                     * playback cursor by IsoPacket[i].Length, so reporting
                     * zero stalls the stream even though the bytes were
                     * actually shipped. */
                } else {
                    pktLen = u->isoc_pkts[i].length;
                }
                turb->u.Isoch.IsoPacket[i].Length = pktLen;
                /* OHCI 1.0a §4.3.2.4: PSW.CC is well-defined for IN packets
                 * but NOT for OUT — qemu and many real HCs leave it as junk
                 * (commonly DataUnderrun) on OUT retire. Reporting that
                 * back to usbaudio.sys causes it to treat every successful
                 * audio packet as "device under-consumed" and stall the
                 * stream. For OUT, force SUCCESS unless the CC is one of
                 * the unambiguous hard-error codes (STALL/CRC/timeout etc.).
                 * Inbox usbohci.sys does the equivalent. */
                USBD_STATUS pktStatus;
                if (isOut) {
                    switch (u->isoc_pkts[i].cc) {
                    case OHCI_CC_STALL:               pktStatus = USBD_STATUS_STALL_PID;          break;
                    case OHCI_CC_CRC:                 pktStatus = USBD_STATUS_CRC;                break;
                    case OHCI_CC_DEVICENOTRESPONDING: pktStatus = USBD_STATUS_DEV_NOT_RESPONDING; break;
                    default:                          pktStatus = USBD_STATUS_SUCCESS;            break;
                    }
                } else {
                    pktStatus = OhciPci_CcToUsbd(u->isoc_pkts[i].cc);
                }
                turb->u.Isoch.IsoPacket[i].Status = pktStatus;
                totalLen += pktLen;
                /* Match per-packet Status: only count real hard errors,
                 * and for OUT only when the cc is unambiguously fatal.
                 * Otherwise junk OUT cc inflates ErrorCount and trips
                 * usbaudio's per-URB error threshold. */
                if (pktStatus != USBD_STATUS_SUCCESS) {
                    totalErr++;
                }
            }
            turb->u.Isoch.ErrorCount   = totalErr;
            turb->TransferBufferLength = totalLen;
            {
                static ULONG s_isocCompletes = 0;
                ULONG n = ++s_isocCompletes;
                /* Per-URB cadence delta in µs from QPC. First retire on
                 * a fresh EP has LastTraceQpc==0 -> suppress the
                 * meaningless huge delta on the first sample. */
                LARGE_INTEGER freq;
                LARGE_INTEGER nowQpc = KeQueryPerformanceCounter(&freq);
                ULONGLONG deltaUs = 0;
                POHCIPCI_EP_CONTEXT epc = uc->EpCtx;
                if (epc->IsocLastTraceQpc.QuadPart != 0 && freq.QuadPart != 0) {
                    deltaUs =
                        ((ULONGLONG)(nowQpc.QuadPart - epc->IsocLastTraceQpc.QuadPart)
                         * 1000000ULL) / (ULONGLONG)freq.QuadPart;
                }
                epc->IsocLastTraceQpc = nowQpc;
                if (n <= 8 || (n & 0x1F) == 0) {
                    LOG("isoc[%lu] dir=%s urbLen=%lu totalLen=%lu err=%lu nPkts=%lu "
                        "delta_us=%llu cc=%u %u %u %u %u %u %u %u %u %u",
                        n, isOut ? "OUT" : "IN", urbLen, totalLen, totalErr, nPkts,
                        deltaUs,
                        u->isoc_pkts[0].cc, u->isoc_pkts[1].cc,
                        u->isoc_pkts[2].cc, u->isoc_pkts[3].cc,
                        u->isoc_pkts[4].cc, u->isoc_pkts[5].cc,
                        u->isoc_pkts[6].cc, u->isoc_pkts[7].cc,
                        u->isoc_pkts[8].cc, u->isoc_pkts[9].cc);
                }
            }
            turb->Hdr.Status           = USBD_STATUS_SUCCESS;
        }
        /* Unlink from EP's in-flight list and release the page-straddle
         * bounce slab (if BuildAndSubmit allocated one). */
        OhciPci_IsocOnUrbRetire_Locked(uc);
        if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
        uc->DeferredStatus = STATUS_SUCCESS;
        uc->DeferredInfo   = (ULONG_PTR)u->transferred;
        InsertTailList(&uc->EpCtx->Dc->DeferredCompletions, &uc->DeferredEntry);
        return;
    }

    /* IN data copy-back. If the destination map fails (low resources),
     * we MUST NOT complete the request as success — caller would read
     * stale memory and treat it as valid received data. Override status. */
    BOOLEAN copyBackFailed = FALSE;
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
        } else {
            copyBackFailed = TRUE;
        }
    }

    /* For isoch, per-packet IsoPacket[i].Status is the source of truth
     * (audio-class drivers aggregate it themselves and the URB-level
     * status is widely ignored). PSW.CC is undefined for OUT per OHCI
     * §4.3.2.4 and qemu writes garbage that flips du->status to OVERRUN
     * even on a perfectly-shipped audio frame. Mapping that to
     * STATUS_DEVICE_DATA_ERROR makes usbaudio drop the URB without
     * advancing its playback cursor → permanent silence. Always report
     * SUCCESS at the WdfRequest level for isoch. */
    NTSTATUS status;
    if (u->is_isoc) {
        status = STATUS_SUCCESS;
    } else {
        status = (u->status == OHCI_URB_STATUS_OK && !copyBackFailed)
                     ? STATUS_SUCCESS
                     : STATUS_DEVICE_DATA_ERROR;
    }
    ULONG_PTR info = (ULONG_PTR)u->transferred;
    /* Always log control completions while diagnosing enumeration. The
     * copyback-failed case used to be silent (status got overridden to
     * STATUS_DEVICE_DATA_ERROR but core_status stayed OK), which masks
     * the failure mode that makes UCX retry enumeration with successive
     * addresses — exactly what we see for the Logitech mouse. */
    LOG("UrbComplete: core_status=%d transferred=%u dir=%u copyBackFailed=%u "
        "len=%u userVa=%p userMdl=%p",
        u->status, u->transferred, uc->DataDirection,
        (unsigned)copyBackFailed, uc->DataLength, uc->UserVa, uc->UserMdl);

    /* Write the result back into the TRANSFER_URB. UCX reads
     * Hdr.Status + TransferBufferLength to learn how the transfer went;
     * the WDF request status alone is not enough.
     * Lifetime: TRANSFER_URB lives as long as the WDFREQUEST, and WDF
     * holds a reference to the request until we call WdfRequestComplete
     * below. Anyone adding a cancel path that completes the request
     * before the HC retires the TD must NOT also let this code run. */
    POHCIPCI_TRANSFER_URB turb = (POHCIPCI_TRANSFER_URB)uc->TransferUrb;
    if (turb != NULL) {
        turb->TransferBufferLength = copyBackFailed ? 0 : u->transferred;
        if (copyBackFailed) {
            turb->Hdr.Status = USBD_STATUS_INSUFFICIENT_RESOURCES;
        } else
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

    /* Release the WdfDmaTransaction (Bulk path) before completing the
     * request — frees map registers + (if HAL bounced) the low-mem buffer
     * the SG list pointed at. Per WDF docs we must signal completion with
     * the actual byte count before deleting the transaction. */
    if (uc->DmaTransaction) {
        NTSTATUS dmaSt;
        (void)WdfDmaTransactionDmaCompletedFinal(uc->DmaTransaction,
                                                 (size_t)u->transferred,
                                                 &dmaSt);
        WdfObjectDelete(uc->DmaTransaction);
        uc->DmaTransaction = NULL;
    }
    if (uc->OurMdl) {
        IoFreeMdl(uc->OurMdl);
        uc->OurMdl = NULL;
    }

    /* Return Control bounce buffers to pool before completing the request. */
    if (uc->SetupBounce) {
        OhciPci_BounceFree(uc->EpCtx->Dc, uc->SetupBounce);
        uc->SetupBounce = NULL;
    }
    if (uc->DataBounce) {
        OhciPci_BounceFree(uc->EpCtx->Dc, uc->DataBounce);
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
 * OhciPci_BulkProgramDma
 *
 * EVT_WDF_PROGRAM_DMA — invoked by HAL once map registers are reserved and
 * the SCATTER_GATHER_LIST is built. Every SgList->Elements[i].Address is
 * guaranteed within our 32-bit DMA mask (HAL bounces high-mem pages into
 * map registers transparently). Walk the SG list, build one OHCI TD per
 * element via ohci_bulk_submit_sg, and return TRUE.
 *
 * Runs at IRQL <= DISPATCH_LEVEL.
 * -------------------------------------------------------------------------- */
/* Release map registers + tear down the WdfDmaTransaction and complete
 * the request with failure. Used by every BulkProgramDma early-out path —
 * returning FALSE alone leaks map registers and never completes the URB,
 * which hangs the endpoint. After this returns, uc is gone (WDF freed
 * the request context); caller must not touch it. */
static VOID
OhciPci_BulkProgramDmaFail(OHCIPCI_URB_CTX *uc, NTSTATUS status)
{
    NTSTATUS dmaSt;
    (void)WdfDmaTransactionDmaCompletedFinal(uc->DmaTransaction, 0, &dmaSt);
    WdfObjectDelete(uc->DmaTransaction);
    uc->DmaTransaction = NULL;
    if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
    WdfRequestComplete(uc->Request, status);
}

static BOOLEAN
OhciPci_BulkProgramDma(
    WDFDMATRANSACTION Transaction,
    WDFDEVICE         Device,
    PVOID             Context,
    WDF_DMA_DIRECTION Direction,
    PSCATTER_GATHER_LIST SgList)
{
    UNREFERENCED_PARAMETER(Transaction);
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(Direction);

    OHCIPCI_URB_CTX    *uc = (OHCIPCI_URB_CTX *)Context;
    OHCIPCI_EP_CONTEXT *ep = uc->EpCtx;
    PDEVICE_CONTEXT     dc = ep->Dc;

    if (SgList->NumberOfElements == 0) {
        LOG("BulkProgramDma: empty SG list");
        OhciPci_BulkProgramDmaFail(uc, STATUS_INVALID_PARAMETER);
        return TRUE;
    }

    /* Split each SG element down to PAGE_SIZE chunks aligned on page
     * boundaries. OHCI §4.3.1.4 allows a TD's CBP→BE to straddle at most
     * one page boundary (~8 KB max), but a SCATTER_GATHER_ELEMENT can be
     * a long physically-contiguous run. Building one TD per multi-page
     * element causes the HC to silently send only the first 4–8 KB
     * (CC=NoError, no underrun) — observed as a USB-MSD CSW STALL after
     * an apparently-successful 16896-byte WRITE_10 OUT. */
    struct ohci_bulk_sg_page sg[OHCI_BULK_MAX_SG_PAGES];
    uint32_t urbOff = 0;
    ULONG    sgCount = 0;
    for (ULONG i = 0; i < SgList->NumberOfElements; i++) {
        uint32_t base = SgList->Elements[i].Address.LowPart;
        ULONG    rem  = SgList->Elements[i].Length;
        uint32_t pos  = base;
        while (rem > 0) {
            /* Guard: pos in the top page would wrap pageEnd to 0 and
             * underflow chunk. HAL with our 32-bit mask shouldn't hand
             * us anything ≥ 0xFFFFF000, but cheap to check. */
            if (pos >= 0xFFFFF000u) {
                LOG("BulkProgramDma: SG addr 0x%08X near 4 GB", pos);
                OhciPci_BulkProgramDmaFail(uc, STATUS_INVALID_PARAMETER);
                return TRUE;
            }
            uint32_t pageEnd = (pos + PAGE_SIZE) & ~(PAGE_SIZE - 1u);
            uint32_t chunk   = pageEnd - pos;
            if (chunk > rem) chunk = rem;
            if (sgCount >= OHCI_BULK_MAX_SG_PAGES) {
                LOG("BulkProgramDma: post-split SG count exceeds %u",
                    OHCI_BULK_MAX_SG_PAGES);
                OhciPci_BulkProgramDmaFail(uc, STATUS_INSUFFICIENT_RESOURCES);
                return TRUE;
            }
            sg[sgCount].phys   = pos;
            sg[sgCount].length = chunk;
            sg[sgCount].off    = urbOff;
            sgCount++;
            urbOff += chunk;
            pos    += chunk;
            rem    -= chunk;
        }
    }

    /* CoreUrb's buffer_phys must yield the per-TD CBP via
     * (buffer_phys + chunk_off). With the SG layout above, that's
     * sg[0].phys (since sg[0].off==0). The drain accumulator uses
     * data_tds[].chunk_off for per-TD math, so this is mainly a
     * back-compat field. */
    uc->CoreUrb.buffer      = NULL;
    uc->CoreUrb.buffer_phys = sg[0].phys;
    uc->CoreUrb.length      = urbOff;
    uc->CoreUrb.direction   = uc->DataDirection;
    uc->CoreUrb.complete    = OhciPci_UrbComplete;

    WdfSpinLockAcquire(dc->CoreLock);
    int rc = ohci_bulk_submit_sg(&dc->Hc, &ep->Core.Bulk, &uc->CoreUrb,
                                  sg, sgCount);
    WdfSpinLockRelease(dc->CoreLock);
    if (rc != 0) {
        LOG("BulkProgramDma: ohci_bulk_submit_sg rc=%d", rc);
        OhciPci_BulkProgramDmaFail(uc, STATUS_INSUFFICIENT_RESOURCES);
        return TRUE;
    }
    /* Async completion path runs OhciPci_UrbComplete; that releases the
     * WdfDmaTransaction via WdfDmaTransactionDmaCompletedFinal. */
    return TRUE;
}

/* --------------------------------------------------------------------------
 * OhciPci_HandleBulkUrb
 *
 * Builds a WdfDmaTransaction for the caller's MDL and hands it to the HAL.
 * The HAL deals with 32-bit address constraints (map-register bouncing) and
 * SG fragmentation transparently — no driver-side bounce slab needed.
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

    if (length == 0) {
        urb->TransferBufferLength = 0;
        urb->Hdr.Status = USBD_STATUS_SUCCESS;
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }
    /* WdfDmaTransactionInitialize needs an MDL. Storage class driver
     * occasionally hands us a flat KVA buffer with no MDL (e.g. an
     * 18-byte INQUIRY response staged in the URB itself). MmBuildMdlForNonPagedPool
     * is only safe if the VA is in non-paged pool — a buggy or malicious
     * upper driver could pass a paged or session-space VA, which would
     * yield bogus PFNs that the HC then DMAs against (kernel memory
     * corruption). Cap the no-MDL case at a small size that matches the
     * legitimate use (short class-staged buffers) so a large flat-KVA
     * URB cannot weaponise this path. */
    PMDL ourMdl = NULL;
    if (bufMdl == NULL) {
        const ULONG OHCIPCI_BULK_NO_MDL_MAX = 512;
        if (bufVa == NULL || length > OHCIPCI_BULK_NO_MDL_MAX) {
            LOG("HandleBulkUrb: rejecting MDL-less URB len=%lu (max %lu)",
                length, OHCIPCI_BULK_NO_MDL_MAX);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        ourMdl = IoAllocateMdl(bufVa, length, FALSE, FALSE, NULL);
        if (ourMdl == NULL) {
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INSUFFICIENT_RESOURCES;
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        MmBuildMdlForNonPagedPool(ourMdl);
        bufMdl = ourMdl;
    }

    /* Allocate per-URB context on the WDFREQUEST. */
    WDF_OBJECT_ATTRIBUTES reqAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&reqAttrs, OHCIPCI_URB_CTX);
    NTSTATUS status = WdfObjectAllocateContext(Request, &reqAttrs, NULL);
    if (!NT_SUCCESS(status)) {
        if (ourMdl) IoFreeMdl(ourMdl);
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
    uc->OurMdl        = ourMdl;

    /* Build a WdfDmaTransaction over the caller's MDL. The HAL owns the
     * SG mapping lifetime; we release in OhciPci_UrbComplete. */
    WDF_OBJECT_ATTRIBUTES txnAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT(&txnAttrs);
    status = WdfDmaTransactionCreate(dc->DmaEnabler, &txnAttrs,
                                      &uc->DmaTransaction);
    if (!NT_SUCCESS(status)) {
        LOG("WdfDmaTransactionCreate failed: 0x%08X", status);
        if (ourMdl) { IoFreeMdl(ourMdl); uc->OurMdl = NULL; }
        WdfRequestComplete(Request, status);
        return;
    }

    PVOID va = bufVa ? bufVa : MmGetMdlVirtualAddress(bufMdl);
    status = WdfDmaTransactionInitialize(uc->DmaTransaction,
                                          OhciPci_BulkProgramDma,
                                          isIn ? WdfDmaDirectionReadFromDevice
                                               : WdfDmaDirectionWriteToDevice,
                                          bufMdl,
                                          va,
                                          length);
    if (!NT_SUCCESS(status)) {
        LOG("WdfDmaTransactionInitialize failed: 0x%08X", status);
        WdfObjectDelete(uc->DmaTransaction);
        uc->DmaTransaction = NULL;
        if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
        WdfRequestComplete(Request, status);
        return;
    }

    /* Execute kicks off SG mapping. HAL invokes OhciPci_BulkProgramDma at
     * DISPATCH_LEVEL, where the actual ohci_bulk_submit_sg happens. */
    status = WdfDmaTransactionExecute(uc->DmaTransaction, uc);
    if (!NT_SUCCESS(status)) {
        LOG("WdfDmaTransactionExecute failed: 0x%08X", status);
        WdfObjectDelete(uc->DmaTransaction);
        uc->DmaTransaction = NULL;
        if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
        WdfRequestComplete(Request, status);
        return;
    }
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
        if (src == NULL) {
            /* Submitting without copying would leak prior bounce-slab
             * contents to the device (info disclosure across endpoints)
             * and corrupt the OUT payload. Fail the request instead. */
            OhciPci_BounceFree(dc, uc->DataBounce);
            uc->DataBounce = NULL;
            WdfSpinLockRelease(dc->CoreLock);
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        RtlCopyMemory(uc->DataBounce, src, length);
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
 * Plan 8 Task 7 — UCXENDPOINT context cleanup.
 *
 * Registered on epAttrs.EvtCleanupCallback for ALL EP kinds; no-ops for
 * non-Isoc. Without this, when an isoch EP is torn down (UCX hot-unplug
 * or driver unload) the EP context remains spliced into dc->IsocEps via
 * IsocEpEntry. The next IsocRefillAll_Locked walks the freed memory =>
 * use-after-free.
 *
 * Cleanup runs at PASSIVE with no CoreLock in scope, so it is safe to
 * call WdfRequestComplete inline (no DeferredCompletions detour needed).
 * The silence buffer is part of dc->DmaRegion — bump-allocator with no
 * per-allocation free, so accept the bounded leak.
 * -------------------------------------------------------------------------- */
static EVT_WDF_OBJECT_CONTEXT_CLEANUP OhciPci_EpContextCleanup;
static VOID
OhciPci_EpContextCleanup(WDFOBJECT Object)
{
    POHCIPCI_EP_CONTEXT ep = OhciPci_EpContextGet((UCXENDPOINT)Object);
    if (ep == NULL) return;
    PDEVICE_CONTEXT dc = ep->Dc;

    /* Unlink from owning device's EndpointList first. Cleanup runs at
     * PASSIVE; spinlock is brief (one RemoveEntryList). The Flink-NULL
     * check makes this safe for EPs whose create failed before insert. */
    if (ep->Udc != NULL && ep->Udc->EndpointListLock != NULL &&
        ep->DeviceEpEntry.Flink != NULL)
    {
        WdfSpinLockAcquire(ep->Udc->EndpointListLock);
        if (ep->DeviceEpEntry.Flink != NULL) {
            RemoveEntryList(&ep->DeviceEpEntry);
            ep->DeviceEpEntry.Flink = NULL;
        }
        WdfSpinLockRelease(ep->Udc->EndpointListLock);
    }

    /* Refund any periodic-bandwidth budget this EP charged at create
     * time. Done unconditionally (any Kind) — PeriodicBudgetCharged is
     * 0 for EPs that never charged (Bulk/Control, or rolled-back creates),
     * so the subtract is a no-op there. Read+clear under DeferredLock-free
     * paths is fine: cleanup runs after UcxEndpointPurge so no concurrent
     * EpAdd touches PeriodicBytesPerFrame for this EP. The summed counter
     * itself is updated under no lock today (single-controller assumption);
     * keep that invariant. */
    if (dc != NULL && ep->PeriodicBudgetCharged != 0) {
        ULONG charged = ep->PeriodicBudgetCharged;
        ep->PeriodicBudgetCharged = 0;
        if (dc->PeriodicBytesPerFrame >= charged) {
            dc->PeriodicBytesPerFrame -= charged;
        } else {
            /* Defensive: shouldn't happen if charges/refunds are paired. */
            LOG("EpCleanup: budget underflow refund=%lu sum=%lu",
                charged, dc->PeriodicBytesPerFrame);
            dc->PeriodicBytesPerFrame = 0;
        }
    }

    /* Release OHCI ED back to its pool for non-Isoch EPs. UCX destroys
     * the UCXENDPOINT object on device disable / reconfigure, and
     * without this the ED leaks — after a few enumeration retries the
     * 16-slot control_ed_pool is empty and DefaultEndpointAdd fails
     * with ohci_control_endpoint_create returning -1. The
     * EvtUsbDeviceEndpointsConfigure path destroys non-default EPs
     * explicitly when UCX hands them in EndpointsToDisable[]; this
     * cleanup catches the default EP plus any non-default EP whose
     * UCXENDPOINT outlived its core ED for any reason. */
    if (dc != NULL && ep->Kind != OhciPciEpKindIsoc) {
        WdfSpinLockAcquire(dc->CoreLock);
        switch (ep->Kind) {
        case OhciPciEpKindControl:
            if (ep->Core.Control.ed != NULL) {
                ohci_control_endpoint_destroy(&dc->Hc, &ep->Core.Control);
                ep->Core.Control.ed = NULL;
            }
            break;
        case OhciPciEpKindBulk:
            if (ep->Core.Bulk.ed != NULL) {
                ohci_bulk_endpoint_destroy(&dc->Hc, &ep->Core.Bulk);
                ep->Core.Bulk.ed = NULL;
            }
            break;
        case OhciPciEpKindInterrupt:
            if (ep->Core.Interrupt.ed != NULL) {
                ohci_interrupt_endpoint_destroy(&dc->Hc, &ep->Core.Interrupt);
                ep->Core.Interrupt.ed = NULL;
            }
            break;
        default:
            break;
        }
        WdfSpinLockRelease(dc->CoreLock);
        return;
    }

    /* Isoch-only teardown follows. */
    if (ep->Kind != OhciPciEpKindIsoc || dc == NULL) return;

    /* Unlink from dc->IsocEps so the refill walker stops touching us.
     * Defensive: if Flink is NULL we were never spliced (e.g. EP create
     * failed before InsertTailList) — skip. */
    if (dc->IsocEpsLock) {
        WdfSpinLockAcquire(dc->IsocEpsLock);
        if (ep->IsocEpEntry.Flink != NULL) {
            RemoveEntryList(&ep->IsocEpEntry);
            ep->IsocEpEntry.Flink = NULL;
        }
        WdfSpinLockRelease(dc->IsocEpsLock);
    }

    /* Drain in-flight URBs (ITDs already linked into the ED) first.
     * Defensive — OHCI core's destroy normally retires these via the
     * per-URB complete callback, but Skip/halt edge cases can leave
     * them stranded. */
    OhciPci_IsocEpTeardown(ep);

    /* Drain queued URBs (not yet handed to BuildAndSubmit) and complete
     * each with STATUS_CANCELLED.
     *
     * Done AFTER teardown to close the parallel-dispatch race: with
     * isoch on WdfIoQueueDispatchParallel, a HandleIsocUrb running on
     * another CPU can insert into IsocQueuedUrbs while teardown is
     * walking IsocInFlightUrbs. By draining queued last we catch
     * anything inserted during the teardown window. */
    if (ep->IsocQueueLock) {
        LIST_ENTRY local;
        InitializeListHead(&local);
        WdfSpinLockAcquire(ep->IsocQueueLock);
        while (!IsListEmpty(&ep->IsocQueuedUrbs)) {
            PLIST_ENTRY le = RemoveHeadList(&ep->IsocQueuedUrbs);
            InsertTailList(&local, le);
        }
        WdfSpinLockRelease(ep->IsocQueueLock);

        while (!IsListEmpty(&local)) {
            PLIST_ENTRY le = RemoveHeadList(&local);
            POHCIPCI_URB_CTX uc =
                CONTAINING_RECORD(le, OHCIPCI_URB_CTX, QueueEntry);
            /* Isoch path never allocates a WdfDmaTransaction (MDL-walk
             * goes straight to ITDs), so no DMA teardown is needed here. */
            if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
            WdfRequestComplete(uc->Request, STATUS_CANCELLED);
        }
    }
}

/* --------------------------------------------------------------------------
 * Plan 8 Task 7 — refill DPC + silence ITDs.
 *
 * The OHCI HC walks the periodic schedule per frame and dispatches whatever
 * ITDs are currently in the chain. If the chain is empty (caller hasn't
 * delivered the next URB yet) the HC silently skips the slot — audible click
 * to the device. To prevent that we keep the chain extended at least
 * OHCIPCI_ISOC_REFILL_HIGH frames ahead of HcFmNumber, padding with
 * silence ITDs when no caller URB is queued.
 *
 * The refill walker runs from EvtDpc (after each WDH drain) AND from a
 * 1ms WDFTIMER backstop, so caller stalls don't starve the chain.
 * -------------------------------------------------------------------------- */
static VOID
OhciPci_IsocRefillOne_Locked(POHCIPCI_EP_CONTEXT ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    struct ohci_isoc_endpoint *ie = &ep->Core.Isoc;

    /* Skip non-isoc EPs (defensive — shouldn't be on the list). */
    if (ep->Kind != OhciPciEpKindIsoc) return;

    uint16_t fmNumber = (uint16_t)(dc->MmioOps.read32(dc->MmioOps.context, 0x3C) & 0xFFFFu);

    /* Drain queued URBs first — they take precedence over silence. The
     * very first URB on a fresh EP also primes the chain (sf = HcFmNumber
     * + lookahead, set inside IsocProgramDma when ie->primed == 0). */
    for (;;) {
        SHORT lead = ie->primed ? (SHORT)(ie->ed_tail_frame - fmNumber) : 0;
        if (ie->primed && lead >= (SHORT)OHCIPCI_ISOC_REFILL_HIGH) break;

        OHCIPCI_URB_CTX *uc = NULL;
        WdfSpinLockAcquire(ep->IsocQueueLock);
        if (!IsListEmpty(&ep->IsocQueuedUrbs)) {
            PLIST_ENTRY e = RemoveHeadList(&ep->IsocQueuedUrbs);
            uc = CONTAINING_RECORD(e, OHCIPCI_URB_CTX, QueueEntry);
        }
        WdfSpinLockRelease(ep->IsocQueueLock);

        if (uc == NULL) {
            break;
        }

        /* Walk the MDL and emit ITDs directly. CoreLock stays held across
         * the call. On failure BuildAndSubmit stages the URB on
         * dc->DeferredCompletions itself. */
        NTSTATUS s = OhciPci_IsocBuildAndSubmit_Locked(ep, uc);
        (void)s;   /* failure already routed onto DeferredCompletions */

        /* Re-read fmNumber: BuildAndSubmit may have consumed enough cycles
         * for the frame counter to advance. */
        fmNumber = (uint16_t)(dc->MmioOps.read32(dc->MmioOps.context, 0x3C) & 0xFFFFu);
    }

    /* Don't emit silence ITDs at all. Earlier "silence-on-underrun" logic
     * stretched ed_tail_frame 8 frames forward whenever lead<=0, which
     * snowballed the per-URB cadence from 10ms to ~18ms (10ms audio +
     * 8ms padded silence) and caused qemu's audio FIFO to perpetually
     * starve — audible as total silence at the host. Fix: when a real
     * URB arrives after the chain has stalled, IsocProgramDma above
     * snaps sf to (fmNumber + lookahead) so it plays now, not 8ms late.
     * Walking dead chain in the gap is harmless (HC sends nothing). */
    (void)fmNumber;
}

VOID
OhciPci_IsocRefillAll_Locked(PDEVICE_CONTEXT dc)
{
    /* CoreLock must be held by the caller (EvtDpc, IsocBackstopTimer, or
     * HandleIsocUrb's queue-then-refill path). IsocEpsLock is acquired
     * second; no path takes both in the opposite order. */
    if (dc->IsocEpsLock == NULL) {
        LOG("IsocRefillAll: IsocEpsLock NULL — refill disabled");
        return;
    }
    WdfSpinLockAcquire(dc->IsocEpsLock);
    PLIST_ENTRY e;
    ULONG nEps = 0;
    for (e = dc->IsocEps.Flink; e != &dc->IsocEps; e = e->Flink) {
        nEps++;
        POHCIPCI_EP_CONTEXT ep =
            CONTAINING_RECORD(e, OHCIPCI_EP_CONTEXT, IsocEpEntry);
        OhciPci_IsocRefillOne_Locked(ep);
    }
    UNREFERENCED_PARAMETER(nEps);
    WdfSpinLockRelease(dc->IsocEpsLock);
}

/* WDFTIMER callback — periodic backstop. Acquires CoreLock, calls refill,
 * then drains any DeferredCompletions outside CoreLock per the standard
 * pattern (see EvtDpc + feedback_wdf_complete_under_spinlock.md). */
EVT_WDF_TIMER OhciPci_EvtIsocBackstopTimer;
VOID
OhciPci_EvtIsocBackstopTimer(WDFTIMER Timer)
{
    PDEVICE_CONTEXT dc = DeviceContextGet(WdfTimerGetParentObject(Timer));

    LIST_ENTRY local;
    InitializeListHead(&local);

    WdfSpinLockAcquire(dc->CoreLock);
    OhciPci_IsocRefillAll_Locked(dc);
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

static VOID
OhciPci_HandleIsocUrb(
    OHCIPCI_EP_CONTEXT   *ep,
    WDFREQUEST            Request,
    POHCIPCI_TRANSFER_URB urb)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    ULONG   length = urb->TransferBufferLength;
    PVOID   bufVa  = urb->TransferBuffer;
    PMDL    bufMdl = urb->TransferBufferMDL;
    BOOLEAN isIn   = !!(urb->TransferFlags & USBD_TRANSFER_DIRECTION_IN);

    {
        static ULONG s_isocSubmits = 0;
        ULONG n = ++s_isocSubmits;
        if (n <= 8 || (n & 0x1F) == 0) {
            LOG("isoc-submit[%lu] len=%lu nPkts=%lu dir=%s",
                n, length, urb->u.Isoch.NumberOfPackets, isIn ? "IN" : "OUT");
        }
    }

    if (length == 0 || urb->u.Isoch.NumberOfPackets == 0) {
        urb->TransferBufferLength = 0;
        urb->Hdr.Status = USBD_STATUS_SUCCESS;
        WdfRequestComplete(Request, STATUS_SUCCESS);
        return;
    }

    /* MDL handling — same shape as HandleBulkUrb. Cap MDL-less buffers
     * tightly to defend against weaponised paged-VA URBs. */
    PMDL ourMdl = NULL;
    if (bufMdl == NULL) {
        const ULONG OHCIPCI_ISOC_NO_MDL_MAX = 512;
        if (bufVa == NULL || length > OHCIPCI_ISOC_NO_MDL_MAX) {
            LOG("HandleIsocUrb: rejecting MDL-less URB len=%lu (max %lu)",
                length, OHCIPCI_ISOC_NO_MDL_MAX);
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INVALID_PARAMETER;
            WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
            return;
        }
        ourMdl = IoAllocateMdl(bufVa, length, FALSE, FALSE, NULL);
        if (ourMdl == NULL) {
            urb->TransferBufferLength = 0;
            urb->Hdr.Status = USBD_STATUS_INSUFFICIENT_RESOURCES;
            WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
            return;
        }
        MmBuildMdlForNonPagedPool(ourMdl);
        bufMdl = ourMdl;
    }

    /* Per-URB context. */
    WDF_OBJECT_ATTRIBUTES reqAttrs;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&reqAttrs, OHCIPCI_URB_CTX);
    NTSTATUS status = WdfObjectAllocateContext(Request, &reqAttrs, NULL);
    if (!NT_SUCCESS(status)) {
        if (ourMdl) IoFreeMdl(ourMdl);
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
    uc->OurMdl        = ourMdl;
    uc->UserMdl       = bufMdl;       /* may be ourMdl if caller had no MDL */
    uc->UserVa        = bufVa;

    /* Isoch uses no WdfDmaTransaction. The MDL-walk path
     * (OhciPci_IsocBuildAndSubmit_Locked) emits ITDs against the user MDL
     * directly. uc->DmaTransaction stays NULL on the isoch path; the
     * (now-removed) `if (uc->DmaTransaction)` branch in OhciPci_UrbComplete
     * therefore never fired for isoch and would have bug-checked
     * WDF_VIOLATION 0x10D sub 8 if it had.
     *
     * Queue the URB onto the EP's pending list. The refill walker drives
     * OhciPci_IsocBuildAndSubmit_Locked when the ED chain has headroom
     * (lead < OHCIPCI_ISOC_REFILL_HIGH frames). Trigger one immediate
     * refill cycle so single-shot callers drain right away if there's room.
     * Failures from BuildAndSubmit land on DeferredCompletions. */
    WdfSpinLockAcquire(ep->IsocQueueLock);
    InsertTailList(&ep->IsocQueuedUrbs, &uc->QueueEntry);
    WdfSpinLockRelease(ep->IsocQueueLock);

    WdfSpinLockAcquire(dc->CoreLock);
    OhciPci_IsocRefillAll_Locked(dc);
    /* Drain any deferred completions/failures the synchronous Execute may
     * have staged, mirroring EvtDpc's pattern. */
    LIST_ENTRY local;
    InitializeListHead(&local);
    while (!IsListEmpty(&dc->DeferredCompletions)) {
        PLIST_ENTRY le = RemoveHeadList(&dc->DeferredCompletions);
        InsertTailList(&local, le);
    }
    WdfSpinLockRelease(dc->CoreLock);

    while (!IsListEmpty(&local)) {
        PLIST_ENTRY le = RemoveHeadList(&local);
        POHCIPCI_URB_CTX duc =
            CONTAINING_RECORD(le, OHCIPCI_URB_CTX, DeferredEntry);
        WdfRequestCompleteWithInformation(duc->Request,
                                           duc->DeferredStatus,
                                           duc->DeferredInfo);
    }
    /* Async completion via OhciPci_UrbComplete. */
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
    if (fn == URB_FUNCTION_ISOCH_TRANSFER) {
        OhciPci_HandleIsocUrb(ep, Request, urb);
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
            /* OHCI §6.4.4: HCD must not write HeadP while the ED is on the
             * schedule. Set K (skip) and wait one frame so the HC retires
             * its current walk past this ED before we mutate HeadP. */
            target->Control |= OHCI_ED_K;
            dc->MmioOps.barrier(dc->MmioOps.context);
            uint32_t f0 = dc->MmioOps.read32(dc->MmioOps.context, 0x3C);
            for (int i = 0; i < 10000; i++) {
                if (dc->MmioOps.read32(dc->MmioOps.context, 0x3C) != f0) break;
            }
            target->HeadP &= ~(uint32_t)(OHCI_ED_HEADP_H | OHCI_ED_HEADP_C);
            dc->MmioOps.barrier(dc->MmioOps.context);
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
            if (src == NULL) {
                OhciPci_BounceFree(dc, uc->SetupBounce);
                OhciPci_BounceFree(dc, uc->DataBounce);
                WdfSpinLockRelease(dc->CoreLock);
                WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
                return;
            }
            RtlCopyMemory(uc->DataBounce, src, length);
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
    /* Diagnostic: snapshot OHCI register state immediately after submit so
     * a stuck schedule (HC not Operational, control list disabled, CLF not
     * latched, ED halted, etc.) is visible from the boot log. */
    {
        PUCHAR mmio = (PUCHAR)dc->MmioBase;
        ULONG hcControl     = READ_REGISTER_ULONG((PULONG)(mmio + 0x04));
        ULONG hcCmdStatus   = READ_REGISTER_ULONG((PULONG)(mmio + 0x08));
        ULONG hcIntStatus   = READ_REGISTER_ULONG((PULONG)(mmio + 0x0C));
        ULONG hcIntEnable   = READ_REGISTER_ULONG((PULONG)(mmio + 0x10));
        ULONG hcFmNumber    = READ_REGISTER_ULONG((PULONG)(mmio + 0x3C));
        ULONG hcCtlHeadED   = READ_REGISTER_ULONG((PULONG)(mmio + 0x20));
        ULONG hcCtlCurED    = READ_REGISTER_ULONG((PULONG)(mmio + 0x24));
        ULONG hcFmInterval  = READ_REGISTER_ULONG((PULONG)(mmio + 0x34));
        ULONG hcPerStart    = READ_REGISTER_ULONG((PULONG)(mmio + 0x40));
        ULONG hcHccaReg     = READ_REGISTER_ULONG((PULONG)(mmio + 0x18));
        struct ohci_ed *ed  = ep->Core.Control.ed;
        LOG("post-submit: HcControl=0x%08X CmdStat=0x%08X IntSt=0x%08X "
            "IntEn=0x%08X FmNum=0x%04X CtlHead=0x%08X CtlCur=0x%08X",
            hcControl, hcCmdStatus, hcIntStatus, hcIntEnable,
            hcFmNumber & 0xFFFF, hcCtlHeadED, hcCtlCurED);
        LOG("post-submit: HcFmInterval=0x%08X HcPeriodicStart=0x%08X HcHCCA=0x%08X",
            hcFmInterval, hcPerStart, hcHccaReg);
        /* HCCA.FrameNumber is written by HC every SOF. If it lags
         * HcFmNumber, the HC's DMA-write path to memory is broken; if
         * it matches, writes work and only TD/ED reads are suspect. */
        if (dc->Hc.hcca) {
            uint16_t hccaFn  = dc->Hc.hcca->FrameNumber;
            uint16_t hccaPad = dc->Hc.hcca->PadFrameNumber;
            uint32_t hccaDh  = dc->Hc.hcca->DoneHead;
            LOG("post-submit HCCA: FrameNumber=0x%04X Pad=0x%04X DoneHead=0x%08X (HcFmNum=0x%04X)",
                hccaFn, hccaPad, hccaDh, hcFmNumber & 0xFFFF);
        }
        LOG("post-submit ED: Control=0x%08X TailP=0x%08X HeadP=0x%08X NextED=0x%08X",
            ed->Control, ed->TailP, ed->HeadP, ed->NextED);
        /* Walk the TD chain HeadP..TailP from CPU side and log raw dwords —
         * if HC stalls on a TD, comparing what we wrote vs what HC reads
         * will tell us whether the descriptors are reaching DRAM at all. */
        uint32_t headPhys = ed->HeadP & ~0xFu;
        uint32_t tailPhys = ed->TailP & ~0xFu;
        for (int i = 0; i < 4 && headPhys != 0 && headPhys != tailPhys; i++) {
            if (headPhys < dc->DmaRegion.phys_base ||
                headPhys >= dc->DmaRegion.phys_base + dc->DmaRegion.size) {
                LOG("  TD@0x%08X: out of DMA region — bogus pointer", headPhys);
                break;
            }
            ULONG off = headPhys - dc->DmaRegion.phys_base;
            volatile ULONG *td = (volatile ULONG *)((PUCHAR)dc->DmaRegion.base + off);
            LOG("  TD@0x%08X: cw=0x%08X CBP=0x%08X NextTD=0x%08X BE=0x%08X",
                headPhys, td[0], td[1], td[2], td[3]);
            headPhys = td[2] & ~0xFu;  /* NextTD */
        }
    }
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
struct ohci_ed *
OhciPci_EpEd(OHCIPCI_EP_CONTEXT *ep)
{
    switch (ep->Kind) {
    case OhciPciEpKindControl:   return ep->Core.Control.ed;
    case OhciPciEpKindBulk:      return ep->Core.Bulk.ed;
    case OhciPciEpKindInterrupt: return ep->Core.Interrupt.ed;
    case OhciPciEpKindIsoc:      return ep->Core.Isoc.ed;
    }
    return NULL;
}

/* OHCI §6.4.4: HCD must not write ED.HeadP while the ED is on the
 * schedule. Set K (skip), wait one HcFmNumber tick so the HC retires
 * its current walk past this ED, edit HeadP via the supplied callback,
 * then clear K. CoreLock alone is insufficient because it doesn't pause
 * the HC's schedule walker. Caller must NOT hold CoreLock (we acquire). */
VOID
OhciPci_EditHeadPSafely(PDEVICE_CONTEXT dc, struct ohci_ed *ed,
                         ohcipci_headp_edit_fn edit, void *ctx)
{
    WdfSpinLockAcquire(dc->CoreLock);
    ed->Control |= OHCI_ED_K;
    dc->MmioOps.barrier(dc->MmioOps.context);
    uint32_t f0 = dc->MmioOps.read32(dc->MmioOps.context, 0x3C);
    for (int i = 0; i < 10000; i++) {
        if (dc->MmioOps.read32(dc->MmioOps.context, 0x3C) != f0) break;
    }
    edit(ed, ctx);
    dc->MmioOps.barrier(dc->MmioOps.context);
    ed->Control &= ~OHCI_ED_K;
    dc->MmioOps.barrier(dc->MmioOps.context);
    WdfSpinLockRelease(dc->CoreLock);
}

VOID OhciPci_HeadPClearHC(struct ohci_ed *ed, void *ctx)
{
    UNREFERENCED_PARAMETER(ctx);
    ed->HeadP &= ~(uint32_t)(OHCI_ED_HEADP_H | OHCI_ED_HEADP_C);
}

/* HeadP-edit callback: force HeadP = TailP & ~0xF. Used after URB cancel
 * to flush stale TD pointers — ohci_urb_cancel_for_ed removes URBs from
 * the in-flight list and frees their TDs, but does not advance HeadP, so
 * HeadP may still point at a freed/reused TD. Resetting HeadP to TailP
 * yields an empty queue from the HC's perspective; the next submit
 * appends at TailP and HC starts cleanly. Also clears H and C. */
VOID OhciPci_HeadPFlushToTail(struct ohci_ed *ed, void *ctx)
{
    UNREFERENCED_PARAMETER(ctx);
    ed->HeadP = ed->TailP & ~(uint32_t)0xF;
}

/* HeadP-edit callback for OhciPci_EditHeadPSafely. ctx is a pointer to a
 * uint16_t carrying the new MaxPacketSize. Rewrites the MPS field
 * (OHCI §4.2.1, bits OHCI_ED_MPS_SHIFT+10..OHCI_ED_MPS_SHIFT in ed->Control). */
VOID OhciPci_HeadPSetMps(struct ohci_ed *ed, void *ctx)
{
    uint16_t mps = *(uint16_t *)ctx;
    ed->Control = (ed->Control & ~((uint32_t)0x7FFu << OHCI_ED_MPS_SHIFT)) |
                  (((uint32_t)mps & 0x7FFu) << OHCI_ED_MPS_SHIFT);
}

/* Re-enable an EP whose ED was halted by a prior purge/abort. UCX will
 * call Start after Purge — without clearing Skip, the HC ignores the
 * ED and the next URB on it sits forever (HC never raises WDH → timeout). */
VOID
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

/* Symmetric to OhciPci_StartEp: set ED.K so the HC stops scheduling new
 * TDs from this ED. In-flight TDs already submitted retire normally.
 * Unlike EditHeadPSafely we do NOT busy-wait one frame here — caller
 * is the device-level Disable path which doesn't immediately edit HeadP. */
VOID
OhciPci_HaltEp(OHCIPCI_EP_CONTEXT *ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    struct ohci_ed *ed = OhciPci_EpEd(ep);
    if (ed == NULL) return;
    WdfSpinLockAcquire(dc->CoreLock);
    ed->Control |= OHCI_ED_K;
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

    OhciPci_EditHeadPSafely(dc, ed, OhciPci_HeadPFlushToTail, NULL);

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
    PDEVICE_CONTEXT dc = ep->Dc;
    LOG("DefaultEp Purge");
    /* Snapshot HC state at Purge time so we can compare with the
     * post-submit dump and tell whether HC made any progress between
     * submit and the unplug-driven Purge. */
    if (dc && dc->MmioBase) {
        PUCHAR mmio = (PUCHAR)dc->MmioBase;
        ULONG hcControl   = READ_REGISTER_ULONG((PULONG)(mmio + 0x04));
        ULONG hcCmdStat   = READ_REGISTER_ULONG((PULONG)(mmio + 0x08));
        ULONG hcIntStat   = READ_REGISTER_ULONG((PULONG)(mmio + 0x0C));
        ULONG hcFmNumber  = READ_REGISTER_ULONG((PULONG)(mmio + 0x3C));
        ULONG hcCtlHead   = READ_REGISTER_ULONG((PULONG)(mmio + 0x20));
        ULONG hcCtlCur    = READ_REGISTER_ULONG((PULONG)(mmio + 0x24));
        struct ohci_ed *ed = ep->Core.Control.ed;
        LOG("Purge HC: HcCtl=0x%08X CmdStat=0x%08X IntSt=0x%08X FmNum=0x%04X "
            "CtlHead=0x%08X CtlCur=0x%08X",
            hcControl, hcCmdStat, hcIntStat, hcFmNumber & 0xFFFF,
            hcCtlHead, hcCtlCur);
        LOG("Purge ED: Control=0x%08X TailP=0x%08X HeadP=0x%08X NextED=0x%08X",
            ed->Control, ed->TailP, ed->HeadP, ed->NextED);
        if (dc->Hc.hcca) {
            LOG("Purge HCCA: FrameNumber=0x%04X DoneHead=0x%08X",
                dc->Hc.hcca->FrameNumber, dc->Hc.hcca->DoneHead);
        }
    }
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
    /* Cleanup unlinks isoch EPs from dc->IsocEps and cancels queued URBs.
     * No-ops for Control/Bulk/Interrupt — safe to register universally. */
    epAttrs.EvtCleanupCallback = OhciPci_EpContextCleanup;

    UCXENDPOINT ucxEp;
    NTSTATUS status = UcxEndpointCreate(UcxUsbDevice,
                                        &UcxEndpointInit,
                                        &epAttrs,
                                        &ucxEp);
    LOG("UcxEndpointCreate (default) -> 0x%08X", status);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    /* Initialise per-EP context. dc comes from the UCXUSBDEVICE's
     * udc->Dc back-pointer (set in OhciPci_UsbDeviceAdd) — this is the
     * multi-instance-safe source of truth, NOT the legacy
     * g_DeviceContext module-static. */
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(ucxEp);
    RtlZeroMemory(ep, sizeof(*ep));
    OHCIPCI_USBDEV_CTX *udc_link = OhciPci_UsbDevContextGet(UcxUsbDevice);
    ep->Dc    = (udc_link != NULL) ? udc_link->Dc : NULL;
    ep->UcxEp = ucxEp;

    /* Link this EP into the owning device's EndpointList for device-level
     * callbacks (Enable/Disable/Reset) to fan out. */
    ep->Udc = udc_link;
    if (udc_link != NULL) {
        WdfSpinLockAcquire(udc_link->EndpointListLock);
        InsertTailList(&udc_link->EndpointList, &ep->DeviceEpEntry);
        WdfSpinLockRelease(udc_link->EndpointListLock);
    }

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
        /* USB CLEAR_FEATURE(ENDPOINT_HALT) / SET_INTERFACE resets the
         * data toggle to DATA0 on this endpoint. Mirror that in the ED:
         * clear both H (halted) and C (toggle carry) bits in HeadP. Must
         * use the K-pause dance (OHCI §6.4.4) — direct HeadP write while
         * the ED is on the schedule corrupts Hc*CurrentED. */
        OhciPci_EditHeadPSafely(dc, ed, OhciPci_HeadPClearHC, NULL);
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

    if (attrs == 0x00) {
        LOG("EndpointAdd: non-default control endpoints not supported");
        return STATUS_NOT_SUPPORTED;
    }
    if (attrs == 0x01) {
        /* Isochronous (Plan 8). v1: assume period 1 (one MPS-sized
         * packet/frame). Refuse if the device is low-speed (USB spec
         * forbids isoch on LS entirely). */
        OHCIPCI_USBDEV_CTX *udcCheck = OhciPci_UsbDevContextGet(UcxUsbDevice);
        if (udcCheck && udcCheck->Speed == UsbLowSpeed) {
            LOG("EndpointAdd: isoch rejected - low-speed devices have no isoch");
            return STATUS_NOT_SUPPORTED;
        }
    }

    /* Periodic-bandwidth pre-check for Isoc (attrs==0x01) and Interrupt
     * (attrs==0x03). Worst-case period-1 accounting; see device_context.h
     * PeriodicBytesPerFrame comment. Done BEFORE we touch any allocator
     * so a rejection leaks nothing. The matching charge happens at the
     * end of the per-kind branch on success and is refunded by
     * EpContextCleanup via ep->PeriodicBudgetCharged. */
    OHCIPCI_USBDEV_CTX *udc_pre = OhciPci_UsbDevContextGet(UcxUsbDevice);
    PDEVICE_CONTEXT dc_pre = (udc_pre != NULL) ? udc_pre->Dc : NULL;
    if (dc_pre == NULL) {
        LOG("EndpointAdd: NULL device context");
        return STATUS_INVALID_DEVICE_STATE;
    }
    if (attrs == 0x01 || attrs == 0x03) {
        if (dc_pre->PeriodicBytesPerFrame + mps > OHCIPCI_PERIODIC_BUDGET_BYTES) {
            LOG("EndpointAdd: %s rejected - periodic budget exceeded "
                "(%lu + %u > %u)",
                attrs == 0x01 ? "isoch" : "interrupt",
                dc_pre->PeriodicBytesPerFrame, mps,
                OHCIPCI_PERIODIC_BUDGET_BYTES);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
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
    /* Cleanup unlinks isoch EPs from dc->IsocEps and cancels queued URBs.
     * No-ops for Control/Bulk/Interrupt — safe to register universally. */
    epAttrs.EvtCleanupCallback = OhciPci_EpContextCleanup;
    UCXENDPOINT ucxEp;
    NTSTATUS status = UcxEndpointCreate(UcxUsbDevice, &EpInit, &epAttrs, &ucxEp);
    if (!NT_SUCCESS(status)) {
        LOG("UcxEndpointCreate (non-default) -> 0x%08X", status);
        return status;
    }
    OHCIPCI_EP_CONTEXT *ep = OhciPci_EpContextGet(ucxEp);
    RtlZeroMemory(ep, sizeof(*ep));
    ep->Dc    = dc_pre;
    ep->UcxEp = ucxEp;

    /* Link this EP into the owning device's EndpointList for device-level
     * callbacks (Enable/Disable/Reset) to fan out. */
    OHCIPCI_USBDEV_CTX *udc_link = udc_pre;
    ep->Udc = udc_link;
    if (udc_link != NULL) {
        WdfSpinLockAcquire(udc_link->EndpointListLock);
        InsertTailList(&udc_link->EndpointList, &ep->DeviceEpEntry);
        WdfSpinLockRelease(udc_link->EndpointListLock);
    }

    /* 3. Per-EP WDFQUEUE.
     *
     * Sequential dispatch for Control/Bulk/Interrupt — the existing WDF
     * DMA transaction lifecycle assumes one Execute outstanding per EP.
     *
     * Parallel for Isoch — the MDL-walk path
     * (OhciPci_IsocBuildAndSubmit_Locked) emits ITDs without going through
     * WdfDmaTransaction, so depth >= 2 doesn't trip the single-channel
     * constraint of WdfDmaProfilePacket that previously forced Sequential
     * dispatch. Pipelined submission keeps the ~10 ms isoch cadence. */
    WDF_IO_QUEUE_CONFIG qCfg;
    WDF_IO_QUEUE_CONFIG_INIT(&qCfg,
        (attrs == 0x01) ? WdfIoQueueDispatchParallel
                        : WdfIoQueueDispatchSequential);
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
    } else if (attrs == 0x03) { /* Interrupt */
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
        if (rc == 0) {
            /* Charge against the periodic-bandwidth budget. Refunded by
             * EpContextCleanup via ep->PeriodicBudgetCharged. */
            ep->Dc->PeriodicBytesPerFrame += mps;
            ep->PeriodicBudgetCharged = mps;
        }
        LOG("Interrupt EP created: addr=%u ep=%u dir=%s mps=%u "
            "bInterval=%u scheduled=%ums rc=%d budget=%lu/%u",
            funcAddr, epNum, isIn ? "IN" : "OUT", mps,
            bInterval, ep->Core.Interrupt.poll_interval_frames, rc,
            ep->Dc->PeriodicBytesPerFrame, OHCIPCI_PERIODIC_BUDGET_BYTES);
    } else { /* attrs == 0x01 - Isochronous */
        ep->Kind = OhciPciEpKindIsoc;
        /* Initialise the per-EP isoch lists BEFORE the create call.
         * Setting Kind=Isoc means EpContextCleanup may run IsocEpTeardown
         * if anything below this point fails (UCX rolls back via
         * WdfObjectDelete on a non-success NTSTATUS return). Teardown
         * walks IsocInFlightUrbs / IsocQueuedUrbs — those must be valid
         * empty lists, not RtlZeroMemory's NULL/NULL state, or
         * RemoveHeadList bugchecks dereferencing NULL Flink/Blink. */
        InitializeListHead(&ep->IsocQueuedUrbs);
        InitializeListHead(&ep->IsocInFlightUrbs);
        struct ohci_isoc_endpoint_config cfg;
        cfg.func_addr       = funcAddr;
        cfg.ep_num          = epNum;
        cfg.max_packet_size = mps;
        cfg.direction       = isIn ? OHCI_URB_DIR_IN : OHCI_URB_DIR_OUT;
        cfg.low_speed       = lowSpeed;
        WdfSpinLockAcquire(ep->Dc->CoreLock);
        rc = ohci_isoc_endpoint_create(&ep->Dc->Hc, &cfg, &ep->Core.Isoc);
        WdfSpinLockRelease(ep->Dc->CoreLock);
        if (rc == 0) {
            /* Charge against the periodic-bandwidth budget. Refunded by
             * EpContextCleanup via ep->PeriodicBudgetCharged. */
            ep->Dc->PeriodicBytesPerFrame += mps;
            ep->PeriodicBudgetCharged = mps;

            /* Plan 8 Task 7 — refill state. Silence buffer is one
             * zero-filled PAGE from the DMA region; alloc failure is
             * non-fatal (caller URBs still work, just no underrun
             * silence). IsocQueueLock is mandatory — HandleIsocUrb
             * relies on it. */
            /* IsocEpEntry.Flink stays NULL (RtlZeroMemory above) until
             * we splice onto dc->IsocEps below; the cleanup callback uses
             * "Flink == NULL" as the "not on any list" sentinel. */

            ep->IsocSilenceVa = ohci_dma_alloc(&ep->Dc->DmaRegion,
                                                PAGE_SIZE, PAGE_SIZE,
                                                &ep->IsocSilencePhys);
            if (ep->IsocSilenceVa) {
                RtlZeroMemory(ep->IsocSilenceVa, PAGE_SIZE);
            } else {
                LOG("Isoc EP: silence buffer alloc failed - underrun"
                    " protection disabled for this EP");
            }

            WDF_OBJECT_ATTRIBUTES qlAttrs;
            WDF_OBJECT_ATTRIBUTES_INIT(&qlAttrs);
            qlAttrs.ParentObject = ucxEp;
            NTSTATUS qlSt = WdfSpinLockCreate(&qlAttrs, &ep->IsocQueueLock);
            if (!NT_SUCCESS(qlSt)) {
                LOG("Isoc EP: WdfSpinLockCreate (IsocQueueLock) failed 0x%08X",
                    qlSt);
                /* Roll back the core EP. EpContextCleanup will refund
                 * PeriodicBudgetCharged when UCX deletes the WDFOBJECT
                 * on this non-success return. */
                WdfSpinLockAcquire(ep->Dc->CoreLock);
                ohci_isoc_endpoint_destroy(&ep->Dc->Hc, &ep->Core.Isoc);
                WdfSpinLockRelease(ep->Dc->CoreLock);
                return qlSt;
            }

            /* Splice onto dc->IsocEps so the refill walker sees us. */
            WdfSpinLockAcquire(ep->Dc->IsocEpsLock);
            InsertTailList(&ep->Dc->IsocEps, &ep->IsocEpEntry);
            WdfSpinLockRelease(ep->Dc->IsocEpsLock);

            /* Lazy-start the periodic backstop (idempotent — calling
             * WdfTimerStart on an already-running periodic timer is a
             * no-op per WDF docs). */
            if (ep->Dc->IsocRefillTimer) {
                WdfTimerStart(ep->Dc->IsocRefillTimer,
                              WDF_REL_TIMEOUT_IN_MS(OHCIPCI_ISOC_BACKSTOP_TIMER_MS));
            }
        }
        LOG("Isoc EP created: addr=%u ep=%u dir=%s mps=%u rc=%d budget=%lu/%u",
            funcAddr, epNum, isIn ? "IN" : "OUT", mps, rc,
            ep->Dc->PeriodicBytesPerFrame, OHCIPCI_PERIODIC_BUDGET_BYTES);
    }
    if (rc != 0) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    return STATUS_SUCCESS;
}
