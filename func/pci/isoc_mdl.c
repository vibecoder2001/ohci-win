/*++

Module Name:

    isoc_mdl.c

Abstract:

    Spike (MDL-walk) replacement for the WdfDmaTransaction-based isochronous
    OUT path. Task 3 of the OHCI isoch MDL-walk spike — implements only
    OhciPci_IsocBuildAndSubmit_Locked; the other three functions in
    isoc_mdl.h get bodies in T4 (retire), T5 (cancel), and T6 (teardown).

    NOT YET WIRED INTO ANY CALLER. Build only.

Environment:

    Kernel mode only. All "_Locked" entry points run with dc->CoreLock held.

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

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
                                  "OhciPci: " fmt "\n", ##__VA_ARGS__)

/* Mirrors the typedef in ucx_endpoint.c. UCX puts a pointer to this layout
 * in Parameters.Others.Arg1 of every URB request enqueued to a per-EP
 * WDFQUEUE; the contract is UCX-private so we duplicate the struct here. */
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
 * Failure helper — stage the URB on dc->DeferredCompletions exactly once.
 * Mirrors the failure half of OhciPci_IsocProgramDmaFail in ucx_endpoint.c
 * but does NOT touch uc->DmaTransaction (the spike leaves it allocated but
 * unused). uc->OurMdl is left for HandleIsocUrb cleanup, same as the
 * reference's success path.
 * -------------------------------------------------------------------------- */
static VOID
OhciPci_IsocBuildFail_Locked(OHCIPCI_URB_CTX *uc, NTSTATUS status)
{
    /* Failure path completes via DeferredCompletions, which bypasses
     * OhciPci_UrbComplete — so this is the only place a bounce allocated
     * by an earlier line in BuildAndSubmit can be freed before the URB
     * is finalised. Safe to call on URBs that never allocated one
     * (IsocBounceVa stays NULL). */
    if (uc->IsocBounceVa) {
        OhciPci_BounceFree(uc->EpCtx->Dc, uc->IsocBounceVa);
        uc->IsocBounceVa   = NULL;
        uc->IsocBouncePhys = 0;
    }
    uc->DeferredStatus = status;
    uc->DeferredInfo   = 0;
    InsertTailList(&uc->EpCtx->Dc->DeferredCompletions, &uc->DeferredEntry);
}

NTSTATUS
OhciPci_IsocBuildAndSubmit_Locked(
    _In_ POHCIPCI_EP_CONTEXT ep,
    _In_ OHCIPCI_URB_CTX    *uc)
{
    PDEVICE_CONTEXT       dc   = ep->Dc;
    POHCIPCI_TRANSFER_URB turb = (POHCIPCI_TRANSFER_URB)uc->TransferUrb;

    if (turb == NULL) {
        LOG("IsocBuildAndSubmit: NULL TransferUrb");
        OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
        return STATUS_INVALID_PARAMETER;
    }

    ULONG nPkts = turb->u.Isoch.NumberOfPackets;
    if (nPkts == 0) {
        LOG("IsocBuildAndSubmit: zero packets");
        OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
        return STATUS_INVALID_PARAMETER;
    }
    if (nPkts > OHCI_URB_MAX_ISOC_PACKETS) {
        LOG("IsocBuildAndSubmit: %lu packets exceeds cap %u",
            nPkts, OHCI_URB_MAX_ISOC_PACKETS);
        OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
        return STATUS_INVALID_PARAMETER;
    }

    PMDL mdl = uc->UserMdl;
    if (mdl == NULL) {
        LOG("IsocBuildAndSubmit: NULL UserMdl (HandleIsocUrb invariant)");
        OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
        return STATUS_INVALID_PARAMETER;
    }

    /* Cache user VA for diagnostics / future packet inspection. NULL is
     * fine — only used by the optional LOG, never dereferenced blindly. */
    if (uc->MdlSysVa == NULL) {
        uc->MdlSysVa = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority);
    }

    ULONG       mdlByteOffset = MmGetMdlByteOffset(mdl);
    PPFN_NUMBER pfns          = MmGetMdlPfnArray(mdl);
    ULONG       mdlByteCount  = MmGetMdlByteCount(mdl);
    ULONG       urbLen        = turb->TransferBufferLength;
    ULONG       mdlPageCount  = ADDRESS_AND_SIZE_TO_SPAN_PAGES(
                                    (PVOID)(ULONG_PTR)mdlByteOffset,
                                    mdlByteCount);

    /* Detect non-contiguous PFNs once. usbaudio.sys URB buffers come from
     * paged-pool and routinely fragment across non-adjacent physical pages
     * (observed: pfn[0]=372319 pfn[1]=384391, delta +12072). When that
     * happens we can't emit ITDs against the user pages directly — bounce
     * the URB through OHCIPCI_BOUNCE_SLAB_BYTES of contiguous DMA memory.
     *
     * Also flag if the URB extends above 4 GB: OHCI is 32-bit phys, the
     * bounce slab base lives below 4 GB so bouncing also resolves >4 GB. */
    BOOLEAN fragmented = FALSE;
    for (ULONG p = 0; p < mdlPageCount; p++) {
        if (pfns[p] >= 0x100000ull) {
            fragmented = TRUE;   /* >4 GB — bounce regardless */
            break;
        }
        if (p > 0 && pfns[p] != pfns[p - 1] + 1) {
            fragmented = TRUE;
            break;
        }
    }

    uint32_t bouncePhys = 0;
    if (fragmented) {
        if (urbLen > OHCIPCI_BOUNCE_SLAB_BYTES) {
            LOG("IsocBuildAndSubmit: fragmented URB len=%lu exceeds bounce "
                "slab %u", urbLen, OHCIPCI_BOUNCE_SLAB_BYTES);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INSUFFICIENT_RESOURCES);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
        void *bounceVa = OhciPci_BounceAlloc(dc, &bouncePhys);
        if (bounceVa == NULL) {
            LOG("IsocBuildAndSubmit: bounce pool exhausted");
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INSUFFICIENT_RESOURCES);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
        /* OUT: copy URB bytes into bounce now. IN: leave bounce uninitialised;
         * UrbComplete copies bounce -> MDL on retire. (Spike is OUT-only —
         * IN copy-back left as a stub, see retire path.) */
        if (uc->DataDirection == OHCI_URB_DIR_OUT) {
            if (uc->MdlSysVa == NULL) {
                uc->MdlSysVa = MmGetSystemAddressForMdlSafe(mdl,
                                                            NormalPagePriority);
            }
            if (uc->MdlSysVa == NULL) {
                OhciPci_BounceFree(dc, bounceVa);
                LOG("IsocBuildAndSubmit: MmGetSystemAddressForMdlSafe failed");
                OhciPci_IsocBuildFail_Locked(uc, STATUS_INSUFFICIENT_RESOURCES);
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            RtlCopyMemory(bounceVa, uc->MdlSysVa, urbLen);
        }
        uc->IsocBounceVa   = bounceVa;
        uc->IsocBouncePhys = bouncePhys;
    }

    /* Per-packet (phys, len) table — identical layout to the reference. */
    typedef struct { uint32_t phys; uint16_t len; } isoc_pkt_t;
    isoc_pkt_t pkt[OHCI_URB_MAX_ISOC_PACKETS];

    for (ULONG i = 0; i < nPkts; i++) {
        ULONG pktOff = turb->u.Isoch.IsoPacket[i].Offset;
        ULONG pktEnd = (i + 1 < nPkts)
                        ? turb->u.Isoch.IsoPacket[i+1].Offset
                        : urbLen;
        if (pktEnd < pktOff || pktEnd > urbLen) {
            LOG("IsocBuildAndSubmit: pkt %lu off/end out of range "
                "(off=%lu end=%lu urbLen=%lu)", i, pktOff, pktEnd, urbLen);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
        }
        ULONG pktLen = pktEnd - pktOff;
        /* PSW size is 12 bits — single-slot cap. */
        if (pktLen > 0xFFFu) {
            LOG("IsocBuildAndSubmit: pkt %lu len=%lu exceeds 0xFFF", i, pktLen);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
        }

        if (fragmented) {
            /* Bounce path: per-packet phys is just bounce_phys + pkt_off.
             * Bounce is contiguous so no straddle check needed. */
            pkt[i].phys = bouncePhys + pktOff;
            pkt[i].len  = (uint16_t)pktLen;
            continue;
        }

        ULONG absoluteOff = mdlByteOffset + pktOff;
        ULONG pageIdx     = absoluteOff >> PAGE_SHIFT;
        ULONG intraPage   = absoluteOff & (PAGE_SIZE - 1);

        if (pageIdx >= mdlPageCount) {
            LOG("IsocBuildAndSubmit: pkt %lu pageIdx=%lu >= mdlPageCount=%lu",
                i, pageIdx, mdlPageCount);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
        }
        /* Contiguous-PFN path: PFN < 0x100000 and contiguity already
         * verified above, so neither single-page packets nor 2-page
         * straddles need re-checking. */
        pkt[i].phys = (uint32_t)((pfns[pageIdx] << PAGE_SHIFT) | intraPage);
        pkt[i].len  = (uint16_t)pktLen;
    }

    /* CoreUrb is shared across all windows; populate before first submit. */
    uc->CoreUrb.complete  = OhciPci_UrbComplete;
    uc->CoreUrb.direction = uc->DataDirection;

    /* Caller holds CoreLock already — no additional lock taken. */
    struct ohci_isoc_endpoint *ie = &ep->Core.Isoc;
    uint16_t sf;
    uint16_t fmNumber = (uint16_t)(dc->MmioOps.read32(dc->MmioOps.context, 0x3C) & 0xFFFFu);
    if (ie->primed) {
        SHORT lead = (SHORT)(ie->ed_tail_frame - fmNumber);
        if (lead > 0) {
            sf = ie->ed_tail_frame;
        } else {
            /* Underrun snap-forward — see the reference for rationale. */
            sf = (uint16_t)((fmNumber + OHCIPCI_ISOC_PRIME_LOOKAHEAD) & 0xFFFFu);
        }
    } else {
        sf = (uint16_t)((fmNumber + OHCIPCI_ISOC_PRIME_LOOKAHEAD) & 0xFFFFu);
    }

    /* Greedy-pack into windows of <=8 packets sharing one BP0/BP0+0x1000
     * page window AND physically contiguous. Verbatim from the reference. */
    ULONG i  = 0;
    int   rc = 0;
    int   firstWindow = 1;
    while (i < nPkts && rc == 0) {
        uint8_t  windowCount = 1;
        uint32_t bp0     = pkt[i].phys & 0xFFFFF000u;
        uint32_t winPhys = pkt[i].phys;
        uint32_t winLen  = pkt[i].len;
        uint32_t expectNext = pkt[i].phys + pkt[i].len;
        while (windowCount < 8 && i + windowCount < nPkts) {
            uint32_t nxtPhys = pkt[i + windowCount].phys;
            if (nxtPhys != expectNext) break;
            uint32_t nxtLen = pkt[i + windowCount].len;
            if (nxtLen == 0) {
                winLen     += 0;
                expectNext  = nxtPhys;
                windowCount++;
                continue;
            }
            uint32_t nxtEnd  = nxtPhys + nxtLen - 1;
            uint32_t nxtPage = nxtEnd & 0xFFFFF000u;
            if (nxtPage != bp0 && nxtPage != bp0 + 0x1000u) break;
            winLen     += nxtLen;
            expectNext  = nxtPhys + nxtLen;
            windowCount++;
        }

        uint16_t lens[8];
        for (uint8_t k = 0; k < windowCount; k++) lens[k] = pkt[i + k].len;

        rc = ohci_isoc_submit_window(&dc->Hc, ie, &uc->CoreUrb,
                                      sf, windowCount, lens,
                                      winPhys, winLen, firstWindow);
        if (firstWindow && rc == 0) {
            uc->CoreUrb.isoc_pkt_count = (uint8_t)nPkts;
        }
        sf  = (uint16_t)(sf + windowCount);
        i  += windowCount;
        firstWindow = 0;
    }

    if (rc != 0) {
        LOG("IsocBuildAndSubmit: ohci_isoc_submit_window rc=%d at packet %lu/%lu",
            rc, i, nPkts);
        if (i == 0 || firstWindow) {
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INSUFFICIENT_RESOURCES);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
        LOG("IsocBuildAndSubmit: partial submit (%lu of %lu packets queued)",
            i, nPkts);
    }

    /* Track in-flight so retire/cancel/teardown can find this URB. T5
     * (cancel) wires WdfRequestMarkCancelable; T3 leaves it off. */
    InsertTailList(&ep->IsocInFlightUrbs, &uc->InFlightEntry);

    return STATUS_SUCCESS;
}

/* --------------------------------------------------------------------------
 * OhciPci_IsocOnUrbRetire_Locked
 *
 * Called from OhciPci_UrbComplete's isoch branch once per retiring URB.
 * Caller already holds CoreLock (UrbComplete fires from the retire DPC).
 *
 * Safe to call on URBs that never went through BuildAndSubmit — e.g. legacy
 * path URBs allocated before T7 wires the spike. The Flink==NULL guard
 * skips unlinking a list entry that was zero-initialised but never inserted.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocOnUrbRetire_Locked(_In_ OHCIPCI_URB_CTX *uc)
{
    if (uc->InFlightEntry.Flink != NULL) {
        RemoveEntryList(&uc->InFlightEntry);
        uc->InFlightEntry.Flink = NULL;
        uc->InFlightEntry.Blink = NULL;
    }

    /* Free the page-straddle bounce slab if BuildAndSubmit allocated one.
     * IN copy-back is left as a TODO — spike is OUT-only (audio sink).
     * Setting Va=NULL after free is required because UrbComplete and the
     * teardown helper both consult IsocBounceVa as the "owns a bounce"
     * sentinel, and the URB context can outlive the slab in the bounce
     * pool's free bitmap. */
    if (uc->IsocBounceVa) {
        OHCIPCI_EP_CONTEXT *ep = uc->EpCtx;
        if (ep && ep->Dc) {
            OhciPci_BounceFree(ep->Dc, uc->IsocBounceVa);
        }
        uc->IsocBounceVa   = NULL;
        uc->IsocBouncePhys = 0;
    }
}

VOID
OhciPci_IsocRetireEmitted_Locked(_In_ POHCIPCI_EP_CONTEXT ep)
{
    /* No-op: per-URB retire is handled by OhciPci_IsocOnUrbRetire_Locked,
     * called from OhciPci_UrbComplete's isoch branch. This entry point is
     * preserved for symmetry with the header. */
    UNREFERENCED_PARAMETER(ep);
}

/* --------------------------------------------------------------------------
 * Cancel callback — intentionally a no-op for the spike.
 *
 * OhciPci_IsocBuildAndSubmit_Locked deliberately does NOT call
 * WdfRequestMarkCancelable, so WDF never invokes this callback. Per-URB
 * cancellation isn't required by the spike GO criterion (start->stop->start
 * of a usbaudio stream): usbaudio.sys "stops" by issuing SET_INTERFACE(alt=0),
 * which triggers UcxEndpointPurge -> WDF queue purge (for URBs not yet
 * handed to BuildAndSubmit) plus OhciPci_IsocEpTeardown_Locked (T6, for URBs
 * with ITDs already linked into the ED). Those two paths cover stop/restart.
 *
 * If a future plan needs interruptible per-URB cancel, fill in here:
 * pause via OhciPci_EditHeadPSafely, wait one SOF, unlink the URB's ITDs,
 * complete USBD_STATUS_CANCELED, then clear Skip if more work remains.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocCancelEmitted(_In_ WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(Request);
}

/* --------------------------------------------------------------------------
 * EP teardown — drains URBs the spike is tracking. Runs at PASSIVE from
 * OhciPci_EpContextCleanup, AFTER the EP has been unlinked from
 * dc->IsocEps so the refill walker no longer touches us. The refill DPC
 * is therefore not racing the IsocQueuedUrbs drain (caller already covers
 * that); we only need to additionally drain IsocInFlightUrbs (URBs whose
 * ITDs were linked into the ED via BuildAndSubmit).
 *
 * By the time this fires, the OHCI core's destroy_endpoint should already
 * have walked the ED and fired each URB's `complete` callback, draining
 * IsocInFlightUrbs through OhciPci_IsocOnUrbRetire_Locked. The list-empty
 * fast-path is the expected case; the explicit drain here is a safety net
 * for partial/aborted teardowns.
 *
 * CoreLock is taken briefly because UrbComplete (and BuildAndSubmit) hold
 * it when mutating IsocInFlightUrbs. Splice into a local list under the
 * lock, then complete outside the lock — same shape as the IsocQueuedUrbs
 * drain in OhciPci_EpContextCleanup.
 *
 * Naming note: this function is named *_Locked for symmetry with the rest
 * of the spike's API but actually acquires CoreLock itself, since it runs
 * at PASSIVE from cleanup. Callers must NOT hold CoreLock.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocEpTeardown_Locked(_In_ POHCIPCI_EP_CONTEXT ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    if (dc == NULL || dc->CoreLock == NULL) return;

    LIST_ENTRY local;
    InitializeListHead(&local);

    WdfSpinLockAcquire(dc->CoreLock);
    while (!IsListEmpty(&ep->IsocInFlightUrbs)) {
        PLIST_ENTRY le = RemoveHeadList(&ep->IsocInFlightUrbs);
        OHCIPCI_URB_CTX *uc =
            CONTAINING_RECORD(le, OHCIPCI_URB_CTX, InFlightEntry);
        uc->InFlightEntry.Flink = NULL;
        uc->InFlightEntry.Blink = NULL;
        InsertTailList(&local, le);
    }
    WdfSpinLockRelease(dc->CoreLock);

    while (!IsListEmpty(&local)) {
        PLIST_ENTRY le = RemoveHeadList(&local);
        OHCIPCI_URB_CTX *uc =
            CONTAINING_RECORD(le, OHCIPCI_URB_CTX, InFlightEntry);
        if (uc->IsocBounceVa) {
            OhciPci_BounceFree(dc, uc->IsocBounceVa);
            uc->IsocBounceVa   = NULL;
            uc->IsocBouncePhys = 0;
        }
        if (uc->OurMdl) { IoFreeMdl(uc->OurMdl); uc->OurMdl = NULL; }
        WdfRequestComplete(uc->Request, STATUS_CANCELLED);
    }
}
