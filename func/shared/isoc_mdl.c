/*++

Module Name:

    isoc_mdl.c

Abstract:

    MDL-walk submitter for the OHCI isochronous OUT path. Walks each URB's
    MDL via MmGetMdlPfnArray, packs per-packet phys addresses into ITD
    windows, and links them into the EP's ED — bypassing the WDF DMA
    framework so that depth >= 2 URB pipelining doesn't trip the single-
    channel constraint of WdfDmaProfilePacket.

    When the URB's pages aren't physically contiguous, a per-URB bounce
    slab from OhciPci_BounceAlloc holds the contiguous copy.

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
#include "ohci_log.h"

/* OHCIPCI_TRANSFER_URB defined in device_context.h. */

/* --------------------------------------------------------------------------
 * Failure helper — stage the URB on dc->DeferredCompletions exactly once.
 * uc->OurMdl is left for OhciPci_UrbComplete / EpContextCleanup to reclaim.
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

    /* Cache the system VA once. The OUT bounce path below dereferences it
     * via RtlCopyMemory; that path NULL-checks before deref. Other readers
     * (diagnostic LOGs, future packet inspection) are also NULL-tolerant. */
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
        if (pfns[p] >= 0xFFFFFull) {
            /* PFN 0xFFFFF puts the last byte at exactly 4 GB and any window
             * arithmetic past it wraps uint32_t; treat it as out-of-range
             * the same as anything strictly above 4 GB. */
            fragmented = TRUE;
            break;
        }
        if (p > 0 && pfns[p] != pfns[p - 1] + 1) {
            fragmented = TRUE;
            break;
        }
    }

    /* Pre-validate per-packet offsets and lengths before allocating a
     * bounce slab. Bounces are scarce (64×4 KB pool); rejecting a malformed
     * URB on cheap arithmetic avoids burning a slab we'd just free in
     * BuildFail. */
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
        /* PSW size is 12 bits — single-slot cap. */
        if ((pktEnd - pktOff) > 0xFFFu) {
            LOG("IsocBuildAndSubmit: pkt %lu len=%lu exceeds 0xFFF",
                i, pktEnd - pktOff);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
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
         * OhciPci_IsocOnUrbRetire_Locked copies bounce -> user MDL on
         * retire. Both directions live behind the bounce path because
         * usbaudio.sys URB MDLs always fragment across non-contiguous
         * PFNs (see feedback_usbaudio_urb_pfn_fragmentation). */
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

    /* Fill phys/len. Offset/length validation already happened in the
     * pre-scan above, so this loop assumes well-formed inputs. */
    for (ULONG i = 0; i < nPkts; i++) {
        ULONG pktOff = turb->u.Isoch.IsoPacket[i].Offset;
        ULONG pktEnd = (i + 1 < nPkts)
                        ? turb->u.Isoch.IsoPacket[i+1].Offset
                        : urbLen;
        ULONG pktLen = pktEnd - pktOff;

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
                /* Zero-length packet: consumes a window slot but contributes
                 * no bytes; expectNext also stays put for contiguity. */
                expectNext = nxtPhys;
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
        /* Partial submit: trim isoc_pkt_count down to what actually got
         * linked into the ED. Otherwise the drain code waits forever for
         * ITDs that were never emitted -> URB never retires, never unlinks
         * from IsocInFlightUrbs, bounce slab leaks, audio stream stalls. */
        LOG("IsocBuildAndSubmit: partial submit (%lu of %lu packets queued)",
            i, nPkts);
        uc->CoreUrb.isoc_pkt_count = (uint8_t)i;
    }

    /* Track in-flight so retire/cancel/teardown can find this URB.
     * WdfRequestMarkCancelable intentionally not called — see the
     * cancel-callback comment below for why. */
    InsertTailList(&ep->IsocInFlightUrbs, &uc->InFlightEntry);

    return STATUS_SUCCESS;
}

/* --------------------------------------------------------------------------
 * OhciPci_IsocOnUrbRetire_Locked
 *
 * Called from OhciPci_UrbComplete's isoch branch once per retiring URB.
 * Caller already holds CoreLock (UrbComplete fires from the retire DPC).
 *
 * Safe to call on URBs that never went through BuildAndSubmit — e.g. URBs
 * whose InFlightEntry is still zero-initialised. The Flink==NULL guard
 * skips unlinking a list entry that was never inserted.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocOnUrbRetire_Locked(_In_ OHCIPCI_URB_CTX *uc)
{
    if (uc->InFlightEntry.Flink != NULL) {
        RemoveEntryList(&uc->InFlightEntry);
        uc->InFlightEntry.Flink = NULL;
        uc->InFlightEntry.Blink = NULL;
    }

    /* IN copy-back. usbaudio.sys URB MDLs come from paged-pool and routinely
     * fragment across non-adjacent physical pages, so BuildAndSubmit takes
     * the bounce path on every microphone URB. The HC wrote captured audio
     * into the bounce slab; copy it back to the user MDL before freeing
     * the bounce — without this the user buffer stays whatever it was at
     * submit time (zero on a fresh allocation) and the input stream is
     * silent.
     *
     * Per-packet IsoPacket[i].Length writeback already happened in
     * OhciPci_UrbComplete above (PSW.Size is well-defined for IN per
     * OHCI §4.3.2.4). We just need the bytes to land in user memory. */
    if (uc->IsocBounceVa &&
        uc->DataDirection == OHCI_URB_DIR_IN &&
        uc->UserMdl != NULL &&
        uc->DataLength > 0)
    {
        if (uc->MdlSysVa == NULL) {
            uc->MdlSysVa = MmGetSystemAddressForMdlSafe(uc->UserMdl,
                                                        NormalPagePriority);
        }
        if (uc->MdlSysVa != NULL) {
            RtlCopyMemory(uc->MdlSysVa, uc->IsocBounceVa, uc->DataLength);
        } else {
            /* Low-resources MDL map failed. Drop the captured audio rather
             * than landing it at a garbage VA. usbaudio sees a zero-length
             * frame which it tolerates as a transient glitch — preferable
             * to a wild kernel write or memory corruption. */
            LOG("IsocOnUrbRetire: IN copy-back skipped — MDL map failed");
        }
    }

    /* Free the page-straddle bounce slab if BuildAndSubmit allocated one.
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

/* --------------------------------------------------------------------------
 * Cancel callback — intentionally a no-op.
 *
 * OhciPci_IsocBuildAndSubmit_Locked deliberately does NOT call
 * WdfRequestMarkCancelable, so WDF never invokes this callback. Per-URB
 * cancellation isn't required for the usbaudio stop/restart cycle:
 * usbaudio.sys "stops" by issuing SET_INTERFACE(alt=0), which triggers
 * UcxEndpointPurge -> WDF queue purge (for URBs not yet handed to
 * BuildAndSubmit) plus OhciPci_IsocEpTeardown (for URBs with
 * ITDs already linked into the ED). Those two paths cover stop/restart.
 *
 * If interruptible per-URB cancel is needed later, fill in here: pause
 * via OhciPci_EditHeadPSafely, wait one SOF, unlink the URB's ITDs,
 * complete USBD_STATUS_CANCELED, then clear Skip if more work remains.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocCancelEmitted(_In_ WDFREQUEST Request)
{
    UNREFERENCED_PARAMETER(Request);
}

/* --------------------------------------------------------------------------
 * EP teardown — drains URBs whose ITDs were linked into the ED via
 * BuildAndSubmit. Runs at PASSIVE from OhciPci_EpContextCleanup, AFTER
 * the EP has been unlinked from dc->IsocEps so the refill walker no
 * longer touches us. The refill DPC is therefore not racing the
 * IsocQueuedUrbs drain (the caller already handles that); we only need
 * to additionally drain IsocInFlightUrbs.
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
 * Callers must NOT hold CoreLock — this function acquires it itself.
 * -------------------------------------------------------------------------- */
VOID
OhciPci_IsocEpTeardown(_In_ POHCIPCI_EP_CONTEXT ep)
{
    PDEVICE_CONTEXT dc = ep->Dc;
    if (dc == NULL || dc->CoreLock == NULL) return;

    /* Skip teardown if IsocInFlightUrbs was never initialised — happens
     * when EP create set Kind=Isoc but ohci_isoc_endpoint_create (or any
     * later step) failed before the InitializeListHead block ran. UCX
     * then rolls back via WdfObjectDelete and EpContextCleanup fires on
     * an EP whose isoch lists are still RtlZeroMemory's NULL/NULL state.
     * IsListEmpty/RemoveHeadList on that bugchecks. EndpointAdd now
     * pre-initialises the lists, but keep this guard as a backstop. */
    if (ep->IsocInFlightUrbs.Flink == NULL) return;

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
