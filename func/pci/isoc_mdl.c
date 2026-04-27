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

        ULONG absoluteOff = mdlByteOffset + pktOff;
        ULONG pageIdx     = absoluteOff >> PAGE_SHIFT;
        ULONG intraPage   = absoluteOff & (PAGE_SIZE - 1);

        if (pageIdx >= mdlPageCount) {
            LOG("IsocBuildAndSubmit: pkt %lu pageIdx=%lu >= mdlPageCount=%lu",
                i, pageIdx, mdlPageCount);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
        }

        /* OHCI is 32-bit phys. PFN >= 0x100000 means PA >= 4 GB. */
        if (pfns[pageIdx] >= 0x100000ull) {
            LOG("IsocBuildAndSubmit: pkt %lu pfn=%llu above 4 GB",
                i, (unsigned long long)pfns[pageIdx]);
            OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
            return STATUS_INVALID_PARAMETER;
        }

        /* Page-straddle check. If the packet crosses into pageIdx+1, the
         * two physical pages must themselves be contiguous (PFN+1) or the
         * packet can't fit in one ITD slot. The reference path falls into
         * the same "spans non-contiguous SG runs" failure here. */
        if (pktLen > 0 && intraPage + pktLen > PAGE_SIZE) {
            if (pageIdx + 1 >= mdlPageCount) {
                LOG("IsocBuildAndSubmit: pkt %lu straddles past MDL end", i);
                OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
                return STATUS_INVALID_PARAMETER;
            }
            if (pfns[pageIdx + 1] != pfns[pageIdx] + 1) {
                LOG("IsocBuildAndSubmit: pkt %lu spans non-contiguous PFNs "
                    "(pfn[%lu]=%llu pfn[%lu]=%llu)",
                    i, pageIdx, (unsigned long long)pfns[pageIdx],
                    pageIdx + 1, (unsigned long long)pfns[pageIdx + 1]);
                OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
                return STATUS_INVALID_PARAMETER;
            }
            if (pfns[pageIdx] + 1 >= 0x100000ull) {
                LOG("IsocBuildAndSubmit: pkt %lu straddle past 4 GB", i);
                OhciPci_IsocBuildFail_Locked(uc, STATUS_INVALID_PARAMETER);
                return STATUS_INVALID_PARAMETER;
            }
        }

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

/* T6 fills in OhciPci_IsocEpTeardown_Locked. */
VOID
OhciPci_IsocEpTeardown_Locked(_In_ POHCIPCI_EP_CONTEXT ep)
{
    UNREFERENCED_PARAMETER(ep);
}
