#include <string.h>
#include "ohci_drain.h"
#include "ohci_regs.h"
#include "ohci_td.h"
#include "ohci_itd.h"
#include "ohci_isoc.h"
#include "ohci_urb.h"
#include "ohci_dma.h"
#include "ohci_pool.h"
#include "ohci_ed.h"
#include "ohci_hc.h"

static int cc_to_urb_status(uint8_t cc) {
    switch (cc) {
    case OHCI_CC_NOERROR:             return OHCI_URB_STATUS_OK;
    case OHCI_CC_STALL:               return OHCI_URB_STATUS_STALL;
    case OHCI_CC_CRC:                 return OHCI_URB_STATUS_CRC;
    case OHCI_CC_DEVICENOTRESPONDING: return OHCI_URB_STATUS_TIMEOUT;
    case OHCI_CC_DATAOVERRUN:
    case OHCI_CC_BUFFEROVERRUN:       return OHCI_URB_STATUS_OVERRUN;
    case OHCI_CC_DATAUNDERRUN:
    case OHCI_CC_BUFFERUNDERRUN:      return OHCI_URB_STATUS_UNDERRUN;
    default:                          return OHCI_URB_STATUS_OTHER;
    }
}

/* Decode per-packet PSW for an isoch URB whose ITD just retired.
 * OHCI §4.3.2.4: each PSW out word holds CC[15:12] | size[11:0]. Sum
 * sizes into urb->transferred; treat DataUnderrun as a normal short
 * packet (URB stays OK). Any other non-NoError code -> URB-level
 * OVERRUN. */
static void decode_isoc_itd(struct ohci_urb *du, struct ohci_itd *itd) {
    uint8_t fc = (uint8_t)((itd->Control & OHCI_ITD_FC_MASK) >> OHCI_ITD_FC_SHIFT);
    uint8_t pkt_count = (uint8_t)(fc + 1);

    /* Per-window pkt base: each ITD covers a window of packets within
     * the URB. isoc_pkts_filled is the running index of packets that
     * have already been decoded from prior ITDs; this ITD's results
     * land at [base .. base+pkt_count). Cap defensively against HW
     * writing FC out of expected range. */
    uint8_t base = du->isoc_pkts_filled;
    if (base >= du->isoc_pkt_count) {
        /* All packets already accounted for — nothing more to decode. */
        return;
    }
    if ((uint16_t)base + pkt_count > du->isoc_pkt_count) {
        pkt_count = (uint8_t)(du->isoc_pkt_count - base);
    }

    uint32_t total = 0;
    int hard_err = 0;
    for (uint8_t i = 0; i < pkt_count; i++) {
        uint16_t psw = itd->PSW[i];
        uint8_t  cc  = (uint8_t)((psw & OHCI_PSW_CC_MASK) >> OHCI_PSW_CC_SHIFT);
        uint16_t len = (uint16_t)(psw & OHCI_PSW_SIZE_MASK);
        du->isoc_pkts[base + i].cc     = cc;
        du->isoc_pkts[base + i].length = len;
        total += len;
        if (cc != OHCI_CC_NOERROR && cc != OHCI_CC_DATAUNDERRUN) hard_err = 1;
    }
    du->isoc_pkts_filled = (uint8_t)(base + pkt_count);
    du->transferred += total;
    if (du->status == OHCI_URB_STATUS_PENDING && hard_err) {
        du->status = OHCI_URB_STATUS_OVERRUN;
    }
    /* OK status: only flip when ALL packets have been filled. The URB-
     * completion match (urb_for_td_phys via tail_td) fires off the
     * LAST ITD's retirement, at which point isoc_pkts_filled ==
     * isoc_pkt_count for a healthy URB. */
    if (du->status == OHCI_URB_STATUS_PENDING &&
        du->isoc_pkts_filled == du->isoc_pkt_count) {
        du->status = OHCI_URB_STATUS_OK;
    }
}

/* True iff `phys` falls inside the ITD pool's slot range. The drain uses
 * this to free retired descriptors back to the right pool — silence ITDs
 * (Plan 8 Task 7) have no owning URB, so the urb-keyed dispatch in
 * earlier tasks would route them to td_pool and corrupt both pools. */
static int td_phys_is_itd(struct ohci_hc *hc, uint32_t phys) {
    if (hc->itd_pool.elems == NULL) return 0;
    uint32_t start = hc->itd_pool.elems_phys;
    uint32_t end   = start + (uint32_t)hc->itd_pool.capacity * (uint32_t)sizeof(struct ohci_itd);
    return (phys >= start && phys < end);
}

/* Match retired TDs against URBs by their tail TD phys — that TD is
 * unique per live URB and its slot stays allocated until we free it. */
static struct ohci_urb *urb_for_td_phys(struct ohci_hc *hc, uint32_t td_phys,
                                         int remove) {
    struct ohci_urb *prev = NULL, *u = hc->in_flight;
    while (u) {
        if (u->tail_td) {
            uint32_t tail_phys = hc->dma->phys_base +
                (uint32_t)((uint8_t*)u->tail_td - hc->dma->base);
            if (tail_phys == td_phys) {
                if (remove) {
                    if (prev) prev->next_pending = u->next_pending;
                    else      hc->in_flight = u->next_pending;
                }
                return u;
            }
        }
        prev = u;
        u = u->next_pending;
    }
    return NULL;
}

/* Find the URB and TD-record-index for a retired data TD at td_phys. */
static struct ohci_urb *urb_for_data_td_phys(struct ohci_hc *hc,
                                              uint32_t td_phys,
                                              int *out_idx) {
    for (struct ohci_urb *u = hc->in_flight; u != NULL; u = u->next_pending) {
        for (int i = 0; i < u->data_td_count; i++) {
            if (u->data_tds[i].td_phys == td_phys) {
                if (out_idx) *out_idx = i;
                return u;
            }
        }
    }
    return NULL;
}

void ohci_drain_done(struct ohci_hc *hc) {
    uint32_t istat = hc->ops.read32(hc->ops.context, 0x0C);
    if (!(istat & OHCI_INT_WDH)) return;

    /* OHCI §4.4.4: HC writes the chain pointer with bit 0 set when at
     * least one queued TD's DelayInterrupt has expired. §5.2.5.4 says
     * the HCD must ignore bits 3:0. Mask here so virt_from_phys gets a
     * 16-byte-aligned pointer regardless of which HC implementation we
     * land on (some clones never set bit 0; spec-compliant ones do). */
    uint32_t done_head = hc->hcca->DoneHead & ~0xFu;
    hc->hcca->DoneHead = 0;
    hc->ops.write32(hc->ops.context, 0x0C, OHCI_INT_WDH);

    /* Reverse LIFO → FIFO. */
    uint32_t reversed = 0;
    uint32_t cur = done_head;
    while (cur) {
        struct ohci_td *td = ohci_dma_virt_from_phys(hc->dma, cur);
        if (!td) break;
        uint32_t next = td->NextTD;
        td->NextTD = reversed;
        reversed = cur;
        cur = next;
    }

    cur = reversed;
    while (cur) {
        struct ohci_td *td = ohci_dma_virt_from_phys(hc->dma, cur);
        if (!td) break;
        uint32_t next = td->NextTD;

        uint8_t cc = (td->Control >> OHCI_TD_CC_SHIFT) & 0xF;
        int s = cc_to_urb_status(cc);

        /* If this TD belongs to a URB's data_tds[] array, accumulate its
         * bytes-transferred onto urb->transferred. OHCI §4.3.1.4: CBP=0
         * when the chunk fully transferred; otherwise CBP points at the
         * next un-transferred byte (within the chunk's range). Multi-TD
         * URBs (Bulk SG) get summed; single-TD URBs match once. */
        int td_idx = -1;
        struct ohci_urb *du = urb_for_data_td_phys(hc, cur, &td_idx);
        if (du && td_idx >= 0) {
            if (du->is_isoc) {
                decode_isoc_itd(du, (struct ohci_itd *)td);
            } else {
                uint32_t chunk_off = du->data_tds[td_idx].chunk_off;
                uint32_t chunk_len = du->data_tds[td_idx].chunk_len;
                uint32_t per_td_done;
                if (td->CBP == 0) {
                    per_td_done = chunk_len;
                } else if (td->CBP >= du->buffer_phys + chunk_off) {
                    per_td_done = td->CBP - (du->buffer_phys + chunk_off);
                } else {
                    per_td_done = 0;
                }
                du->transferred += per_td_done;
                if (cc != OHCI_CC_NOERROR && du->status == OHCI_URB_STATUS_PENDING) {
                    du->status = s;
                }
            }
        }

        struct ohci_urb *u = urb_for_td_phys(hc, cur, /*remove=*/1);
        if (u) {
            if (u->status == OHCI_URB_STATUS_PENDING) u->status = s;
            if (u->complete) u->complete(u);
        } else if (du && !du->is_isoc && cc != OHCI_CC_NOERROR) {
            /* OHCI §6.4.4: when a non-isoc TD reports a hard error, HC
             * halts the ED at this TD and queues only THIS TD to DoneHead;
             * subsequent TDs in the chain (e.g. STATUS in a SETUP/DATA/
             * STATUS control transfer) never retire. Without this branch
             * the URB sits in hc->in_flight forever — UCX never sees the
             * STALL and enumeration freezes. Symptom: Logitech mouse
             * STALLing DEVICE_QUALIFIER (0x06) hangs the bus.
             *
             * Complete the URB on the failing data TD and remove it from
             * in_flight. Surviving TDs (the unretired tail and any later
             * data TDs) leak their pool slots; the next ohci_control_submit
             * reuses the placeholder slot via the K-toggle bracket and
             * the orphans get reclaimed when the ED itself is destroyed. */
            struct ohci_urb *prev = NULL;
            for (struct ohci_urb *iter = hc->in_flight; iter; iter = iter->next_pending) {
                if (iter == du) {
                    if (prev) prev->next_pending = du->next_pending;
                    else      hc->in_flight       = du->next_pending;
                    break;
                }
                prev = iter;
            }
            if (du->complete) du->complete(du);
        }

        /* Pool dispatch keys on physical address range, not URB ownership,
         * so orphan silence ITDs (no URB) free correctly back to itd_pool.
         * decode_isoc_itd above is gated on (du && du->is_isoc) which
         * silently skips orphans — they have no URB to write into. */
        if (td_phys_is_itd(hc, cur)) {
            ohci_itd_pool_free(&hc->itd_pool, (struct ohci_itd *)td);
        } else {
            ohci_td_pool_free(&hc->td_pool, td);
        }
        cur = next;
    }
}

void ohci_urb_cancel_for_ed(struct ohci_hc *hc, struct ohci_ed *ed) {
    if (!ed) return;
    ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);

    struct ohci_urb *prev = NULL;
    struct ohci_urb *u    = hc->in_flight;
    while (u) {
        struct ohci_urb *next = u->next_pending;
        if (u->ed == ed) {
            if (prev) prev->next_pending = next;
            else      hc->in_flight       = next;
            if (u->status == OHCI_URB_STATUS_PENDING)
                u->status = OHCI_URB_STATUS_OTHER;
            if (u->complete) u->complete(u);
        } else {
            prev = u;
        }
        u = next;
    }
}
