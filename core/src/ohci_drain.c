#include <string.h>
#include "ohci_drain.h"
#include "ohci_regs.h"
#include "ohci_td.h"
#include "ohci_urb.h"
#include "ohci_dma.h"
#include "ohci_pool.h"

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

    uint32_t done_head = hc->hcca->DoneHead;
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

        struct ohci_urb *u = urb_for_td_phys(hc, cur, /*remove=*/1);
        if (u) {
            if (u->status == OHCI_URB_STATUS_PENDING) u->status = s;
            if (u->complete) u->complete(u);
        }

        ohci_td_pool_free(&hc->td_pool, td);
        cur = next;
    }
}
