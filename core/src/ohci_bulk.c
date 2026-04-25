#include <string.h>
#include "ohci_bulk.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"

#define OHCI_MAX_TD_BYTES 4096u  /* one 4 KB page per TD: safe, simple */

static void ed_set_bulk(struct ohci_ed *ed,
                        const struct ohci_bulk_endpoint_config *cfg) {
    uint32_t c = 0;
    c |= ((uint32_t)cfg->func_addr & 0x7F)              << OHCI_ED_FA_SHIFT;
    c |= ((uint32_t)cfg->ep_num & 0x0F)                 << OHCI_ED_EN_SHIFT;
    /* Bulk ED carries direction; TDs use T_FROM_ED so toggle auto-carries. */
    c |= (cfg->direction == OHCI_URB_DIR_IN) ? OHCI_ED_D_IN : OHCI_ED_D_OUT;
    if (cfg->low_speed) c |= OHCI_ED_S;
    c |= ((uint32_t)cfg->max_packet_size & 0x7FF)       << OHCI_ED_MPS_SHIFT;
    ed->Control = c;
}

int ohci_bulk_endpoint_create(struct ohci_hc *hc,
                              const struct ohci_bulk_endpoint_config *cfg,
                              struct ohci_bulk_endpoint *ep) {
    uint32_t ed_phys, ph_phys;
    struct ohci_ed *ed = ohci_ed_pool_alloc(&hc->bulk_ed_pool, &ed_phys);
    struct ohci_td *ph = ohci_td_pool_alloc(&hc->td_pool, &ph_phys);
    if (!ed || !ph) {
        if (ed) ohci_ed_pool_free(&hc->bulk_ed_pool, ed);
        if (ph) ohci_td_pool_free(&hc->td_pool, ph);
        return -1;
    }
    ed_set_bulk(ed, cfg);
    ed->HeadP  = ph_phys;
    ed->TailP  = ph_phys;
    ed->NextED = 0;

    ep->ed                    = ed;
    ep->ed_phys               = ed_phys;
    ep->tail_placeholder      = ph;
    ep->tail_placeholder_phys = ph_phys;
    ep->direction             = cfg->direction;

    /* Splice onto Bulk list head.
     *
     * TODO(Plan 4): pause BLE around this sequence when a real HC may
     * be mid-list-walk (OHCI §5.2.5). Safe in Tier-1 harness. */
    uint32_t old_head = hc->ops.read32(hc->ops.context, 0x28 /* HcBulkHeadED */);
    ed->NextED = old_head;
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x28, ed_phys);

    ep->next = hc->bulk_head;
    hc->bulk_head = ep;
    return 0;
}

static struct ohci_td *build_bulk_data_td(struct ohci_td_pool *tdp,
                                           uint32_t buffer_phys, uint32_t length,
                                           uint32_t *phys_out) {
    struct ohci_td *td = ohci_td_pool_alloc(tdp, phys_out);
    if (!td) return NULL;
    /* DP=FROM_ED (00 for Bulk DATA), T=FROM_ED so ED.C toggles automatically.
     * R=1 so the HC may complete early on a short packet. DP 0 means
     * "take from ED" — OHCI §4.3.1.3 — so ED.D field controls IN/OUT. */
    uint32_t ctrl = OHCI_TD_DI_NO_INTR | OHCI_TD_T_FROM_ED | OHCI_TD_R;
    ctrl |= (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT);
    td->Control = ctrl;
    td->CBP     = buffer_phys;
    td->BE      = buffer_phys + length - 1;
    td->NextTD  = 0;
    return td;
}

int ohci_bulk_submit(struct ohci_hc *hc,
                     struct ohci_bulk_endpoint *ep,
                     struct ohci_urb *urb) {
    if (urb->length == 0 || urb->buffer == NULL) return -1;

    urb->status      = OHCI_URB_STATUS_PENDING;
    urb->transferred = 0;
    urb->ed          = ep->ed;

    /* Build one or more data TDs covering the URB buffer. Each TD carries
     * up to OHCI_MAX_TD_BYTES. Chain them via NextTD; the LAST TD's
     * NextTD gets set to the new placeholder below. */
    struct ohci_td *first_td      = NULL;
    struct ohci_td *last_td       = NULL;
    uint32_t first_td_phys = 0;
    uint32_t last_td_phys  = 0;
    uint32_t remaining = urb->length;
    uint32_t offset    = 0;

    while (remaining) {
        uint32_t chunk = remaining > OHCI_MAX_TD_BYTES ? OHCI_MAX_TD_BYTES : remaining;
        uint32_t td_phys;
        struct ohci_td *td = build_bulk_data_td(&hc->td_pool,
            urb->buffer_phys + offset, chunk, &td_phys);
        if (!td) {
            /* Roll back: free all TDs we allocated so far. */
            struct ohci_td *t = first_td;
            while (t) {
                uint32_t n_phys = t->NextTD;
                ohci_td_pool_free(&hc->td_pool, t);
                if (t == last_td) break;
                t = n_phys ? ohci_dma_virt_from_phys(hc->dma, n_phys) : NULL;
            }
            return -1;
        }
        if (!first_td) {
            first_td      = td;
            first_td_phys = td_phys;
        } else {
            last_td->NextTD = td_phys;
        }
        last_td      = td;
        last_td_phys = td_phys;
        offset    += chunk;
        remaining -= chunk;
    }
    (void)first_td_phys;  /* Not needed after folding; suppress warning. */

    uint32_t new_ph_phys;
    struct ohci_td *new_ph = ohci_td_pool_alloc(&hc->td_pool, &new_ph_phys);
    if (!new_ph) {
        struct ohci_td *t = first_td;
        while (t) {
            uint32_t n_phys = t->NextTD;
            ohci_td_pool_free(&hc->td_pool, t);
            if (t == last_td) break;
            t = n_phys ? ohci_dma_virt_from_phys(hc->dma, n_phys) : NULL;
        }
        return -1;
    }

    uint32_t head_td_phys = ep->tail_placeholder_phys;

    /* Fold FIRST TD into the old placeholder. The rest of the chain
     * (if any) is already in pool slots reachable via NextTD. */
    *ep->tail_placeholder = *first_td;
    ohci_td_pool_free(&hc->td_pool, first_td);

    /* Point last TD's NextTD at the new placeholder. If there's only one
     * data TD (first == last), last_td was freed above — but its phys slot
     * (its pool entry) was the same as first_td. After freeing, we no
     * longer have a valid `last_td` virtual pointer. Special-case it: */
    if (last_td == first_td) {
        /* Single-TD chain: the data now lives in the old placeholder.
         * Update the placeholder's NextTD to the new placeholder. */
        ep->tail_placeholder->NextTD = new_ph_phys;
        /* tail_td for drain matching = the data TD at head_td_phys. */
    } else {
        /* Multi-TD chain: last_td still lives in its pool slot (not freed),
         * and its NextTD field is currently 0 from build_bulk_data_td.
         * Point it at the new placeholder. */
        last_td->NextTD = new_ph_phys;
    }

    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;

    urb->head_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    /* tail_td is the last data TD the drain will see retired. For single-TD
     * chains that's the old placeholder slot; for multi-TD it's last_td. */
    urb->tail_td = (last_td == first_td)
        ? ohci_dma_virt_from_phys(hc->dma, head_td_phys)
        : ohci_dma_virt_from_phys(hc->dma, last_td_phys);

    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;

    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x08 /* HcCommandStatus */, OHCI_CMD_BLF);

    urb->next_pending = hc->in_flight;
    hc->in_flight = urb;
    return 0;
}

void ohci_bulk_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_bulk_endpoint *ep) {
    ep->ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);

    uint32_t head = hc->ops.read32(hc->ops.context, 0x28);
    if (head == ep->ed_phys) {
        hc->ops.write32(hc->ops.context, 0x28, ep->ed->NextED);
    } else {
        uint32_t cur = head;
        while (cur) {
            struct ohci_ed *ed = ohci_dma_virt_from_phys(hc->dma, cur);
            if (!ed) break;
            if (ed->NextED == ep->ed_phys) {
                ed->NextED = ep->ed->NextED;
                break;
            }
            cur = ed->NextED;
        }
    }

    struct ohci_bulk_endpoint **pp = &hc->bulk_head;
    while (*pp) {
        if (*pp == ep) { *pp = ep->next; break; }
        pp = &(*pp)->next;
    }

    ohci_td_pool_free(&hc->td_pool, ep->tail_placeholder);
    ohci_ed_pool_free(&hc->bulk_ed_pool, ep->ed);
    memset(ep, 0, sizeof(*ep));
}
