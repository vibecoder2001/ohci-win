#include <string.h>
#include "ohci_bulk.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"

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

    /* Build single data TD; chain it with a new placeholder. */
    uint32_t data_td_phys;
    struct ohci_td *data_td = build_bulk_data_td(&hc->td_pool,
        urb->buffer_phys, urb->length, &data_td_phys);
    if (!data_td) return -1;

    uint32_t new_ph_phys;
    struct ohci_td *new_ph = ohci_td_pool_alloc(&hc->td_pool, &new_ph_phys);
    if (!new_ph) {
        ohci_td_pool_free(&hc->td_pool, data_td);
        return -1;
    }

    uint32_t head_td_phys = ep->tail_placeholder_phys;

    /* Fold data_td into the old placeholder; point the data TD's
     * NextTD at the new placeholder. */
    *ep->tail_placeholder = *data_td;
    ep->tail_placeholder->NextTD = new_ph_phys;
    ohci_td_pool_free(&hc->td_pool, data_td);

    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;

    urb->head_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    /* Drain matches by URB tail_td phys. For a single-TD Bulk URB the
     * "tail" is that one data TD — which now lives in the old placeholder
     * slot (at head_td_phys) after the in-place swap. Task 4 (SG) will
     * point tail_td at the LAST data TD when the chain has multiple. */
    urb->tail_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);

    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;

    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x08 /* HcCommandStatus */, OHCI_CMD_BLF);

    /* Track in-flight. */
    urb->next_pending = hc->in_flight;
    hc->in_flight = urb;
    return 0;
}
