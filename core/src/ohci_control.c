#include <string.h>
#include "ohci_control.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_dma.h"

/* Build a General TD representing one stage of a Control transfer. */
static void fill_td(struct ohci_td *td,
                    uint32_t dp,
                    uint32_t toggle,
                    uint32_t cbp,
                    uint32_t be,
                    uint32_t next_td_phys,
                    int allow_short) {
    uint32_t ctrl = OHCI_TD_DI_NO_INTR | dp | toggle;
    ctrl |= (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT);
    if (allow_short) ctrl |= OHCI_TD_R;
    td->Control = ctrl;
    td->CBP     = cbp;
    td->NextTD  = next_td_phys;
    td->BE      = be;
}

int ohci_build_control_chain(struct ohci_urb *urb,
                             struct ohci_td_pool *tdp,
                             struct ohci_td **head_out,
                             struct ohci_td **tail_out) {
    /* Always SETUP + STATUS; DATA present iff buffer && length. */
    int have_data = urb->buffer && urb->length > 0;

    uint32_t setup_td_phys = 0, data_td_phys = 0, status_td_phys = 0;
    struct ohci_td *setup_td  = ohci_td_pool_alloc(tdp, &setup_td_phys);
    struct ohci_td *data_td   = NULL;
    struct ohci_td *status_td = ohci_td_pool_alloc(tdp, &status_td_phys);

    if (have_data) {
        data_td = ohci_td_pool_alloc(tdp, &data_td_phys);
    }
    if (!setup_td || !status_td || (have_data && !data_td)) {
        if (setup_td)  ohci_td_pool_free(tdp, setup_td);
        if (data_td)   ohci_td_pool_free(tdp, data_td);
        if (status_td) ohci_td_pool_free(tdp, status_td);
        return -1;
    }

    /* SETUP: DATA0, 8 bytes at urb->setup_phys, no short-packet rounding. */
    uint32_t next_after_setup = have_data ? data_td_phys : status_td_phys;
    fill_td(setup_td,
            OHCI_TD_DP_SETUP,
            OHCI_TD_T_DATA0,
            urb->setup_phys,
            urb->setup_phys + 7,
            next_after_setup,
            /*allow_short=*/0);

    /* DATA: DATA1, direction from URB, short packets allowed. */
    if (have_data) {
        uint32_t dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_IN : OHCI_TD_DP_OUT;
        fill_td(data_td,
                dp,
                OHCI_TD_T_DATA1,
                urb->buffer_phys,
                urb->buffer_phys + urb->length - 1,
                status_td_phys,
                /*allow_short=*/1);
    }

    /* STATUS: DATA1, opposite direction. Zero-length (CBP=BE=0). */
    uint32_t status_dp;
    if (have_data) {
        status_dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_OUT : OHCI_TD_DP_IN;
    } else {
        /* No data stage: STATUS is always IN (device acks). */
        status_dp = OHCI_TD_DP_IN;
    }
    fill_td(status_td,
            status_dp,
            OHCI_TD_T_DATA1,
            /*cbp=*/0,
            /*be =*/0,
            /*next=*/0,
            /*allow_short=*/0);

    *head_out = setup_td;
    *tail_out = status_td;
    return 0;
}

static void ed_set_control(struct ohci_ed *ed,
                           const struct ohci_control_endpoint_config *cfg) {
    uint32_t c = 0;
    c |= ((uint32_t)cfg->func_addr & 0x7F)              << OHCI_ED_FA_SHIFT;
    c |= ((uint32_t)cfg->ep_num & 0x0F)                 << OHCI_ED_EN_SHIFT;
    c |= OHCI_ED_D_FROM_TD;     /* Control endpoints take direction from TD */
    if (cfg->low_speed) c |= OHCI_ED_S;
    c |= ((uint32_t)cfg->max_packet_size & 0x7FF)       << OHCI_ED_MPS_SHIFT;
    ed->Control = c;
}

int ohci_control_endpoint_create(struct ohci_hc *hc,
                                 struct ohci_ed_pool *edp,
                                 const struct ohci_control_endpoint_config *cfg,
                                 struct ohci_control_endpoint *ep) {
    uint32_t ed_phys, ph_phys;
    struct ohci_ed *ed = ohci_ed_pool_alloc(edp, &ed_phys);
    struct ohci_td *ph = ohci_td_pool_alloc(&hc->td_pool, &ph_phys);
    if (!ed || !ph) {
        if (ed) ohci_ed_pool_free(edp, ed);
        if (ph) ohci_td_pool_free(&hc->td_pool, ph);
        return -1;
    }
    ed_set_control(ed, cfg);
    ed->HeadP  = ph_phys;   /* HeadP == TailP == placeholder: empty queue */
    ed->TailP  = ph_phys;
    ed->NextED = 0;

    ep->ed                    = ed;
    ep->ed_phys               = ed_phys;
    ep->tail_placeholder      = ph;
    ep->tail_placeholder_phys = ph_phys;

    /* Splice onto Control list head.
     *
     * TODO(Plan 3): pause CLE (HcControl.CLE=0) around this sequence
     * when a real HC may be mid-list-walk (OHCI §5.2.3). Safe in
     * Tier-1 harness where no HC is actively walking. */
    uint32_t old_head = hc->ops.read32(hc->ops.context, 0x20);
    ed->NextED = old_head;
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x20, ed_phys);

    /* SW-side list for iteration in done-queue handling. */
    ep->next = hc->control_head;
    hc->control_head = ep;
    return 0;
}

int ohci_control_submit(struct ohci_hc *hc,
                        struct ohci_control_endpoint *ep,
                        struct ohci_urb *urb) {
    urb->status      = OHCI_URB_STATUS_PENDING;
    urb->transferred = 0;
    urb->ed          = ep->ed;

    /* Build a fresh 2- or 3-TD chain in a new set of pool TDs. */
    struct ohci_td *chain_head = NULL, *chain_tail = NULL;
    int rc = ohci_build_control_chain(urb, &hc->td_pool, &chain_head, &chain_tail);
    if (rc != 0) return rc;

    /* Allocate a NEW placeholder to become the future tail. */
    uint32_t new_ph_phys;
    struct ohci_td *new_ph = ohci_td_pool_alloc(&hc->td_pool, &new_ph_phys);
    if (!new_ph) {
        /* Roll back the chain on failure. */
        struct ohci_td *t = chain_head;
        while (t) {
            struct ohci_td *n = (t->NextTD == 0) ? NULL
                : ohci_dma_virt_from_phys(hc->dma, t->NextTD);
            ohci_td_pool_free(&hc->td_pool, t);
            if (t == chain_tail) break;
            t = n;
        }
        return -1;
    }

    /* Capture the old placeholder's phys BEFORE the swap — it's the
     * physical address of what will become the head TD of this URB
     * (the in-place overwrite below fills that slot with SETUP contents).
     * ED.HeadP still points here because no HC has retired a TD yet. */
    uint32_t head_td_phys = ep->tail_placeholder_phys;

    /* OHCI placeholder convention: fold chain_head's contents into the
     * existing placeholder (so ED.HeadP still points to a live TD), then
     * make chain_tail->NextTD point at the new placeholder. */
    *ep->tail_placeholder = *chain_head;
    chain_tail->NextTD = new_ph_phys;

    /* chain_head's pool slot is redundant now (its content lives in
     * the old placeholder). Free it back to the pool. */
    ohci_td_pool_free(&hc->td_pool, chain_head);

    /* Install the new placeholder as the tail. */
    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;

    urb->head_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    urb->tail_td = chain_tail;

    /* Update ED.TailP — publishes the new chain to the HC. */
    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;

    /* Doorbell: HcCommandStatus.CLF = 1 signals the HC to walk the list. */
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x08 /* HcCommandStatus */, OHCI_CMD_CLF);
    return 0;
}
