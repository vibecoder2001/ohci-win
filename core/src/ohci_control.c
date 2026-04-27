#include <string.h>
#include "ohci_control.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_dma.h"

/* Wait until at least one SOF has elapsed by polling HcFmNumber. The HC
 * advances the frame counter every 1ms; one transition is enough to
 * guarantee any in-progress list walk has completed (OHCI §5.2.7). The
 * spin is bounded so a wedged controller can't hang the host. */
static void wait_one_frame(struct ohci_hc *hc) {
    uint32_t f0 = hc->ops.read32(hc->ops.context, 0x3C /* HcFmNumber */);
    for (int i = 0; i < 10000; i++) {
        uint32_t f = hc->ops.read32(hc->ops.context, 0x3C);
        if (f != f0) return;
    }
}

/* Build a General TD representing one stage of a Control transfer. */
static void fill_td(struct ohci_td *td,
                    uint32_t dp,
                    uint32_t toggle,
                    uint32_t cbp,
                    uint32_t be,
                    uint32_t next_td_phys,
                    int allow_short,
                    uint32_t di) {
    uint32_t ctrl = di | dp | toggle;
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
            /*allow_short=*/0,
            OHCI_TD_DI_NO_INTR);

    /* DATA: DATA1, direction from URB, short packets allowed. */
    if (have_data) {
        uint32_t dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_IN : OHCI_TD_DP_OUT;
        fill_td(data_td,
                dp,
                OHCI_TD_T_DATA1,
                urb->buffer_phys,
                urb->buffer_phys + urb->length - 1,
                status_td_phys,
                /*allow_short=*/1,
                OHCI_TD_DI_NO_INTR);
    }

    /* STATUS: DATA1, opposite direction. Zero-length (CBP=BE=0). */
    uint32_t status_dp;
    if (have_data) {
        status_dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_OUT : OHCI_TD_DP_IN;
    } else {
        /* No data stage: STATUS is always IN (device acks). */
        status_dp = OHCI_TD_DP_IN;
    }
    /* DI=0 on STATUS so the controller raises WDH as soon as this TD is
     * retired — that's how we learn the URB completed. Earlier stages use
     * DI=7 (no IOC) to avoid extra interrupts mid-transfer. */
    fill_td(status_td,
            status_dp,
            OHCI_TD_T_DATA1,
            /*cbp=*/0,
            /*be =*/0,
            /*next=*/0,
            /*allow_short=*/0,
            OHCI_TD_DI_IMMEDIATE);

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
                                 const struct ohci_control_endpoint_config *cfg,
                                 struct ohci_control_endpoint *ep) {
    uint32_t ed_phys, ph_phys;
    struct ohci_ed *ed = ohci_ed_pool_alloc(&hc->control_ed_pool, &ed_phys);
    struct ohci_td *ph = ohci_td_pool_alloc(&hc->td_pool, &ph_phys);
    if (!ed || !ph) {
        if (ed) ohci_ed_pool_free(&hc->control_ed_pool, ed);
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

    /* Splice onto Control list head. Per OHCI §6.2.1 we must pause the
     * Control list (clear HcControl.CLE), wait for the HC to drain the
     * current SOF window, link the ED, then re-enable. Skip the pause
     * if CLE is already off (e.g. Tier-1 fake HC). */
    uint32_t hc_ctrl = hc->ops.read32(hc->ops.context, 0x04 /* HcControl */);
    int was_enabled  = (hc_ctrl & OHCI_CTRL_CLE) != 0;
    if (was_enabled) {
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl & ~OHCI_CTRL_CLE);
        hc->ops.barrier(hc->ops.context);
        wait_one_frame(hc);
    }
    uint32_t old_head = hc->ops.read32(hc->ops.context, 0x20);
    ed->NextED = old_head;
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x20, ed_phys);
    if (was_enabled) {
        hc->ops.barrier(hc->ops.context);
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl | OHCI_CTRL_CLE);
    }

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
    /* If there's a data stage, the SETUP TD's NextTD now points at it.
     * Record the data TD so the drain can compute bytes transferred. */
    if (urb->buffer && urb->length > 0) {
        urb->data_tds[0].td_phys   = urb->head_td->NextTD;
        urb->data_tds[0].chunk_off = 0;
        urb->data_tds[0].chunk_len = urb->length;
        urb->data_td_count = 1;
    } else {
        urb->data_td_count = 0;
    }

    /* Publish the new chain to the HC. OHCI 1.0a §5.2.7.2 allows TailP
     * modification without a K-toggle, but real silicon (observed on
     * RK3588's OHCI) caches HeadP/TailP from the ED's first scan and
     * never re-fetches after a TailP-only update — symptom: HC parks at
     * the ED with CtlCur==CtlHead and never starts the SETUP TD.
     * Bracket the writes with a K-toggle so the HC drops its cached ED
     * snapshot and re-reads on the next frame.
     *
     * Also rewrite HeadP unconditionally to point at the freshly-built
     * SETUP TD (head_td_phys, with H=0 C=0). This serves two purposes:
     *  1. Recovers from a prior STALL: OHCI §6.4.4 sets HeadP[0] (H,
     *     Halted) when a TD reports a non-zero CC. The HCD must clear
     *     it before new TDs on this ED will dispatch. We never get an
     *     EP0-level reset callback from UCX for control STALLs, so the
     *     submit path itself has to clear the halt.
     *  2. EP0 toggle always restarts at DATA0 on SETUP per USB §8.5.3,
     *     so dropping the C (toggle Carry) bit by writing HeadP=head_td_phys
     *     is correct for control endpoints. (NOT correct for bulk/interrupt
     *     where toggle must persist — those EPs use the EditHeadPSafely
     *     helper for selective edits.) */
    ep->ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;
    ep->ed->HeadP = head_td_phys;
    hc->ops.barrier(hc->ops.context);
    ep->ed->Control &= ~OHCI_ED_K;

    /* Doorbell: HcCommandStatus.CLF = 1 signals the HC to walk the list. */
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x08 /* HcCommandStatus */, OHCI_CMD_CLF);

    /* Track this URB as in-flight. */
    urb->next_pending = hc->in_flight;
    hc->in_flight = urb;
    return 0;
}

void ohci_control_endpoint_destroy(struct ohci_hc *hc,
                                   struct ohci_control_endpoint *ep) {
    /* 1) Skip bit so the HC stops dispatching new TDs on this ED. */
    ep->ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);

    /* OHCI §6.2.1: pause CLE around link-edit. Same reasoning as
     * ohci_bulk_endpoint_destroy — unsafe link mutation while HC walks
     * leaves HcControlCurrentED dangling. */
    uint32_t hc_ctrl     = hc->ops.read32(hc->ops.context, 0x04);
    int      was_enabled = (hc_ctrl & OHCI_CTRL_CLE) != 0;
    if (was_enabled) {
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl & ~OHCI_CTRL_CLE);
        hc->ops.barrier(hc->ops.context);
        wait_one_frame(hc);
    }
    uint32_t cur_ed = hc->ops.read32(hc->ops.context, 0x24 /* HcControlCurrentED */);
    if (cur_ed == ep->ed_phys) {
        hc->ops.write32(hc->ops.context, 0x24, 0);
    }

    /* 2) Unlink from HcControlHeadED list. */
    uint32_t head = hc->ops.read32(hc->ops.context, 0x20);
    if (head == ep->ed_phys) {
        hc->ops.write32(hc->ops.context, 0x20, ep->ed->NextED);
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

    if (was_enabled) {
        hc->ops.barrier(hc->ops.context);
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl | OHCI_CTRL_CLE);
    }

    /* 3) Unlink from SW-side control_head list. */
    struct ohci_control_endpoint **pp = &hc->control_head;
    while (*pp) {
        if (*pp == ep) { *pp = ep->next; break; }
        pp = &(*pp)->next;
    }

    /* 4) Free pool resources. */
    ohci_td_pool_free(&hc->td_pool, ep->tail_placeholder);
    ohci_ed_pool_free(&hc->control_ed_pool, ep->ed);
    memset(ep, 0, sizeof(*ep));
}
