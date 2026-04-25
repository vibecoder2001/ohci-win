#include <string.h>
#include "ohci_interrupt.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"
#include "ohci_hcca.h"

static void ed_set_interrupt(struct ohci_ed *ed,
                             const struct ohci_interrupt_endpoint_config *cfg) {
    uint32_t c = 0;
    c |= ((uint32_t)cfg->func_addr & 0x7F)              << OHCI_ED_FA_SHIFT;
    c |= ((uint32_t)cfg->ep_num & 0x0F)                 << OHCI_ED_EN_SHIFT;
    c |= (cfg->direction == OHCI_URB_DIR_IN) ? OHCI_ED_D_IN : OHCI_ED_D_OUT;
    if (cfg->low_speed) c |= OHCI_ED_S;
    c |= ((uint32_t)cfg->max_packet_size & 0x7FF)       << OHCI_ED_MPS_SHIFT;
    ed->Control = c;
}

static uint8_t pick_slot_32ms(struct ohci_hc *hc) {
    uint8_t count = 0;
    struct ohci_interrupt_endpoint *e = hc->interrupt_head;
    while (e) { count++; e = e->next; }
    return count & 31;
}

int ohci_interrupt_endpoint_create(struct ohci_hc *hc,
                                   const struct ohci_interrupt_endpoint_config *cfg,
                                   struct ohci_interrupt_endpoint *ep) {
    if (cfg->poll_interval_frames != 32) return -1;

    uint32_t ed_phys, ph_phys;
    struct ohci_ed *ed = ohci_ed_pool_alloc(&hc->interrupt_ed_pool, &ed_phys);
    struct ohci_td *ph = ohci_td_pool_alloc(&hc->td_pool, &ph_phys);
    if (!ed || !ph) {
        if (ed) ohci_ed_pool_free(&hc->interrupt_ed_pool, ed);
        if (ph) ohci_td_pool_free(&hc->td_pool, ph);
        return -1;
    }
    memset(ph, 0, sizeof(*ph));
    ed_set_interrupt(ed, cfg);
    ed->HeadP  = ph_phys;
    ed->TailP  = ph_phys;

    uint8_t slot = pick_slot_32ms(hc);
    /* Insert at head of the slot chain: ep.NextED = current slot head;
     * HCCA.InterruptTable[slot] = ep_phys. The previous chain still leads
     * down through the skeleton (or through another user endpoint). */
    ed->NextED = hc->hcca->InterruptTable[slot];
    hc->hcca->InterruptTable[slot] = ed_phys;

    ep->ed                    = ed;
    ep->ed_phys               = ed_phys;
    ep->tail_placeholder      = ph;
    ep->tail_placeholder_phys = ph_phys;
    ep->direction             = cfg->direction;
    ep->slot_index            = slot;
    ep->poll_interval_frames  = cfg->poll_interval_frames;

    ep->next = hc->interrupt_head;
    hc->interrupt_head = ep;
    return 0;
}

int ohci_interrupt_submit(struct ohci_hc *hc,
                          struct ohci_interrupt_endpoint *ep,
                          struct ohci_urb *urb) {
    if (urb->length == 0 || urb->buffer == NULL) return -1;
    urb->status      = OHCI_URB_STATUS_PENDING;
    urb->transferred = 0;
    urb->ed          = ep->ed;

    uint32_t data_td_phys;
    struct ohci_td *data_td = ohci_td_pool_alloc(&hc->td_pool, &data_td_phys);
    if (!data_td) return -1;
    /* DI=0 (immediate IOC). With only one TD per Interrupt URB this is the
     * TD that signals completion; DI=7 here means the HC retires the TD but
     * never raises WDH and the URB hangs in PENDING. Same root cause as
     * the Plan 5 STATUS TD fix. */
    uint32_t ctrl = OHCI_TD_DI_IMMEDIATE | OHCI_TD_T_FROM_ED | OHCI_TD_R;
    ctrl |= (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT);
    data_td->Control = ctrl;
    data_td->CBP     = urb->buffer_phys;
    data_td->BE      = urb->buffer_phys + urb->length - 1;
    data_td->NextTD  = 0;

    uint32_t new_ph_phys;
    struct ohci_td *new_ph = ohci_td_pool_alloc(&hc->td_pool, &new_ph_phys);
    if (!new_ph) {
        ohci_td_pool_free(&hc->td_pool, data_td);
        return -1;
    }
    memset(new_ph, 0, sizeof(*new_ph));

    uint32_t head_td_phys = ep->tail_placeholder_phys;

    *ep->tail_placeholder = *data_td;
    ep->tail_placeholder->NextTD = new_ph_phys;
    ohci_td_pool_free(&hc->td_pool, data_td);

    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;

    urb->head_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    urb->tail_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    /* Single TD per Interrupt URB; drain reads CBP from it. */
    urb->data_tds[0].td_phys   = head_td_phys;
    urb->data_tds[0].chunk_off = 0;
    urb->data_tds[0].chunk_len = urb->length;
    urb->data_td_count = 1;

    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;
    /* No doorbell for Interrupt — the HC visits the slot per frame. */

    urb->next_pending = hc->in_flight;
    hc->in_flight = urb;
    return 0;
}

void ohci_interrupt_endpoint_destroy(struct ohci_hc *hc,
                                     struct ohci_interrupt_endpoint *ep) {
    ep->ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);

    uint8_t slot = ep->slot_index;
    /* Unlink from the slot chain. If ep is the slot head, patch
     * HCCA.InterruptTable[slot]; otherwise walk NextED. */
    if (hc->hcca->InterruptTable[slot] == ep->ed_phys) {
        hc->hcca->InterruptTable[slot] = ep->ed->NextED;
    } else {
        uint32_t cur = hc->hcca->InterruptTable[slot];
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

    struct ohci_interrupt_endpoint **pp = &hc->interrupt_head;
    while (*pp) {
        if (*pp == ep) { *pp = ep->next; break; }
        pp = &(*pp)->next;
    }

    ohci_td_pool_free(&hc->td_pool, ep->tail_placeholder);
    ohci_ed_pool_free(&hc->interrupt_ed_pool, ep->ed);
    memset(ep, 0, sizeof(*ep));
}
