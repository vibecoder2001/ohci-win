#include <string.h>
#include "ohci_bulk.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"

/* See ohci_control.c — bounded wait for one HcFmNumber tick. */
static void wait_one_frame(struct ohci_hc *hc) {
    uint32_t f0 = hc->ops.read32(hc->ops.context, 0x3C /* HcFmNumber */);
    for (int i = 0; i < 10000; i++) {
        uint32_t f = hc->ops.read32(hc->ops.context, 0x3C);
        if (f != f0) return;
    }
}

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

    /* Splice onto Bulk list head. Per OHCI §6.2.1 pause BLE around the
     * link-edit so the HC isn't mid-walk when we update head/NextED. */
    uint32_t hc_ctrl = hc->ops.read32(hc->ops.context, 0x04 /* HcControl */);
    int was_enabled  = (hc_ctrl & OHCI_CTRL_BLE) != 0;
    if (was_enabled) {
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl & ~OHCI_CTRL_BLE);
        hc->ops.barrier(hc->ops.context);
        wait_one_frame(hc);
    }
    uint32_t old_head = hc->ops.read32(hc->ops.context, 0x28 /* HcBulkHeadED */);
    ed->NextED = old_head;
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x28, ed_phys);
    if (was_enabled) {
        hc->ops.barrier(hc->ops.context);
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl | OHCI_CTRL_BLE);
    }

    ep->next = hc->bulk_head;
    hc->bulk_head = ep;
    return 0;
}

int ohci_bulk_submit_sg(struct ohci_hc *hc,
                        struct ohci_bulk_endpoint *ep,
                        struct ohci_urb *urb,
                        const struct ohci_bulk_sg_page *pages,
                        unsigned page_count) {
    if (page_count == 0 || page_count > OHCI_BULK_MAX_SG_PAGES) return -1;

    urb->status        = OHCI_URB_STATUS_PENDING;
    urb->transferred   = 0;
    urb->ed            = ep->ed;
    urb->data_td_count = 0;

    struct ohci_td *first_td = NULL, *last_td = NULL;
    uint32_t first_td_phys = 0, last_td_phys = 0;

    for (unsigned i = 0; i < page_count; i++) {
        if (pages[i].length == 0) continue;
        uint32_t td_phys;
        struct ohci_td *td = ohci_td_pool_alloc(&hc->td_pool, &td_phys);
        if (!td) {
            struct ohci_td *t = first_td;
            while (t) {
                uint32_t n = t->NextTD;
                ohci_td_pool_free(&hc->td_pool, t);
                if (t == last_td) break;
                t = n ? ohci_dma_virt_from_phys(hc->dma, n) : NULL;
            }
            return -1;
        }
        uint32_t ctrl = OHCI_TD_DI_NO_INTR | OHCI_TD_T_FROM_ED | OHCI_TD_R;
        ctrl |= (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT);
        td->Control = ctrl;
        td->CBP     = pages[i].phys;
        td->BE      = pages[i].phys + pages[i].length - 1;
        td->NextTD  = 0;
        if (!first_td) {
            first_td      = td;
            first_td_phys = td_phys;
        } else {
            last_td->NextTD = td_phys;
        }
        last_td      = td;
        last_td_phys = td_phys;

        urb->data_tds[urb->data_td_count].td_phys   = td_phys;
        urb->data_tds[urb->data_td_count].chunk_off = pages[i].off;
        urb->data_tds[urb->data_td_count].chunk_len = pages[i].length;
        urb->data_td_count++;
    }
    if (!first_td) return -1;
    (void)first_td_phys;

    uint32_t new_ph_phys;
    struct ohci_td *new_ph = ohci_td_pool_alloc(&hc->td_pool, &new_ph_phys);
    if (!new_ph) {
        struct ohci_td *t = first_td;
        while (t) {
            uint32_t n = t->NextTD;
            ohci_td_pool_free(&hc->td_pool, t);
            if (t == last_td) break;
            t = n ? ohci_dma_virt_from_phys(hc->dma, n) : NULL;
        }
        return -1;
    }
    memset(new_ph, 0, sizeof(*new_ph));

    uint32_t head_td_phys = ep->tail_placeholder_phys;

    /* Fold first_td into the existing placeholder so ED.HeadP still
     * points at a live TD. */
    *ep->tail_placeholder = *first_td;
    ohci_td_pool_free(&hc->td_pool, first_td);
    if (last_td == first_td) {
        ep->tail_placeholder->NextTD = new_ph_phys;
    } else {
        last_td->NextTD = new_ph_phys;
    }

    /* Patch the first TD record: its physical address is now the
     * placeholder's slot (head_td_phys), not the freed pool slot. */
    urb->data_tds[0].td_phys = head_td_phys;

    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;

    urb->head_td = ohci_dma_virt_from_phys(hc->dma, head_td_phys);
    urb->tail_td = (last_td == first_td)
        ? ohci_dma_virt_from_phys(hc->dma, head_td_phys)
        : ohci_dma_virt_from_phys(hc->dma, last_td_phys);

    /* Last TD must IOC so WDH fires when the chain completes. */
    if (last_td == first_td) {
        ep->tail_placeholder->Control =
            (ep->tail_placeholder->Control & ~OHCI_TD_DI_MASK) | OHCI_TD_DI_IMMEDIATE;
    } else {
        last_td->Control =
            (last_td->Control & ~OHCI_TD_DI_MASK) | OHCI_TD_DI_IMMEDIATE;
    }

    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;
    hc->ops.barrier(hc->ops.context);
    hc->ops.write32(hc->ops.context, 0x08, OHCI_CMD_BLF);

    urb->next_pending = hc->in_flight;
    hc->in_flight     = urb;
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
