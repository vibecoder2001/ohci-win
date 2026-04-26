#include <string.h>
#include "ohci_isoc.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"
#include "ohci_urb.h"
#include "ohci_td.h"

/* Skeleton root index — visited every frame; v1 only supports period-1 isoch. */
#define OHCI_ISOC_ROOT_SKEL_IDX 62

static void ed_set_isoc(struct ohci_ed *ed,
                        const struct ohci_isoc_endpoint_config *cfg) {
    uint32_t c = 0;
    c |= ((uint32_t)cfg->func_addr & 0x7F)        << OHCI_ED_FA_SHIFT;
    c |= ((uint32_t)cfg->ep_num & 0x0F)           << OHCI_ED_EN_SHIFT;
    c |= (cfg->direction == OHCI_URB_DIR_IN) ? OHCI_ED_D_IN : OHCI_ED_D_OUT;
    if (cfg->low_speed) c |= OHCI_ED_S;
    c |= OHCI_ED_F;   /* Format=Isochronous (§4.2.2) */
    c |= ((uint32_t)cfg->max_packet_size & 0x7FF) << OHCI_ED_MPS_SHIFT;
    ed->Control = c;
}

int ohci_isoc_endpoint_create(struct ohci_hc *hc,
                              const struct ohci_isoc_endpoint_config *cfg,
                              struct ohci_isoc_endpoint *ep)
{
    uint32_t ed_phys = 0, ph_phys = 0;
    struct ohci_ed  *ed = ohci_ed_pool_alloc(&hc->isoc_ed_pool, &ed_phys);
    struct ohci_itd *ph = ohci_itd_pool_alloc(&hc->itd_pool, &ph_phys);
    if (!ed || !ph) {
        if (ed) ohci_ed_pool_free(&hc->isoc_ed_pool, ed);
        if (ph) ohci_itd_pool_free(&hc->itd_pool, ph);
        return -1;
    }
    memset(ph, 0, sizeof(*ph));

    ed_set_isoc(ed, cfg);
    ed->HeadP = ph_phys;
    ed->TailP = ph_phys;

    struct ohci_ed *root = hc->interrupt_skeleton[OHCI_ISOC_ROOT_SKEL_IDX];
    ed->NextED   = root->NextED;
    root->NextED = ed_phys;

    ep->ed                    = ed;
    ep->ed_phys               = ed_phys;
    ep->tail_placeholder      = ph;
    ep->tail_placeholder_phys = ph_phys;
    ep->max_packet_size       = cfg->max_packet_size;
    ep->direction             = cfg->direction;
    ep->ed_tail_frame         = 0;
    ep->primed                = 0;

    ep->next = hc->isoc_head;
    hc->isoc_head = ep;
    return 0;
}

void ohci_isoc_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_isoc_endpoint *ep)
{
    ep->ed->Control |= OHCI_ED_K;
    hc->ops.barrier(hc->ops.context);

    /* OHCI §6.2.1: pause PLE around link-edit. Same dance as
     * ohci_interrupt_endpoint_destroy — without the pause the HC may be
     * mid-walk through the periodic chain when we patch NextED, leaving
     * HcPeriodCurrentED dangling at our about-to-be-freed slot. */
    uint32_t hc_ctrl     = hc->ops.read32(hc->ops.context, 0x04);
    int      was_enabled = (hc_ctrl & OHCI_CTRL_PLE) != 0;
    if (was_enabled) {
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl & ~OHCI_CTRL_PLE);
        hc->ops.barrier(hc->ops.context);
        uint32_t f0 = hc->ops.read32(hc->ops.context, 0x3C);
        for (int i = 0; i < 10000; i++) {
            uint32_t f = hc->ops.read32(hc->ops.context, 0x3C);
            if (f != f0) break;
        }
    }
    uint32_t cur_ed = hc->ops.read32(hc->ops.context, 0x1C /* HcPeriodCurrentED */);
    if (cur_ed == ep->ed_phys) {
        hc->ops.write32(hc->ops.context, 0x1C, 0);
    }

    struct ohci_ed *root = hc->interrupt_skeleton[OHCI_ISOC_ROOT_SKEL_IDX];
    if (root->NextED == ep->ed_phys) {
        root->NextED = ep->ed->NextED;
    } else {
        uint32_t cur = root->NextED;
        while (cur) {
            struct ohci_ed *e = ohci_dma_virt_from_phys(hc->dma, cur);
            if (!e) break;
            if (e->NextED == ep->ed_phys) {
                e->NextED = ep->ed->NextED;
                break;
            }
            cur = e->NextED;
        }
    }

    if (was_enabled) {
        hc->ops.barrier(hc->ops.context);
        hc->ops.write32(hc->ops.context, 0x04, hc_ctrl | OHCI_CTRL_PLE);
    }

    struct ohci_isoc_endpoint **pp = &hc->isoc_head;
    while (*pp) {
        if (*pp == ep) { *pp = ep->next; break; }
        pp = &(*pp)->next;
    }

    ohci_itd_pool_free(&hc->itd_pool, ep->tail_placeholder);
    ohci_ed_pool_free(&hc->isoc_ed_pool, ep->ed);
    memset(ep, 0, sizeof(*ep));
}

int ohci_isoc_submit_window(struct ohci_hc *hc,
                            struct ohci_isoc_endpoint *ep,
                            struct ohci_urb *urb,
                            uint16_t sf,
                            uint8_t  pkt_count,
                            const uint16_t *pkt_lens,
                            uint32_t buf_phys,
                            uint32_t buf_len,
                            int first_window)
{
    if (pkt_count == 0 || pkt_count > 8) return -1;
    if (buf_len == 0) return -1;

    /* OHCI §4.3.2: BP0 = top 20 bits of buf_phys; the buffer may cross at
     * most one 4 KB page boundary (BP0 + 0x1000). Reject anything else;
     * caller is responsible for splitting larger windows. */
    uint32_t bp0       = buf_phys & 0xFFFFF000u;
    uint32_t last      = buf_phys + buf_len - 1;
    uint32_t last_page = last & 0xFFFFF000u;
    if (last_page != bp0 && last_page != bp0 + 0x1000u) return -1;

    uint32_t data_phys = 0;
    struct ohci_itd *data = ohci_itd_pool_alloc(&hc->itd_pool, &data_phys);
    if (!data) return -1;

    /* ITD Control: SF in low 16, FC = pkt_count-1 at [26:24], DI=NO_INTR,
     * CC=NOTACCESSED until HW retires. Task 7 will toggle DI on a subset of
     * ITDs so the refill DPC fires. */
    data->Control = ((uint32_t)sf & OHCI_ITD_SF_MASK)
                  | (((uint32_t)(pkt_count - 1) << OHCI_ITD_FC_SHIFT) & OHCI_ITD_FC_MASK)
                  | OHCI_ITD_DI_NO_INTR
                  | ((uint32_t)OHCI_CC_NOTACCESSED << OHCI_ITD_CC_SHIFT);
    data->BP0    = bp0;
    data->BE     = last;
    data->NextTD = 0;

    /* Per-packet PSW: in = byte offset[12:0] from BP0. HW resolves the
     * BP0/BP0+0x1000 page selection from bit 12 of the offset (§4.3.2.2). */
    uint32_t off = buf_phys - bp0;
    for (uint8_t i = 0; i < pkt_count; i++) {
        data->PSW[i] = (uint16_t)(off & 0x1FFFu);
        off += pkt_lens[i];
    }
    for (uint8_t i = pkt_count; i < 8; i++) data->PSW[i] = 0;

    uint32_t new_ph_phys = 0;
    struct ohci_itd *new_ph = ohci_itd_pool_alloc(&hc->itd_pool, &new_ph_phys);
    if (!new_ph) {
        ohci_itd_pool_free(&hc->itd_pool, data);
        return -1;
    }
    memset(new_ph, 0, sizeof(*new_ph));

    /* Promote the current placeholder to a data ITD (head_phys stays stable
     * for HW), point its NextTD at the new placeholder, free the temp slot.
     * Mirrors ohci_interrupt_submit's promote-tail dance. */
    uint32_t head_phys = ep->tail_placeholder_phys;
    *ep->tail_placeholder = *data;
    ep->tail_placeholder->NextTD = new_ph_phys;
    ohci_itd_pool_free(&hc->itd_pool, data);

    ep->tail_placeholder      = new_ph;
    ep->tail_placeholder_phys = new_ph_phys;
    ep->ed_tail_frame         = (uint16_t)(sf + pkt_count);
    ep->primed                = 1;

    /* The pointer at head_phys is actually an ohci_itd, not an ohci_td.
     * Drain MUST dispatch on urb->is_isoc (or ed->Control & OHCI_ED_F)
     * before dereferencing — never reinterpret without checking. */
    struct ohci_td *new_td_virt =
        (struct ohci_td *)ohci_dma_virt_from_phys(hc->dma, head_phys);

    if (first_window) {
        urb->is_isoc          = 1;
        urb->isoc_pkt_count   = pkt_count;
        urb->isoc_pkts_filled = 0;
        urb->ed               = ep->ed;
        urb->status           = OHCI_URB_STATUS_PENDING;
        urb->transferred      = 0;
        urb->buffer           = NULL;
        urb->buffer_phys      = buf_phys;
        urb->length           = buf_len;
        urb->direction        = ep->direction;
        urb->head_td          = new_td_virt;
        urb->tail_td          = new_td_virt;
        urb->data_tds[0].td_phys   = head_phys;
        urb->data_tds[0].chunk_off = 0;
        urb->data_tds[0].chunk_len = buf_len;
        urb->data_td_count  = 1;
        for (uint8_t i = 0; i < pkt_count; i++) {
            urb->isoc_pkts[i].length = 0;
            urb->isoc_pkts[i].cc     = 0;
        }
    } else {
        /* Continuation: append ITD to data_tds[], advance tail_td so the
         * URB-completion lookup fires only when the LAST ITD retires.
         * URB-level fields (transferred, status, isoc_pkt_count, head_td)
         * are left as-is. */
        if (urb->data_td_count < OHCI_URB_MAX_DATA_TDS) {
            uint8_t idx = urb->data_td_count;
            urb->data_tds[idx].td_phys   = head_phys;
            /* chunk_off/chunk_len are unused for isoc (decode_isoc_itd
             * walks PSW directly), but populate sensibly anyway. */
            urb->data_tds[idx].chunk_off = urb->length;
            urb->data_tds[idx].chunk_len = buf_len;
            urb->data_td_count = (uint8_t)(idx + 1);
        }
        urb->length += buf_len;
        urb->tail_td = new_td_virt;
        /* URB stays on hc->in_flight from the first window; do not relink. */
    }

    hc->ops.barrier(hc->ops.context);
    ep->ed->TailP = new_ph_phys;
    /* No doorbell — the HC visits the periodic schedule per frame. */

    if (first_window) {
        urb->next_pending = hc->in_flight;
        hc->in_flight = urb;
    }
    return 0;
}
