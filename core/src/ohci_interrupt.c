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

/* Round requested interval DOWN to nearest power of 2 in [1,32], return
 * skeleton level (0..5) where period = 32 / (1 << level):
 *   interval >= 32 -> level 0 (period 32 frames)
 *   interval >= 16 -> level 1
 *   interval >= 8  -> level 2
 *   interval >= 4  -> level 3
 *   interval >= 2  -> level 4
 *   else           -> level 5 (period 1 frame)
 */
static uint8_t level_for_interval(uint16_t interval_frames) {
    if (interval_frames == 0) interval_frames = 1;
    uint16_t p = 1;
    while ((p << 1) <= interval_frames && (p << 1) <= 32) p <<= 1;
    switch (p) {
    case 32: return 0;
    case 16: return 1;
    case  8: return 2;
    case  4: return 3;
    case  2: return 4;
    default: return 5;   /* p == 1 */
    }
}

/* Skeleton index for the start of each level, per build_interrupt_skeleton:
 *   level 0: leaves [ 0..31] (start  0)
 *   level 1:        [32..47] (start 32)
 *   level 2:        [48..55] (start 48)
 *   level 3:        [56..59] (start 56)
 *   level 4:        [60..61] (start 60)
 *   level 5: root   [62..62] (start 62)
 */
static int skeleton_index_for_level(uint8_t level) {
    static const int starts[6] = { 0, 32, 48, 56, 60, 62 };
    return starts[level];
}

int ohci_interrupt_endpoint_create(struct ohci_hc *hc,
                                   const struct ohci_interrupt_endpoint_config *cfg,
                                   struct ohci_interrupt_endpoint *ep) {

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

    uint8_t level    = level_for_interval(cfg->poll_interval_frames);
    int     skel_idx = skeleton_index_for_level(level);
    struct ohci_ed *skel_ed = hc->interrupt_skeleton[skel_idx];

    /* Insert user ED at the head of skel_ed's NextED chain. The HC reaches
     * skel_ed via HCCA[i] -> ... -> skel_ed once every (32 >> level) frames
     * by skeleton design (build_interrupt_skeleton). */
    ed->NextED      = skel_ed->NextED;
    skel_ed->NextED = ed_phys;

    ep->ed                    = ed;
    ep->ed_phys               = ed_phys;
    ep->tail_placeholder      = ph;
    ep->tail_placeholder_phys = ph_phys;
    ep->direction             = cfg->direction;
    ep->slot_index            = (uint8_t)skel_idx;     /* repurposed: skeleton slot */
    ep->poll_interval_frames  = (uint16_t)(32u >> level);

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

    /* OHCI §6.2.1: pause PLE around link-edit. Periodic schedule is
     * walked per frame; without the pause the HC can be mid-walk through
     * the skeleton + leaf chain when we patch NextED, leaving
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

    /* slot_index now means skeleton slot; unlink from skel_ed's NextED chain. */
    int skel_idx = ep->slot_index;
    struct ohci_ed *skel_ed = hc->interrupt_skeleton[skel_idx];
    if (skel_ed->NextED == ep->ed_phys) {
        skel_ed->NextED = ep->ed->NextED;
    } else {
        uint32_t cur = skel_ed->NextED;
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

    struct ohci_interrupt_endpoint **pp = &hc->interrupt_head;
    while (*pp) {
        if (*pp == ep) { *pp = ep->next; break; }
        pp = &(*pp)->next;
    }

    ohci_td_pool_free(&hc->td_pool, ep->tail_placeholder);
    ohci_ed_pool_free(&hc->interrupt_ed_pool, ep->ed);
    memset(ep, 0, sizeof(*ep));
}
