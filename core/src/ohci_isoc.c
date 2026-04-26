#include <string.h>
#include "ohci_isoc.h"
#include "ohci_hc.h"
#include "ohci_regs.h"
#include "ohci_pool.h"
#include "ohci_dma.h"
#include "ohci_urb.h"

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
