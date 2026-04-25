#include <string.h>
#include "fake_hc.h"
#include "fake_hc_exec.h"
#include "ohci_regs.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_hcca.h"
#include "ohci_dma.h"

static uint32_t rd(struct fake_hc *hc, uint32_t off) {
    uint32_t v; memcpy(&v, hc->regs + off, sizeof(v)); return v;
}
static void wr(struct fake_hc *hc, uint32_t off, uint32_t v) {
    memcpy(hc->regs + off, &v, sizeof(v));
}

static int walk_ed_list(struct fake_hc *hc, uint32_t ed_phys,
                         struct ohci_hcca *hcca) {
    int retired = 0;
    while (ed_phys) {
        struct ohci_ed *ed = ohci_dma_virt_from_phys(hc->dma, ed_phys);
        if (!ed) break;

        if (!(ed->Control & OHCI_ED_K) && !(ed->HeadP & OHCI_ED_HEADP_H)) {
            uint32_t head = ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
            uint32_t tail = ed->TailP & OHCI_ED_HEADP_ADDR_MASK;
            while (head && head != tail) {
                struct ohci_td *td = ohci_dma_virt_from_phys(hc->dma, head);
                if (!td) break;
                uint32_t next = td->NextTD;
                td->Control &= ~(OHCI_TD_CC_MASK | OHCI_TD_EC_MASK);
                td->Control |=  (OHCI_CC_NOERROR << OHCI_TD_CC_SHIFT);
                td->NextTD = hcca->DoneHead;
                hcca->DoneHead = head;
                head = next;
                retired++;
            }
            ed->HeadP = (ed->HeadP & (OHCI_ED_HEADP_H | OHCI_ED_HEADP_C))
                      | (tail & OHCI_ED_HEADP_ADDR_MASK);
        }
        ed_phys = ed->NextED;
    }
    return retired;
}

void fake_hc_exec_step(struct fake_hc *hc) {
    if (!hc->dma) return;
    uint32_t hcca_phys = rd(hc, 0x18);
    struct ohci_hcca *hcca = ohci_dma_virt_from_phys(hc->dma, hcca_phys);
    if (!hcca) return;

    int retired = 0;
    retired += walk_ed_list(hc, rd(hc, 0x20 /* HcControlHeadED */), hcca);
    retired += walk_ed_list(hc, rd(hc, 0x28 /* HcBulkHeadED    */), hcca);

    /* Periodic list: advance frame counter and walk the one slot pointed
     * at by HCCA.InterruptTable[frame & 31]. Each leaf chains through
     * user EPs then the skeleton tree; skeleton EDs have K=1 and are
     * skipped by walk_ed_list. */
    uint32_t slot = hc->frame_number & 31;
    retired += walk_ed_list(hc, hcca->InterruptTable[slot], hcca);
    hc->frame_number++;
    hcca->FrameNumber = (uint16_t)hc->frame_number;

    if (retired) {
        uint32_t s = rd(hc, 0x0C);
        s |= OHCI_INT_WDH;
        wr(hc, 0x0C, s);
    }

    /* Clear both doorbells. */
    uint32_t cmd = rd(hc, 0x08);
    cmd &= ~(OHCI_CMD_CLF | OHCI_CMD_BLF);
    wr(hc, 0x08, cmd);
}
