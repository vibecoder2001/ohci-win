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

void fake_hc_exec_step(struct fake_hc *hc) {
    if (!hc->dma) return;
    uint32_t hcca_phys = rd(hc, 0x18 /* HcHCCA */);
    struct ohci_hcca *hcca = ohci_dma_virt_from_phys(hc->dma, hcca_phys);
    if (!hcca) return;

    int retired = 0;
    uint32_t ed_phys = rd(hc, 0x20 /* HcControlHeadED */);
    while (ed_phys) {
        struct ohci_ed *ed = ohci_dma_virt_from_phys(hc->dma, ed_phys);
        if (!ed) break;

        /* Skip if Halted or Skip bit set. */
        if (!(ed->Control & OHCI_ED_K) && !(ed->HeadP & OHCI_ED_HEADP_H)) {
            uint32_t head = ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
            uint32_t tail = ed->TailP & OHCI_ED_HEADP_ADDR_MASK;
            while (head && head != tail) {
                struct ohci_td *td = ohci_dma_virt_from_phys(hc->dma, head);
                if (!td) break;
                uint32_t next = td->NextTD;
                /* Mark TD successful. Clear CC bits, EC, leave other fields. */
                td->Control &= ~(OHCI_TD_CC_MASK | OHCI_TD_EC_MASK);
                td->Control |=  (OHCI_CC_NOERROR << OHCI_TD_CC_SHIFT);
                /* Prepend to HCCA.DoneHead (LIFO). */
                td->NextTD = hcca->DoneHead;
                hcca->DoneHead = head;
                head = next;
                retired++;
            }
            /* Drained up to placeholder; advance ED.HeadP. Carry bit
             * semantics (toggle) aren't simulated here — real HC preserves
             * ED.HeadP.C; for the fake it doesn't matter because we always
             * build TDs with explicit toggle. */
            ed->HeadP = (ed->HeadP & (OHCI_ED_HEADP_H | OHCI_ED_HEADP_C))
                      | (tail & OHCI_ED_HEADP_ADDR_MASK);
        }
        ed_phys = ed->NextED;
    }

    if (retired) {
        uint32_t s = rd(hc, 0x0C /* HcInterruptStatus */);
        s |= OHCI_INT_WDH;
        wr(hc, 0x0C, s);
    }

    /* Clear HcCommandStatus.CLF (software asserts it, HC clears when
     * the list is walked and empty). */
    uint32_t cmd = rd(hc, 0x08);
    cmd &= ~OHCI_CMD_CLF;
    wr(hc, 0x08, cmd);
}
