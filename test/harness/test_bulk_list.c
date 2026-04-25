#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_pool.h"
#include "ohci_dma.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

/* Assemble a single-TD Bulk queue by hand on a fresh ED. Retires via
 * fake_hc_exec_step. Validates the executor walks HcBulkHeadED. */
int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xA0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config cfg = { .td_pool_size = 16,
        .control_ed_count=1, .bulk_ed_count=1, .interrupt_ed_count=1 };
    ohci_hc_init(&hc, &ops, &dma, &cfg);

    /* Alloc ED + a TD + a placeholder manually, wire up as Bulk IN EP. */
    uint32_t ed_phys, td_phys, ph_phys;
    struct ohci_ed *ed = ohci_ed_pool_alloc(&hc.bulk_ed_pool, &ed_phys);
    struct ohci_td *td = ohci_td_pool_alloc(&hc.td_pool, &td_phys);
    (void)ohci_td_pool_alloc(&hc.td_pool, &ph_phys);

    ed->Control = (1u << 7) /* EN=1 */
                 | OHCI_ED_D_IN
                 | (64u << OHCI_ED_MPS_SHIFT);
    ed->TailP  = ph_phys;
    ed->HeadP  = td_phys;
    ed->NextED = 0;

    /* One data TD: no real buffer needed for this layout-only test. */
    td->Control = OHCI_TD_DI_NO_INTR | OHCI_TD_DP_IN | OHCI_TD_T_DATA0
                 | (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT) | OHCI_TD_R;
    td->CBP    = 0;
    td->BE     = 0;
    td->NextTD = ph_phys;

    ops.write32(ops.context, 0x28 /* HcBulkHeadED */, ed_phys);
    ops.write32(ops.context, 0x08 /* HcCommandStatus */, OHCI_CMD_BLF);

    fake_hc_exec_step(&fake);

    /* TD must be on DoneHead and marked successful. */
    if (hc.hcca->DoneHead != td_phys) {
        fprintf(stderr,"FAIL: DoneHead = 0x%x, expected 0x%x\n",
                hc.hcca->DoneHead, td_phys); return 1;
    }
    if (((td->Control >> OHCI_TD_CC_SHIFT) & 0xF) != OHCI_CC_NOERROR) {
        fprintf(stderr,"FAIL: TD CC != NOERROR\n"); return 1;
    }
    /* ED drained. */
    if ((ed->HeadP & OHCI_ED_HEADP_ADDR_MASK) != ed->TailP) {
        fprintf(stderr,"FAIL: bulk ED not drained\n"); return 1;
    }
    /* WDH. */
    if (!(ops.read32(ops.context, 0x0C) & OHCI_INT_WDH)) {
        fprintf(stderr,"FAIL: WDH not set\n"); return 1;
    }
    /* BLF cleared by HC simulation. */
    if (ops.read32(ops.context, 0x08) & OHCI_CMD_BLF) {
        fprintf(stderr,"FAIL: BLF not cleared\n"); return 1;
    }

    printf("PASS: fake_hc_exec retires Bulk TD\n");
    return 0;
}
