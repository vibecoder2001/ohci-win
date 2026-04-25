#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_ed.h"
#include "ohci_dma.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xD0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 8,
        .control_ed_count=1, .bulk_ed_count=1, .interrupt_ed_count=4 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) { fprintf(stderr,"FAIL: init\n"); return 1; }

    /* All 32 HCCA.InterruptTable entries must equal the 32 leaves. */
    for (int i = 0; i < 32; i++) {
        if (hc.hcca->InterruptTable[i] != hc.interrupt_skeleton_phys[i]) {
            fprintf(stderr,"FAIL: InterruptTable[%d] = 0x%x, expected 0x%x\n",
                    i, hc.hcca->InterruptTable[i], hc.interrupt_skeleton_phys[i]);
            return 1;
        }
    }

    /* Every skeleton ED must have K=1. */
    for (int i = 0; i < 63; i++) {
        if (!(hc.interrupt_skeleton[i]->Control & OHCI_ED_K)) {
            fprintf(stderr,"FAIL: skeleton[%d] K not set\n", i); return 1;
        }
    }

    /* Verify a specific path: leaf 0 -> level1 32 -> level2 48 -> ... -> root 62. */
    int idx = 0, expected[6] = {0, 32, 48, 56, 60, 62};
    uint32_t cur = hc.interrupt_skeleton_phys[0];
    while (cur && idx < 6) {
        if (cur != hc.interrupt_skeleton_phys[expected[idx]]) {
            fprintf(stderr,"FAIL: path step %d: cur=0x%x, expected skeleton[%d]=0x%x\n",
                    idx, cur, expected[idx], hc.interrupt_skeleton_phys[expected[idx]]);
            return 1;
        }
        struct ohci_ed *ed = ohci_dma_virt_from_phys(&dma, cur);
        if (!ed) { fprintf(stderr,"FAIL: cur->ED resolve\n"); return 1; }
        cur = ed->NextED;
        idx++;
    }
    if (idx != 6 || cur != 0) {
        fprintf(stderr,"FAIL: tree walk idx=%d cur=0x%x\n", idx, cur); return 1;
    }

    /* HcControl.PLE must be enabled. */
    uint32_t ctrl = ops.read32(ops.context, 0x04);
    if (!(ctrl & OHCI_CTRL_PLE)) { fprintf(stderr,"FAIL: PLE not set\n"); return 1; }

    printf("PASS: interrupt skeleton + PLE enabled\n");
    return 0;
}
