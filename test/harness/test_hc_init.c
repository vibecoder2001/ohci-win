#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ohci_regs.h"
#include "ohci_hcca.h"
#include "ohci_hc.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc fake;
    fake_hc_init(&fake);

    /* 64 KiB DMA region — plenty for HCCA + handful of EDs/TDs */
    static uint8_t dma_backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, dma_backing, 0x20000000u, sizeof(dma_backing));

    struct ohci_mmio_ops ops;
    fake_hc_get_ops(&fake, &ops);

    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size       = 64,
        .control_ed_count   = 4,
        .bulk_ed_count      = 4,
        .interrupt_ed_count = 4,
    };
    int rc = ohci_hc_init(&hc, &ops, &dma, &hcfg);
    if (rc != 0) { fprintf(stderr, "FAIL: ohci_hc_init -> %d\n", rc); return 1; }

    /* HcHCCA must equal the phys addr of the allocated HCCA */
    uint32_t hc_hcca = ops.read32(ops.context, 0x18);
    if (hc_hcca != hc.hcca_phys) {
        fprintf(stderr, "FAIL: HcHCCA = 0x%x, expected 0x%x\n", hc_hcca, hc.hcca_phys);
        return 1;
    }

    /* HCCA must be 256-byte aligned */
    if ((hc.hcca_phys & 0xFF) != 0) {
        fprintf(stderr, "FAIL: HCCA unaligned 0x%x\n", hc.hcca_phys);
        return 1;
    }

    /* HcControlHeadED starts empty (0) */
    if (ops.read32(ops.context, 0x20) != 0) {
        fprintf(stderr, "FAIL: HcControlHeadED not zero\n"); return 1;
    }

    /* InterruptTable[i] must now point at skeleton leaves (non-zero). */
    for (int i = 0; i < 32; i++) {
        if (hc.hcca->InterruptTable[i] == 0) {
            fprintf(stderr, "FAIL: HCCA.InterruptTable[%d] = 0 (skeleton expected)\n", i);
            return 1;
        }
    }

    /* HcControl: OPER state, CLE + BLE + PLE + IE set */
    uint32_t ctrl = ops.read32(ops.context, 0x04);
    if ((ctrl & OHCI_CTRL_HCFS_MASK) != OHCI_CTRL_HCFS_OPER) {
        fprintf(stderr, "FAIL: HCFS != OPER (ctrl=0x%x)\n", ctrl); return 1;
    }
    if (!(ctrl & OHCI_CTRL_CLE)) { fprintf(stderr, "FAIL: CLE not set\n"); return 1; }
    if (!(ctrl & OHCI_CTRL_IE))  { fprintf(stderr, "FAIL: IE not set\n"); return 1; }
    if (!(ctrl & OHCI_CTRL_BLE)) { fprintf(stderr, "FAIL: BLE not set\n"); return 1; }
    if (!(ctrl & OHCI_CTRL_PLE)) { fprintf(stderr, "FAIL: PLE should be set\n"); return 1; }

    /* HcInterruptEnable: WDH + MIE */
    uint32_t ie = ops.read32(ops.context, 0x10);
    if (!(ie & OHCI_INT_WDH)) { fprintf(stderr, "FAIL: WDH not enabled\n"); return 1; }
    if (!(ie & OHCI_INT_MIE)) { fprintf(stderr, "FAIL: MIE not enabled\n"); return 1; }

    printf("PASS: hc_init OK\n");
    return 0;
}
