#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_ed.h"
#include "ohci_dma.h"
#include "ohci_isoc.h"
#include "ohci_urb.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size       = 8,
        .control_ed_count   = 1,
        .bulk_ed_count      = 1,
        .interrupt_ed_count = 1,
        .isoc_ed_count      = 2,
        .itd_pool_size      = 4,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) {
        fprintf(stderr, "FAIL: hc_init\n"); return 1;
    }

    /* Case 1: create succeeds. */
    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 5, .ep_num = 2, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 192,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) {
        fprintf(stderr, "FAIL: isoc create\n"); return 1;
    }

    /* Case 2: ED Format=Iso bit set. */
    if (!(ep.ed->Control & OHCI_ED_F)) {
        fprintf(stderr, "FAIL: OHCI_ED_F not set in Control=0x%08x\n", ep.ed->Control);
        return 1;
    }

    /* Case 3: encoded fields. */
    uint32_t c = ep.ed->Control;
    if (((c & OHCI_ED_FA_MASK) >> OHCI_ED_FA_SHIFT) != 5) {
        fprintf(stderr, "FAIL: func_addr encode\n"); return 1;
    }
    if (((c & OHCI_ED_EN_MASK) >> OHCI_ED_EN_SHIFT) != 2) {
        fprintf(stderr, "FAIL: ep_num encode\n"); return 1;
    }
    if ((c & OHCI_ED_D_MASK) != OHCI_ED_D_IN) {
        fprintf(stderr, "FAIL: direction IN\n"); return 1;
    }
    if (((c & OHCI_ED_MPS_MASK) >> OHCI_ED_MPS_SHIFT) != 192) {
        fprintf(stderr, "FAIL: mps encode\n"); return 1;
    }

    /* Case 4: linked off skeleton root index 62. */
    if (hc.interrupt_skeleton[62]->NextED != ep.ed_phys) {
        fprintf(stderr, "FAIL: root->NextED=0x%x expected 0x%x\n",
                hc.interrupt_skeleton[62]->NextED, ep.ed_phys);
        return 1;
    }

    /* Case 5: HeadP == TailP == ph_phys (queue empty). */
    if (ep.ed->HeadP != ep.tail_placeholder_phys ||
        ep.ed->TailP != ep.tail_placeholder_phys) {
        fprintf(stderr, "FAIL: HeadP/TailP not at placeholder\n"); return 1;
    }

    /* Case 6: second EP inserted at head of root's chain. */
    struct ohci_isoc_endpoint ep2;
    struct ohci_isoc_endpoint_config cfg2 = {
        .func_addr = 7, .ep_num = 3, .direction = OHCI_URB_DIR_OUT,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg2, &ep2) != 0) {
        fprintf(stderr, "FAIL: second isoc create\n"); return 1;
    }
    if (hc.interrupt_skeleton[62]->NextED != ep2.ed_phys) {
        fprintf(stderr, "FAIL: root->NextED should be ep2 after second create\n");
        return 1;
    }
    if (ep2.ed->NextED != ep.ed_phys) {
        fprintf(stderr, "FAIL: ep2->NextED should be ep1\n"); return 1;
    }

    /* Case 7: destroy removes from chain and zeroes struct. */
    uint32_t ep_ed_phys_saved = ep.ed_phys;
    ohci_isoc_endpoint_destroy(&hc, &ep);

    /* Walk root's chain; ep.ed_phys must not appear. */
    uint32_t cur = hc.interrupt_skeleton[62]->NextED;
    while (cur) {
        if (cur == ep_ed_phys_saved) {
            fprintf(stderr, "FAIL: destroyed ED still in chain\n"); return 1;
        }
        struct ohci_ed *e = ohci_dma_virt_from_phys(&dma, cur);
        if (!e) break;
        cur = e->NextED;
    }
    /* Verify ep struct memset to 0. */
    if (ep.ed != NULL || ep.ed_phys != 0 || ep.tail_placeholder != NULL) {
        fprintf(stderr, "FAIL: ep not zeroed after destroy\n"); return 1;
    }

    /* Case 8: pool reusable. Create another EP — should succeed. */
    struct ohci_isoc_endpoint ep3;
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep3) != 0) {
        fprintf(stderr, "FAIL: ep3 create after destroy (pool exhaustion?)\n");
        return 1;
    }

    /* Cleanup. */
    ohci_isoc_endpoint_destroy(&hc, &ep3);
    ohci_isoc_endpoint_destroy(&hc, &ep2);

    printf("PASS: ohci_isoc endpoint create/destroy\n");
    return 0;
}
