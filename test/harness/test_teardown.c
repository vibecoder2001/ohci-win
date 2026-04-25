#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_control.h"
#include "ohci_bulk.h"
#include "ohci_interrupt.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xF0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 16,
        .control_ed_count=2, .bulk_ed_count=2, .interrupt_ed_count=2 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    /* Control endpoint: create, destroy, verify list head cleared. */
    struct ohci_control_endpoint cep;
    struct ohci_control_endpoint_config ccfg = { .max_packet_size = 8 };
    ohci_control_endpoint_create(&hc, &ccfg, &cep);
    if (ops.read32(ops.context, 0x20) != cep.ed_phys) {
        fprintf(stderr,"FAIL: control head not set\n"); return 1;
    }
    ohci_control_endpoint_destroy(&hc, &cep);
    if (ops.read32(ops.context, 0x20) != 0) {
        fprintf(stderr,"FAIL: control head not cleared\n"); return 1;
    }
    if (hc.control_head != NULL) { fprintf(stderr,"FAIL: sw control_head\n"); return 1; }

    /* Bulk endpoint: same test. */
    struct ohci_bulk_endpoint bep;
    struct ohci_bulk_endpoint_config bcfg = { .func_addr=1, .ep_num=1,
        .max_packet_size=64, .direction=OHCI_URB_DIR_IN };
    ohci_bulk_endpoint_create(&hc, &bcfg, &bep);
    if (ops.read32(ops.context, 0x28) != bep.ed_phys) {
        fprintf(stderr,"FAIL: bulk head not set\n"); return 1;
    }
    ohci_bulk_endpoint_destroy(&hc, &bep);
    if (ops.read32(ops.context, 0x28) != 0) {
        fprintf(stderr,"FAIL: bulk head not cleared\n"); return 1;
    }

    /* Interrupt endpoint (Plan 7): user ED attaches as skeleton-ED's
     * NextED head, not directly into HCCA.InterruptTable. After destroy
     * the skeleton ED's NextED returns to its previous value (0 for the
     * first attached EP). HCCA[i] never moves off the skeleton phys. */
    struct ohci_interrupt_endpoint iep;
    struct ohci_interrupt_endpoint_config icfg = { .func_addr=2, .ep_num=1,
        .max_packet_size=8, .direction=OHCI_URB_DIR_IN,
        .poll_interval_frames=32 };
    /* Snapshot the skeleton ED's NextED before insert — for a leaf this is
     * the level-1 skeleton ED phys per build_interrupt_skeleton, not 0. */
    ohci_interrupt_endpoint_create(&hc, &icfg, &iep);
    uint8_t skel_idx = iep.slot_index;
    uint32_t saved_next = iep.ed->NextED;   /* what skel_ed->NextED was before insert */
    if (hc.interrupt_skeleton[skel_idx]->NextED != iep.ed_phys) {
        fprintf(stderr,"FAIL: skeleton[%u]->NextED != ep.ed_phys\n", skel_idx);
        return 1;
    }
    ohci_interrupt_endpoint_destroy(&hc, &iep);
    if (hc.interrupt_skeleton[skel_idx]->NextED != saved_next) {
        fprintf(stderr,"FAIL: skeleton[%u]->NextED=0x%x not restored to 0x%x\n",
                skel_idx, hc.interrupt_skeleton[skel_idx]->NextED, saved_next);
        return 1;
    }

    /* Pool reuse check. */
    struct ohci_control_endpoint cep2, cep3;
    if (ohci_control_endpoint_create(&hc, &ccfg, &cep2) != 0) {
        fprintf(stderr,"FAIL: realloc cep2\n"); return 1;
    }
    if (ohci_control_endpoint_create(&hc, &ccfg, &cep3) != 0) {
        fprintf(stderr,"FAIL: realloc cep3\n"); return 1;
    }

    printf("PASS: endpoint teardown + pool reuse\n");
    return 0;
}
