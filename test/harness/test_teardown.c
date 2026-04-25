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

    /* Interrupt endpoint: InterruptTable slot returns to pointing at
     * the skeleton leaf after destroy. */
    struct ohci_interrupt_endpoint iep;
    struct ohci_interrupt_endpoint_config icfg = { .func_addr=2, .ep_num=1,
        .max_packet_size=8, .direction=OHCI_URB_DIR_IN,
        .poll_interval_frames=32 };
    ohci_interrupt_endpoint_create(&hc, &icfg, &iep);
    uint8_t slot = iep.slot_index;
    if (hc.hcca->InterruptTable[slot] != iep.ed_phys) {
        fprintf(stderr,"FAIL: int slot not set\n"); return 1;
    }
    ohci_interrupt_endpoint_destroy(&hc, &iep);
    if (hc.hcca->InterruptTable[slot] != hc.interrupt_skeleton_phys[slot]) {
        fprintf(stderr,"FAIL: int slot not restored to skeleton\n"); return 1;
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
