#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_urb.h"
#include "ohci_interrupt.h"
#include "ohci_drain.h"
#include "ohci_dma.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int completed;
static void on_done(struct ohci_urb *u) { (void)u; completed++; }

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xE0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 16,
        .control_ed_count=1, .bulk_ed_count=1, .interrupt_ed_count=2 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    struct ohci_interrupt_endpoint ep;
    struct ohci_interrupt_endpoint_config epcfg = {
        .func_addr = 3, .ep_num = 1, .max_packet_size = 8,
        .direction = OHCI_URB_DIR_IN, .low_speed = 1,
        .poll_interval_frames = 32,
    };
    if (ohci_interrupt_endpoint_create(&hc, &epcfg, &ep) != 0) {
        fprintf(stderr,"FAIL: int ep_create\n"); return 1;
    }

    /* ep attaches at one leaf; verify the HCCA.InterruptTable entry for
     * that leaf now points at ep.ed (ep inserted before the skeleton). */
    int slot = ep.slot_index;
    if (hc.hcca->InterruptTable[slot] != ep.ed_phys) {
        fprintf(stderr,"FAIL: slot %d = 0x%x, expected ep 0x%x\n",
                slot, hc.hcca->InterruptTable[slot], ep.ed_phys); return 1;
    }
    /* ep.ed->NextED == skeleton leaf phys. */
    if (ep.ed->NextED != hc.interrupt_skeleton_phys[slot]) {
        fprintf(stderr,"FAIL: ep->NextED not skeleton leaf\n"); return 1;
    }

    uint32_t buf_phys; uint8_t *buf = ohci_dma_alloc(&dma, 8, 4, &buf_phys);
    memset(buf, 0, 8);

    struct ohci_urb urb = {0};
    urb.buffer = buf; urb.buffer_phys = buf_phys; urb.length = 8;
    urb.direction = OHCI_URB_DIR_IN;
    urb.complete = on_done;

    ohci_interrupt_submit(&hc, &ep, &urb);

    /* Step frames until this slot is polled. */
    for (int i = 0; i < 64 && !completed; i++) {
        fake_hc_exec_step(&fake);
        ohci_drain_done(&hc);
    }

    if (!completed) { fprintf(stderr,"FAIL: URB never completed\n"); return 1; }
    if (urb.status != OHCI_URB_STATUS_OK) {
        fprintf(stderr,"FAIL: status=%d\n", urb.status); return 1;
    }

    printf("PASS: Interrupt IN 32ms end-to-end\n");
    return 0;
}
