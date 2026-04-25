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

    /* Plan 7: ep attaches as skel_ed->NextED head (not into HCCA directly).
     * For poll_interval_frames=32 the picker chooses level 0 -> skel_idx 0.
     * HCCA[0] still points at skeleton[0]; skel_ed->NextED == ep.ed_phys. */
    int skel_idx = ep.slot_index;
    if (skel_idx != 0) {
        fprintf(stderr,"FAIL: expected skel_idx=0 for interval=32, got %d\n", skel_idx);
        return 1;
    }
    if (hc.hcca->InterruptTable[0] != hc.interrupt_skeleton_phys[0]) {
        fprintf(stderr,"FAIL: HCCA[0] should still point at skeleton[0]\n"); return 1;
    }
    if (hc.interrupt_skeleton[0]->NextED != ep.ed_phys) {
        fprintf(stderr,"FAIL: skeleton[0]->NextED != ep.ed_phys\n"); return 1;
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
