#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "ohci_drain.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int done[4] = {0};
static void complete_idx(struct ohci_urb *urb) {
    int *slot = (int *)urb->context;
    *slot = 1;
}

static void submit_and_wait(struct ohci_hc *hc,
                             struct fake_hc *fake,
                             struct ohci_control_endpoint *ep,
                             struct ohci_urb *urb) {
    if (ohci_control_submit(hc, ep, urb) != 0) { fprintf(stderr,"FAIL: submit\n"); exit(1); }
    fake_hc_exec_step(fake);
    ohci_drain_done(hc);
}

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[256 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x90000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=128,
        .control_ed_count=4, .bulk_ed_count=4, .interrupt_ed_count=4 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) { fprintf(stderr,"FAIL: hc_init\n"); return 1; }

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .func_addr = 0, .ep_num = 0,
                                                 .max_packet_size = 8 };
    ohci_control_endpoint_create(&hc, &cfg, &ep);

    /* First URB: GET_DEVICE_DESCRIPTOR */
    uint32_t s1_phys; uint8_t *s1 = ohci_dma_alloc(&dma, 8, 4, &s1_phys);
    uint32_t d1_phys; uint8_t *d1 = ohci_dma_alloc(&dma, 18, 4, &d1_phys);
    s1[0]=0x80; s1[1]=0x06; s1[2]=0x00; s1[3]=0x01;
    s1[4]=0x00; s1[5]=0x00; s1[6]=0x12; s1[7]=0x00;

    struct ohci_urb u1 = {0};
    memcpy(u1.setup, s1, 8);
    u1.setup_phys = s1_phys;
    u1.buffer = d1; u1.buffer_phys = d1_phys; u1.length = 18;
    u1.direction = OHCI_URB_DIR_IN;
    u1.complete = complete_idx; u1.context = &done[0];

    submit_and_wait(&hc, &fake, &ep, &u1);

    if (!done[0]) { fprintf(stderr,"FAIL: u1 not completed\n"); return 1; }
    if (u1.status != OHCI_URB_STATUS_OK) { fprintf(stderr,"FAIL: u1.status=%d\n", u1.status); return 1; }

    /* Second URB back-to-back: SET_ADDRESS (no data stage). */
    uint32_t s2_phys; uint8_t *s2 = ohci_dma_alloc(&dma, 8, 4, &s2_phys);
    s2[0]=0x00; s2[1]=0x05; s2[2]=0x02; s2[3]=0x00;
    s2[4]=0x00; s2[5]=0x00; s2[6]=0x00; s2[7]=0x00;

    struct ohci_urb u2 = {0};
    memcpy(u2.setup, s2, 8);
    u2.setup_phys = s2_phys;
    /* No data stage */
    u2.buffer = NULL; u2.buffer_phys = 0; u2.length = 0;
    u2.direction = OHCI_URB_DIR_OUT;
    u2.complete = complete_idx; u2.context = &done[1];

    submit_and_wait(&hc, &fake, &ep, &u2);

    if (!done[1]) { fprintf(stderr,"FAIL: u2 not completed\n"); return 1; }
    if (u2.status != OHCI_URB_STATUS_OK) { fprintf(stderr,"FAIL: u2.status=%d\n", u2.status); return 1; }

    /* Verify pool sanity: ED list head still points at ep.ed. */
    if (ops.read32(ops.context, 0x20) != ep.ed_phys) {
        fprintf(stderr,"FAIL: HcControlHeadED drifted\n"); return 1;
    }

    /* HCCA.DoneHead cleared. */
    if (hc.hcca->DoneHead != 0) { fprintf(stderr,"FAIL: DoneHead not cleared at end\n"); return 1; }

    printf("PASS: end-to-end Control transfer (with + without data stage)\n");
    return 0;
}
