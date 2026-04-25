#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "ohci_drain.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int completion_count = 0;
static struct ohci_urb *last_completed = NULL;

static void on_complete(struct ohci_urb *urb) {
    completion_count++;
    last_completed = urb;
}

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=64,
        .control_ed_count=4, .bulk_ed_count=4, .interrupt_ed_count=4 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .max_packet_size = 8 };
    ohci_control_endpoint_create(&hc, &cfg, &ep);

    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    setup[0]=0x80; setup[1]=0x06; setup[2]=0x00; setup[3]=0x01;
    setup[4]=0x00; setup[5]=0x00; setup[6]=0x12; setup[7]=0x00;

    struct ohci_urb urb = {0};
    memcpy(urb.setup, setup, 8);
    urb.setup_phys = setup_phys;
    urb.buffer = data; urb.buffer_phys = data_phys; urb.length = 18;
    urb.direction = OHCI_URB_DIR_IN;
    urb.complete  = on_complete;

    ohci_control_submit(&hc, &ep, &urb);
    fake_hc_exec_step(&fake);       /* HC retires the chain */
    ohci_drain_done(&hc);           /* core drains + completes */

    if (completion_count != 1)
        { fprintf(stderr,"FAIL: completion_count=%d\n", completion_count); return 1; }
    if (last_completed != &urb)
        { fprintf(stderr,"FAIL: wrong urb completed\n"); return 1; }
    if (urb.status != OHCI_URB_STATUS_OK)
        { fprintf(stderr,"FAIL: urb.status=%d\n", urb.status); return 1; }

    /* HCCA.DoneHead must be cleared after drain. */
    if (hc.hcca->DoneHead != 0)
        { fprintf(stderr,"FAIL: DoneHead not cleared\n"); return 1; }
    /* WDH must be cleared. */
    if (ops.read32(ops.context, 0x0C) & OHCI_INT_WDH)
        { fprintf(stderr,"FAIL: WDH not cleared\n"); return 1; }

    printf("PASS: drain_done completes URB\n");
    return 0;
}
