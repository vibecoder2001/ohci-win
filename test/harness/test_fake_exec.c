#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x70000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops;
    fake_hc_get_ops(&fake, &ops);

    struct ohci_hc hc;
    ohci_hc_init(&hc, &ops, &dma, 64);

    struct ohci_ed_pool edp;
    ohci_ed_pool_init(&edp, &dma, 2);

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .max_packet_size=8 };
    ohci_control_endpoint_create(&hc, &edp, &cfg, &ep);

    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    setup[0]=0x80; setup[1]=0x06; setup[2]=0x00; setup[3]=0x01;
    setup[4]=0x00; setup[5]=0x00; setup[6]=0x12; setup[7]=0x00;

    struct ohci_urb urb = {0};
    memcpy(urb.setup, setup, 8);
    urb.setup_phys = setup_phys;
    urb.buffer = data; urb.buffer_phys = data_phys; urb.length = 18;
    urb.direction = OHCI_URB_DIR_IN;
    ohci_control_submit(&hc, &ep, &urb);

    /* Execute — simulate the HC walking the list. */
    fake_hc_exec_step(&fake);

    /* HCCA.DoneHead must be non-zero (TDs retired). */
    if (hc.hcca->DoneHead == 0) { fprintf(stderr,"FAIL: DoneHead empty\n"); return 1; }

    /* HcInterruptStatus.WDH must be set. */
    if (!(ops.read32(ops.context, 0x0C) & OHCI_INT_WDH))
        { fprintf(stderr,"FAIL: WDH not set\n"); return 1; }

    /* ED should now have HeadP == TailP (queue drained except placeholder). */
    uint32_t hp = ep.ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
    if (hp != ep.ed->TailP) { fprintf(stderr,"FAIL: ED not drained, hp=0x%x tp=0x%x\n",
                                      hp, ep.ed->TailP); return 1; }

    /* Walk DoneHead — should have 3 TDs (SETUP, DATA, STATUS) in LIFO order. */
    int count = 0;
    uint32_t cur = hc.hcca->DoneHead;
    while (cur && count < 10) {
        struct ohci_td *td = ohci_dma_virt_from_phys(&dma, cur);
        if (!td) { fprintf(stderr,"FAIL: DoneHead chain phys %x\n", cur); return 1; }
        /* All TDs retired successfully: CC = NOERROR. */
        if (((td->Control >> OHCI_TD_CC_SHIFT) & 0xF) != OHCI_CC_NOERROR)
            { fprintf(stderr,"FAIL: TD CC != NOERROR\n"); return 1; }
        cur = td->NextTD;
        count++;
    }
    if (count != 3) { fprintf(stderr,"FAIL: expected 3 TDs on DoneHead, got %d\n", count); return 1; }

    printf("PASS: fake_hc_exec retires Control chain\n");
    return 0;
}
