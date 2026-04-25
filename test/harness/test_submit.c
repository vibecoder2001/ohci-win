#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x60000000u, sizeof(backing));

    struct ohci_mmio_ops ops;
    fake_hc_get_ops(&fake, &ops);

    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=64,
        .control_ed_count=4, .bulk_ed_count=4, .interrupt_ed_count=4 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) { fprintf(stderr,"FAIL: hc_init\n"); return 1; }

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = {
        .func_addr = 0, .ep_num = 0, .max_packet_size = 8, .low_speed = 0
    };
    if (ohci_control_endpoint_create(&hc, &cfg, &ep) != 0)
        { fprintf(stderr,"FAIL: ep_create\n"); return 1; }

    /* HcControlHeadED must equal ep.ed_phys */
    if (ops.read32(ops.context, 0x20) != ep.ed_phys)
        { fprintf(stderr,"FAIL: HcControlHeadED != ep_phys\n"); return 1; }

    /* ED must be in "empty" state: HeadP == TailP (placeholder). */
    uint32_t hp = ep.ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
    if (hp != ep.ed->TailP)
        { fprintf(stderr,"FAIL: empty ED HeadP != TailP\n"); return 1; }

    /* Build + submit a GET_DEVICE_DESCRIPTOR URB. */
    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    setup[0]=0x80; setup[1]=0x06; setup[2]=0x00; setup[3]=0x01;
    setup[4]=0x00; setup[5]=0x00; setup[6]=0x12; setup[7]=0x00;

    struct ohci_urb urb = {0};
    memcpy(urb.setup, setup, 8);
    urb.setup_phys = setup_phys;
    urb.buffer = data; urb.buffer_phys = data_phys; urb.length = 18;
    urb.direction = OHCI_URB_DIR_IN;

    if (ohci_control_submit(&hc, &ep, &urb) != 0)
        { fprintf(stderr,"FAIL: submit\n"); return 1; }

    /* After submit: ED.HeadP (addr bits) != ED.TailP — queue non-empty. */
    uint32_t hp2 = ep.ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
    if (hp2 == ep.ed->TailP)
        { fprintf(stderr,"FAIL: TailP did not advance\n"); return 1; }

    /* Doorbell must have been written. */
    if ((ops.read32(ops.context, 0x08) & OHCI_CMD_CLF) == 0)
        { fprintf(stderr,"FAIL: CLF doorbell not set\n"); return 1; }

    /* HeadP should point at a TD whose DP=SETUP. */
    struct ohci_td *setup_td = ohci_dma_virt_from_phys(&dma, hp2);
    if ((setup_td->Control & OHCI_TD_DP_MASK) != OHCI_TD_DP_SETUP)
        { fprintf(stderr,"FAIL: head TD DP != SETUP\n"); return 1; }
    if (setup_td->CBP != setup_phys)
        { fprintf(stderr,"FAIL: head TD CBP\n"); return 1; }

    printf("PASS: submit advances TailP + rings doorbell\n");
    return 0;
}
