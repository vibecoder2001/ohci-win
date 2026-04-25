#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_dma.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"

int main(void) {
    static uint8_t backing[4096];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x50000000u, sizeof(backing));

    struct ohci_td_pool tdp;
    if (ohci_td_pool_init(&tdp, &dma, 16) != 0) { fprintf(stderr,"FAIL: td pool\n"); return 1; }

    /* Allocate a SETUP packet and a data buffer inside the DMA region
     * (in the real driver these are mapped URB buffers; here they're
     * just co-resident with the descriptors). */
    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    /* GET_DEVICE_DESCRIPTOR: bmRequestType=0x80, bRequest=0x06, wValue=0x0100, wIndex=0, wLength=18 */
    setup[0]=0x80; setup[1]=0x06; setup[2]=0x00; setup[3]=0x01;
    setup[4]=0x00; setup[5]=0x00; setup[6]=0x12; setup[7]=0x00;

    struct ohci_urb urb = {0};
    memcpy(urb.setup, setup, 8);
    urb.setup_phys = setup_phys;
    urb.buffer     = data;
    urb.buffer_phys= data_phys;
    urb.length     = 18;
    urb.direction  = OHCI_URB_DIR_IN;

    struct ohci_td *head = NULL, *tail = NULL;
    int rc = ohci_build_control_chain(&urb, &tdp, &head, &tail);
    if (rc != 0) { fprintf(stderr,"FAIL: build -> %d\n", rc); return 1; }

    /* SETUP TD */
    if ((head->Control & OHCI_TD_DP_MASK) != OHCI_TD_DP_SETUP)
        { fprintf(stderr,"FAIL: SETUP DP\n"); return 1; }
    if ((head->Control & OHCI_TD_T_MASK) != OHCI_TD_T_DATA0)
        { fprintf(stderr,"FAIL: SETUP toggle\n"); return 1; }
    if (head->CBP != setup_phys)
        { fprintf(stderr,"FAIL: SETUP CBP 0x%x\n", head->CBP); return 1; }
    if (head->BE  != setup_phys + 7)
        { fprintf(stderr,"FAIL: SETUP BE\n"); return 1; }

    /* DATA TD */
    struct ohci_td *data_td = ohci_dma_virt_from_phys(&dma, head->NextTD);
    if (!data_td) { fprintf(stderr,"FAIL: DATA td phys->virt\n"); return 1; }
    if ((data_td->Control & OHCI_TD_DP_MASK) != OHCI_TD_DP_IN)
        { fprintf(stderr,"FAIL: DATA DP\n"); return 1; }
    if ((data_td->Control & OHCI_TD_T_MASK) != OHCI_TD_T_DATA1)
        { fprintf(stderr,"FAIL: DATA toggle\n"); return 1; }
    if (!(data_td->Control & OHCI_TD_R))
        { fprintf(stderr,"FAIL: DATA should allow short packets\n"); return 1; }
    if (data_td->CBP != data_phys || data_td->BE != data_phys + 17)
        { fprintf(stderr,"FAIL: DATA CBP/BE\n"); return 1; }

    /* STATUS TD (opposite direction, zero-length) */
    struct ohci_td *status_td = ohci_dma_virt_from_phys(&dma, data_td->NextTD);
    if (!status_td) { fprintf(stderr,"FAIL: STATUS td phys->virt\n"); return 1; }
    if ((status_td->Control & OHCI_TD_DP_MASK) != OHCI_TD_DP_OUT)
        { fprintf(stderr,"FAIL: STATUS DP for IN control\n"); return 1; }
    if ((status_td->Control & OHCI_TD_T_MASK) != OHCI_TD_T_DATA1)
        { fprintf(stderr,"FAIL: STATUS toggle\n"); return 1; }
    if (status_td->CBP != 0 || status_td->BE != 0)
        { fprintf(stderr,"FAIL: STATUS CBP/BE should be zero\n"); return 1; }

    /* Tail returned by builder should match the STATUS TD */
    if (tail != status_td) { fprintf(stderr,"FAIL: tail != status_td\n"); return 1; }

    printf("PASS: control-in chain build\n");
    return 0;
}
