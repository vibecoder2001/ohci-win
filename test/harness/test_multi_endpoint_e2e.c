#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "ohci_bulk.h"
#include "ohci_interrupt.h"
#include "ohci_drain.h"
#include "ohci_dma.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int done_ctrl, done_bin, done_bout, done_int;
static void cb_ctrl(struct ohci_urb *u) { (void)u; done_ctrl++; }
static void cb_bin (struct ohci_urb *u) { (void)u; done_bin++;  }
static void cb_bout(struct ohci_urb *u) { (void)u; done_bout++; }
static void cb_int (struct ohci_urb *u) { (void)u; done_int++;  }

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[256 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xF0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 64,
        .control_ed_count=2, .bulk_ed_count=4, .interrupt_ed_count=2 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    /* Create all four endpoint types. */
    struct ohci_control_endpoint cep;
    struct ohci_control_endpoint_config ccfg = { .max_packet_size = 8 };
    ohci_control_endpoint_create(&hc, &ccfg, &cep);

    struct ohci_bulk_endpoint bep_in, bep_out;
    struct ohci_bulk_endpoint_config bin  = { .func_addr=4,.ep_num=1,
        .max_packet_size=64, .direction=OHCI_URB_DIR_IN };
    struct ohci_bulk_endpoint_config bout = { .func_addr=4,.ep_num=2,
        .max_packet_size=64, .direction=OHCI_URB_DIR_OUT };
    ohci_bulk_endpoint_create(&hc, &bin,  &bep_in);
    ohci_bulk_endpoint_create(&hc, &bout, &bep_out);

    struct ohci_interrupt_endpoint iep;
    struct ohci_interrupt_endpoint_config icfg = { .func_addr=4,.ep_num=3,
        .max_packet_size=8, .direction=OHCI_URB_DIR_IN,
        .poll_interval_frames=32 };
    ohci_interrupt_endpoint_create(&hc, &icfg, &iep);

    /* Submit one URB on each. */
    uint32_t sc_p; uint8_t *sc = ohci_dma_alloc(&dma, 8, 4, &sc_p);
    uint32_t dc_p; uint8_t *dc = ohci_dma_alloc(&dma, 8, 4, &dc_p);
    sc[0]=0x80; sc[1]=0x06; sc[2]=0x00; sc[3]=0x01;
    sc[4]=0x00; sc[5]=0x00; sc[6]=0x08; sc[7]=0x00;
    struct ohci_urb uc = {0};
    memcpy(uc.setup, sc, 8);
    uc.setup_phys = sc_p;
    uc.buffer = dc; uc.buffer_phys = dc_p; uc.length = 8;
    uc.direction = OHCI_URB_DIR_IN; uc.complete = cb_ctrl;
    ohci_control_submit(&hc, &cep, &uc);

    uint32_t bi_p; uint8_t *bi = ohci_dma_alloc(&dma, 256, 4, &bi_p);
    struct ohci_urb ubi = {0};
    ubi.buffer = bi; ubi.buffer_phys = bi_p; ubi.length = 256;
    ubi.direction = OHCI_URB_DIR_IN; ubi.complete = cb_bin;
    ohci_bulk_submit(&hc, &bep_in, &ubi);

    uint32_t bo_p; uint8_t *bo = ohci_dma_alloc(&dma, 512, 4, &bo_p);
    struct ohci_urb ubo = {0};
    ubo.buffer = bo; ubo.buffer_phys = bo_p; ubo.length = 512;
    ubo.direction = OHCI_URB_DIR_OUT; ubo.complete = cb_bout;
    ohci_bulk_submit(&hc, &bep_out, &ubo);

    uint32_t ii_p; uint8_t *ii = ohci_dma_alloc(&dma, 8, 4, &ii_p);
    struct ohci_urb uii = {0};
    uii.buffer = ii; uii.buffer_phys = ii_p; uii.length = 8;
    uii.direction = OHCI_URB_DIR_IN; uii.complete = cb_int;
    ohci_interrupt_submit(&hc, &iep, &uii);

    /* Run frames until the Interrupt URB completes (Control/Bulk complete
     * in one step; Interrupt requires the right frame slot). */
    for (int i = 0; i < 64 && !done_int; i++) {
        fake_hc_exec_step(&fake);
        ohci_drain_done(&hc);
    }

    if (done_ctrl != 1) { fprintf(stderr,"FAIL: ctrl not done\n"); return 1; }
    if (done_bin  != 1) { fprintf(stderr,"FAIL: bulk-in not done\n"); return 1; }
    if (done_bout != 1) { fprintf(stderr,"FAIL: bulk-out not done\n"); return 1; }
    if (done_int  != 1) { fprintf(stderr,"FAIL: int not done\n"); return 1; }

    /* Teardown all endpoints. */
    ohci_interrupt_endpoint_destroy(&hc, &iep);
    ohci_bulk_endpoint_destroy(&hc, &bep_out);
    ohci_bulk_endpoint_destroy(&hc, &bep_in);
    ohci_control_endpoint_destroy(&hc, &cep);

    /* Verify all lists emptied to pre-endpoint state. */
    if (ops.read32(ops.context, 0x20) != 0) { fprintf(stderr,"FAIL: ctrl list\n"); return 1; }
    if (ops.read32(ops.context, 0x28) != 0) { fprintf(stderr,"FAIL: bulk list\n"); return 1; }

    printf("PASS: multi-endpoint lifecycle\n");
    return 0;
}
