#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_urb.h"
#include "ohci_bulk.h"
#include "ohci_drain.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int completed;
static void on_done(struct ohci_urb *u) { (void)u; completed++; }

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xB0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 32,
        .control_ed_count=1, .bulk_ed_count=2, .interrupt_ed_count=1 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    struct ohci_bulk_endpoint ep;
    struct ohci_bulk_endpoint_config epcfg = {
        .func_addr = 1, .ep_num = 2, .max_packet_size = 64,
        .direction = OHCI_URB_DIR_IN, .low_speed = 0
    };
    if (ohci_bulk_endpoint_create(&hc, &epcfg, &ep) != 0) {
        fprintf(stderr,"FAIL: bulk ep_create\n"); return 1;
    }

    /* HcBulkHeadED must equal the new ED. */
    if (ops.read32(ops.context, 0x28) != ep.ed_phys) {
        fprintf(stderr,"FAIL: HcBulkHeadED != ep.ed_phys\n"); return 1;
    }
    /* ED direction = IN. */
    if ((ep.ed->Control & OHCI_ED_D_MASK) != OHCI_ED_D_IN) {
        fprintf(stderr,"FAIL: bulk ED D != IN\n"); return 1;
    }

    uint32_t buf_phys; uint8_t *buf = ohci_dma_alloc(&dma, 512, 4, &buf_phys);
    memset(buf, 0, 512);

    struct ohci_urb urb = {0};
    urb.buffer = buf; urb.buffer_phys = buf_phys; urb.length = 512;
    urb.direction = OHCI_URB_DIR_IN;
    urb.complete = on_done;

    if (ohci_bulk_submit(&hc, &ep, &urb) != 0) { fprintf(stderr,"FAIL: submit\n"); return 1; }

    /* BLF must be asserted. */
    if ((ops.read32(ops.context, 0x08) & OHCI_CMD_BLF) == 0) {
        fprintf(stderr,"FAIL: BLF not set\n"); return 1;
    }

    fake_hc_exec_step(&fake);
    ohci_drain_done(&hc);

    if (completed != 1) { fprintf(stderr,"FAIL: completed=%d\n", completed); return 1; }
    if (urb.status != OHCI_URB_STATUS_OK) {
        fprintf(stderr,"FAIL: urb.status=%d\n", urb.status); return 1;
    }

    printf("PASS: Bulk IN end-to-end\n");
    return 0;
}
