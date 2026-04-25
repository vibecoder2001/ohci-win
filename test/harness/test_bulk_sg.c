#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_urb.h"
#include "ohci_bulk.h"
#include "ohci_drain.h"
#include "ohci_dma.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

static int completed;
static void on_done(struct ohci_urb *u) { (void)u; completed++; }

int main(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[128 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size = 64,
        .control_ed_count=1, .bulk_ed_count=1, .interrupt_ed_count=1 };
    ohci_hc_init(&hc, &ops, &dma, &hcfg);

    struct ohci_bulk_endpoint ep;
    struct ohci_bulk_endpoint_config epcfg = {
        .func_addr = 2, .ep_num = 1, .max_packet_size = 512,
        .direction = OHCI_URB_DIR_OUT
    };
    ohci_bulk_endpoint_create(&hc, &epcfg, &ep);

    /* Allocate a 10 KB buffer aligned to 4 KB so split points fall at
     * 4 KB boundaries. */
    uint32_t buf_phys; uint8_t *buf = ohci_dma_alloc(&dma, 10 * 1024, 4096, &buf_phys);
    memset(buf, 0xA5, 10 * 1024);

    struct ohci_urb urb = {0};
    urb.buffer = buf; urb.buffer_phys = buf_phys; urb.length = 10 * 1024;
    urb.direction = OHCI_URB_DIR_OUT;
    urb.complete = on_done;

    ohci_bulk_submit(&hc, &ep, &urb);

    /* Walk the queue from ED.HeadP to count TDs excluding placeholder.
     * 10 KB / 4 KB per TD = 3 TDs (4 + 4 + 2). */
    uint32_t cur = ep.ed->HeadP & OHCI_ED_HEADP_ADDR_MASK;
    uint32_t tail = ep.ed->TailP;
    int tds = 0;
    while (cur && cur != tail && tds < 10) {
        struct ohci_td *td = ohci_dma_virt_from_phys(&dma, cur);
        if (!td) break;
        cur = td->NextTD;
        tds++;
    }
    if (tds != 3) { fprintf(stderr,"FAIL: expected 3 TDs, got %d\n", tds); return 1; }

    fake_hc_exec_step(&fake);
    ohci_drain_done(&hc);

    if (completed != 1) { fprintf(stderr,"FAIL: completed=%d\n", completed); return 1; }
    if (urb.status != OHCI_URB_STATUS_OK) {
        fprintf(stderr,"FAIL: status=%d\n", urb.status); return 1;
    }

    printf("PASS: Bulk SG 10 KB split into 3 TDs\n");
    return 0;
}
