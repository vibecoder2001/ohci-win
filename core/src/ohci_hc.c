#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"

/* Negative error codes used internally. Kept small and symbolic. */
#define OHCI_ERR_NOMEM      -1
#define OHCI_ERR_RESET_FAIL -2

/* Register offsets — mirror ohci_regs.h for readability. */
#define REG_HcControl          0x04
#define REG_HcCommandStatus    0x08
#define REG_HcInterruptStatus  0x0C
#define REG_HcInterruptEnable  0x10
#define REG_HcHCCA             0x18
#define REG_HcControlHeadED    0x20
#define REG_HcBulkHeadED       0x28
#define REG_HcFmInterval       0x34
#define REG_HcPeriodicStart    0x40

static uint32_t r32(struct ohci_hc *hc, uint32_t off) {
    return hc->ops.read32(hc->ops.context, off);
}
static void w32(struct ohci_hc *hc, uint32_t off, uint32_t v) {
    hc->ops.write32(hc->ops.context, off, v);
}
static void barrier(struct ohci_hc *hc) {
    hc->ops.barrier(hc->ops.context);
}

int ohci_hc_init(struct ohci_hc *hc,
                 const struct ohci_mmio_ops *ops,
                 struct ohci_dma_region *dma,
                 uint16_t td_pool_size) {
    memset(hc, 0, sizeof(*hc));
    hc->ops = *ops;
    hc->dma = dma;
    if (ohci_td_pool_init(&hc->td_pool, dma, td_pool_size) != 0) return OHCI_ERR_NOMEM;
    hc->control_head = NULL;

    /* 1) Reset: write HCR, wait for hardware to clear it. The fake HC
     * clears it synchronously; real hardware typically < 10 µs. */
    w32(hc, REG_HcCommandStatus, OHCI_CMD_HCR);
    barrier(hc);
    for (int i = 0; i < 1000; i++) {
        if (!(r32(hc, REG_HcCommandStatus) & OHCI_CMD_HCR)) break;
    }
    if (r32(hc, REG_HcCommandStatus) & OHCI_CMD_HCR) return OHCI_ERR_RESET_FAIL;

    /* 2) Allocate HCCA — 256 bytes, 256-byte aligned (§4.4). */
    uint32_t phys;
    void *v = ohci_dma_alloc(dma, sizeof(struct ohci_hcca), 256, &phys);
    if (!v) return OHCI_ERR_NOMEM;
    hc->hcca      = (struct ohci_hcca *)v;
    hc->hcca_phys = phys;
    memset(hc->hcca, 0, sizeof(*hc->hcca));

    /* 3) Program HcHCCA + empty Control/Bulk list heads. */
    w32(hc, REG_HcHCCA,          hc->hcca_phys);
    w32(hc, REG_HcControlHeadED, 0);
    w32(hc, REG_HcBulkHeadED,    0);

    /* 4) Program frame timing — values per §7.3. The fake HC seeds these
     * already, but a real controller needs them after reset. */
    w32(hc, REG_HcFmInterval,   0x2EDF);        /* 11999 */
    w32(hc, REG_HcPeriodicStart, (0x2EDF * 9) / 10); /* 90% of FI */

    /* 5) Clear any stale interrupt status. */
    w32(hc, REG_HcInterruptStatus, 0xFFFFFFFFu);

    /* 6) Enable WDH + master interrupt enable. */
    w32(hc, REG_HcInterruptEnable, OHCI_INT_WDH | OHCI_INT_MIE);

    /* 7) Enable Control-list processing, IE; leave PLE and BLE off. */
    uint32_t ctrl = r32(hc, REG_HcControl);
    ctrl &= ~(OHCI_CTRL_HCFS_MASK | OHCI_CTRL_PLE | OHCI_CTRL_BLE);
    ctrl |= OHCI_CTRL_CLE | OHCI_CTRL_IE | OHCI_CTRL_HCFS_OPER;
    barrier(hc);
    w32(hc, REG_HcControl, ctrl);

    return 0;
}
