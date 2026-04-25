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
                 const struct ohci_hc_config *cfg) {
    memset(hc, 0, sizeof(*hc));
    hc->ops = *ops;
    hc->dma = dma;

    if (ohci_td_pool_init(&hc->td_pool, dma, cfg->td_pool_size)   != 0) return OHCI_ERR_NOMEM;
    if (ohci_ed_pool_init(&hc->control_ed_pool,   dma, cfg->control_ed_count)   != 0) return OHCI_ERR_NOMEM;
    if (ohci_ed_pool_init(&hc->bulk_ed_pool,      dma, cfg->bulk_ed_count)      != 0) return OHCI_ERR_NOMEM;
    if (ohci_ed_pool_init(&hc->interrupt_ed_pool, dma, cfg->interrupt_ed_count) != 0) return OHCI_ERR_NOMEM;

    hc->control_head   = NULL;
    hc->bulk_head      = NULL;
    hc->interrupt_head = NULL;
    hc->in_flight      = NULL;

    /* 1) Reset. */
    w32(hc, REG_HcCommandStatus, OHCI_CMD_HCR);
    barrier(hc);
    for (int i = 0; i < 1000; i++) {
        if (!(r32(hc, REG_HcCommandStatus) & OHCI_CMD_HCR)) break;
    }
    if (r32(hc, REG_HcCommandStatus) & OHCI_CMD_HCR) return OHCI_ERR_RESET_FAIL;

    /* 2) Allocate HCCA, 256-byte aligned. */
    uint32_t phys;
    void *v = ohci_dma_alloc(dma, sizeof(struct ohci_hcca), 256, &phys);
    if (!v) return OHCI_ERR_NOMEM;
    hc->hcca      = (struct ohci_hcca *)v;
    hc->hcca_phys = phys;
    memset(hc->hcca, 0, sizeof(*hc->hcca));

    /* 3) Program HCCA + empty list heads + frame timing + interrupts. */
    w32(hc, REG_HcHCCA,            hc->hcca_phys);
    w32(hc, REG_HcControlHeadED,   0);
    w32(hc, REG_HcBulkHeadED,      0);
    w32(hc, REG_HcFmInterval,      0x2EDF);
    w32(hc, REG_HcPeriodicStart,   (0x2EDF * 9) / 10);
    w32(hc, REG_HcInterruptStatus, 0xFFFFFFFFu);
    w32(hc, REG_HcInterruptEnable, OHCI_INT_WDH | OHCI_INT_MIE);

    /* 4) Enable Control + Bulk lists; Periodic stays off until Task 5. */
    uint32_t ctrl = r32(hc, REG_HcControl);
    ctrl &= ~(OHCI_CTRL_HCFS_MASK | OHCI_CTRL_PLE);
    ctrl |= OHCI_CTRL_CLE | OHCI_CTRL_BLE | OHCI_CTRL_IE | OHCI_CTRL_HCFS_OPER;
    barrier(hc);
    w32(hc, REG_HcControl, ctrl);

    return 0;
}
