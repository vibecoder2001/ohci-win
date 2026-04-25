#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_ed.h"

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

static int build_interrupt_skeleton(struct ohci_hc *hc) {
    /* Allocate a dedicated ED pool for the 63 skeleton entries, distinct
     * from hc->interrupt_ed_pool which holds user endpoints. The static
     * keeps pool state across hc lifetime; for multi-hc scenarios in Plan 4+
     * this would move into struct ohci_hc. */
    static struct ohci_ed_pool skel_pool;
    if (ohci_ed_pool_init(&skel_pool, hc->dma, 63) != 0) return -1;

    uint32_t phys[63];
    for (int i = 0; i < 63; i++) {
        struct ohci_ed *ed = ohci_ed_pool_alloc(&skel_pool, &phys[i]);
        if (!ed) return -1;
        ed->Control = OHCI_ED_K;   /* Skip bit so HC never dispatches work */
        ed->HeadP   = 0;
        ed->TailP   = 0;
        ed->NextED  = 0;
        hc->interrupt_skeleton[i]      = ed;
        hc->interrupt_skeleton_phys[i] = phys[i];
    }

    /* Link children -> parent. Levels: 0..31 leaves, 32..47 L1,
     * 48..55 L2, 56..59 L3, 60..61 L4, 62 root. */
    for (int i = 0; i < 32; i++) hc->interrupt_skeleton[i]->NextED      = phys[32 + i / 2];
    for (int i = 0; i < 16; i++) hc->interrupt_skeleton[32+i]->NextED   = phys[48 + i / 2];
    for (int i = 0; i <  8; i++) hc->interrupt_skeleton[48+i]->NextED   = phys[56 + i / 2];
    for (int i = 0; i <  4; i++) hc->interrupt_skeleton[56+i]->NextED   = phys[60 + i / 2];
    for (int i = 0; i <  2; i++) hc->interrupt_skeleton[60+i]->NextED   = phys[62];
    hc->interrupt_skeleton[62]->NextED = 0;

    /* HCCA.InterruptTable[i] -> leaf[i]. */
    for (int i = 0; i < 32; i++) hc->hcca->InterruptTable[i] = phys[i];
    return 0;
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

    if (build_interrupt_skeleton(hc) != 0) return OHCI_ERR_NOMEM;

    /* 3) Program HCCA + empty list heads + frame timing + interrupts. */
    w32(hc, REG_HcHCCA,            hc->hcca_phys);
    w32(hc, REG_HcControlHeadED,   0);
    w32(hc, REG_HcBulkHeadED,      0);
    w32(hc, REG_HcFmInterval,      0x2EDF);
    w32(hc, REG_HcPeriodicStart,   (0x2EDF * 9) / 10);
    w32(hc, REG_HcInterruptStatus, 0xFFFFFFFFu);
    w32(hc, REG_HcInterruptEnable, OHCI_INT_WDH | OHCI_INT_MIE);

    /* 4) Enable Control + Bulk + Periodic lists. */
    uint32_t ctrl = r32(hc, REG_HcControl);
    ctrl &= ~OHCI_CTRL_HCFS_MASK;
    ctrl |= OHCI_CTRL_CLE | OHCI_CTRL_BLE | OHCI_CTRL_PLE | OHCI_CTRL_IE | OHCI_CTRL_HCFS_OPER;
    barrier(hc);
    w32(hc, REG_HcControl, ctrl);

    return 0;
}
