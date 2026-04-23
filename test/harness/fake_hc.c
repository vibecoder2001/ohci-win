#include <string.h>
#include "fake_hc.h"
#include "ohci_regs.h"

static uint32_t read32(void *ctx, uint32_t offset) {
    struct fake_hc *hc = (struct fake_hc *)ctx;
    uint32_t v;
    memcpy(&v, hc->regs + offset, sizeof(v));
    return v;
}

static void write32(void *ctx, uint32_t offset, uint32_t value) {
    struct fake_hc *hc = (struct fake_hc *)ctx;
    /* Simulate a handful of hardware side effects so core code that polls
     * these bits terminates. Extend as needed. */
    if (offset == 0x08 /* HcCommandStatus */) {
        /* HCR is W1S in real HC; we clear it immediately to simulate a
         * completed reset. */
        value &= ~OHCI_CMD_HCR;
    }
    memcpy(hc->regs + offset, &value, sizeof(value));
}

static void barrier(void *ctx) {
    struct fake_hc *hc = (struct fake_hc *)ctx;
    hc->barrier_calls++;
}

void fake_hc_init(struct fake_hc *hc) {
    memset(hc, 0, sizeof(*hc));
    /* Seed post-reset register values per OHCI 1.0a §7. */
    uint32_t rev = 0x10;
    memcpy(hc->regs + 0x00, &rev, sizeof(rev));
    uint32_t fm_interval = 0x2EDF; /* 11999 — standard 1 ms frame */
    memcpy(hc->regs + 0x34, &fm_interval, sizeof(fm_interval));
    uint32_t ls_thresh = 0x0628;   /* 1576 */
    memcpy(hc->regs + 0x44, &ls_thresh, sizeof(ls_thresh));
}

void fake_hc_get_ops(struct fake_hc *hc, struct ohci_mmio_ops *ops_out) {
    ops_out->context = hc;
    ops_out->read32  = read32;
    ops_out->write32 = write32;
    ops_out->barrier = barrier;
}
