#ifndef FAKE_HC_H
#define FAKE_HC_H

#include <stdint.h>
#include "ohci_mmio.h"

/* 4 KB of shadow register space, which covers OHCI's operational register
 * file with headroom for root-hub port registers on a reasonable NDP. */
#define FAKE_HC_MMIO_SIZE 0x1000

struct fake_hc {
    uint8_t  regs[FAKE_HC_MMIO_SIZE];
    uint32_t barrier_calls;
};

void fake_hc_init(struct fake_hc *hc);
void fake_hc_get_ops(struct fake_hc *hc, struct ohci_mmio_ops *ops_out);

#endif /* FAKE_HC_H */
