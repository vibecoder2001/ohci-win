#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_mmio.h"
#include "fake_hc.h"

int main(void) {
    struct fake_hc hc;
    fake_hc_init(&hc);

    struct ohci_mmio_ops ops;
    fake_hc_get_ops(&hc, &ops);

    /* OHCI 1.0a: HcRevision should read 0x10 at reset */
    uint32_t rev = ops.read32(ops.context, 0x00);
    if (rev != 0x10) {
        fprintf(stderr, "FAIL: HcRevision = 0x%x, expected 0x10\n", rev);
        return 1;
    }

    /* Writing HCR in HcCommandStatus should simulate controller reset:
     * the hardware clears it once reset is complete (we simulate instantaneously). */
    ops.write32(ops.context, 0x08, OHCI_CMD_HCR);
    uint32_t cmd = ops.read32(ops.context, 0x08);
    if (cmd & OHCI_CMD_HCR) {
        fprintf(stderr, "FAIL: HCR not auto-cleared by fake HC\n");
        return 1;
    }

    printf("PASS: fake_hc reset and readback\n");
    return 0;
}
