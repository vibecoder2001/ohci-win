#include <stddef.h>
#include <stdio.h>
#include "ohci_regs.h"
#include "ohci_hcca.h"

static int check_offset(const char *type_name, const char *field_name, size_t got, size_t expected) {
    if (got != expected) {
        fprintf(stderr, "FAIL: offsetof(%s,%s) = %zu, expected %zu\n", type_name, field_name, got, expected);
        return 1;
    }
    return 0;
}

static int check_size(const char *type_name, size_t got, size_t expected) {
    if (got != expected) {
        fprintf(stderr, "FAIL: sizeof(%s) = %zu, expected %zu\n", type_name, got, expected);
        return 1;
    }
    return 0;
}

#define EXPECT_OFFSET(type, field, expected) \
    check_offset(#type, #field, offsetof(type, field), expected)

#define EXPECT_SIZE(type, expected) \
    check_size(#type, sizeof(type), expected)

int main(void) {
    int ret = 0;
    /* OHCI 1.0a section 7: operational register offsets */
    if (EXPECT_OFFSET(struct ohci_regs, HcRevision,          0x00)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcControl,           0x04)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcCommandStatus,     0x08)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcInterruptStatus,   0x0C)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcInterruptEnable,   0x10)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcInterruptDisable,  0x14)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcHCCA,              0x18)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcPeriodCurrentED,   0x1C)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcControlHeadED,     0x20)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcControlCurrentED,  0x24)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcBulkHeadED,        0x28)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcBulkCurrentED,     0x2C)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcDoneHead,          0x30)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcFmInterval,        0x34)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcFmRemaining,       0x38)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcFmNumber,          0x3C)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcPeriodicStart,     0x40)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcLSThreshold,       0x44)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcRhDescriptorA,     0x48)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcRhDescriptorB,     0x4C)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcRhStatus,          0x50)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_regs, HcRhPortStatus,      0x54)) ret = 1;

    /* OHCI 1.0a section 4.4: HCCA is 256 bytes */
    if (EXPECT_SIZE(struct ohci_hcca, 256)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_hcca, InterruptTable,      0x00)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_hcca, FrameNumber,         0x80)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_hcca, PadFrameNumber,      0x82)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_hcca, DoneHead,            0x84)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_hcca, Reserved,            0x88)) ret = 1;

    if (ret == 0) {
        printf("PASS: register + HCCA layouts verified\n");
    }
    return ret;
}
