/* OHCI 1.0a operational registers — section 7 */
#ifndef OHCI_REGS_H
#define OHCI_REGS_H

#include <stdint.h>

#pragma pack(push, 1)
struct ohci_regs {
    volatile uint32_t HcRevision;          /* 0x00 */
    volatile uint32_t HcControl;           /* 0x04 */
    volatile uint32_t HcCommandStatus;     /* 0x08 */
    volatile uint32_t HcInterruptStatus;   /* 0x0C */
    volatile uint32_t HcInterruptEnable;   /* 0x10 */
    volatile uint32_t HcInterruptDisable;  /* 0x14 */
    volatile uint32_t HcHCCA;              /* 0x18 */
    volatile uint32_t HcPeriodCurrentED;   /* 0x1C */
    volatile uint32_t HcControlHeadED;     /* 0x20 */
    volatile uint32_t HcControlCurrentED;  /* 0x24 */
    volatile uint32_t HcBulkHeadED;        /* 0x28 */
    volatile uint32_t HcBulkCurrentED;     /* 0x2C */
    volatile uint32_t HcDoneHead;          /* 0x30 */
    volatile uint32_t HcFmInterval;        /* 0x34 */
    volatile uint32_t HcFmRemaining;       /* 0x38 */
    volatile uint32_t HcFmNumber;          /* 0x3C */
    volatile uint32_t HcPeriodicStart;     /* 0x40 */
    volatile uint32_t HcLSThreshold;       /* 0x44 */
    volatile uint32_t HcRhDescriptorA;     /* 0x48 */
    volatile uint32_t HcRhDescriptorB;     /* 0x4C */
    volatile uint32_t HcRhStatus;          /* 0x50 */
    volatile uint32_t HcRhPortStatus[1];   /* 0x54..0x54+4*(N-1); N from HcRhDescriptorA.NDP */
};
#pragma pack(pop)

/* HcControl bits */
#define OHCI_CTRL_CBSR_MASK   0x00000003u
#define OHCI_CTRL_PLE         0x00000004u
#define OHCI_CTRL_IE          0x00000008u
#define OHCI_CTRL_CLE         0x00000010u
#define OHCI_CTRL_BLE         0x00000020u
#define OHCI_CTRL_HCFS_MASK   0x000000C0u
#define OHCI_CTRL_HCFS_RESET  0x00000000u
#define OHCI_CTRL_HCFS_RESUME 0x00000040u
#define OHCI_CTRL_HCFS_OPER   0x00000080u
#define OHCI_CTRL_HCFS_SUSP   0x000000C0u
#define OHCI_CTRL_IR          0x00000100u
#define OHCI_CTRL_RWC         0x00000200u
#define OHCI_CTRL_RWE         0x00000400u

/* HcCommandStatus bits */
#define OHCI_CMD_HCR          0x00000001u
#define OHCI_CMD_CLF          0x00000002u
#define OHCI_CMD_BLF          0x00000004u
#define OHCI_CMD_OCR          0x00000008u

/* HcInterruptStatus / Enable / Disable bits */
#define OHCI_INT_SO           0x00000001u
#define OHCI_INT_WDH          0x00000002u
#define OHCI_INT_SF           0x00000004u
#define OHCI_INT_RD           0x00000008u
#define OHCI_INT_UE           0x00000010u
#define OHCI_INT_FNO          0x00000020u
#define OHCI_INT_RHSC         0x00000040u
#define OHCI_INT_OC           0x40000000u
#define OHCI_INT_MIE          0x80000000u

#endif /* OHCI_REGS_H */
