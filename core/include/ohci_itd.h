/* OHCI 1.0a Isochronous Transfer Descriptor — §4.3.2 */
#ifndef OHCI_ITD_H
#define OHCI_ITD_H
#include "ohci_types.h"

#pragma pack(push, 4)
struct ohci_itd {
    uint32_t Control;     /* CC[31:28] FC[26:24] DI[23:21] SF[15:0] */
    uint32_t BP0;         /* page-aligned base (top 20 bits used) */
    uint32_t NextTD;
    uint32_t BE;          /* physical address of last byte */
    uint16_t PSW[8];      /* per-packet offset-in / status-out */
};
#pragma pack(pop)

#define OHCI_ITD_SF_MASK    0x0000FFFFu
#define OHCI_ITD_DI_SHIFT   21
#define OHCI_ITD_DI_MASK    (0x07u << 21)
#define OHCI_ITD_DI_NO_INTR (0x07u << 21)
#define OHCI_ITD_FC_SHIFT   24
#define OHCI_ITD_FC_MASK    (0x07u << 24)   /* packet_count - 1 */
#define OHCI_ITD_CC_SHIFT   28
#define OHCI_ITD_CC_MASK    (0x0Fu << 28)

/* PSW: in = offset[12:0] from BP0; out = CC[15:12] | size[11:0] */
#define OHCI_PSW_CC_SHIFT   12
#define OHCI_PSW_CC_MASK    0xF000u
#define OHCI_PSW_SIZE_MASK  0x0FFFu

#endif /* OHCI_ITD_H */
