/* OHCI 1.0a Host Controller Communications Area — section 4.4 */
#ifndef OHCI_HCCA_H
#define OHCI_HCCA_H

#include "ohci_types.h"

#pragma pack(push, 1)
struct ohci_hcca {
    uint32_t InterruptTable[32]; /* 0x000..0x07F */
    uint16_t FrameNumber;        /* 0x080 */
    uint16_t PadFrameNumber;     /* 0x082 */
    uint32_t DoneHead;           /* 0x084 */
    uint8_t  Reserved[116];      /* 0x088..0x0FB */
    uint8_t  VendorPrivate[4];   /* 0x0FC..0x0FF */
};
#pragma pack(pop)

#endif /* OHCI_HCCA_H */
