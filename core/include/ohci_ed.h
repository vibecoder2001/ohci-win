/* OHCI 1.0a Endpoint Descriptor — §4.2 */
#ifndef OHCI_ED_H
#define OHCI_ED_H

#include "ohci_types.h"

#pragma pack(push, 4)
struct ohci_ed {
    /* Control word — funcAddr/EN/D/S/K/F/MPS */
    uint32_t Control;
    /* Physical address of ED's TD queue tail (low 4 bits reserved) */
    uint32_t TailP;
    /* Physical address of current TD; low bits H (Halted), C (toggle Carry) */
    uint32_t HeadP;
    /* Physical address of next ED in list */
    uint32_t NextED;
};
#pragma pack(pop)

/* ED Control word bit layout (§4.2.2) */
#define OHCI_ED_FA_SHIFT     0
#define OHCI_ED_FA_MASK      (0x7Fu << 0)    /* Function address */
#define OHCI_ED_EN_SHIFT     7
#define OHCI_ED_EN_MASK      (0x0Fu << 7)    /* Endpoint number */
#define OHCI_ED_D_SHIFT      11
#define OHCI_ED_D_MASK       (0x03u << 11)   /* Direction: 00=from TD, 01=OUT, 10=IN, 11=from TD */
#define OHCI_ED_D_FROM_TD    (0x00u << 11)
#define OHCI_ED_D_OUT        (0x01u << 11)
#define OHCI_ED_D_IN         (0x02u << 11)
#define OHCI_ED_S            (1u << 13)      /* Speed: 0=Full, 1=Low */
#define OHCI_ED_K            (1u << 14)      /* sKip */
#define OHCI_ED_F            (1u << 15)      /* Format: 0=General, 1=Isochronous */
#define OHCI_ED_MPS_SHIFT    16
#define OHCI_ED_MPS_MASK     (0x07FFu << 16) /* Max packet size */

/* ED HeadP low-bit flags */
#define OHCI_ED_HEADP_H      (1u << 0)       /* Halted */
#define OHCI_ED_HEADP_C      (1u << 1)       /* toggle Carry */
#define OHCI_ED_HEADP_ADDR_MASK  0xFFFFFFF0u /* 16-byte aligned TD phys */

#endif /* OHCI_ED_H */
