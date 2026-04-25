/* OHCI 1.0a General Transfer Descriptor — §4.3 */
#ifndef OHCI_TD_H
#define OHCI_TD_H

#include "ohci_types.h"

#pragma pack(push, 4)
struct ohci_td {
    /* Control word — CC/EC/T/DI/DP/R */
    uint32_t Control;
    /* Current buffer pointer (physical); 0 when no data stage */
    uint32_t CBP;
    /* Physical address of next TD in queue (4-byte aligned) */
    uint32_t NextTD;
    /* Physical address of the last byte of the buffer (inclusive) */
    uint32_t BE;
};
#pragma pack(pop)

/* TD Control word bit layout (§4.3.1) */
#define OHCI_TD_R            (1u << 18)  /* bufferRounding — allow short packet */
#define OHCI_TD_DP_SHIFT     19
#define OHCI_TD_DP_MASK      (0x03u << 19)
#define OHCI_TD_DP_SETUP     (0x00u << 19)
#define OHCI_TD_DP_OUT       (0x01u << 19)
#define OHCI_TD_DP_IN        (0x02u << 19)
#define OHCI_TD_DP_RESERVED  (0x03u << 19)
#define OHCI_TD_DI_SHIFT     21
#define OHCI_TD_DI_MASK      (0x07u << 21) /* Delay interrupt (7 = no IOC) */
#define OHCI_TD_DI_NO_INTR   (0x07u << 21)
#define OHCI_TD_DI_IMMEDIATE (0x00u << 21)
#define OHCI_TD_T_SHIFT      24
#define OHCI_TD_T_MASK       (0x03u << 24) /* Toggle: 00/01 = take from ED.C, 10=DATA0, 11=DATA1 */
#define OHCI_TD_T_FROM_ED    (0x00u << 24)
#define OHCI_TD_T_DATA0      (0x02u << 24)
#define OHCI_TD_T_DATA1      (0x03u << 24)
#define OHCI_TD_EC_SHIFT     26
#define OHCI_TD_EC_MASK      (0x03u << 26) /* Error count (HW-written) */
#define OHCI_TD_CC_SHIFT     28
#define OHCI_TD_CC_MASK      (0x0Fu << 28) /* Condition code (HW-written) */

/* Common condition codes (§4.3.3) */
#define OHCI_CC_NOERROR            0x0
#define OHCI_CC_CRC                0x1
#define OHCI_CC_BITSTUFFING        0x2
#define OHCI_CC_DATATOGGLEMISMATCH 0x3
#define OHCI_CC_STALL              0x4
#define OHCI_CC_DEVICENOTRESPONDING 0x5
#define OHCI_CC_PIDCHECKFAILURE    0x6
#define OHCI_CC_UNEXPECTEDPID      0x7
#define OHCI_CC_DATAOVERRUN        0x8
#define OHCI_CC_DATAUNDERRUN       0x9
#define OHCI_CC_BUFFEROVERRUN      0xC
#define OHCI_CC_BUFFERUNDERRUN     0xD
#define OHCI_CC_NOTACCESSED        0xF  /* HW default until TD is retired */

#endif /* OHCI_TD_H */
