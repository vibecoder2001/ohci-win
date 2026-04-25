#ifndef OHCI_URB_H
#define OHCI_URB_H

#include "ohci_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Direction for Control-data stage and non-Control transfers. */
#define OHCI_URB_DIR_OUT 0
#define OHCI_URB_DIR_IN  1

/* Completion status. Mirrors a subset of OHCI condition codes, translated
 * to positive semantics. Plan 3+ may add more. */
#define OHCI_URB_STATUS_PENDING  -1
#define OHCI_URB_STATUS_OK        0
#define OHCI_URB_STATUS_STALL     1
#define OHCI_URB_STATUS_CRC       2
#define OHCI_URB_STATUS_TIMEOUT   3
#define OHCI_URB_STATUS_OVERRUN   4
#define OHCI_URB_STATUS_UNDERRUN  5
#define OHCI_URB_STATUS_OTHER     6

struct ohci_urb;
typedef void (*ohci_urb_complete_fn)(struct ohci_urb *urb);

struct ohci_urb {
    /* Control-transfer setup packet (ignored for non-Control). */
    uint8_t  setup[8];
    uint32_t setup_phys;

    /* Data buffer for the Data stage (Control) or the entire transfer
     * (Bulk/Interrupt later). NULL + length=0 means "no data stage". */
    void    *buffer;
    uint32_t buffer_phys;
    uint32_t length;

    /* 0=OUT, 1=IN. For Control: direction of the Data stage. */
    uint8_t  direction;

    /* Completion. Set by core on retirement. status=PENDING while queued. */
    int32_t  status;
    uint32_t transferred;
    ohci_urb_complete_fn complete;
    void    *context;

    /* Internal — populated by the submit path. */
    struct ohci_ed *ed;
    struct ohci_td *head_td;
    struct ohci_td *tail_td;
    struct ohci_urb *next_pending;
};

#ifdef __cplusplus
}
#endif

#endif /* OHCI_URB_H */
