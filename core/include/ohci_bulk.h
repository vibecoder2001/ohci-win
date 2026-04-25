#ifndef OHCI_BULK_H
#define OHCI_BULK_H

#include <stdint.h>
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_urb.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_hc;

struct ohci_bulk_endpoint {
    struct ohci_ed *ed;
    uint32_t        ed_phys;
    struct ohci_td *tail_placeholder;
    uint32_t        tail_placeholder_phys;
    uint8_t         direction;   /* OHCI_URB_DIR_IN/OUT — fixed at create */
    struct ohci_bulk_endpoint *next;
};

struct ohci_bulk_endpoint_config {
    uint8_t  func_addr;
    uint8_t  ep_num;          /* 1..15 for Bulk */
    uint16_t max_packet_size;
    uint8_t  direction;       /* OHCI_URB_DIR_IN or OHCI_URB_DIR_OUT */
    uint8_t  low_speed;       /* Bulk is never low-speed in real USB, but param exists for symmetry */
};

int ohci_bulk_endpoint_create(struct ohci_hc *hc,
                              const struct ohci_bulk_endpoint_config *cfg,
                              struct ohci_bulk_endpoint *ep);

/* Submit a Bulk URB. Splits the URB buffer into one or more General TDs
 * each covering at most 4 KB, chained via NextTD. urb->direction is
 * ignored for Bulk — direction comes from the ED. */
int ohci_bulk_submit(struct ohci_hc *hc,
                     struct ohci_bulk_endpoint *ep,
                     struct ohci_urb *urb);

void ohci_bulk_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_bulk_endpoint *ep);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_BULK_H */
