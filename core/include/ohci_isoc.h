#ifndef OHCI_ISOC_H
#define OHCI_ISOC_H

#include "ohci_types.h"
#include "ohci_ed.h"
#include "ohci_itd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_hc;

struct ohci_isoc_endpoint_config {
    uint8_t  func_addr;
    uint8_t  ep_num;
    uint8_t  direction;        /* OHCI_URB_DIR_IN / OHCI_URB_DIR_OUT */
    uint8_t  low_speed;
    uint16_t max_packet_size;
};

struct ohci_isoc_endpoint {
    struct ohci_ed   *ed;
    uint32_t          ed_phys;
    struct ohci_itd  *tail_placeholder;
    uint32_t          tail_placeholder_phys;
    uint16_t          max_packet_size;
    uint8_t           direction;
    /* Refill bookkeeping — populated by submit/refill paths in later tasks. */
    uint16_t          ed_tail_frame;     /* SF + FC + 1 of last queued ITD */
    uint8_t           primed;
    struct ohci_isoc_endpoint *next;
};

int  ohci_isoc_endpoint_create(struct ohci_hc *hc,
                               const struct ohci_isoc_endpoint_config *cfg,
                               struct ohci_isoc_endpoint *ep);
void ohci_isoc_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_isoc_endpoint *ep);

#ifdef __cplusplus
}
#endif

#endif
