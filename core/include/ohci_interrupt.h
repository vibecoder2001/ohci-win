#ifndef OHCI_INTERRUPT_H
#define OHCI_INTERRUPT_H

#include "ohci_types.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_urb.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_hc;

struct ohci_interrupt_endpoint {
    struct ohci_ed *ed;
    uint32_t        ed_phys;
    struct ohci_td *tail_placeholder;
    uint32_t        tail_placeholder_phys;
    uint8_t         direction;
    uint8_t         slot_index;           /* Leaf index in HCCA.InterruptTable */
    uint16_t        poll_interval_frames; /* Plan 3 supports 32 only */
    struct ohci_interrupt_endpoint *next;
};

struct ohci_interrupt_endpoint_config {
    uint8_t  func_addr;
    uint8_t  ep_num;
    uint16_t max_packet_size;
    uint8_t  direction;
    uint8_t  low_speed;
    uint16_t poll_interval_frames;  /* desired period in frames; rounded down
                                     * to nearest power of two in {1,2,4,8,16,32}. */
};

int ohci_interrupt_endpoint_create(struct ohci_hc *hc,
                                   const struct ohci_interrupt_endpoint_config *cfg,
                                   struct ohci_interrupt_endpoint *ep);

int ohci_interrupt_submit(struct ohci_hc *hc,
                          struct ohci_interrupt_endpoint *ep,
                          struct ohci_urb *urb);

void ohci_interrupt_endpoint_destroy(struct ohci_hc *hc,
                                     struct ohci_interrupt_endpoint *ep);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_INTERRUPT_H */
