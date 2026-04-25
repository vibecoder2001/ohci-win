#ifndef OHCI_BULK_H
#define OHCI_BULK_H

#include "ohci_types.h"
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

/* Per-page descriptor for SG submit. The caller fills one of these per
 * physically-contiguous page (typically obtained by walking an MDL). */
struct ohci_bulk_sg_page {
    uint32_t phys;     /* physical base address of this page chunk */
    uint32_t length;   /* bytes to transfer within this page (≤ PAGE_SIZE) */
    uint32_t off;      /* offset of this chunk within the overall URB */
};

#define OHCI_BULK_MAX_SG_PAGES OHCI_URB_MAX_DATA_TDS

/* Submit a Bulk URB across an SG page list. One TD per page; each TD
 * carries DI=7 except the last which gets DI=0 to fire WDH on completion.
 * urb->length must equal sum(pages[].length). */
int ohci_bulk_submit_sg(struct ohci_hc *hc,
                        struct ohci_bulk_endpoint *ep,
                        struct ohci_urb *urb,
                        const struct ohci_bulk_sg_page *pages,
                        unsigned page_count);

/* Single-page convenience wrapper around ohci_bulk_submit_sg. Used by the
 * Tier-1 harness (which has no MDL plumbing) and any caller that already
 * has a single physically-contiguous buffer ≤ 4 KB. */
static inline int ohci_bulk_submit(struct ohci_hc *hc,
                                   struct ohci_bulk_endpoint *ep,
                                   struct ohci_urb *urb) {
    struct ohci_bulk_sg_page page;
    page.phys   = urb->buffer_phys;
    page.length = urb->length;
    page.off    = 0;
    return ohci_bulk_submit_sg(hc, ep, urb, &page, 1);
}

void ohci_bulk_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_bulk_endpoint *ep);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_BULK_H */
