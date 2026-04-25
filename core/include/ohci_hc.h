#ifndef OHCI_HC_H
#define OHCI_HC_H

#include <stdint.h>
#include "ohci_mmio.h"
#include "ohci_dma.h"
#include "ohci_hcca.h"
#include "ohci_pool.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_control_endpoint;

/* Software state for one OHCI controller. Zero-initialised by ohci_hc_init. */
struct ohci_hc {
    struct ohci_mmio_ops    ops;    /* register read/write seam */
    struct ohci_dma_region *dma;    /* DMA-coherent memory for HCCA + descriptors */

    struct ohci_hcca       *hcca;     /* virtual pointer */
    uint32_t                hcca_phys;/* physical address */

    /* TD pool used for all endpoints on this controller. */
    struct ohci_td_pool    td_pool;

    /* Software-side head of the Control list (for teardown later).
     * NULL when empty. */
    struct ohci_control_endpoint *control_head;
};

/* Reset the controller, allocate HCCA in the DMA region, program HcHCCA,
 * enable the Control list + interrupts, and move HCFS to Operational.
 * Additional param: number of TDs the TD pool should reserve. Typical
 * sizing: 3 TDs per outstanding URB times a small queue depth, e.g. 64.
 * Returns 0 on success or a negative errno-ish code on failure. */
int ohci_hc_init(struct ohci_hc *hc,
                 const struct ohci_mmio_ops *ops,
                 struct ohci_dma_region *dma,
                 uint16_t td_pool_size);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_HC_H */
