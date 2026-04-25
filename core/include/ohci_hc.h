#ifndef OHCI_HC_H
#define OHCI_HC_H

#include <stdint.h>
#include "ohci_mmio.h"
#include "ohci_dma.h"
#include "ohci_hcca.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Software state for one OHCI controller. Zero-initialised by ohci_hc_init.
 * Later Plan 2 tasks will extend this struct with TD pool + endpoint list. */
struct ohci_hc {
    struct ohci_mmio_ops    ops;    /* register read/write seam */
    struct ohci_dma_region *dma;    /* DMA-coherent memory for HCCA + descriptors */

    struct ohci_hcca       *hcca;     /* virtual pointer */
    uint32_t                hcca_phys;/* physical address */
};

/* Reset the controller, allocate HCCA in the DMA region, program HcHCCA,
 * enable the Control list + interrupts, and move HCFS to Operational.
 * Returns 0 on success or a negative errno-ish code on failure. */
int ohci_hc_init(struct ohci_hc *hc,
                 const struct ohci_mmio_ops *ops,
                 struct ohci_dma_region *dma);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_HC_H */
