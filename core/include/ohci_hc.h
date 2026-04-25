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
struct ohci_bulk_endpoint;
struct ohci_interrupt_endpoint;
struct ohci_urb;

/* Capacity parameters for the HC's statically-sized internal pools. */
struct ohci_hc_config {
    uint16_t td_pool_size;
    uint16_t control_ed_count;
    uint16_t bulk_ed_count;
    uint16_t interrupt_ed_count;
};

/* Software state for one OHCI controller. Zero-initialised by ohci_hc_init. */
struct ohci_hc {
    struct ohci_mmio_ops    ops;
    struct ohci_dma_region *dma;

    struct ohci_hcca       *hcca;
    uint32_t                hcca_phys;

    struct ohci_td_pool     td_pool;

    struct ohci_ed_pool     control_ed_pool;
    struct ohci_ed_pool     bulk_ed_pool;
    struct ohci_ed_pool     interrupt_ed_pool;

    /* Interrupt skeleton (Task 5 populates; zero-initialised for now). */
    struct ohci_ed         *interrupt_skeleton[63];
    uint32_t                interrupt_skeleton_phys[63];

    struct ohci_control_endpoint   *control_head;
    struct ohci_bulk_endpoint      *bulk_head;
    struct ohci_interrupt_endpoint *interrupt_head;

    struct ohci_urb        *in_flight;
};

int ohci_hc_init(struct ohci_hc *hc,
                 const struct ohci_mmio_ops *ops,
                 struct ohci_dma_region *dma,
                 const struct ohci_hc_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_HC_H */
