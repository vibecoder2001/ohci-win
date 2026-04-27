#ifndef OHCI_HC_H
#define OHCI_HC_H

#include "ohci_types.h"
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
struct ohci_isoc_endpoint;
struct ohci_urb;

/* Capacity parameters for the HC's statically-sized internal pools. */
struct ohci_hc_config {
    uint16_t td_pool_size;
    uint16_t control_ed_count;
    uint16_t bulk_ed_count;
    uint16_t interrupt_ed_count;
    uint16_t isoc_ed_count;
    uint16_t itd_pool_size;
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
    struct ohci_ed_pool     isoc_ed_pool;
    struct ohci_itd_pool    itd_pool;

    /* Interrupt skeleton (Task 5 populates; zero-initialised for now). */
    struct ohci_ed         *interrupt_skeleton[63];
    uint32_t                interrupt_skeleton_phys[63];

    struct ohci_control_endpoint   *control_head;
    struct ohci_bulk_endpoint      *bulk_head;
    struct ohci_interrupt_endpoint *interrupt_head;
    struct ohci_isoc_endpoint      *isoc_head;

    struct ohci_urb        *in_flight;
};

int ohci_hc_init(struct ohci_hc *hc,
                 const struct ohci_mmio_ops *ops,
                 struct ohci_dma_region *dma,
                 const struct ohci_hc_config *cfg);

/* OHCI §6.5.1: UE (Unrecoverable Error) leaves the HC in a fatal state
 * that requires HCR + full re-initialisation. Steps performed:
 *   1. Splice every URB off hc->in_flight, mark non-PENDING ones, and
 *      invoke urb->complete so callers (UCX glue) can stage WDFREQUEST
 *      completions onto their Deferred lists.
 *   2. Issue HcCommandStatus.HCR and wait for the reset to clear.
 *   3. Re-publish HCCA, list head registers (rebuilt from hc->control_head
 *      / hc->bulk_head), HcFmInterval (with FSMPS), HcPeriodicStart,
 *      HcInterruptEnable (WDH|MIE), and HcControl in OPERATIONAL with
 *      CLE|BLE|PLE|IE.
 * Caller is responsible for any extra HcInterruptEnable bits the driver
 * glue normally sets (RHSC|UE|SO) and for re-running CLEAR_PORT if the
 * UE took down a port reset mid-flight. TDs hanging off EDs at UE time
 * leak their pool slots — acceptable since UE forces device-level
 * re-enumeration anyway. */
void ohci_hc_reinit_after_ue(struct ohci_hc *hc);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_HC_H */
