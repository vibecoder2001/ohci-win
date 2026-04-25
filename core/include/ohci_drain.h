#ifndef OHCI_DRAIN_H
#define OHCI_DRAIN_H

#include "ohci_hc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Poll the HC's done queue. If WDH is set, snapshot HCCA.DoneHead, clear
 * it + WDH, reverse the chain to FIFO order, and for each TD match it
 * against an in-flight URB via its tail_td phys address. URBs whose tail
 * TD retires are completed via urb->complete. Retired TDs are returned
 * to hc->td_pool.
 *
 * Works for any transfer type (Control/Bulk/Interrupt) — the match is
 * type-agnostic. */
void ohci_drain_done(struct ohci_hc *hc);

/* Cancel any in-flight URBs whose ED is `ed`. Sets the ED's Skip bit so the
 * HC stops dispatching new TDs on it, then walks hc->in_flight, removes
 * matching URBs, sets status=OHCI_URB_STATUS_OTHER, and invokes
 * urb->complete for each. Used by EP purge/abort paths so UCX can proceed
 * with EP teardown without waiting for a poll that will never come.
 *
 * Note: does NOT free TDs from the pool or unlink the ED. Caller is
 * expected to call the type-specific endpoint_destroy when tearing down
 * the EP for good. */
void ohci_urb_cancel_for_ed(struct ohci_hc *hc, struct ohci_ed *ed);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_DRAIN_H */
