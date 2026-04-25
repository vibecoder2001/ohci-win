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

#ifdef __cplusplus
}
#endif

#endif /* OHCI_DRAIN_H */
