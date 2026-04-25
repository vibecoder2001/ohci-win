#ifndef OHCI_CONTROL_H
#define OHCI_CONTROL_H

#include "ohci_urb.h"
#include "ohci_pool.h"
#include "ohci_ed.h"
#include "ohci_td.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build a 2- or 3-TD Control-transfer chain from the URB's SETUP packet,
 * optional data buffer, and direction. TDs are pulled from `tdp`.
 *
 * On success: returns 0; *head = SETUP TD; *tail = STATUS TD; NextTD links
 * are written; URB data pointers are NOT dereferenced.
 *
 * On failure: returns a negative code and frees any TDs it allocated.
 *
 * Does NOT touch any ED or submit anything to hardware. Pure function. */
int ohci_build_control_chain(struct ohci_urb *urb,
                             struct ohci_td_pool *tdp,
                             struct ohci_td **head_out,
                             struct ohci_td **tail_out);

/* One Control-class endpoint. Owns its ED plus an ED-dedicated pool of
 * TDs (tail-placeholder convention). */
struct ohci_control_endpoint {
    struct ohci_ed *ed;
    uint32_t        ed_phys;

    /* Always-valid tail-placeholder TD. After each submit, this is
     * overwritten in-place and a fresh TD becomes the new placeholder. */
    struct ohci_td *tail_placeholder;
    uint32_t        tail_placeholder_phys;

    struct ohci_control_endpoint *next; /* SW-side control-list chain */
};

/* Endpoint configuration. Only FS Control in scope for this plan. */
struct ohci_control_endpoint_config {
    uint8_t  func_addr;     /* 0..127 */
    uint8_t  ep_num;        /* 0..15  */
    uint16_t max_packet_size;
    uint8_t  low_speed;     /* 0=Full, 1=Low */
};

struct ohci_hc;

/* Create and splice a Control endpoint onto the HC's Control list head.
 * Uses hc->control_ed_pool and hc->td_pool internally. Caller provides
 * pre-allocated ohci_control_endpoint storage. */
int ohci_control_endpoint_create(struct ohci_hc *hc,
                                 const struct ohci_control_endpoint_config *cfg,
                                 struct ohci_control_endpoint *ep);

/* Submit a Control URB: build TD chain, splice after the placeholder,
 * advance TailP, write HcCommandStatus.CLF. Returns 0 on success. */
int ohci_control_submit(struct ohci_hc *hc,
                        struct ohci_control_endpoint *ep,
                        struct ohci_urb *urb);

/* Remove the endpoint from the Control list, set Skip, and return
 * its ED + placeholder TD to the pools. Caller must ensure no URBs
 * are in flight on this endpoint (Plan 4 adds concurrency guards). */
void ohci_control_endpoint_destroy(struct ohci_hc *hc,
                                   struct ohci_control_endpoint *ep);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_CONTROL_H */
