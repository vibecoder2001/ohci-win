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

#ifdef __cplusplus
}
#endif

#endif /* OHCI_CONTROL_H */
