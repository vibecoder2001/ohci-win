#ifndef OHCI_ISOC_H
#define OHCI_ISOC_H

#include "ohci_types.h"
#include "ohci_ed.h"
#include "ohci_itd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_hc;
struct ohci_urb;

struct ohci_isoc_endpoint_config {
    uint8_t  func_addr;
    uint8_t  ep_num;
    uint8_t  direction;        /* OHCI_URB_DIR_IN / OHCI_URB_DIR_OUT */
    uint8_t  low_speed;
    uint16_t max_packet_size;
};

struct ohci_isoc_endpoint {
    struct ohci_ed   *ed;
    uint32_t          ed_phys;
    struct ohci_itd  *tail_placeholder;
    uint32_t          tail_placeholder_phys;
    uint16_t          max_packet_size;
    uint8_t           direction;
    /* Refill bookkeeping — populated by submit/refill paths in later tasks. */
    uint16_t          ed_tail_frame;     /* SF + FC + 1 of last queued ITD */
    uint8_t           primed;
    struct ohci_isoc_endpoint *next;
};

int  ohci_isoc_endpoint_create(struct ohci_hc *hc,
                               const struct ohci_isoc_endpoint_config *cfg,
                               struct ohci_isoc_endpoint *ep);
void ohci_isoc_endpoint_destroy(struct ohci_hc *hc,
                                struct ohci_isoc_endpoint *ep);

/* Build one ITD covering pkt_count packets (1..8) starting at frame `sf`.
 * Each packet's length is in pkt_lens[i]; the buffer at buf_phys/buf_len
 * holds the concatenated packet payload (packets are placed back-to-back
 * starting at buf_phys, so buf_len must equal sum of pkt_lens).
 *
 * Returns 0 on success, -1 if pkt_count is out of range, the buffer
 * straddles more than one page boundary, or pool allocation fails.
 *
 * On success the URB is queued in_flight; complete() will fire when the
 * drain path retires the ITD. ep->ed_tail_frame advances to sf + pkt_count
 * and ep->primed becomes 1. */
/* `first_window` selects URB-init semantics:
 *   1: this is the first window of the URB — reset transferred,
 *      isoc_pkts_filled, status=PENDING, isoc_pkt_count=pkt_count,
 *      head_td/tail_td both point at the new ITD.
 *   0: continuation — leave URB-level fields alone, only emit the new
 *      ITD, append it to data_tds[], advance ep->ed_tail_frame, and
 *      bump tail_td so URB completion fires from the LAST ITD. The
 *      caller must have set urb->isoc_pkt_count to the URB-total
 *      packet count before the first call (submit_window does that
 *      when first_window=1). */
int ohci_isoc_submit_window(struct ohci_hc *hc,
                            struct ohci_isoc_endpoint *ep,
                            struct ohci_urb *urb,
                            uint16_t sf,
                            uint8_t  pkt_count,
                            const uint16_t *pkt_lens,
                            uint32_t buf_phys,
                            uint32_t buf_len,
                            int first_window);

/* Silence-window variant (Plan 8 Task 7).
 *
 * Emits an ITD with no URB tracking — used by the refill DPC to keep the
 * isoch ED chain non-empty when no caller URB is queued. The HC sources
 * `pkt_count` packets of bytes from buf_phys (typically a zero-filled
 * page) and sends them on the wire; on retirement the orphan ITD is
 * dispatched back to the ITD pool by the drain (which routes by
 * phys-range, not by URB ownership). No URB is linked to hc->in_flight.
 *
 * Same windowing constraints as ohci_isoc_submit_window (≤8 packets,
 * buffer fits in BP0 + BP0+0x1000). Returns 0 on success, -1 on bad
 * input or pool exhaustion. */
int ohci_isoc_submit_silence_window(struct ohci_hc *hc,
                                    struct ohci_isoc_endpoint *ep,
                                    uint16_t sf,
                                    uint8_t  pkt_count,
                                    const uint16_t *pkt_lens,
                                    uint32_t buf_phys,
                                    uint32_t buf_len);

#ifdef __cplusplus
}
#endif

#endif
