#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_ed.h"
#include "ohci_itd.h"
#include "ohci_dma.h"
#include "ohci_isoc.h"
#include "ohci_urb.h"
#include "fake_hc.h"

#define FAIL(msg, ...) do { fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); return 1; } while (0)

static int run_basic_submit(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size       = 8,
        .control_ed_count   = 1,
        .bulk_ed_count      = 1,
        .interrupt_ed_count = 1,
        .isoc_ed_count      = 1,
        .itd_pool_size      = 8,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 5, .ep_num = 2, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 192,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    /* Case 1: 4-packet window at frame 100, page-aligned buffer. */
    struct ohci_urb urb; memset(&urb, 0, sizeof(urb));
    uint16_t lens[4] = {192, 192, 192, 192};
    uint32_t buf_phys = 0xB0001000u;
    uint32_t buf_len  = 4 * 192;
    if (ohci_isoc_submit_window(&hc, &ep, &urb, 100, 4, lens, buf_phys, buf_len) != 0)
        FAIL("submit returned non-zero");

    struct ohci_itd *itd = (struct ohci_itd *)ohci_dma_virt_from_phys(&dma, urb.data_tds[0].td_phys);
    if (!itd) FAIL("data ITD virt lookup failed");

    uint32_t sf = itd->Control & OHCI_ITD_SF_MASK;
    uint32_t fc = (itd->Control & OHCI_ITD_FC_MASK) >> OHCI_ITD_FC_SHIFT;
    if (sf != 100) FAIL("SF=%u expected 100", sf);
    if (fc != 3)   FAIL("FC=%u expected 3", fc);
    if (itd->BP0 != 0xB0001000u) FAIL("BP0=0x%08x", itd->BP0);
    if (itd->BE  != 0xB0001000u + buf_len - 1) FAIL("BE=0x%08x", itd->BE);
    if (itd->PSW[0] != 0)   FAIL("PSW[0]=%u", itd->PSW[0]);
    if (itd->PSW[1] != 192) FAIL("PSW[1]=%u", itd->PSW[1]);
    if (itd->PSW[2] != 384) FAIL("PSW[2]=%u", itd->PSW[2]);
    if (itd->PSW[3] != 576) FAIL("PSW[3]=%u", itd->PSW[3]);
    for (int i = 4; i < 8; i++)
        if (itd->PSW[i] != 0) FAIL("PSW[%d]=%u expected 0", i, itd->PSW[i]);

    if (ep.ed_tail_frame != 104) FAIL("ed_tail_frame=%u expected 104", ep.ed_tail_frame);
    if (ep.primed != 1)          FAIL("primed=%u expected 1", ep.primed);
    if (urb.is_isoc != 1)        FAIL("is_isoc=%u", urb.is_isoc);
    if (urb.isoc_pkt_count != 4) FAIL("isoc_pkt_count=%u", urb.isoc_pkt_count);
    if (urb.ed != ep.ed)         FAIL("urb.ed mismatch");
    if (urb.status != OHCI_URB_STATUS_PENDING) FAIL("urb.status=%d", urb.status);
    if (ep.ed->TailP != ep.tail_placeholder_phys) FAIL("TailP not advanced");
    if (hc.in_flight != &urb)    FAIL("in_flight not pointing to urb");

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

static int run_page_straddle_accept(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 8,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 1, .ep_num = 1, .direction = OHCI_URB_DIR_OUT,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    struct ohci_urb urb; memset(&urb, 0, sizeof(urb));
    uint16_t lens[1] = {32};
    uint32_t buf_phys = 0xB0001FF0u;     /* crosses one page boundary */
    if (ohci_isoc_submit_window(&hc, &ep, &urb, 50, 1, lens, buf_phys, 32) != 0)
        FAIL("page-straddle (one boundary) should be accepted");
    struct ohci_itd *itd = (struct ohci_itd *)ohci_dma_virt_from_phys(&dma, urb.data_tds[0].td_phys);
    if (itd->BP0 != 0xB0001000u) FAIL("BP0=0x%08x expected 0xB0001000", itd->BP0);
    if (itd->BE  != 0xB000200Fu) FAIL("BE=0x%08x expected 0xB000200F", itd->BE);
    if (itd->PSW[0] != 0xFF0)    FAIL("PSW[0]=0x%x expected 0xFF0", itd->PSW[0]);

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

static int run_two_page_straddle_reject(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 8,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 1, .ep_num = 1, .direction = OHCI_URB_DIR_OUT,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    struct ohci_urb urb; memset(&urb, 0, sizeof(urb));
    uint16_t lens[1] = {0x1020};
    uint32_t buf_phys = 0xB0001FF0u;
    if (ohci_isoc_submit_window(&hc, &ep, &urb, 60, 1, lens, buf_phys, 0x1020) != -1)
        FAIL("two-page straddle should be rejected");

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

static int run_pkt_count_bounds(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 8,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 1, .ep_num = 1, .direction = OHCI_URB_DIR_OUT,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    struct ohci_urb urb; memset(&urb, 0, sizeof(urb));
    uint16_t lens[9] = {8,8,8,8,8,8,8,8,8};
    if (ohci_isoc_submit_window(&hc, &ep, &urb, 0, 0, lens, 0xB0001000u, 0) != -1)
        FAIL("pkt_count=0 should reject");
    if (ohci_isoc_submit_window(&hc, &ep, &urb, 0, 9, lens, 0xB0001000u, 72) != -1)
        FAIL("pkt_count=9 should reject");

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

static int run_two_consecutive(void) {
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 8,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 5, .ep_num = 2, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    uint16_t lens[8] = {64,64,64,64,64,64,64,64};
    struct ohci_urb urb1; memset(&urb1, 0, sizeof(urb1));
    if (ohci_isoc_submit_window(&hc, &ep, &urb1, 200, 8, lens, 0xB0002000u, 512) != 0)
        FAIL("submit1");
    if (ep.ed_tail_frame != 208) FAIL("after #1 ed_tail_frame=%u", ep.ed_tail_frame);

    struct ohci_urb urb2; memset(&urb2, 0, sizeof(urb2));
    if (ohci_isoc_submit_window(&hc, &ep, &urb2, 208, 8, lens, 0xB0003000u, 512) != 0)
        FAIL("submit2");
    if (ep.ed_tail_frame != 216) FAIL("after #2 ed_tail_frame=%u", ep.ed_tail_frame);

    /* First ITD's NextTD == second ITD's phys. */
    struct ohci_itd *itd1 = (struct ohci_itd *)ohci_dma_virt_from_phys(&dma, urb1.data_tds[0].td_phys);
    if (itd1->NextTD != urb2.data_tds[0].td_phys)
        FAIL("itd1.NextTD=0x%08x expected 0x%08x", itd1->NextTD, urb2.data_tds[0].td_phys);

    /* LIFO: urb2 was submitted last, so it's at head; urb2.next_pending == &urb1. */
    if (hc.in_flight != &urb2)            FAIL("in_flight head not urb2");
    if (urb2.next_pending != &urb1)       FAIL("urb2.next_pending != &urb1");

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

static int run_pool_exhaustion(void) {
    /* itd_pool_size = 4: endpoint create consumes 1 placeholder, leaving 3.
     * Each submit nets +1 (alloc data + new_ph, free data, promote placeholder).
     * So submits should succeed until the new_ph alloc fails. */
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 4,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_isoc_endpoint ep;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 5, .ep_num = 2, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_isoc_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep create");

    uint16_t lens[1] = {64};
    int succeeded = 0;
    int rc;
    struct ohci_urb urbs[8];
    for (int i = 0; i < 8; i++) {
        memset(&urbs[i], 0, sizeof(urbs[i]));
        rc = ohci_isoc_submit_window(&hc, &ep, &urbs[i],
                                     (uint16_t)(300 + i), 1, lens,
                                     0xB0001000u + (uint32_t)i * 64u, 64);
        if (rc == 0) succeeded++;
        else break;
    }
    if (rc != -1) FAIL("expected exhaustion to return -1");
    if (succeeded == 0) FAIL("no submits succeeded; pool too small");
    /* Queue must remain valid (ed_tail_frame = SF + 1 of last successful). */
    if (ep.ed_tail_frame != (uint16_t)(300 + succeeded))
        FAIL("ed_tail_frame=%u expected %u", ep.ed_tail_frame, 300 + succeeded);

    ohci_isoc_endpoint_destroy(&hc, &ep);
    return 0;
}

int main(void) {
    if (run_basic_submit())             return 1;
    if (run_page_straddle_accept())     return 1;
    if (run_two_page_straddle_reject()) return 1;
    if (run_pkt_count_bounds())         return 1;
    if (run_two_consecutive())          return 1;
    if (run_pool_exhaustion())          return 1;
    printf("PASS: ohci_isoc_submit_window\n");
    return 0;
}
