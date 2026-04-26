#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_itd.h"
#include "ohci_dma.h"
#include "ohci_isoc.h"
#include "ohci_bulk.h"
#include "ohci_urb.h"
#include "ohci_drain.h"
#include "ohci_pool.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

#define FAIL(msg, ...) do { fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); return 1; } while (0)

static int g_completions = 0;
static struct ohci_urb *g_last_completed = NULL;
static void on_complete(struct ohci_urb *u) { g_completions++; g_last_completed = u; }

/* Fake-HC raw interrupt-status set: write32() to 0x0C is W1C in our fake,
 * so to simulate the HC raising WDH we poke the shadow regs directly. */
static void inject_wdh(struct fake_hc *fake, uint32_t done_head_phys,
                       struct ohci_hc *hc) {
    hc->hcca->DoneHead = done_head_phys;
    uint32_t cur;
    memcpy(&cur, fake->regs + 0x0C, sizeof(cur));
    cur |= OHCI_INT_WDH;
    memcpy(fake->regs + 0x0C, &cur, sizeof(cur));
}

/* Build (hc + isoch ep) and submit a 3-packet ITD. Caller then writes
 * PSW words to simulate HW retirement, posts to DoneHead, and drives drain. */
static int setup_and_submit(struct fake_hc *fake, struct ohci_dma_region *dma,
                            struct ohci_hc *hc,
                            struct ohci_isoc_endpoint *ep,
                            struct ohci_urb *urb,
                            uint8_t pkt_count,
                            const uint16_t *lens,
                            uint32_t *out_itd_phys,
                            struct ohci_itd **out_itd) {
    fake_hc_init(fake);
    fake_hc_set_dma(fake, dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(fake, &ops);
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 8,
    };
    if (ohci_hc_init(hc, &ops, dma, &hcfg) != 0) return 1;
    struct ohci_isoc_endpoint_config cfg = {
        .func_addr = 5, .ep_num = 2, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 192,
    };
    if (ohci_isoc_endpoint_create(hc, &cfg, ep) != 0) return 1;

    memset(urb, 0, sizeof(*urb));
    urb->complete = on_complete;

    uint32_t buf_len = 0;
    for (uint8_t i = 0; i < pkt_count; i++) buf_len += lens[i];
    if (ohci_isoc_submit_window(hc, ep, urb, 100, pkt_count, lens,
                                0xC0001000u, buf_len, 1) != 0) return 1;

    *out_itd_phys = urb->data_tds[0].td_phys;
    *out_itd = (struct ohci_itd *)ohci_dma_virt_from_phys(dma, *out_itd_phys);
    return 0;
}

static int test_happy(void) {
    g_completions = 0; g_last_completed = NULL;
    struct fake_hc fake;
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    struct ohci_hc hc;
    struct ohci_isoc_endpoint ep;
    struct ohci_urb urb;
    uint16_t lens[3] = {192, 192, 192};
    uint32_t itd_phys; struct ohci_itd *itd;
    if (setup_and_submit(&fake, &dma, &hc, &ep, &urb, 3, lens, &itd_phys, &itd))
        FAIL("setup");

    /* Simulate HW retirement: PSW out = CC[15:12] | size[11:0] */
    itd->PSW[0] = (OHCI_CC_NOERROR << OHCI_PSW_CC_SHIFT) | 192;
    itd->PSW[1] = (OHCI_CC_NOERROR << OHCI_PSW_CC_SHIFT) | 192;
    itd->PSW[2] = (OHCI_CC_NOERROR << OHCI_PSW_CC_SHIFT) | 192;
    /* Control CC = NoError. */
    itd->Control = (itd->Control & ~OHCI_ITD_CC_MASK) |
                   ((OHCI_CC_NOERROR << OHCI_ITD_CC_SHIFT) & OHCI_ITD_CC_MASK);
    itd->NextTD = 0;

    inject_wdh(&fake, itd_phys, &hc);
    ohci_drain_done(&hc);

    if (g_completions != 1) FAIL("completions=%d", g_completions);
    if (g_last_completed != &urb) FAIL("wrong urb");
    if (urb.transferred != 576) FAIL("transferred=%u", urb.transferred);
    if (urb.status != OHCI_URB_STATUS_OK) FAIL("status=%d", urb.status);
    for (int i = 0; i < 3; i++) {
        if (urb.isoc_pkts[i].length != 192) FAIL("pkt[%d].length=%u", i, urb.isoc_pkts[i].length);
        if (urb.isoc_pkts[i].cc != OHCI_CC_NOERROR) FAIL("pkt[%d].cc=%u", i, urb.isoc_pkts[i].cc);
    }

    /* Slot returned to itd_pool: alloc must succeed and yield same virt. */
    uint32_t reaped_phys;
    struct ohci_itd *reaped = ohci_itd_pool_alloc(&hc.itd_pool, &reaped_phys);
    if (reaped != itd) FAIL("itd not returned to pool");
    return 0;
}

static int test_underrun(void) {
    g_completions = 0;
    struct fake_hc fake;
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    struct ohci_hc hc;
    struct ohci_isoc_endpoint ep;
    struct ohci_urb urb;
    uint16_t lens[3] = {192, 192, 192};
    uint32_t itd_phys; struct ohci_itd *itd;
    if (setup_and_submit(&fake, &dma, &hc, &ep, &urb, 3, lens, &itd_phys, &itd))
        FAIL("setup");

    itd->PSW[0] = (OHCI_CC_NOERROR     << OHCI_PSW_CC_SHIFT) | 192;
    itd->PSW[1] = (OHCI_CC_NOERROR     << OHCI_PSW_CC_SHIFT) | 192;
    itd->PSW[2] = (OHCI_CC_DATAUNDERRUN << OHCI_PSW_CC_SHIFT) | 100;
    itd->Control = (itd->Control & ~OHCI_ITD_CC_MASK) |
                   ((OHCI_CC_NOERROR << OHCI_ITD_CC_SHIFT) & OHCI_ITD_CC_MASK);
    itd->NextTD = 0;

    inject_wdh(&fake, itd_phys, &hc);
    ohci_drain_done(&hc);

    if (urb.transferred != 484) FAIL("transferred=%u", urb.transferred);
    if (urb.status != OHCI_URB_STATUS_OK) FAIL("status=%d (underrun should not fail URB)", urb.status);
    if (urb.isoc_pkts[2].cc != OHCI_CC_DATAUNDERRUN) FAIL("pkt[2].cc=%u", urb.isoc_pkts[2].cc);
    if (urb.isoc_pkts[2].length != 100) FAIL("pkt[2].length=%u", urb.isoc_pkts[2].length);
    return 0;
}

static int test_hard_error(void) {
    g_completions = 0;
    struct fake_hc fake;
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xC0000000u, sizeof(backing));
    struct ohci_hc hc;
    struct ohci_isoc_endpoint ep;
    struct ohci_urb urb;
    uint16_t lens[3] = {192, 192, 192};
    uint32_t itd_phys; struct ohci_itd *itd;
    if (setup_and_submit(&fake, &dma, &hc, &ep, &urb, 3, lens, &itd_phys, &itd))
        FAIL("setup");

    itd->PSW[0] = (OHCI_CC_NOERROR << OHCI_PSW_CC_SHIFT) | 192;
    itd->PSW[1] = (OHCI_CC_STALL   << OHCI_PSW_CC_SHIFT) | 0;
    itd->PSW[2] = (OHCI_CC_NOERROR << OHCI_PSW_CC_SHIFT) | 192;
    itd->Control = (itd->Control & ~OHCI_ITD_CC_MASK) |
                   ((OHCI_CC_NOERROR << OHCI_ITD_CC_SHIFT) & OHCI_ITD_CC_MASK);
    itd->NextTD = 0;

    inject_wdh(&fake, itd_phys, &hc);
    ohci_drain_done(&hc);

    if (urb.status != OHCI_URB_STATUS_OVERRUN) FAIL("status=%d expected OVERRUN", urb.status);
    if (urb.isoc_pkts[1].cc != OHCI_CC_STALL) FAIL("pkt[1].cc=%u", urb.isoc_pkts[1].cc);
    return 0;
}

static int test_general_td_path_unchanged(void) {
    g_completions = 0; g_last_completed = NULL;
    struct fake_hc fake; fake_hc_init(&fake);
    static uint8_t backing[64 * 1024];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0xD0000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);
    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = {
        .td_pool_size = 8, .control_ed_count = 1, .bulk_ed_count = 1,
        .interrupt_ed_count = 1, .isoc_ed_count = 1, .itd_pool_size = 4,
    };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_bulk_endpoint ep;
    struct ohci_bulk_endpoint_config cfg = {
        .func_addr = 1, .ep_num = 1, .direction = OHCI_URB_DIR_IN,
        .low_speed = 0, .max_packet_size = 64,
    };
    if (ohci_bulk_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("bulk ep create");

    uint32_t buf_phys; uint8_t *buf = ohci_dma_alloc(&dma, 64, 4, &buf_phys);
    memset(buf, 0, 64);

    struct ohci_urb urb = {0};
    urb.buffer = buf; urb.buffer_phys = buf_phys; urb.length = 64;
    urb.direction = OHCI_URB_DIR_IN;
    urb.complete = on_complete;

    if (ohci_bulk_submit(&hc, &ep, &urb) != 0) FAIL("bulk submit");

    /* Capture the data TD pool slot before drain so we can verify return. */
    uint32_t data_td_phys = urb.data_tds[0].td_phys;
    struct ohci_td *data_td = (struct ohci_td *)ohci_dma_virt_from_phys(&dma, data_td_phys);

    fake_hc_exec_step(&fake);
    ohci_drain_done(&hc);

    if (g_completions != 1) FAIL("bulk completions=%d", g_completions);
    if (urb.status != OHCI_URB_STATUS_OK) FAIL("bulk status=%d", urb.status);
    if (urb.is_isoc != 0) FAIL("bulk urb shouldn't be isoc");

    /* Slot should be on td_pool, not itd_pool — re-alloc from td_pool yields it. */
    uint32_t reaped_phys;
    struct ohci_td *reaped = ohci_td_pool_alloc(&hc.td_pool, &reaped_phys);
    if (!reaped) FAIL("td_pool empty after drain");
    /* The TD pool is LIFO; the most-recently-freed TD is the one we get back. */
    if (reaped != data_td) FAIL("data TD not returned to td_pool");
    return 0;
}

int main(void) {
    if (test_happy())                       return 1;
    if (test_underrun())                    return 1;
    if (test_hard_error())                  return 1;
    if (test_general_td_path_unchanged())   return 1;
    printf("PASS: ohci_drain ITD branch (PSW decode + pool dispatch)\n");
    return 0;
}
