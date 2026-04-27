/* Coverage for the four 2026-04-27 error-handling fixes:
 *   1. Drain fallback completes URB on hard-error data TD even when
 *      the chain's tail TD never retires (OHCI §6.4.4).
 *   2. SETUP-TD STALL fallback — same idea but keyed on urb->head_td
 *      instead of data_tds[].
 *   3. ohci_hc_reinit_after_ue completes in-flight URBs and reprograms
 *      list-head registers from preserved SW-side state.
 *   4. Bulk submit's K-toggle bracket around HeadP/TailP rewrite.
 *
 * Pool-leak checks count free slots before/after to confirm orphan TDs
 * get returned to the pool by each fallback path. */
#include <stdio.h>
#include <string.h>
#include "ohci_regs.h"
#include "ohci_hc.h"
#include "ohci_pool.h"
#include "ohci_urb.h"
#include "ohci_control.h"
#include "ohci_bulk.h"
#include "ohci_drain.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_dma.h"
#include "fake_hc.h"
#include "fake_hc_exec.h"

#define FAIL(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr,"\n"); return 1; } while (0)

static int g_completions;
static struct ohci_urb *g_last_completed;
static void on_complete(struct ohci_urb *u) {
    g_completions++;
    g_last_completed = u;
}

/* Walk the TD-pool free-list and return the number of free slots. Used
 * to assert orphan TDs get reclaimed by drain fallbacks. */
static int td_pool_free_count(struct ohci_td_pool *p) {
    int n = 0;
    uint16_t i = p->free_head;
    while (i != 0xFFFF) {
        if (n > p->capacity) return -1;  /* loop / corruption */
        i = p->next[i];
        n++;
    }
    return n;
}

/* Mark `td_phys` with condition code `cc`, splice it onto the head of
 * HCCA.DoneHead (LIFO — the drain reverses it), and raise the WDH bit
 * in shadow regs. fake_hc_exec_step always writes CC=NOERROR, so error
 * paths require a manual injection like this. */
static void inject_done(struct ohci_hc *hc, struct fake_hc *fake,
                        uint32_t td_phys, uint8_t cc) {
    struct ohci_td *td = ohci_dma_virt_from_phys(hc->dma, td_phys);
    td->Control = (td->Control & ~(uint32_t)OHCI_TD_CC_MASK) |
                  ((uint32_t)cc << OHCI_TD_CC_SHIFT);
    td->NextTD  = hc->hcca->DoneHead;
    hc->hcca->DoneHead = td_phys;

    uint32_t istat;
    memcpy(&istat, fake->regs + 0x0C, sizeof(istat));
    istat |= OHCI_INT_WDH;
    memcpy(fake->regs + 0x0C, &istat, sizeof(istat));
}

/* ---- Test 1: hard-error on data TD completes URB + frees STATUS orphan. ---- */
static int test_data_td_stall(void) {
    g_completions = 0; g_last_completed = NULL;
    static uint8_t backing[64 * 1024];
    struct fake_hc fake; fake_hc_init(&fake);
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=16, .control_ed_count=2,
                                   .bulk_ed_count=2, .interrupt_ed_count=2 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .max_packet_size = 8 };
    if (ohci_control_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep_create");

    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    (void)setup; (void)data;

    struct ohci_urb urb = {0};
    urb.setup_phys  = setup_phys;
    urb.buffer      = data;
    urb.buffer_phys = data_phys;
    urb.length      = 18;
    urb.direction   = OHCI_URB_DIR_IN;
    urb.complete    = on_complete;

    int free_before = td_pool_free_count(&hc.td_pool);
    if (ohci_control_submit(&hc, &ep, &urb) != 0) FAIL("submit");

    /* Submit allocated SETUP+DATA+STATUS+new-placeholder = 4 TDs from the
     * pool relative to the post-init steady state. Verify before injecting. */
    int free_after_submit = td_pool_free_count(&hc.td_pool);
    if (free_before - free_after_submit != 3 && free_before - free_after_submit != 4)
        FAIL("unexpected pool delta on submit: %d", free_before - free_after_submit);

    /* Inject STALL on the DATA TD. urb.data_tds[0].td_phys is the DATA TD.
     * Don't include SETUP in DoneHead — HC §6.4.4 says only the failing TD
     * is queued to DoneHead when an ED halts. */
    inject_done(&hc, &fake, urb.data_tds[0].td_phys, OHCI_CC_STALL);
    ohci_drain_done(&hc);

    if (g_completions != 1)               FAIL("completions=%d", g_completions);
    if (g_last_completed != &urb)         FAIL("wrong urb completed");
    if (urb.status != OHCI_URB_STATUS_STALL)
        FAIL("status=%d (expected STALL=%d)", urb.status, OHCI_URB_STATUS_STALL);
    if (hc.in_flight != NULL)             FAIL("in_flight not empty");

    /* Drain freed: the failing DATA TD itself + the STATUS orphan.
     * SETUP is still allocated (folded into the old placeholder, which
     * stays alive as part of the EP). The new placeholder also stays
     * alive. So pool reclaim = 2 TDs. */
    int free_after_drain = td_pool_free_count(&hc.td_pool);
    if (free_after_drain - free_after_submit < 2)
        FAIL("drain didn't free orphans: before=%d after=%d",
             free_after_submit, free_after_drain);

    printf("PASS: data-TD STALL completes URB and frees STATUS orphan\n");
    return 0;
}

/* ---- Test 2: SETUP-TD STALL fallback (rare per USB §8.5.3 but legal). ---- */
static int test_setup_td_stall(void) {
    g_completions = 0; g_last_completed = NULL;
    static uint8_t backing[64 * 1024];
    struct fake_hc fake; fake_hc_init(&fake);
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=16, .control_ed_count=2,
                                   .bulk_ed_count=2, .interrupt_ed_count=2 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .max_packet_size = 8 };
    if (ohci_control_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep_create");

    uint32_t setup_phys; uint8_t *setup = ohci_dma_alloc(&dma, 8, 4, &setup_phys);
    uint32_t data_phys;  uint8_t *data  = ohci_dma_alloc(&dma, 18, 4, &data_phys);
    (void)setup; (void)data;

    struct ohci_urb urb = {0};
    urb.setup_phys  = setup_phys;
    urb.buffer      = data;
    urb.buffer_phys = data_phys;
    urb.length      = 18;
    urb.direction   = OHCI_URB_DIR_IN;
    urb.complete    = on_complete;

    if (ohci_control_submit(&hc, &ep, &urb) != 0) FAIL("submit");
    int free_after_submit = td_pool_free_count(&hc.td_pool);

    /* Inject STALL on the SETUP TD. urb.head_td is a virt pointer; convert
     * to phys via dma->phys_base + (head_td - dma->base). */
    uint32_t setup_td_phys = dma.phys_base +
        (uint32_t)((uint8_t *)urb.head_td - dma.base);
    inject_done(&hc, &fake, setup_td_phys, OHCI_CC_STALL);
    ohci_drain_done(&hc);

    if (g_completions != 1)        FAIL("completions=%d", g_completions);
    if (urb.status != OHCI_URB_STATUS_STALL)
        FAIL("status=%d", urb.status);
    if (hc.in_flight != NULL)      FAIL("in_flight not empty");

    /* SETUP fallback frees: failing SETUP itself + DATA + STATUS = 3 TDs.
     * (SETUP is the OLD placeholder slot; freeing it is the loop-tail's
     * job, not the fallback's. DATA and STATUS are the fallback's job.) */
    int free_after_drain = td_pool_free_count(&hc.td_pool);
    if (free_after_drain - free_after_submit < 3)
        FAIL("setup-stall didn't free orphans: delta=%d",
             free_after_drain - free_after_submit);

    printf("PASS: SETUP-TD STALL completes URB and frees DATA+STATUS orphans\n");
    return 0;
}

/* ---- Test 3: UE recovery completes in-flight URBs + reprograms regs. ---- */
static int test_ue_recovery(void) {
    g_completions = 0; g_last_completed = NULL;
    static uint8_t backing[64 * 1024];
    struct fake_hc fake; fake_hc_init(&fake);
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=16, .control_ed_count=2,
                                   .bulk_ed_count=2, .interrupt_ed_count=2 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_control_endpoint ep;
    struct ohci_control_endpoint_config cfg = { .max_packet_size = 8 };
    if (ohci_control_endpoint_create(&hc, &cfg, &ep) != 0) FAIL("ep_create");

    /* Submit two URBs to confirm the in-flight walk drains all of them,
     * not just one. */
    uint32_t s1_phys; uint8_t *s1 = ohci_dma_alloc(&dma, 8, 4, &s1_phys); (void)s1;
    uint32_t s2_phys; uint8_t *s2 = ohci_dma_alloc(&dma, 8, 4, &s2_phys); (void)s2;

    struct ohci_urb urb1 = {0}, urb2 = {0};
    urb1.setup_phys = s1_phys; urb1.complete = on_complete;
    urb2.setup_phys = s2_phys; urb2.complete = on_complete;

    if (ohci_control_submit(&hc, &ep, &urb1) != 0) FAIL("submit1");
    if (ohci_control_submit(&hc, &ep, &urb2) != 0) FAIL("submit2");
    if (hc.in_flight == NULL) FAIL("expected in_flight after submits");

    /* Snapshot SW-side chain state before reinit. The reinit's contract
     * is that it rebuilds list-head registers from hc->control_head /
     * hc->bulk_head — so post-reinit registers must match these
     * snapshots. Comparing to ep.ed_phys alone is weaker because the
     * test would still pass if reinit accidentally NULL'd
     * hc->control_head and just happened to write the right value
     * from somewhere else. */
    struct ohci_control_endpoint *expected_control_head = hc.control_head;
    uint32_t expected_control_head_phys = expected_control_head
                                          ? expected_control_head->ed_phys
                                          : 0;

    /* Scribble over the list-head registers so we can prove reinit
     * actually rewrites them. */
    ops.write32(ops.context, 0x20 /* HcControlHeadED */, 0xDEADBEEFu);
    ops.write32(ops.context, 0x18 /* HcHCCA          */, 0xDEADBEEFu);

    ohci_hc_reinit_after_ue(&hc);

    if (g_completions != 2)
        FAIL("ue completions=%d (expected 2)", g_completions);
    if (urb1.status != OHCI_URB_STATUS_OTHER ||
        urb2.status != OHCI_URB_STATUS_OTHER)
        FAIL("ue urb status: %d %d", urb1.status, urb2.status);
    if (hc.in_flight != NULL) FAIL("in_flight not drained");

    /* SW chain must survive reinit (URB completion shouldn't touch it). */
    if (hc.control_head != expected_control_head)
        FAIL("control_head changed across reinit: %p -> %p",
             expected_control_head, hc.control_head);

    /* Registers rebuilt FROM the SW chain, not from a stale snapshot or
     * lucky coincidence. */
    if (ops.read32(ops.context, 0x18) != hc.hcca_phys)
        FAIL("HCCA reg not reprogrammed: 0x%08X", ops.read32(ops.context, 0x18));
    if (ops.read32(ops.context, 0x20) != expected_control_head_phys)
        FAIL("HcControlHeadED not rebuilt from control_head: "
             "got 0x%08X expected 0x%08X",
             ops.read32(ops.context, 0x20), expected_control_head_phys);
    /* HcFmInterval must include FSMPS — RK3588 silently skips TDs without
     * it, so reinit MUST reprogram both fields, not just FrameInterval. */
    uint32_t fmi = ops.read32(ops.context, 0x34);
    if ((fmi & 0x3FFF) != 11999u)        FAIL("FrameInterval=0x%X", fmi & 0x3FFF);
    if (((fmi >> 16) & 0x7FFF) != 10104u) FAIL("FSMPS=0x%X", (fmi >> 16) & 0x7FFF);

    printf("PASS: UE reinit completes in-flight URBs and reprograms regs\n");
    return 0;
}

/* ---- Test 4: bulk submit emits K-pause bracket around HeadP/TailP. ---- */
static int test_bulk_k_bracket(void) {
    g_completions = 0; g_last_completed = NULL;
    static uint8_t backing[64 * 1024];
    struct fake_hc fake; fake_hc_init(&fake);
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));
    fake_hc_set_dma(&fake, &dma);

    struct ohci_mmio_ops ops; fake_hc_get_ops(&fake, &ops);
    struct ohci_hc hc;
    struct ohci_hc_config hcfg = { .td_pool_size=16, .control_ed_count=2,
                                   .bulk_ed_count=2, .interrupt_ed_count=2 };
    if (ohci_hc_init(&hc, &ops, &dma, &hcfg) != 0) FAIL("hc_init");

    struct ohci_bulk_endpoint bep;
    struct ohci_bulk_endpoint_config bcfg = {
        .func_addr = 1, .ep_num = 1, .direction = OHCI_URB_DIR_OUT,
        .max_packet_size = 64,
    };
    if (ohci_bulk_endpoint_create(&hc, &bcfg, &bep) != 0) FAIL("bulk_ep");

    /* Pre-set K=1 and H=1 to simulate the post-STALL / post-cancel state
     * the new bracket has to recover from. Old (pre-6609952) bulk submit
     * cleared K once at the start and never re-set it; new submit must
     * still arrive at K=0 / H=0 / valid TailP regardless of entry state.
     * NOTE: this test verifies END-STATE invariants only — it cannot
     * observe the K=1 *during* HeadP/TailP rewrites without a write-
     * intercept hook on the fake HC. The real value of the bracket
     * is cycle-accurate (HC's cached ED snapshot drop), which the fake
     * HC doesn't model. End-state K=0 / H=0 plus a working happy-path
     * is all this harness can prove. */
    bep.ed->Control |= OHCI_ED_K;
    bep.ed->HeadP   |= OHCI_ED_HEADP_H;

    uint32_t buf_phys;
    void *buf = ohci_dma_alloc(&dma, 64, 4, &buf_phys);
    (void)buf;

    struct ohci_bulk_sg_page pages[1] = { { buf_phys, 64, 0 } };
    struct ohci_urb urb = {0};
    urb.complete = on_complete;
    if (ohci_bulk_submit_sg(&hc, &bep, &urb, pages, 1) != 0) FAIL("bulk submit");

    if ((bep.ed->Control & OHCI_ED_K) != 0)
        FAIL("K bit still set after submit (entry-from-K=1 failed)");
    if ((bep.ed->HeadP & OHCI_ED_HEADP_H) != 0)
        FAIL("H bit still set after submit (STALL recovery failed)");

    /* Now exec the HC and confirm normal completion still works (regression
     * guard: the bracket shouldn't break the happy path). */
    fake_hc_exec_step(&fake);
    ohci_drain_done(&hc);
    if (g_completions != 1) FAIL("bulk happy-path completions=%d", g_completions);
    if (urb.status != OHCI_URB_STATUS_OK)
        FAIL("bulk happy-path status=%d", urb.status);

    printf("PASS: bulk submit recovers from K=1/H=1 entry and completes\n");
    return 0;
}

int main(void) {
    if (test_data_td_stall())  return 1;
    if (test_setup_td_stall()) return 1;
    if (test_ue_recovery())    return 1;
    if (test_bulk_k_bracket()) return 1;
    return 0;
}
