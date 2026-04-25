#include <stdio.h>
#include <string.h>
#include "ohci_pool.h"

int main(void) {
    static uint8_t backing[4096];
    struct ohci_dma_region dma;
    ohci_dma_init(&dma, backing, 0x40000000u, sizeof(backing));

    /* ED pool: 4 EDs */
    struct ohci_ed_pool edp;
    if (ohci_ed_pool_init(&edp, &dma, 4) != 0) { fprintf(stderr,"FAIL: ed_pool init\n"); return 1; }

    uint32_t pa, pb;
    struct ohci_ed *a = ohci_ed_pool_alloc(&edp, &pa);
    struct ohci_ed *b = ohci_ed_pool_alloc(&edp, &pb);
    if (!a || !b)                         { fprintf(stderr,"FAIL: pool alloc null\n"); return 1; }
    if ((pa & 0xF) || (pb & 0xF))         { fprintf(stderr,"FAIL: unaligned ED phys\n"); return 1; }
    if (pa == pb)                         { fprintf(stderr,"FAIL: duplicate ED phys\n"); return 1; }

    /* After freeing, the next alloc should reuse */
    ohci_ed_pool_free(&edp, a);
    uint32_t pc;
    struct ohci_ed *c = ohci_ed_pool_alloc(&edp, &pc);
    if (c != a || pc != pa)               { fprintf(stderr,"FAIL: free/realloc didn't reuse\n"); return 1; }

    /* Exhaust the pool */
    uint32_t pd, pe, pf;
    struct ohci_ed *d = ohci_ed_pool_alloc(&edp, &pd);
    struct ohci_ed *e = ohci_ed_pool_alloc(&edp, &pe);
    if (!d || !e)                         { fprintf(stderr,"FAIL: expected 4 total slots\n"); return 1; }
    struct ohci_ed *f = ohci_ed_pool_alloc(&edp, &pf);
    if (f != NULL)                        { fprintf(stderr,"FAIL: 5th alloc should be NULL\n"); return 1; }

    /* TD pool works the same way */
    struct ohci_td_pool tdp;
    if (ohci_td_pool_init(&tdp, &dma, 8) != 0) { fprintf(stderr,"FAIL: td_pool init\n"); return 1; }
    uint32_t tpa;
    struct ohci_td *ta = ohci_td_pool_alloc(&tdp, &tpa);
    if (!ta || (tpa & 0xF))               { fprintf(stderr,"FAIL: td_pool alloc\n"); return 1; }

    /* Roundtrip phys->virt via the shared DMA region */
    if (ohci_dma_virt_from_phys(&dma, tpa) != ta) {
        fprintf(stderr,"FAIL: td phys/virt roundtrip\n"); return 1;
    }

    printf("PASS: ed + td pools\n");
    return 0;
}
