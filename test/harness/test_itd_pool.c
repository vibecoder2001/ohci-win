#include <stdio.h>
#include <string.h>
#include "ohci_itd.h"
#include "ohci_pool.h"
#include "ohci_dma.h"

int main(void) {
    /* 8 KB backing region — enough for 16 ITDs (32 B each) + free-list array */
    static uint8_t backing[8192];
    struct ohci_dma_region dma;
    /* phys_base chosen to be 32-byte aligned so bump-alloc stays aligned */
    ohci_dma_init(&dma, backing, 0x80000000u, sizeof(backing));

    struct ohci_itd_pool p;
    if (ohci_itd_pool_init(&p, &dma, 16) != 0) {
        fprintf(stderr, "FAIL: itd_pool_init\n"); return 1;
    }

    uint32_t pa, pb;
    struct ohci_itd *a = ohci_itd_pool_alloc(&p, &pa);
    struct ohci_itd *b = ohci_itd_pool_alloc(&p, &pb);
    if (!a || !b)            { fprintf(stderr, "FAIL: alloc null\n"); return 1; }
    if (pa & 0x1F)           { fprintf(stderr, "FAIL: pa not 32B-aligned: 0x%X\n", pa); return 1; }
    if (pb & 0x1F)           { fprintf(stderr, "FAIL: pb not 32B-aligned\n"); return 1; }
    if (pa == pb)            { fprintf(stderr, "FAIL: duplicate phys\n"); return 1; }
    if ((uint8_t*)b - (uint8_t*)a != 32) {
        fprintf(stderr, "FAIL: slot stride should be 32 bytes, got %lld\n",
                (long long)((uint8_t*)b - (uint8_t*)a));
        return 1;
    }

    /* free + realloc reuses the same slot */
    ohci_itd_pool_free(&p, a);
    uint32_t pc;
    struct ohci_itd *c = ohci_itd_pool_alloc(&p, &pc);
    if (c != a || pc != pa)  { fprintf(stderr, "FAIL: reuse\n"); return 1; }

    /* Exhaust the pool — 16 slots total; c==a and b are held (2 slots).
     * Allocate the remaining 14. */
    for (int i = 0; i < 14; i++) {
        uint32_t pp;
        struct ohci_itd *t = ohci_itd_pool_alloc(&p, &pp);
        if (!t) { fprintf(stderr, "FAIL: ran out at i=%d\n", i); return 1; }
    }
    uint32_t pf;
    if (ohci_itd_pool_alloc(&p, &pf) != NULL) {
        fprintf(stderr, "FAIL: 17th alloc should be NULL\n"); return 1;
    }

    printf("PASS: itd pool\n");
    return 0;
}
