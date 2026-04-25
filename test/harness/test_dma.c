#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ohci_dma.h"

int main(void) {
    uint8_t backing[4096];
    struct ohci_dma_region r;
    ohci_dma_init(&r, backing, 0x10000000u, sizeof(backing));

    /* First alloc: 256 bytes, 256-aligned (HCCA shape) */
    uint32_t phys_a;
    void *va = ohci_dma_alloc(&r, 256, 256, &phys_a);
    if (!va)                         { fprintf(stderr, "FAIL: alloc A null\n"); return 1; }
    if ((phys_a & 0xFF) != 0)        { fprintf(stderr, "FAIL: phys_a align = 0x%x\n", phys_a); return 1; }
    if (ohci_dma_virt_from_phys(&r, phys_a) != va) { fprintf(stderr, "FAIL: roundtrip A\n"); return 1; }

    /* Second alloc: 16 bytes, 16-aligned (ED shape). Must be after A. */
    uint32_t phys_b;
    void *vb = ohci_dma_alloc(&r, 16, 16, &phys_b);
    if (!vb)                         { fprintf(stderr, "FAIL: alloc B null\n"); return 1; }
    if ((phys_b & 0xF) != 0)         { fprintf(stderr, "FAIL: phys_b align = 0x%x\n", phys_b); return 1; }
    if ((uint8_t*)vb < (uint8_t*)va + 256) {
        fprintf(stderr, "FAIL: B overlaps A\n"); return 1;
    }

    /* phys distance equals virt distance */
    if ((phys_b - phys_a) != (uint32_t)((uint8_t*)vb - (uint8_t*)va)) {
        fprintf(stderr, "FAIL: phys/virt stride mismatch\n"); return 1;
    }

    /* Out-of-space */
    uint32_t phys_c;
    if (ohci_dma_alloc(&r, 8192, 16, &phys_c) != NULL) {
        fprintf(stderr, "FAIL: over-size alloc should return NULL\n"); return 1;
    }

    printf("PASS: dma region bump + phys/virt roundtrip\n");
    return 0;
}
