#include <ntddk.h>
#include <intrin.h>
#include <stddef.h>
#include "device_context.h"

NTSTATUS OhciPci_BounceInit(PDEVICE_CONTEXT dc) {
    size_t total = (size_t)OHCIPCI_BOUNCE_SLAB_COUNT * OHCIPCI_BOUNCE_SLAB_BYTES;
    uint32_t phys;
    void *base = ohci_dma_alloc(&dc->DmaRegion, total, OHCIPCI_BOUNCE_SLAB_BYTES, &phys);
    if (!base) return STATUS_INSUFFICIENT_RESOURCES;

    dc->BouncePool.base      = (uint8_t *)base;
    dc->BouncePool.base_phys = phys;
    /* All slabs free: every bit set. */
    for (int i = 0; i < (OHCIPCI_BOUNCE_SLAB_COUNT + 31) / 32; i++) {
        dc->BouncePool.free_bitmap[i] = 0xFFFFFFFFu;
    }
    /* If count isn't a multiple of 32, mask the unused tail bits. */
    int rem = OHCIPCI_BOUNCE_SLAB_COUNT % 32;
    if (rem) {
        int last = (OHCIPCI_BOUNCE_SLAB_COUNT - 1) / 32;
        dc->BouncePool.free_bitmap[last] = (1u << rem) - 1u;
    }
    return STATUS_SUCCESS;
}

void *OhciPci_BounceAlloc(PDEVICE_CONTEXT dc, uint32_t *phys_out) {
    for (int word = 0; word < (OHCIPCI_BOUNCE_SLAB_COUNT + 31) / 32; word++) {
        ULONG bm = dc->BouncePool.free_bitmap[word];
        if (bm == 0) continue;
        ULONG bit;
        _BitScanForward(&bit, bm);
        int idx = word * 32 + (int)bit;
        if (idx >= OHCIPCI_BOUNCE_SLAB_COUNT) continue;
        dc->BouncePool.free_bitmap[word] &= ~(1u << bit);
        if (phys_out) *phys_out = dc->BouncePool.base_phys + idx * OHCIPCI_BOUNCE_SLAB_BYTES;
        return dc->BouncePool.base + idx * OHCIPCI_BOUNCE_SLAB_BYTES;
    }
    return NULL;
}

void OhciPci_BounceFree(PDEVICE_CONTEXT dc, void *ptr) {
    if (!ptr) return;
    ptrdiff_t off = (uint8_t *)ptr - dc->BouncePool.base;
    int idx = (int)(off / OHCIPCI_BOUNCE_SLAB_BYTES);
    int word = idx / 32;
    int bit  = idx % 32;
    dc->BouncePool.free_bitmap[word] |= (1u << bit);
}

/* Allocate `n_slabs` physically-contiguous slabs as one chunk. The pool's
 * underlying memory is one ohci_dma_alloc, so consecutive slab indices map
 * to consecutive physical pages — finding N consecutive free bits in the
 * bitmap is sufficient. Returns the VA of the first slab; phys_out gets
 * the matching physical address. Pass the same `n_slabs` to FreeBig. */
void *OhciPci_BounceAllocBig(PDEVICE_CONTEXT dc, ULONG n_slabs,
                             uint32_t *phys_out) {
    if (n_slabs == 0) return NULL;
    if (n_slabs == 1) return OhciPci_BounceAlloc(dc, phys_out);
    if (n_slabs > OHCIPCI_BOUNCE_SLAB_COUNT) return NULL;

    /* Linear scan for N consecutive set bits across the bitmap. */
    ULONG run = 0;
    int   start = -1;
    for (int idx = 0; idx < OHCIPCI_BOUNCE_SLAB_COUNT; idx++) {
        int word = idx / 32;
        int bit  = idx % 32;
        if (dc->BouncePool.free_bitmap[word] & (1u << bit)) {
            if (run == 0) start = idx;
            run++;
            if (run == n_slabs) {
                /* Found a run; mark them all used. */
                for (int j = 0; j < (int)n_slabs; j++) {
                    int k    = start + j;
                    int kw   = k / 32;
                    int kb   = k % 32;
                    dc->BouncePool.free_bitmap[kw] &= ~(1u << kb);
                }
                if (phys_out) *phys_out = dc->BouncePool.base_phys
                                          + start * OHCIPCI_BOUNCE_SLAB_BYTES;
                return dc->BouncePool.base + start * OHCIPCI_BOUNCE_SLAB_BYTES;
            }
        } else {
            run = 0;
            start = -1;
        }
    }
    return NULL;
}

void OhciPci_BounceFreeBig(PDEVICE_CONTEXT dc, void *ptr, ULONG n_slabs) {
    if (!ptr || n_slabs == 0) return;
    ptrdiff_t off = (uint8_t *)ptr - dc->BouncePool.base;
    int start = (int)(off / OHCIPCI_BOUNCE_SLAB_BYTES);
    for (int j = 0; j < (int)n_slabs; j++) {
        int idx  = start + j;
        int word = idx / 32;
        int bit  = idx % 32;
        dc->BouncePool.free_bitmap[word] |= (1u << bit);
    }
}
