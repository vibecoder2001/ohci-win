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

