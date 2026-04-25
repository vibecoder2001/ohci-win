#include <string.h>
#include "ohci_dma.h"

void ohci_dma_init(struct ohci_dma_region *r, void *base, uint32_t phys_base, size_t size) {
    r->base      = (uint8_t *)base;
    r->phys_base = phys_base;
    r->size      = size;
    r->offset    = 0;
    if (size) memset(base, 0, size);
}

static size_t align_up(size_t n, size_t align) {
    return (n + (align - 1)) & ~(align - 1);
}

void *ohci_dma_alloc(struct ohci_dma_region *r, size_t size, size_t align, uint32_t *phys_out) {
    size_t aligned = align_up(r->offset, align);
    if (aligned + size > r->size) return NULL;
    void *v = r->base + aligned;
    if (phys_out) *phys_out = r->phys_base + (uint32_t)aligned;
    r->offset = aligned + size;
    return v;
}

void *ohci_dma_virt_from_phys(const struct ohci_dma_region *r, uint32_t phys) {
    if (phys < r->phys_base) return NULL;
    uint32_t off = phys - r->phys_base;
    if (off >= r->size) return NULL;
    return r->base + off;
}
