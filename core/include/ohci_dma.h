#ifndef OHCI_DMA_H
#define OHCI_DMA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A contiguous DMA-coherent memory region with a bump-allocator head.
 * Core code never mallocs; it consumes this region instead. */
struct ohci_dma_region {
    uint8_t *base;
    uint32_t phys_base;  /* 32-bit physical address (OHCI is 32-bit only) */
    size_t   size;
    size_t   offset;     /* Bump pointer; bytes allocated so far. */
};

void   ohci_dma_init(struct ohci_dma_region *r, void *base, uint32_t phys_base, size_t size);

/* Allocate `size` bytes aligned to `align` (power of two). On success returns
 * virtual pointer and writes the physical address to `phys_out`. On failure
 * (out of space) returns NULL and leaves `*phys_out` untouched. */
void  *ohci_dma_alloc(struct ohci_dma_region *r, size_t size, size_t align, uint32_t *phys_out);

/* Convert a physical address that resides within this region to its virtual
 * pointer. Returns NULL if the address is outside the region. */
void  *ohci_dma_virt_from_phys(const struct ohci_dma_region *r, uint32_t phys);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_DMA_H */
