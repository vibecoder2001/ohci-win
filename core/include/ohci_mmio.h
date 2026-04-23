#ifndef OHCI_MMIO_H
#define OHCI_MMIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_mmio_ops {
    void    *context;
    uint32_t (*read32)(void *context, uint32_t offset);
    void     (*write32)(void *context, uint32_t offset, uint32_t value);
    void     (*barrier)(void *context); /* full memory barrier before HC-visible writes */
};

#ifdef __cplusplus
}
#endif

#endif /* OHCI_MMIO_H */
