#include <string.h>
#include "ohci_pool.h"

/* Shared generic implementation via macros so ED and TD share code without
 * runtime indirection. Each pool type has identical structure; only the
 * element type differs. */

#define POOL_INIT(POOL_T, ELEM_T)                                            \
    int POOL_T##_init(struct POOL_T *p, struct ohci_dma_region *r,           \
                      uint16_t count) {                                      \
        if (count == 0 || count == 0xFFFF) return -1;                        \
        uint32_t phys;                                                       \
        void *mem = ohci_dma_alloc(r, sizeof(ELEM_T) * count, 16, &phys);    \
        if (!mem) return -1;                                                 \
        p->elems = (ELEM_T *)mem;                                            \
        p->elems_phys = phys;                                                \
        p->capacity = count;                                                 \
        p->next = (uint16_t *)ohci_dma_alloc(r,                              \
            sizeof(uint16_t) * count, sizeof(uint16_t), NULL);               \
        if (!p->next) return -1;                                             \
        for (uint16_t i = 0; i < count; i++) {                               \
            p->next[i] = (uint16_t)(i + 1 < count ? i + 1 : 0xFFFF);         \
        }                                                                    \
        p->free_head = 0;                                                    \
        memset(p->elems, 0, sizeof(ELEM_T) * count);                         \
        return 0;                                                            \
    }

#define POOL_ALLOC(POOL_T, ELEM_T)                                           \
    ELEM_T *POOL_T##_alloc(struct POOL_T *p, uint32_t *phys_out) {           \
        if (p->free_head == 0xFFFF) return NULL;                             \
        uint16_t idx = p->free_head;                                         \
        p->free_head = p->next[idx];                                         \
        ELEM_T *e = &p->elems[idx];                                          \
        memset(e, 0, sizeof(*e));                                            \
        if (phys_out) *phys_out = p->elems_phys + (uint32_t)(idx * sizeof(ELEM_T)); \
        return e;                                                            \
    }

#define POOL_FREE(POOL_T, ELEM_T)                                            \
    void POOL_T##_free(struct POOL_T *p, ELEM_T *e) {                        \
        uint16_t idx = (uint16_t)(e - p->elems);                             \
        p->next[idx] = p->free_head;                                         \
        p->free_head = idx;                                                  \
    }

POOL_INIT (ohci_ed_pool, struct ohci_ed)
POOL_ALLOC(ohci_ed_pool, struct ohci_ed)
POOL_FREE (ohci_ed_pool, struct ohci_ed)

POOL_INIT (ohci_td_pool, struct ohci_td)
POOL_ALLOC(ohci_td_pool, struct ohci_td)
POOL_FREE (ohci_td_pool, struct ohci_td)
