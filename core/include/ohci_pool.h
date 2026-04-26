#ifndef OHCI_POOL_H
#define OHCI_POOL_H

#include "ohci_types.h"
#include "ohci_dma.h"
#include "ohci_ed.h"
#include "ohci_td.h"
#include "ohci_itd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ohci_ed_pool {
    struct ohci_ed *elems;
    uint32_t        elems_phys;
    uint16_t        capacity;
    uint16_t        free_head;     /* index into elems; 0xFFFF = empty */
    uint16_t       *next;          /* free-list links, capacity entries */
};

struct ohci_td_pool {
    struct ohci_td *elems;
    uint32_t        elems_phys;
    uint16_t        capacity;
    uint16_t        free_head;
    uint16_t       *next;
};

int             ohci_ed_pool_init (struct ohci_ed_pool *p, struct ohci_dma_region *r, uint16_t count);
struct ohci_ed *ohci_ed_pool_alloc(struct ohci_ed_pool *p, uint32_t *phys_out);
void            ohci_ed_pool_free (struct ohci_ed_pool *p, struct ohci_ed *e);

int             ohci_td_pool_init (struct ohci_td_pool *p, struct ohci_dma_region *r, uint16_t count);
struct ohci_td *ohci_td_pool_alloc(struct ohci_td_pool *p, uint32_t *phys_out);
void            ohci_td_pool_free (struct ohci_td_pool *p, struct ohci_td *t);

struct ohci_itd_pool {
    struct ohci_itd *elems;
    uint32_t         elems_phys;
    uint16_t         capacity;
    uint16_t         free_head;
    uint16_t        *next;
};

int              ohci_itd_pool_init (struct ohci_itd_pool *p, struct ohci_dma_region *r, uint16_t count);
struct ohci_itd *ohci_itd_pool_alloc(struct ohci_itd_pool *p, uint32_t *phys_out);
void             ohci_itd_pool_free (struct ohci_itd_pool *p, struct ohci_itd *t);

#ifdef __cplusplus
}
#endif

#endif /* OHCI_POOL_H */
