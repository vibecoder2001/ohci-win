#include <string.h>
#include "ohci_control.h"

/* Build a General TD representing one stage of a Control transfer. */
static void fill_td(struct ohci_td *td,
                    uint32_t dp,
                    uint32_t toggle,
                    uint32_t cbp,
                    uint32_t be,
                    uint32_t next_td_phys,
                    int allow_short) {
    uint32_t ctrl = OHCI_TD_DI_NO_INTR | dp | toggle;
    ctrl |= (OHCI_CC_NOTACCESSED << OHCI_TD_CC_SHIFT);
    if (allow_short) ctrl |= OHCI_TD_R;
    td->Control = ctrl;
    td->CBP     = cbp;
    td->NextTD  = next_td_phys;
    td->BE      = be;
}

int ohci_build_control_chain(struct ohci_urb *urb,
                             struct ohci_td_pool *tdp,
                             struct ohci_td **head_out,
                             struct ohci_td **tail_out) {
    /* Always SETUP + STATUS; DATA present iff buffer && length. */
    int have_data = urb->buffer && urb->length > 0;

    uint32_t setup_td_phys = 0, data_td_phys = 0, status_td_phys = 0;
    struct ohci_td *setup_td  = ohci_td_pool_alloc(tdp, &setup_td_phys);
    struct ohci_td *data_td   = NULL;
    struct ohci_td *status_td = ohci_td_pool_alloc(tdp, &status_td_phys);

    if (have_data) {
        data_td = ohci_td_pool_alloc(tdp, &data_td_phys);
    }
    if (!setup_td || !status_td || (have_data && !data_td)) {
        if (setup_td)  ohci_td_pool_free(tdp, setup_td);
        if (data_td)   ohci_td_pool_free(tdp, data_td);
        if (status_td) ohci_td_pool_free(tdp, status_td);
        return -1;
    }

    /* SETUP: DATA0, 8 bytes at urb->setup_phys, no short-packet rounding. */
    uint32_t next_after_setup = have_data ? data_td_phys : status_td_phys;
    fill_td(setup_td,
            OHCI_TD_DP_SETUP,
            OHCI_TD_T_DATA0,
            urb->setup_phys,
            urb->setup_phys + 7,
            next_after_setup,
            /*allow_short=*/0);

    /* DATA: DATA1, direction from URB, short packets allowed. */
    if (have_data) {
        uint32_t dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_IN : OHCI_TD_DP_OUT;
        fill_td(data_td,
                dp,
                OHCI_TD_T_DATA1,
                urb->buffer_phys,
                urb->buffer_phys + urb->length - 1,
                status_td_phys,
                /*allow_short=*/1);
    }

    /* STATUS: DATA1, opposite direction. Zero-length (CBP=BE=0). */
    uint32_t status_dp;
    if (have_data) {
        status_dp = (urb->direction == OHCI_URB_DIR_IN) ? OHCI_TD_DP_OUT : OHCI_TD_DP_IN;
    } else {
        /* No data stage: STATUS is always IN (device acks). */
        status_dp = OHCI_TD_DP_IN;
    }
    fill_td(status_td,
            status_dp,
            OHCI_TD_T_DATA1,
            /*cbp=*/0,
            /*be =*/0,
            /*next=*/0,
            /*allow_short=*/0);

    *head_out = setup_td;
    *tail_out = status_td;
    return 0;
}
