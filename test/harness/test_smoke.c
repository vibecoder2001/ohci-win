#include <stdio.h>
#include <stdlib.h>
#include "ohci_core.h"

int main(void) {
    int rc = ohci_core_version();
    if (rc != OHCI_CORE_VERSION) {
        fprintf(stderr, "FAIL: version = %d, expected %d\n", rc, OHCI_CORE_VERSION);
        return 1;
    }
    printf("PASS: ohci_core_version = %d\n", rc);
    return 0;
}
