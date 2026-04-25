#include <stddef.h>
#include <stdio.h>
#include "ohci_ed.h"
#include "ohci_td.h"

static int check_offset(const char *type_name, const char *field_name, size_t got, size_t expected) {
    if (got != expected) {
        fprintf(stderr, "FAIL: offsetof(%s,%s) = %zu, expected %zu\n", type_name, field_name, got, expected);
        return 1;
    }
    return 0;
}

static int check_size(const char *type_name, size_t got, size_t expected) {
    if (got != expected) {
        fprintf(stderr, "FAIL: sizeof(%s) = %zu, expected %zu\n", type_name, got, expected);
        return 1;
    }
    return 0;
}

#define EXPECT_OFFSET(type, field, expected) \
    check_offset(#type, #field, offsetof(type, field), expected)

#define EXPECT_SIZE(type, expected) \
    check_size(#type, sizeof(type), expected)

int main(void) {
    int ret = 0;

    /* OHCI 1.0a §4.2: Endpoint Descriptor is 16 bytes, 16-byte aligned */
    if (EXPECT_SIZE(struct ohci_ed, 16)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_ed, Control, 0x0)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_ed, TailP,   0x4)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_ed, HeadP,   0x8)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_ed, NextED,  0xC)) ret = 1;

    /* OHCI 1.0a §4.3: General TD is 16 bytes, 16-byte aligned */
    if (EXPECT_SIZE(struct ohci_td, 16)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_td, Control, 0x0)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_td, CBP,     0x4)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_td, NextTD,  0x8)) ret = 1;
    if (EXPECT_OFFSET(struct ohci_td, BE,      0xC)) ret = 1;

    if (ret == 0) {
        printf("PASS: ED + TD layouts verified\n");
    }
    return ret;
}
