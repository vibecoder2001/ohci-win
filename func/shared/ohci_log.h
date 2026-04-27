#ifndef OHCI_LOG_H
#define OHCI_LOG_H

#include <ntddk.h>

/* Each function driver (func/pci, func/acpi, ...) defines OHCI_LOG_TAG via
 * its vcxproj as the DbgPrint prefix ("OhciPci", "OhciAcpi", ...). Default
 * to "Ohci" if not set so the shared sources still compile standalone. */
#ifndef OHCI_LOG_TAG
#define OHCI_LOG_TAG "Ohci"
#endif

#if DBG
#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,  \
                                  OHCI_LOG_TAG ": " fmt "\n", ##__VA_ARGS__)
#else
/* In release builds we still want the compiler to consider every argument
 * "referenced" so locals and formals that exist only for diagnostics don't
 * trip C4100/C4189-as-error. The 0 && ... guard prevents runtime execution
 * while keeping type-checking and use-tracking intact. */
#define LOG(fmt, ...) ((void)sizeof(DbgPrintEx(DPFLTR_IHVDRIVER_ID,        \
                                               DPFLTR_ERROR_LEVEL,         \
                                               OHCI_LOG_TAG ": " fmt "\n", \
                                               ##__VA_ARGS__)))
#endif

#endif /* OHCI_LOG_H */
