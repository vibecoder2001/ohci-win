#ifndef OHCI_LOG_H
#define OHCI_LOG_H

#include <ntddk.h>

/* Each function driver (func/pci, func/acpi, ...) defines OHCI_LOG_TAG via
 * its vcxproj as the DbgPrint prefix ("OhciPci", "OhciAcpi", ...). Default
 * to "Ohci" if not set so the shared sources still compile standalone. */
#ifndef OHCI_LOG_TAG
#define OHCI_LOG_TAG "Ohci"
#endif

#define LOG(fmt, ...) DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,  \
                                  OHCI_LOG_TAG ": " fmt "\n", ##__VA_ARGS__)

#endif /* OHCI_LOG_H */
