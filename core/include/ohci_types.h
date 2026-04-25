/* ohci_types.h — portable integer typedefs for kernel-mode and user-mode.
 *
 * In kernel-mode (WindowsKernelModeDriver10.0 / WDK) <stdint.h> may be
 * pulled in before ntddk.h has been processed, causing conflicts.  We use
 * the _KERNEL_MODE macro (set by the WDK toolchain) to select a safe path.
 *
 * All core headers MUST include this file instead of <stdint.h>/<stddef.h>
 * directly.
 */
#ifndef OHCI_TYPES_H
#define OHCI_TYPES_H

#ifdef _KERNEL_MODE
/* WDK kernel-mode: use WDM-defined UINT8/UINT16/UINT32/INT32/SIZE_T types. */
#include <wdm.h>
typedef UINT8   uint8_t;
typedef UINT16  uint16_t;
typedef UINT32  uint32_t;
typedef INT32   int32_t;
#ifndef _SIZE_T_DEFINED
#define _SIZE_T_DEFINED
typedef SIZE_T  size_t;
#endif
#else
/* User-mode / Tier-1 CMake build. */
#include <stdint.h>
#include <stddef.h>
#endif /* _KERNEL_MODE */

#endif /* OHCI_TYPES_H */
