/*  isoc_mdl.h — MDL-walk ITD builder for OHCI isochronous OUT.
 *
 *  Spike replacement for the WdfDmaTransaction-based isoc DMA path.
 *  All functions named *_Locked require dc->CoreLock held by the caller.
 */
#pragma once

#include "device_context.h"

/*  Build ITDs for the URB at the head of ep->IsocQueuedUrbs (caller has
 *  already removed it). Walks the URB MDL via MmGetMdlPfnArray, emits one
 *  or more ITDs into the EP's ED, links them, and stages the URB on the
 *  per-EP "in-flight" list. Does NOT call WdfDmaTransactionExecute.
 *
 *  Returns STATUS_SUCCESS or a failure code; on failure the URB is queued
 *  for deferred completion via dc->DeferredCompletions (same convention
 *  the existing path uses on Execute failure).
 */
NTSTATUS
OhciPci_IsocBuildAndSubmit_Locked(
    _In_ POHCIPCI_EP_CONTEXT ep,
    _In_ OHCIPCI_URB_CTX    *uc);

/*  Unlink a retiring isoch URB from the per-EP in-flight list. Called from
 *  OhciPci_UrbComplete's isoch branch once per retiring URB, within the
 *  retire DPC (CoreLock held by caller).
 *
 *  Safe to call on legacy URBs that never went through BuildAndSubmit;
 *  the Flink==NULL check skips any that were zero-initialised but never
 *  inserted into IsocInFlightUrbs.
 */
VOID
OhciPci_IsocOnUrbRetire_Locked(_In_ OHCIPCI_URB_CTX *uc);

/*  Walk the EP's done-queue ITDs that this module emitted, accumulate
 *  per-PSW status into the owning URB, and when an URB is fully retired
 *  defer its WdfRequestComplete via dc->DeferredCompletions (so the lock
 *  release path completes it OUTSIDE CoreLock — see
 *  feedback_wdf_complete_under_spinlock.md).
 *
 *  Called from the existing retire DPC after the WDH drain.
 */
VOID
OhciPci_IsocRetireEmitted_Locked(
    _In_ POHCIPCI_EP_CONTEXT ep);

/*  WDF cancel callback for an isoch Request. Pauses the EP via
 *  OhciPci_EditHeadPSafely (mandatory per feedback_ohci_headp_edit_helper.md),
 *  waits one SOF, unlinks any not-yet-retired ITDs belonging to this URB,
 *  completes the Request USBD_STATUS_CANCELED, and clears Skip if more
 *  URBs remain queued (per feedback_ohci_ed_skip_clear_on_start.md).
 */
EVT_WDF_REQUEST_CANCEL OhciPci_IsocCancelEmitted;

/*  EP teardown — drains in-flight ITDs and IsocQueuedUrbs. Caller MUST
 *  pause the ED's list (BLE/CLE/PLE) before invoking, per
 *  feedback_ohci_destroy_must_pause_list.md.
 */
VOID
OhciPci_IsocEpTeardown_Locked(
    _In_ POHCIPCI_EP_CONTEXT ep);
