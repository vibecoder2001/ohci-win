/*  isoc_mdl.h — MDL-walk ITD builder for OHCI isochronous OUT.
 *
 *  All functions named *_Locked require dc->CoreLock held by the caller.
 *  OhciPci_IsocEpTeardown is the one outlier — it acquires CoreLock itself
 *  because it runs at PASSIVE from EP cleanup, hence no _Locked suffix.
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

/*  WDF cancel callback for an isoch Request. Currently a no-op stub —
 *  OhciPci_IsocBuildAndSubmit_Locked deliberately does NOT call
 *  WdfRequestMarkCancelable, so WDF never invokes this callback. The
 *  usbaudio stop/restart cycle is covered by UcxEndpointPurge (drains
 *  IsocQueuedUrbs) plus OhciPci_IsocEpTeardown (drains
 *  IsocInFlightUrbs). See isoc_mdl.c for the implementation rationale
 *  and a sketch of what a real per-URB cancel would look like.
 */
EVT_WDF_REQUEST_CANCEL OhciPci_IsocCancelEmitted;

/*  EP teardown — drains in-flight ITDs and IsocQueuedUrbs. Caller MUST
 *  pause the ED's list (BLE/CLE/PLE) before invoking, per
 *  feedback_ohci_destroy_must_pause_list.md.
 */
VOID
OhciPci_IsocEpTeardown(
    _In_ POHCIPCI_EP_CONTEXT ep);
