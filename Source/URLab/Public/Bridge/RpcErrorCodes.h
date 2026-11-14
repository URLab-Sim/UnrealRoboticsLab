// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "CoreMinimal.h"

// Error code constants matching wire/urlab_wire.yaml (dispatcher_errors and
// active message errors). Checked against the handlers by wire/check_emission.py.
namespace URLabError
{
inline const TCHAR* BadRequest = TEXT("bad_request");
inline const TCHAR* BadValue = TEXT("bad_value");
inline const TCHAR* BootMismatch = TEXT("boot_mismatch");
inline const TCHAR* Busy = TEXT("busy");
inline const TCHAR* CameraInvalid = TEXT("camera_invalid");
inline const TCHAR* CapabilityDisabled = TEXT("capability_disabled");
inline const TCHAR* CaptureInvalid = TEXT("capture_invalid");
inline const TCHAR* ControlClaimed = TEXT("control_claimed");
inline const TCHAR* ControlExpired = TEXT("control_expired");
inline const TCHAR* ControlInvalid = TEXT("control_invalid");
inline const TCHAR* CounterExhausted = TEXT("counter_exhausted");
inline const TCHAR* DataNegotiationRequired = TEXT("data_negotiation_required");
inline const TCHAR* DataSelectionConflict = TEXT("data_selection_conflict");
inline const TCHAR* DataTransferMismatch = TEXT("data_transfer_mismatch");
inline const TCHAR* DimMismatch = TEXT("dim_mismatch");
inline const TCHAR* EditorFailed = TEXT("editor_failed");
inline const TCHAR* EncodingError = TEXT("encoding_error");
inline const TCHAR* ForwardingFailed = TEXT("forwarding_failed");
inline const TCHAR* GrantInvalid = TEXT("grant_invalid");
inline const TCHAR* HistoryMiss = TEXT("history_miss");
inline const TCHAR* InvalidState = TEXT("invalid_state");
inline const TCHAR* InvalidStateMask = TEXT("invalid_state_mask");
inline const TCHAR* MissingField = TEXT("missing_field");
inline const TCHAR* ModelNotReady = TEXT("model_not_ready");
inline const TCHAR* MutationCommittedObservationUnavailable = TEXT("mutation_committed_observation_unavailable");
inline const TCHAR* NotControlOwner = TEXT("not_control_owner");
inline const TCHAR* NotInEditor = TEXT("not_in_editor");
inline const TCHAR* NotReady = TEXT("not_ready");
inline const TCHAR* OwnerInvalid = TEXT("owner_invalid");
inline const TCHAR* OwnerUnavailable = TEXT("owner_unavailable");
inline const TCHAR* PatchInvalid = TEXT("patch_invalid");
inline const TCHAR* PayloadBackingFailure = TEXT("payload_backing_failure");
inline const TCHAR* PayloadChunkLimitExceeded = TEXT("payload_chunk_limit_exceeded");
inline const TCHAR* PayloadChunkTooLarge = TEXT("payload_chunk_too_large");
inline const TCHAR* PayloadClockInvalid = TEXT("payload_clock_invalid");
inline const TCHAR* PayloadConfigurationInvalid = TEXT("payload_configuration_invalid");
inline const TCHAR* PayloadExpired = TEXT("payload_expired");
inline const TCHAR* PayloadIdentityMismatch = TEXT("payload_identity_mismatch");
inline const TCHAR* PayloadIncomplete = TEXT("payload_incomplete");
inline const TCHAR* PayloadIntegrityMismatch = TEXT("payload_integrity_mismatch");
inline const TCHAR* PayloadNotFound = TEXT("payload_not_found");
inline const TCHAR* PayloadOutOfBounds = TEXT("payload_out_of_bounds");
inline const TCHAR* PayloadOverlap = TEXT("payload_overlap");
inline const TCHAR* PayloadQuotaExceeded = TEXT("payload_quota_exceeded");
inline const TCHAR* PayloadTooLarge = TEXT("payload_too_large");
inline const TCHAR* RequestCapacity = TEXT("request_capacity");
inline const TCHAR* RequestIdConflict = TEXT("request_id_conflict");
inline const TCHAR* RequestRetired = TEXT("request_retired");
inline const TCHAR* ResultUnavailable = TEXT("result_unavailable");
inline const TCHAR* RevisionConflict = TEXT("revision_conflict");
inline const TCHAR* ServerDraining = TEXT("server_draining");
inline const TCHAR* ServerSchemaError = TEXT("server_schema_error");
inline const TCHAR* ServiceUnavailable = TEXT("service_unavailable");
inline const TCHAR* SessionExpired = TEXT("session_expired");
inline const TCHAR* ShmChallengeExpired = TEXT("shm_challenge_expired");
inline const TCHAR* ShmChallengeInvalid = TEXT("shm_challenge_invalid");
inline const TCHAR* ShmUnavailable = TEXT("shm_unavailable");
inline const TCHAR* ShuttingDown = TEXT("shutting_down");
inline const TCHAR* SourceInvalid = TEXT("source_invalid");
inline const TCHAR* SourceRevisionConflict = TEXT("source_revision_conflict");
inline const TCHAR* StaleModelGeneration = TEXT("stale_model_generation");
inline const TCHAR* StateUnavailable = TEXT("state_unavailable");
inline const TCHAR* StreamInvalid = TEXT("stream_invalid");
inline const TCHAR* StreamUnavailable = TEXT("stream_unavailable");
inline const TCHAR* Timeout = TEXT("timeout");
inline const TCHAR* UnknownActuator = TEXT("unknown_actuator");
inline const TCHAR* UnknownJob = TEXT("unknown_job");
inline const TCHAR* UnknownOp = TEXT("unknown_op");
inline const TCHAR* Unsupported = TEXT("unsupported");
inline const TCHAR* WrongClockMode = TEXT("wrong_clock_mode");
inline const TCHAR* WrongTransport = TEXT("wrong_transport");
} // namespace URLabError
