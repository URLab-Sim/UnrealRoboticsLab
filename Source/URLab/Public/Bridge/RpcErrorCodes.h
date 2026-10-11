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

// Every code a reply can carry in {op:"error", code, message}. There is exactly
// one error envelope on the wire and exactly one place a code may be spelled, so
// a client can enumerate the failures it must handle; a code typed out at a call
// site is invisible to that enumeration and is what let four upload codes and
// three ownership codes ship undeclared. Nothing here may be dead either: a code
// nothing raises is a promise to a client that cannot be kept.
namespace URLabError
{
// Dispatcher-level errors
inline const TCHAR* BadRequest = TEXT("bad_request");
inline const TCHAR* MissingOp = TEXT("missing_op");
inline const TCHAR* UnknownOp = TEXT("unknown_op");
inline const TCHAR* NotInEditor = TEXT("not_in_editor");
// The two shapes an op can be missing its context in. Split because they are
// different facts about the instance: a render server HAS a world and will never
// have a simulation, so telling a client "no manager" when it asked for a light
// sent it looking for a PIE session that is never coming.
inline const TCHAR* NoSimulation = TEXT("no_simulation");
inline const TCHAR* NoWorld = TEXT("no_world");
inline const TCHAR* SessionExpired = TEXT("session_expired");
inline const TCHAR* MissingField = TEXT("missing_field");
inline const TCHAR* NotReady = TEXT("not_ready");
inline const TCHAR* BadMode = TEXT("bad_mode");
inline const TCHAR* BadValue = TEXT("bad_value");
inline const TCHAR* UnknownArticulation = TEXT("unknown_articulation");
inline const TCHAR* UnknownBody = TEXT("unknown_body");
inline const TCHAR* NotMocapBody = TEXT("not_mocap_body");
inline const TCHAR* UnknownKeyframe = TEXT("unknown_keyframe");
inline const TCHAR* DimMismatch = TEXT("dim_mismatch");
inline const TCHAR* NoJoints = TEXT("no_joints");
inline const TCHAR* StepTimeout = TEXT("step_timeout");
inline const TCHAR* Timeout = TEXT("timeout");
inline const TCHAR* ReplyTooLarge = TEXT("reply_too_large");
inline const TCHAR* WrongTransport = TEXT("wrong_transport");
inline const TCHAR* CapabilityDisabled = TEXT("capability_disabled");
inline const TCHAR* UnknownJob = TEXT("unknown_job");
inline const TCHAR* ShuttingDown = TEXT("shutting_down");
// Control ownership and instance leasing
inline const TCHAR* NotControlOwner = TEXT("not_control_owner");
inline const TCHAR* ControlClaimed = TEXT("control_claimed");
inline const TCHAR* Busy = TEXT("busy");
// Content-addressed model upload
inline const TCHAR* PayloadTooLarge = TEXT("payload_too_large");
inline const TCHAR* UnknownUpload = TEXT("unknown_upload");
inline const TCHAR* HashMismatch = TEXT("hash_mismatch");
inline const TCHAR* ImportFailed = TEXT("import_failed");
inline const TCHAR* PathNotWritable = TEXT("path_not_writable");
inline const TCHAR* PathNotReadable = TEXT("path_not_readable");
} // namespace URLabError
