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

namespace URLabError
{
	// Dispatcher-level errors
	inline const TCHAR* BadRequest      = TEXT("bad_request");
	inline const TCHAR* MissingOp       = TEXT("missing_op");
	inline const TCHAR* UnknownOp       = TEXT("unknown_op");
	inline const TCHAR* NotInEditor     = TEXT("not_in_editor");
	inline const TCHAR* NoActiveManager = TEXT("no_active_manager");
	inline const TCHAR* SessionExpired  = TEXT("session_expired");
	inline const TCHAR* MissingField    = TEXT("missing_field");
	inline const TCHAR* NotReady        = TEXT("not_ready");
	inline const TCHAR* BadMode         = TEXT("bad_mode");
	inline const TCHAR* BadValue        = TEXT("bad_value");
	inline const TCHAR* UnknownArticulation = TEXT("unknown_articulation");
	inline const TCHAR* UnknownBody     = TEXT("unknown_body");
	inline const TCHAR* NotMocapBody    = TEXT("not_mocap_body");
	inline const TCHAR* UnknownKeyframe = TEXT("unknown_keyframe");
	inline const TCHAR* DimMismatch     = TEXT("dim_mismatch");
	inline const TCHAR* NoJoints        = TEXT("no_joints");
	inline const TCHAR* NoTwistController = TEXT("no_twist_controller");
	inline const TCHAR* NoController    = TEXT("no_controller");
	inline const TCHAR* ControllerSchemaViolation = TEXT("controller_schema_violation");
	inline const TCHAR* ModeLockedByServer = TEXT("mode_locked_by_server");
	inline const TCHAR* StepTimeout     = TEXT("step_timeout");
	inline const TCHAR* Timeout         = TEXT("timeout");
	inline const TCHAR* ReplyTooLarge   = TEXT("reply_too_large");
	inline const TCHAR* WrongTransport  = TEXT("wrong_transport");
	inline const TCHAR* UnknownJob      = TEXT("unknown_job");
	inline const TCHAR* ShuttingDown    = TEXT("shutting_down");
	// Recording
	inline const TCHAR* RecordingAlreadyActive = TEXT("recording_already_active");
	inline const TCHAR* RecordingNotActive     = TEXT("recording_not_active");
	inline const TCHAR* PathNotWritable = TEXT("path_not_writable");
	inline const TCHAR* PathNotReadable = TEXT("path_not_readable");
	inline const TCHAR* ReplaySessionNotFound = TEXT("replay_session_not_found");
	inline const TCHAR* ReplayRequiresStepped = TEXT("replay_requires_stepped");
}
