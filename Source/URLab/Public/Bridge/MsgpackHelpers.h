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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/**
 * Bidirectional FJsonObject <-> msgpack helpers (Epic's msgpack-cxx in
 * `clmdep_msgpack`). All JSON numbers pack as float64 — round-trip
 * precision matters for policies that mirror state from MJ.
 *
 * Numerical arrays (qpos / qvel / ctrl / sensors) pack as msgpack arrays
 * of float64, not `bin`: Python's `msgpack.unpackb` decodes `bin` as
 * `bytes` (caller would need `np.frombuffer`), arrays decode straight to
 * `list[float]`. `bin` is reserved for genuinely raw payloads (handshake
 * `mjb` blob, camera frame bytes) — the `__b64__` key suffix marks those
 * on the JSON side so PackJsonObject re-emits as msgpack `bin`.
 */

class URLAB_API FURLabMsgpackUtil
{
public:
    /** Pack an FJsonObject into a binary msgpack buffer. Caller owns OutBuf. */
    static void PackJsonObject(const TSharedPtr<FJsonObject>& Object, TArray<uint8>& OutBuf);

    /** Unpack a binary msgpack buffer into an FJsonObject. Returns false on
     *  parse error. The unpack treats msgpack `bin` as base64-encoded
     *  strings inside the resulting JSON tree, so existing handlers don't
     *  need a separate code path for binary fields. */
    static bool UnpackToJsonObject(const uint8* Data, int32 Size, TSharedPtr<FJsonObject>& OutObject);

    /** Pack-and-set a binary blob under a JSON object's field. Stores as
     *  base64 string with a `__b64__` suffix on the key so PackJsonObject
     *  knows to re-emit it as a real msgpack `bin` rather than a string.
     *  Used by handshake (MJB) and camera replies (pixel buffers). */
    static void SetBinaryField(TSharedPtr<FJsonObject>& Obj, const FString& Field, const uint8* Data, int32 Size);
};
