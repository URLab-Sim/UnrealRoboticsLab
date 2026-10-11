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
 * `clmdep_msgpack`). JSON numbers pack as float64 by default — round-trip
 * precision matters for policies that mirror state from MJ — except fields
 * written through SetIntField, which the wire schema declares as `int` and
 * which pack as msgpack int so a UE frame and a Python frame are comparable.
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

	/** Pack-and-set a binary blob under a JSON object's field so PackJsonObject
	 *  emits it as a real msgpack `bin`. Owns a single copy of the bytes (no
	 *  base64), so the caller's buffer need not outlive the reply. Used by the
	 *  handshake (MJB blob) and any caller without a shareable source buffer. */
	static void SetBinaryField(TSharedPtr<FJsonObject>& Obj, const FString& Field, const uint8* Data, int32 Size);

	/** Zero-copy variant of SetBinaryField: packs `Data`/`Size` straight through
	 *  as msgpack `bin` with no base64 and no intermediate copy. `Keeper` must own
	 *  the buffer that `Data` points into; it is retained until the reply is packed,
	 *  so a caller can hand raw pixel bytes directly from a shared frame. */
	static void SetBinaryFieldShared(TSharedPtr<FJsonObject>& Obj, const FString& Field,
		const uint8* Data, int32 Size, TSharedPtr<const void, ESPMode::ThreadSafe> Keeper);

	/** Set a field the wire schema declares as `int` so PackJsonObject emits a
	 *  msgpack int instead of the float64 every other number gets. Use it for every
	 *  declared-int field -- counts, ids, frame numbers -- because a Python owner
	 *  packs those as int and the two encodings are not byte-comparable otherwise.
	 *
	 *  The field keeps its plain name: the value is a normal JSON number and the
	 *  int-ness is recorded out of band under IntKeysFieldName(), so every reader
	 *  of the FJsonObject, and the JSON transport, are unaffected.
	 *
	 *  Scalars only. An ARRAY of declared ints still packs as float64 -- there is
	 *  nowhere to hang a per-element tag -- so no wire field is currently declared
	 *  as an int array. */
	static void SetIntField(TSharedPtr<FJsonObject>& Obj, const FString& Field, int64 Value);

	/** The reserved key SetIntField keeps its field-name list under. Exposed so a
	 *  reply-schema check can skip the bookkeeping key. */
	static const TCHAR* IntKeysFieldName();
};
