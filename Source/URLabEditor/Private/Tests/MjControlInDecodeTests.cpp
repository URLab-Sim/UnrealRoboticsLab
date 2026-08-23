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
// endorsed by, or sponsored by Epic Games, Inc.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

// ============================================================================
// MjControlInDecodeTests.cpp
//
// Tier-1 GAP 1.d (UE / H9): the async control-in path in
// UURLabZmqSubscribeTransport decodes a msgpack `{ids:[...], vals:[...]}` map
// (source-of-truth §9.3) through FURLabMsgpackUtil -- NOT the retired
// little-endian `[i32 n][i32 id, f32 val]*` binary format parsed with raw
// `*(int32*)` casts. The decode + the ids/vals extraction it feeds are the
// bounds-safe layer H9 hardened.
//
// The socket-read loop itself needs a live SUB socket + a manager, so the pure
// unit under test here is the msgpack parse the loop delegates to: a well-formed
// `{ids,vals}` buffer round-trips to the exact id/value pairs the loop applies,
// and a truncated / garbage buffer is rejected cleanly (no OOB read, no crash) --
// the H9 assertion. The full "ctrl lands on the actuator" step is covered by the
// live socket loop (Tier-2 integration).
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Utils/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

// ---------------------------------------------------------------------------
// URLab.Transport.ControlInIdsValsDecode
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlInIdsValsDecode,
	"URLab.Transport.ControlInIdsValsDecode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlInIdsValsDecode::RunTest(const FString& Parameters)
{
	// --- (1) a well-formed {ids,vals} buffer decodes to matching pairs --------
	// Build the exact map the Python senders emit (ids: ints, vals: floats).
	const TArray<int32> ExpectIds = {5, 6, 7};
	const TArray<double> ExpectVals = {0.1, -0.25, 1.5};

	TSharedPtr<FJsonObject> Src = MakeShared<FJsonObject>();
	{
		TArray<TSharedPtr<FJsonValue>> Ids, Vals;
		for (int32 Id : ExpectIds) Ids.Add(MakeShared<FJsonValueNumber>((double)Id));
		for (double V : ExpectVals) Vals.Add(MakeShared<FJsonValueNumber>(V));
		Src->SetArrayField(TEXT("ids"), Ids);
		Src->SetArrayField(TEXT("vals"), Vals);
	}

	TArray<uint8> Packed;
	FURLabMsgpackUtil::PackJsonObject(Src, Packed);
	TestTrue(TEXT("packed control buffer non-empty"), Packed.Num() > 0);

	TSharedPtr<FJsonObject> Decoded;
	const bool bOk = FURLabMsgpackUtil::UnpackToJsonObject(Packed.GetData(), Packed.Num(), Decoded);
	TestTrue(TEXT("unpack {ids,vals} succeeds"), bOk);
	if (bOk && Decoded.IsValid())
	{
		const TArray<TSharedPtr<FJsonValue>>* IdsArr = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* ValsArr = nullptr;
		const bool bHasIds = Decoded->TryGetArrayField(TEXT("ids"), IdsArr) && IdsArr;
		const bool bHasVals = Decoded->TryGetArrayField(TEXT("vals"), ValsArr) && ValsArr;
		TestTrue(TEXT("decoded has ids array"), bHasIds);
		TestTrue(TEXT("decoded has vals array"), bHasVals);
		if (bHasIds && bHasVals)
		{
			// The loop applies FMath::Min(ids,vals) pairs; here they match.
			TestEqual(TEXT("ids length"), IdsArr->Num(), ExpectIds.Num());
			TestEqual(TEXT("vals length"), ValsArr->Num(), ExpectVals.Num());
			const int32 N = FMath::Min(IdsArr->Num(), ValsArr->Num());
			for (int32 i = 0; i < N; ++i)
			{
				TestEqual(FString::Printf(TEXT("id[%d]"), i),
					(int32)(*IdsArr)[i]->AsNumber(), ExpectIds[i]);
				TestEqual(FString::Printf(TEXT("val[%d]"), i),
					(*ValsArr)[i]->AsNumber(), ExpectVals[i], 1e-6);
			}
		}
	}

	// --- (2) a truncated buffer is rejected cleanly (no OOB read) -------------
	// H9: the decoder must bounds-check before trusting any length header, so a
	// half-buffer neither reads past the end nor crashes -- it fails the unpack.
	if (Packed.Num() >= 4)
	{
		TArray<uint8> Truncated(Packed.GetData(), Packed.Num() / 2);
		TSharedPtr<FJsonObject> Out;
		const bool bTrunc = FURLabMsgpackUtil::UnpackToJsonObject(
			Truncated.GetData(), Truncated.Num(), Out);
		TestFalse(TEXT("truncated control buffer rejected"), bTrunc);
	}

	// --- (3) a garbage / oversized-looking buffer is rejected cleanly ---------
	// A msgpack array16 header (0xdc) claiming 0xFFFF elements with no payload:
	// a raw `4 + n*8` multiply-then-read would run off the end; the bounds-safe
	// parser must reject it without reading the promised elements.
	{
		TArray<uint8> Bogus = {0xdc, 0xFF, 0xFF};
		TSharedPtr<FJsonObject> Out;
		const bool bBogus = FURLabMsgpackUtil::UnpackToJsonObject(
			Bogus.GetData(), Bogus.Num(), Out);
		TestFalse(TEXT("oversized-length buffer rejected (no OOB)"), bBogus);
	}

	// --- (4) empty buffer is a clean no-op reject ----------------------------
	{
		TSharedPtr<FJsonObject> Out;
		const bool bEmpty = FURLabMsgpackUtil::UnpackToJsonObject(nullptr, 0, Out);
		TestFalse(TEXT("empty buffer rejected"), bEmpty);
	}

	return true;
}
