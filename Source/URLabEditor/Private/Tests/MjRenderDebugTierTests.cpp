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
// MjRenderDebugTierTests.cpp
//
// Tier-1 GAP 1.e (UE): the tiered render frame builder must always emit the
// transform tier and gate the debug tier (source-of-truth §8.2) behind
// FMjRenderDebugCaps -- a subscriber that asks for neither StreamContacts nor
// StreamOverlay pays ZERO extra bytes, and the contact list is count-capped.
//
// Exercises AAMjManager::BuildRenderFrame + AppendRenderDebugFields directly
// (the Phase 2.3 / 9.1 seam), no bus, no display. Mirrors the Python
// test_render_debug_tier.py assertions on the UE code path.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include <mujoco/mujoco.h>

namespace
{
// The debug-tier keys AppendRenderDebugFields may add; none may appear with
// empty caps.
static const TCHAR* const kDebugKeys[] = {
	TEXT("contacts"), TEXT("xfrc_applied"), TEXT("subtree_com"), TEXT("ctrl"),
	TEXT("act"), TEXT("wrap_xpos"), TEXT("wrap_obj"), TEXT("ten_wrapadr"),
	TEXT("ten_wrapnum"), TEXT("eq_active"), TEXT("eq_anchor"), TEXT("sensordata"),
	TEXT("light_xpos"), TEXT("light_xdir"),
};

static bool HasAnyDebugKey(const TSharedPtr<FJsonObject>& Frame)
{
	for (const TCHAR* Key : kDebugKeys)
		if (Frame->HasField(Key))
			return true;
	return false;
}
} // namespace

// ---------------------------------------------------------------------------
// URLab.Fast.RenderFrameDebugTierGating
//   Empty caps -> the frame carries only the transform tier (bxpos/bxquat),
//   byte-identical to BuildRenderFrame's output, with no debug keys. Contacts
//   caps add a (capped) contacts[] and nothing else; overlay caps add the
//   derived-decor bundle sized by the model dimensions.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderFrameDebugTierGating,
	"URLab.Fast.RenderFrameDebugTierGating",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenderFrameDebugTierGating::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d)
	{
		AddError(TEXT("Model/data missing"));
		S.Cleanup();
		return false;
	}
	// Populate contacts / derived quantities so the debug tier has data to emit.
	mj_forward(m, d);

	using FCaps = AAMjManager::FMjRenderDebugCaps;

	// --- (1) empty caps -> transforms only, zero debug bytes -----------------
	{
		TSharedPtr<FJsonObject> Frame = S.Manager->BuildRenderFrame(m, d);
		TestTrue(TEXT("transform tier present: bxpos"), Frame->HasField(TEXT("bxpos")));
		TestTrue(TEXT("transform tier present: bxquat"), Frame->HasField(TEXT("bxquat")));
		const int32 KeysBefore = Frame->Values.Num();

		FCaps None; // all off by default
		S.Manager->AppendRenderDebugFields(Frame, m, d, None);
		TestFalse(TEXT("no debug keys with empty caps"), HasAnyDebugKey(Frame));
		TestEqual(TEXT("empty caps add zero keys (transform bytes identical)"),
			Frame->Values.Num(), KeysBefore);
	}

	// --- (2) StreamContacts -> contacts[] present + count-capped, no overlay --
	{
		TSharedPtr<FJsonObject> Frame = S.Manager->BuildRenderFrame(m, d);
		FCaps Caps;
		Caps.bStreamContacts = true;
		Caps.MaxContacts = 4;
		S.Manager->AppendRenderDebugFields(Frame, m, d, Caps);

		TestTrue(TEXT("contacts[] present under StreamContacts"),
			Frame->HasField(TEXT("contacts")));
		// Overlay bundle stays absent (contacts-only subscription).
		TestFalse(TEXT("no xfrc_applied without StreamOverlay"),
			Frame->HasField(TEXT("xfrc_applied")));
		TestFalse(TEXT("no subtree_com without StreamOverlay"),
			Frame->HasField(TEXT("subtree_com")));

		const TArray<TSharedPtr<FJsonValue>>* Contacts = nullptr;
		if (Frame->TryGetArrayField(TEXT("contacts"), Contacts) && Contacts)
		{
			const int32 Ncon = (int32)d->ncon;
			// The cap bounds the list; when the scene has more contacts than the
			// cap the list is exactly the cap, otherwise it is the full count.
			const int32 Expected = (Ncon > 4) ? 4 : Ncon;
			TestEqual(TEXT("contacts capped at MaxContacts"), Contacts->Num(), Expected);
			if (Ncon <= 4)
			{
				AddInfo(FString::Printf(
					TEXT("scene produced %d contacts (<= cap 4); cap-truncation "
						 "asserted trivially. The full > cap truncation is the "
						 "same code path."), Ncon));
			}
			// Each entry carries the documented shape.
			if (Contacts->Num() > 0)
			{
				const TSharedPtr<FJsonObject> C = (*Contacts)[0]->AsObject();
				TestTrue(TEXT("contact has pos"), C->HasField(TEXT("pos")));
				TestTrue(TEXT("contact has frame"), C->HasField(TEXT("frame")));
				TestTrue(TEXT("contact has dist"), C->HasField(TEXT("dist")));
				TestTrue(TEXT("contact has force"), C->HasField(TEXT("force")));
				TestTrue(TEXT("contact has dim"), C->HasField(TEXT("dim")));
				TestTrue(TEXT("contact has g1"), C->HasField(TEXT("g1")));
				TestTrue(TEXT("contact has g2"), C->HasField(TEXT("g2")));
				const TArray<TSharedPtr<FJsonValue>>* Pos = nullptr;
				const TArray<TSharedPtr<FJsonValue>>* Fr = nullptr;
				const TArray<TSharedPtr<FJsonValue>>* Force = nullptr;
				C->TryGetArrayField(TEXT("pos"), Pos);
				C->TryGetArrayField(TEXT("frame"), Fr);
				C->TryGetArrayField(TEXT("force"), Force);
				if (Pos) TestEqual(TEXT("pos is 3-vec"), Pos->Num(), 3);
				if (Fr) TestEqual(TEXT("frame is 9-vec"), Fr->Num(), 9);
				if (Force) TestEqual(TEXT("force is 6-vec"), Force->Num(), 6);
			}
			else
			{
				AddInfo(TEXT("test scene produced 0 contacts; contact-entry shape "
					"not asserted (the empty contacts[] key IS present, which is "
					"the gating assertion)."));
			}
		}
	}

	// --- (3) StreamOverlay -> derived-decor bundle sized by model dims --------
	{
		TSharedPtr<FJsonObject> Frame = S.Manager->BuildRenderFrame(m, d);
		FCaps Caps;
		Caps.bStreamOverlay = true;
		S.Manager->AppendRenderDebugFields(Frame, m, d, Caps);

		TestFalse(TEXT("no contacts without StreamContacts"),
			Frame->HasField(TEXT("contacts")));

		const TArray<TSharedPtr<FJsonValue>>* Xfrc = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* Com = nullptr;
		if (m->nbody > 0)
		{
			TestTrue(TEXT("xfrc_applied present"), Frame->TryGetArrayField(TEXT("xfrc_applied"), Xfrc));
			TestTrue(TEXT("subtree_com present"), Frame->TryGetArrayField(TEXT("subtree_com"), Com));
			if (Xfrc) TestEqual(TEXT("xfrc_applied is 6*nbody"), Xfrc->Num(), 6 * (int32)m->nbody);
			if (Com) TestEqual(TEXT("subtree_com is 3*nbody"), Com->Num(), 3 * (int32)m->nbody);
		}
		const TArray<TSharedPtr<FJsonValue>>* Ctrl = nullptr;
		if (m->nu > 0)
		{
			TestTrue(TEXT("ctrl present when nu>0"), Frame->TryGetArrayField(TEXT("ctrl"), Ctrl));
			if (Ctrl) TestEqual(TEXT("ctrl is nu"), Ctrl->Num(), (int32)m->nu);
		}
		else
		{
			TestFalse(TEXT("ctrl absent when nu==0"), Frame->HasField(TEXT("ctrl")));
		}
	}

	S.Cleanup();
	return true;
}
