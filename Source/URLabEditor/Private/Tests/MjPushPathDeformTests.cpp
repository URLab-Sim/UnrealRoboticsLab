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
// MjPushPathDeformTests.cpp
//
// cleanup_audit_addendum §A1: the push/forced-render drive renders deformables
// FROZEN and draws no overlays. AMjRenderer::ApplyForcedRenderState
// (MjRenderer.cpp:1695-1782, reached via the public RenderForcedRequest that
// the fastpath_render RPC calls) applies body transforms but never calls
// UpdateMirrorFlex / UpdateMirrorSkin / SynthesizeMirrorOverlays -- only the
// stream Tick path does (MjRenderer.cpp:2999-3013). So a -URLabDrive=push
// render/eval server shows flex+skin at rest and no overlays.
//
// These tests assert the INTENDED behavior (a push renderer deforms flex and
// synthesizes overlays exactly like the stream path). Both FAIL today and turn
// green the moment the post-ApplyBodyTransforms deform/overlay block is factored
// into a shared helper both drives call, as the addendum prescribes.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Editor.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Entity/MjOverlayRenderer.h"
#include "MuJoCo/Fast/MjRenderer.h"

class UMjCamera;

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjPushPathTest
{
// Compile inline MJCF and serialize it to an MJB byte buffer the renderer loads
// exactly as it would an over-the-wire model (no on-disk fixture needed).
bool BuildFlexMjb(FString& OutErr, TArray<uint8>& OutMjb, TArray<double>& OutRestXpos,
	TArray<double>& OutRestXquat, int32& OutNBody)
{
	char szErr[1000] = "";
	mjSpec* Spec = mj_parseXMLString(
		"<mujoco>"
		"  <size memory=\"10M\"/>"
		"  <worldbody>"
		"    <flexcomp name=\"soft\" type=\"grid\" count=\"3 3 3\" spacing=\"0.1 0.1 0.1\""
		"              pos=\"0 0 0.5\" dim=\"3\" radius=\"0.005\" mass=\"1\">"
		"      <contact selfcollide=\"none\" internal=\"false\"/>"
		"    </flexcomp>"
		"  </worldbody>"
		"</mujoco>",
		nullptr, szErr, sizeof(szErr));
	if (!Spec)
	{
		OutErr = FString::Printf(TEXT("parse: %hs"), szErr);
		return false;
	}
	mjModel* m = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	if (!m)
	{
		OutErr = TEXT("compile failed");
		return false;
	}
	if (m->nflex < 1)
	{
		OutErr = TEXT("model has no flex");
		mj_deleteModel(m);
		return false;
	}

	mjData* d = mj_makeData(m);
	mj_forward(m, d);
	OutNBody = m->nbody;
	OutRestXpos.SetNumUninitialized(3 * m->nbody);
	OutRestXquat.SetNumUninitialized(4 * m->nbody);
	FMemory::Memcpy(OutRestXpos.GetData(), d->xpos, sizeof(double) * 3 * m->nbody);
	FMemory::Memcpy(OutRestXquat.GetData(), d->xquat, sizeof(double) * 4 * m->nbody);
	mj_deleteData(d);

	const int32 Sz = mj_sizeModel(m);
	OutMjb.SetNumUninitialized(Sz);
	mj_saveModel(m, nullptr, OutMjb.GetData(), Sz);
	mj_deleteModel(m);
	return Sz > 0;
}

// Build a fastpath_render request carrying per-body transforms (the one wire
// format, §8.1) shifted +Dz in world Z so a fixed push would visibly deform.
TSharedPtr<FJsonObject> MakePushRequest(const TArray<double>& Xpos, const TArray<double>& Xquat,
	int32 NBody, double Dz)
{
	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	TArray<TSharedPtr<FJsonValue>> Bxpos, Bxquat;
	Bxpos.Reserve(3 * NBody);
	Bxquat.Reserve(4 * NBody);
	for (int32 b = 0; b < NBody; ++b)
	{
		Bxpos.Add(MakeShared<FJsonValueNumber>(Xpos[3 * b + 0]));
		Bxpos.Add(MakeShared<FJsonValueNumber>(Xpos[3 * b + 1]));
		Bxpos.Add(MakeShared<FJsonValueNumber>(Xpos[3 * b + 2] + Dz));
	}
	for (int32 i = 0; i < 4 * NBody; ++i)
	{
		Bxquat.Add(MakeShared<FJsonValueNumber>(Xquat[i]));
	}
	Req->SetArrayField(TEXT("bxpos"), Bxpos);
	Req->SetArrayField(TEXT("bxquat"), Bxquat);
	Req->SetNumberField(TEXT("sim_time"), 1.0);
	Req->SetNumberField(TEXT("frame_id"), 1);
	return Req;
}

int32 CountFlexWithMesh(AMjRenderer* Scene)
{
	int32 N = 0;
	TArray<UMjFlexcomp*> Comps;
	Scene->GetComponents<UMjFlexcomp>(Comps);
	for (UMjFlexcomp* FC : Comps)
	{
		if (FC && FC->DynamicMesh)
		{
			++N;
		}
	}
	return N;
}

} // namespace MjPushPathTest

// ---------------------------------------------------------------------------
// URLab.Fast.PushDriveDeformsFlex
//   A push (forced-render) renderer must deform its mirror flex from the pushed
//   body transforms, exactly like the stream path -- it must build a mirror
//   UMjFlexcomp with a UDynamicMesh surface.
//
//   EXPECTED FAIL (addendum §A1): ApplyForcedRenderState never calls
//   UpdateMirrorFlex, so BuildFlexcomps never runs on the push path and NO
//   mirror flexcomp is created. Fixing A1 (shared post-transform deform helper)
//   makes CountFlexWithMesh() > 0 and this test pass.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPushDriveDeformsFlex,
	"URLab.Fast.PushDriveDeformsFlex",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjPushDriveDeformsFlex::RunTest(const FString& Parameters)
{
	using namespace MjPushPathTest;

	FString Err;
	TArray<uint8> Mjb;
	TArray<double> RestXpos, RestXquat;
	int32 NBody = 0;
	if (!BuildFlexMjb(Err, Mjb, RestXpos, RestXquat, NBody))
	{
		AddError(FString::Printf(TEXT("fixture build failed: %s"), *Err));
		return false;
	}

	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World))
	{
		return false;
	}

	AMjRenderer* Scene = World->SpawnActor<AMjRenderer>();
	if (!TestNotNull(TEXT("spawned AMjRenderer"), Scene))
	{
		return false;
	}
	// A -URLabDrive=push render server.
	Scene->Drive = EMjDrive::Push;
	Scene->bForcedRenderOnly = true;
	Scene->SetMjbBytes(Mjb);
	const int32 NGeom = Scene->LoadAndBuild();
	TestTrue(FString::Printf(TEXT("push renderer built the flex model (ngeom=%d)"), NGeom), NGeom >= 0);

	// Push a forced-render frame with the flex bodies displaced.
	TSharedPtr<FJsonObject> Req = MakePushRequest(RestXpos, RestXquat, NBody, /*Dz=*/0.2);
	TArray<UMjCamera*> Cams;
	uint64 TargetId = 0;
	Scene->RenderForcedRequest(Req, Cams, TargetId);

	// INTENDED: the push path deformed the flex, so a mirror flexcomp exists with
	// a built surface.
	const int32 FlexWithMesh = CountFlexWithMesh(Scene);
	// EXPECTED FAIL (addendum §A1): push never builds/deforms mirror flex.
	TestTrue(FString::Printf(
				 TEXT("push-drive builds+deforms mirror flex like the stream path "
					  "(got %d flexcomps with a mesh) -- EXPECTED FAIL until A1 is fixed"),
				 FlexWithMesh),
		FlexWithMesh > 0);

	Scene->Destroy();
	return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// URLab.Fast.PushDriveSynthesizesOverlays
//   With a non-zero mj.MirrorOverlayMask a push renderer must synthesize its
//   mirror overlays from the pushed frame, exactly like the stream path -- it
//   must create a UMjOverlayRenderer.
//
//   EXPECTED FAIL (addendum §A1): ApplyForcedRenderState never calls
//   SynthesizeMirrorOverlays, so no overlay renderer is ever created on the push
//   path. Fixing A1 makes the overlay renderer appear and this test pass.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPushDriveSynthesizesOverlays,
	"URLab.Fast.PushDriveSynthesizesOverlays",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjPushDriveSynthesizesOverlays::RunTest(const FString& Parameters)
{
	using namespace MjPushPathTest;

	FString Err;
	TArray<uint8> Mjb;
	TArray<double> RestXpos, RestXquat;
	int32 NBody = 0;
	if (!BuildFlexMjb(Err, Mjb, RestXpos, RestXquat, NBody))
	{
		AddError(FString::Printf(TEXT("fixture build failed: %s"), *Err));
		return false;
	}

	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World))
	{
		return false;
	}

	// Ask the mirror to draw an overlay set (inertia). Save/restore the cvar so
	// this never leaks into a co-running test.
	IConsoleVariable* MaskCVar = IConsoleManager::Get().FindConsoleVariable(TEXT("mj.MirrorOverlayMask"));
	if (!TestNotNull(TEXT("mj.MirrorOverlayMask cvar exists"), MaskCVar))
	{
		return false;
	}
	const int32 SavedMask = MaskCVar->GetInt();
	MaskCVar->Set(1 << mjVIS_INERTIA, ECVF_SetByCode);

	AMjRenderer* Scene = World->SpawnActor<AMjRenderer>();
	if (!TestNotNull(TEXT("spawned AMjRenderer"), Scene))
	{
		MaskCVar->Set(SavedMask, ECVF_SetByCode);
		return false;
	}
	Scene->Drive = EMjDrive::Push;
	Scene->bForcedRenderOnly = true;
	Scene->SetMjbBytes(Mjb);
	Scene->LoadAndBuild();

	TSharedPtr<FJsonObject> Req = MakePushRequest(RestXpos, RestXquat, NBody, /*Dz=*/0.0);
	TArray<UMjCamera*> Cams;
	uint64 TargetId = 0;
	Scene->RenderForcedRequest(Req, Cams, TargetId);

	TArray<UMjOverlayRenderer*> Overlays;
	Scene->GetComponents<UMjOverlayRenderer>(Overlays);
	// EXPECTED FAIL (addendum §A1): push never synthesizes overlays, so no overlay
	// renderer is created even with a non-zero mask.
	TestTrue(FString::Printf(
				 TEXT("push-drive synthesizes mirror overlays with a non-zero mask "
					  "(got %d overlay renderers) -- EXPECTED FAIL until A1 is fixed"),
				 Overlays.Num()),
		Overlays.Num() > 0);

	Scene->Destroy();
	MaskCVar->Set(SavedMask, ECVF_SetByCode);
	return !HasAnyErrors();
}

#endif // WITH_EDITOR
