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
// MjFlexSkinReplicationTests.cpp
//
// Tier-1 GAPs 1.h / 1.i (UE): the RUNTIME flex/skin replication on a mirror --
//   UMjFlexcomp::UpdateFromBodyTransforms (mj_flex, both interp modes) and
//   UMjSkincomp::UpdateFromBodyTransforms (CPU LBS vs mjv_updateActiveSkin) --
// compared vertex-by-vertex against a hand-computed reference.
//
// DEFERRED (documented skip, no source change): these cannot be driven as a pure
// automation unit test with the current API. Both UpdateFromBodyTransforms
// implementations early-return unless a UDynamicMesh SURFACE has been built, and
// that surface is a precondition, not a product, of the vertex math:
//
//   * Flex (MjFlexcomp.cpp:439-540): the function bails unless EnsureFlex()
//     resolves a compiled flex AND a child UStaticMeshComponent exists to build
//     the surface + the raw->welded vertex map (NumRenderVerts != 0). A grid
//     flexcomp authored in a headless test has no child static mesh, so the
//     surface never builds and the function is a no-op. Supplying a real
//     UStaticMesh whose LOD vertex buffer welds 1:1 onto the flex vertices is
//     asset scaffolding a headless automation test cannot stand up, and the weld
//     map is entangled with the very math under test. (The existing
//     MjFlexcompTests cover compile/import/pin/elasticity only, for this reason.)
//
//   * Skin (MjSkincomp.cpp): CreateProceduralMesh builds from the model's skin_*
//     arrays (no external mesh), but the test still needs a COMPILED model that
//     actually contains a <skin> with vertices/faces/bones/bindpos/bindquat and
//     >4-influence weights to exercise LBS + renormalize + skin_inflate; there is
//     no existing spec-authoring path in the test harness for that, and
//     MjSkinSinkTests cover asset collection only.
//
// In both cases the produced vertices land in the private UDynamicMesh writeback
// (ApplyFlexWorldPositions / the skin equivalent); the public DynamicMesh
// UPROPERTY exposes the result mesh, so a comparison is READABLE -- the blocker is
// building the INPUT (a surfaced flex/skin model) headlessly.
//
// A faithful vertex-by-vertex test would be straightforward if the pure math were
// factored into a free function, e.g.
//   ComputeFlexVertsFromBodyTransforms(model, flexId, bxpos, bxquat) -> TArray<FVector>
//   ComputeSkinVertsFromBodyTransforms(model, skinId, bxpos, bxquat, ...) -> TArray<FVector>
// callable without a UDynamicMesh. That is a SOURCE refactor and is out of scope
// for this test-only change; it is the recommended enabling step.
//
// Coverage today: the flex/skin math is validated by the Tier-3 visual checks
// (deformable surface deforms identically on owner + mirror). The tests below are
// registered under the audit's intended names and WARN so the gap stays visible.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

static const TCHAR* const kFlexSkinDeferReason =
	TEXT("DEFERRED: runtime flex/skin replication is not unit-testable without a "
		 "surfaced flex/skin model (asset scaffolding) or a pure-math seam "
		 "(source refactor). See file header. Covered by Tier-3 visual checks.");

// ---------------------------------------------------------------------------
// URLab.Flexcomp.ReplicatesMjFlexVertexMode  (vertex mode, flex_interp==0)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompReplicatesMjFlexVertexMode,
	"URLab.Flexcomp.ReplicatesMjFlexVertexMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFlexcompReplicatesMjFlexVertexMode::RunTest(const FString& Parameters)
{
	AddWarning(kFlexSkinDeferReason);
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flexcomp.ReplicatesMjFlexTrilinear  (trilinear cage, flex_interp>=1)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompReplicatesMjFlexTrilinear,
	"URLab.Flexcomp.ReplicatesMjFlexTrilinear",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFlexcompReplicatesMjFlexTrilinear::RunTest(const FString& Parameters)
{
	AddWarning(kFlexSkinDeferReason);
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Skin.LbsMatchesUpdateActiveSkin  (CPU LBS vs mjv_updateActiveSkin)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSkinLbsMatchesUpdateActiveSkin,
	"URLab.Skin.LbsMatchesUpdateActiveSkin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSkinLbsMatchesUpdateActiveSkin::RunTest(const FString& Parameters)
{
	AddWarning(kFlexSkinDeferReason);
	return true;
}
