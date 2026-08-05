// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The editor seam: spec to preview transform, and back again.
//
// Two properties are load-bearing and neither is visible in the compiled model,
// which is why they need tests of their own rather than falling out of the
// round-trip gates.
//
// Presence. An element that inherits `pos` from its default class, or does not
// author one at all, must still be inheriting it after someone drags its parent.
// The write-back's baseline is transient, so the case that matters is the one
// where there is no baseline: a freshly loaded level.
//
// Frame. An authored `quat` is MJCF's [w, x, y, z] in a right-handed frame. The
// accessor must hand back exactly that, and the conversion to an Unreal rotation
// must apply both the permutation and the sign flip -- a quaternion with a zero
// X and Z cannot tell the two apart, so the witness here has neither.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"

namespace
{
/** 90 degrees about MuJoCo's (1, 1, 1)/sqrt(3): every component is non-zero. */
constexpr double kQuatW = 0.7071067811865476;
constexpr double kQuatXyz = 0.4082482904638630;

/** The hook a gizmo drag delivers. The editor sends it to descendants too. */
void DeliverMove(UMjNodeComponent& Node)
{
	Node.PostEditComponentMove(true);
}
}  // namespace

// =============================================================================
// Presence: dragging a parent must not author a pose onto unset children
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_DragParentLeavesChildrenUnset,
	"URLab.Preview.DragParentLeavesChildrenUnset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_DragParentLeavesChildrenUnset::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <default>
            <default class="shell">
              <geom pos="0 0 0.25" size="0.1"/>
            </default>
          </default>
          <worldbody>
            <body name="parent" pos="1 2 3">
              <body name="child">
                <geom name="plain" size="0.1"/>
                <geom name="classed" class="shell"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjBody* Parent = S.FindTemplate<UMjBody>(TEXT("parent"));
	UMjBody* Child = S.FindTemplate<UMjBody>(TEXT("child"));
	UMjGeom* Plain = S.FindTemplate<UMjGeom>(TEXT("plain"));
	UMjGeom* Classed = S.FindTemplate<UMjGeom>(TEXT("classed"));
	if (Parent == nullptr || Child == nullptr || Plain == nullptr || Classed == nullptr)
	{
		AddError(TEXT("imported tree is missing an element"));
		S.Cleanup();
		return false;
	}

	TestFalse(TEXT("child body authored no pos"), Child->HasPos());
	TestFalse(TEXT("plain geom authored no pos"), Plain->HasPos());
	TestFalse(TEXT("classed geom authored no pos"), Classed->HasPos());

	// The state a freshly opened level is in: the baseline is transient, so a
	// load has to re-derive it. Without that, the write-back below has nothing to
	// compare against and every child gets a pos it never had.
	Parent->PostLoad();
	Child->PostLoad();
	Plain->PostLoad();
	Classed->PostLoad();

	// Drag the parent. PostEditComponentMove reaches every descendant, whose own
	// relative transform did not move at all.
	Parent->SetRelativeLocation(Parent->GetRelativeLocation() + FVector(50.0, 0.0, 0.0));
	DeliverMove(*Parent);
	DeliverMove(*Child);
	DeliverMove(*Plain);
	DeliverMove(*Classed);

	TestTrue(TEXT("the dragged parent authored its own pos"), Parent->HasPos());
	TestNearlyEqual(TEXT("parent pos x moved by 0.5 m"), (float)Parent->GetPos().X, 1.5f, 1e-4f);

	TestFalse(TEXT("child body pos still unset"), Child->HasPos());
	TestFalse(TEXT("plain geom pos still unset"), Plain->HasPos());
	TestFalse(TEXT("classed geom pos still unset, so its class still governs"), Classed->HasPos());

	S.Cleanup();
	return true;
}

// =============================================================================
// Presence: a class-inherited pose previews at the value it inherits
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_InheritedPosePreviewsEffective,
	"URLab.Preview.InheritedPosePreviewsEffective",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_InheritedPosePreviewsEffective::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <default>
            <default class="shell">
              <geom pos="0 0 0.25" size="0.1"/>
            </default>
          </default>
          <worldbody>
            <body name="b1">
              <geom name="classed" class="shell"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* Classed = S.FindTemplate<UMjGeom>(TEXT("classed"));
	if (Classed == nullptr)
	{
		AddError(TEXT("geom 'classed' not found"));
		S.Cleanup();
		return false;
	}

	TestFalse(TEXT("pos is inherited, not authored"), Classed->HasPos());
	Classed->SyncPreviewFromSpec();

	// 0.25 m up in MuJoCo is 25 cm up in Unreal, and Z does not flip.
	TestNearlyEqual(TEXT("preview Z = 25 cm from the class"), (float)Classed->GetRelativeLocation().Z, 25.0f, 0.1f);
	TestFalse(TEXT("previewing an inherited pos does not author it"), Classed->HasPos());

	// The size is inherited too, and the scale handle has to show it.
	TestNearlyEqual(TEXT("preview scale from the class size"), (float)Classed->GetRelativeScale3D().X, 0.2f, 1e-3f);
	TestFalse(TEXT("previewing an inherited size does not author it"), Classed->HasSize());

	S.Cleanup();
	return true;
}

// =============================================================================
// Frame: an authored quaternion, out through the accessor and back in
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_QuatAccessorRoundTrip,
	"URLab.Preview.QuatAccessorRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_QuatAccessorRoundTrip::RunTest(const FString&)
{
	// 90 degrees about (1, 1, 1) in MuJoCo's right-handed frame.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1" quat="0.7071067811865476 0.4082482904638630 0.4082482904638630 0.4082482904638630">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjBody* B = S.FindTemplate<UMjBody>(TEXT("b1"));
	if (B == nullptr)
	{
		AddError(TEXT("body 'b1' not found"));
		S.Cleanup();
		return false;
	}

	// The accessor hands back MJCF's own spelling, verbatim and unpermuted.
	const FMjQuatRot Authored = B->GetQuat();
	TestNearlyEqual(TEXT("w verbatim"), (float)Authored.W, (float)kQuatW, 1e-5f);
	TestNearlyEqual(TEXT("x verbatim"), (float)Authored.X, (float)kQuatXyz, 1e-5f);
	TestNearlyEqual(TEXT("y verbatim"), (float)Authored.Y, (float)kQuatXyz, 1e-5f);
	TestNearlyEqual(TEXT("z verbatim"), (float)Authored.Z, (float)kQuatXyz, 1e-5f);

	// Crossing into Unreal permutes AND flips the sign of X and Z. A stored FQuat
	// carrying the permutation alone would pass every type check and land here
	// with two components of the wrong sign.
	const FQuat Unreal = Authored.ToUnreal();
	const FQuat Expected(-kQuatXyz, kQuatXyz, -kQuatXyz, kQuatW);
	TestTrue(TEXT("ToUnreal applies the sign flip"), Unreal.Equals(Expected, 1e-4f));
	TestFalse(TEXT("the unflipped permutation is a different rotation"),
		Unreal.Equals(FQuat(kQuatXyz, kQuatXyz, kQuatXyz, kQuatW), 1e-4f));

	// The preview transform is the same conversion, reached the way the editor
	// reaches it.
	B->SyncPreviewFromSpec();
	TestTrue(TEXT("preview rotation matches"), B->GetRelativeRotation().Quaternion().Equals(Expected, 1e-3f));

	// And back: an Unreal rotation, spelled the way MJCF authors one, restores
	// exactly the four numbers the spec started with.
	B->SetQuat(FMjQuatRot::FromUnreal(Unreal));
	const FMjQuatRot Returned = B->GetQuat();
	TestNearlyEqual(TEXT("round trip w"), (float)Returned.W, (float)kQuatW, 1e-5f);
	TestNearlyEqual(TEXT("round trip x"), (float)Returned.X, (float)kQuatXyz, 1e-5f);
	TestNearlyEqual(TEXT("round trip y"), (float)Returned.Y, (float)kQuatXyz, 1e-5f);
	TestNearlyEqual(TEXT("round trip z"), (float)Returned.Z, (float)kQuatXyz, 1e-5f);

	S.Cleanup();
	return true;
}

// =============================================================================
// Size through the scale handle, one round trip per shape with a preview
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_SizeScaleRoundTripPerShape,
	"URLab.Preview.SizeScaleRoundTripPerShape",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_SizeScaleRoundTripPerShape::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <geom name="s" type="sphere"   size="0.1"/>
              <geom name="c" type="capsule"  size="0.1 0.3"/>
              <geom name="y" type="cylinder" size="0.1 0.3"/>
              <geom name="x" type="box"      size="0.1 0.2 0.3"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	struct FCase
	{
		const TCHAR* Name;
		TArray<double> Size;
		FVector Scale;
	};
	// Scale is size * 100 cm/m / 50 cm half-extent = size * 2, on the mapped axes;
	// the axes the shape does not map follow the lock, X mastering.
	const TArray<FCase> Cases = {
		{TEXT("s"), {0.1}, FVector(0.2, 0.2, 0.2)},
		{TEXT("c"), {0.1, 0.3}, FVector(0.2, 0.2, 0.6)},
		{TEXT("y"), {0.1, 0.3}, FVector(0.2, 0.2, 0.6)},
		{TEXT("x"), {0.1, 0.2, 0.3}, FVector(0.2, 0.4, 0.6)},
	};

	for (const FCase& Case : Cases)
	{
		UMjGeom* G = S.FindTemplate<UMjGeom>(Case.Name);
		if (G == nullptr)
		{
			AddError(FString::Printf(TEXT("geom '%s' not found"), Case.Name));
			continue;
		}

		// Forward: the authored size decides the scale.
		G->SyncPreviewFromSpec();
		TestTrue(FString::Printf(TEXT("%s: size -> scale"), Case.Name),
			G->GetRelativeScale3D().Equals(Case.Scale, 1e-4));

		// Back: dragging the scale handle to twice the size authors twice the size,
		// off the same rows, for every slot the shape has.
		G->SetRelativeScale3D(Case.Scale * 2.0);
		DeliverMove(*G);

		const TArray<double> Written = G->GetSize();
		TestEqual(FString::Printf(TEXT("%s: whole size array authored"), Case.Name),
			Written.Num(), Case.Size.Num());
		for (int32 Index = 0; Index < Case.Size.Num() && Index < Written.Num(); ++Index)
		{
			TestNearlyEqual(FString::Printf(TEXT("%s: size[%d] doubled"), Case.Name, Index),
				(float)Written[Index], (float)(Case.Size[Index] * 2.0), 1e-4f);
		}
	}

	S.Cleanup();
	return true;
}

// =============================================================================
// A shape that cannot hold a non-uniform scale constrains it, visibly
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_NonUniformScaleIsConstrained,
	"URLab.Preview.NonUniformScaleIsConstrained",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_NonUniformScaleIsConstrained::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <geom name="s" type="sphere"  size="0.1"/>
              <geom name="c" type="capsule" size="0.1 0.3"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* Sphere = S.FindTemplate<UMjGeom>(TEXT("s"));
	UMjGeom* Capsule = S.FindTemplate<UMjGeom>(TEXT("c"));
	if (Sphere == nullptr || Capsule == nullptr)
	{
		AddError(TEXT("imported tree is missing a geom"));
		S.Cleanup();
		return false;
	}

	// A sphere has one radius, so X is the master and the other two snap to it on
	// the spot rather than being silently dropped at write time.
	Sphere->SyncPreviewFromSpec();
	Sphere->SetRelativeScale3D(FVector(0.4, 0.9, 1.7));
	DeliverMove(*Sphere);
	TestTrue(TEXT("sphere scale snapped uniform"), Sphere->GetRelativeScale3D().Equals(FVector(0.4), 1e-4));
	TestEqual(TEXT("sphere authored one radius"), Sphere->GetSize().Num(), 1);
	TestNearlyEqual(TEXT("sphere radius from X"), (float)Sphere->GetSize()[0], 0.2f, 1e-4f);

	// A capsule's cross-section is round, so Y follows X and Z stays free.
	Capsule->SyncPreviewFromSpec();
	Capsule->SetRelativeScale3D(FVector(0.4, 0.9, 0.6));
	DeliverMove(*Capsule);
	TestTrue(TEXT("capsule Y snapped to X"), Capsule->GetRelativeScale3D().Equals(FVector(0.4, 0.4, 0.6), 1e-4));
	TestEqual(TEXT("capsule authored radius and half-length"), Capsule->GetSize().Num(), 2);
	TestNearlyEqual(TEXT("capsule radius from X"), (float)Capsule->GetSize()[0], 0.2f, 1e-4f);
	TestNearlyEqual(TEXT("capsule half-length from Z"), (float)Capsule->GetSize()[1], 0.3f, 1e-4f);

	S.Cleanup();
	return true;
}

// =============================================================================
// An element with pos but no quat drops rotation deltas rather than authoring pos
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjPreview_QuatlessElementIgnoresRotation,
	"URLab.Preview.QuatlessElementIgnoresRotation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjPreview_QuatlessElementIgnoresRotation::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <joint name="j" type="hinge" axis="0 0 1"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjJoint* J = S.FindTemplate<UMjJoint>(TEXT("j"));
	if (J == nullptr)
	{
		AddError(TEXT("joint 'j' not found"));
		S.Cleanup();
		return false;
	}

	TestFalse(TEXT("joint authored no pos"), J->HasPos());
	J->PostLoad();

	// A pure rotation drag. <joint> has `pos` but no `quat`, so there is nowhere
	// for the rotation to go; authoring `pos` on the way past would be worse than
	// dropping it.
	J->SetRelativeRotation(FRotator(0.0, 45.0, 0.0));
	DeliverMove(*J);

	TestFalse(TEXT("a rotation-only drag authored nothing"), J->HasPos());

	S.Cleanup();
	return true;
}
