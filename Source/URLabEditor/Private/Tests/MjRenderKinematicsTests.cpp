// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "MuJoCo/Render/MjRenderKinematics.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
const char* RenderKinematicsXml = R"(
<mujoco model="render_kinematics">
  <option timestep="0.01" gravity="1 0 0"/>
  <worldbody>
    <body name="slider">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom name="box" type="box" size="0.1 0.1 0.1" mass="1"/>
      <camera name="mounted" pos="0 0 0.25"/>
    </body>
  </worldbody>
</mujoco>)";

TArray<uint8> RenderKinematicsBytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool RenderKinematicsPrepare(const FString& Revision,
	FMjRenderKinematicsCandidate& Out, FString& OutError)
{
	FMjModelSourcePayload Source;
	Source.Kind = EMjModelSourceKind::XmlWithAssets;
	Source.PrimaryBytes = RenderKinematicsBytes(RenderKinematicsXml);
	Source.Label = Revision;
	return FMjRenderKinematicsCandidate::Prepare(MoveTemp(Source), Revision,
		FMjRenderKinematicsOptions(), Out, OutError);
}

bool RenderKinematicsSameTransforms(const FRenderFrame& A, const FRenderFrame& B)
{
	if (A.BodySceneTransforms.Num() != B.BodySceneTransforms.Num()
		|| A.CameraSceneTransforms.Num() != B.CameraSceneTransforms.Num()) return false;
	for (int32 Index = 0; Index < A.BodySceneTransforms.Num(); ++Index)
	{
		if (!A.BodySceneTransforms[Index].Equals(B.BodySceneTransforms[Index])) return false;
	}
	for (int32 Index = 0; Index < A.CameraSceneTransforms.Num(); ++Index)
	{
		if (!A.CameraSceneTransforms[Index].Equals(B.CameraSceneTransforms[Index])) return false;
	}
	return A.UserCameraSceneTransform.IsSet() == B.UserCameraSceneTransform.IsSet()
		&& (!A.UserCameraSceneTransform.IsSet()
			|| A.UserCameraSceneTransform.GetValue().Equals(
				B.UserCameraSceneTransform.GetValue()));
}

bool RenderKinematicsSameFrame(const FRenderFrame& A, const FRenderFrame& B)
{
	return A.Identity.SceneRevision == B.Identity.SceneRevision
		&& A.Identity.FrameId == B.Identity.FrameId
		&& A.Identity.SourceTime == B.Identity.SourceTime
		&& A.Identity.ClockEpoch == B.Identity.ClockEpoch
		&& A.Identity.bHasSourceTime == B.Identity.bHasSourceTime
		&& RenderKinematicsSameTransforms(A, B);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderKinematicsForwardOnlyTest,
	"URLab.RenderKinematics.ForwardOnlyDeterministicFrames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderKinematicsForwardOnlyTest::RunTest(const FString&)
{
	FString Error;
	FMjRenderKinematicsCandidate Candidate;
	if (!TestTrue(TEXT("render candidate prepares"),
		RenderKinematicsPrepare(TEXT("scene-a"), Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Initial;
	TestTrue(TEXT("neutral scene package transfers once"),
		Candidate.TakeNeutralScene(Scene, Resources, Initial, Error));
	TestEqual(TEXT("neutral revision is exact"), Scene.SceneRevision, FString(TEXT("scene-a")));
	TestFalse(TEXT("heavy scene package cannot transfer twice"),
		Candidate.TakeNeutralScene(Scene, Resources, Initial, Error));

	FMjRenderKinematics Kinematics;
	if (!TestTrue(TEXT("initial kinematics install commits"),
		Kinematics.Commit(MoveTemp(Candidate), FString(), Error)))
	{
		AddError(Error);
		return false;
	}
	TestEqual(TEXT("qpos width comes from installed model"), Kinematics.GetQposWidth(), 1);

	const TArray<double> Positive = {0.5};
	const TArray<double> Negative = {-0.25};
	FRenderFrame First;
	FRenderFrame Second;
	FRenderFrame Repeated;
	TestTrue(TEXT("positive qpos runs FK"), Kinematics.FrameFromQpos(TEXT("scene-a"),
		Positive, 4, 12.5, true, 7, First, Error));
	TestTrue(TEXT("negative qpos runs FK"), Kinematics.FrameFromQpos(TEXT("scene-a"),
		Negative, 5, 13.0, true, 7, Second, Error));
	if (!TestTrue(TEXT("FK emits the installed body and camera arrays"),
		First.BodySceneTransforms.IsValidIndex(1)
		&& Second.BodySceneTransforms.IsValidIndex(1)
		&& First.CameraSceneTransforms.IsValidIndex(0)
		&& Second.CameraSceneTransforms.IsValidIndex(0)))
	{
		return false;
	}
	TestFalse(TEXT("body pose changes with qpos"),
		First.BodySceneTransforms[1].Equals(Second.BodySceneTransforms[1]));
	TestFalse(TEXT("mounted camera follows qpos FK"),
		First.CameraSceneTransforms[0].Equals(Second.CameraSceneTransforms[0]));
	TestTrue(TEXT("same qpos repeats deterministically after another request"),
		Kinematics.FrameFromQpos(TEXT("scene-a"), Positive, 6, 14.0, true, 8,
			Repeated, Error));
	TestTrue(TEXT("repeated qpos yields identical Cartesian transforms"),
		RenderKinematicsSameTransforms(First, Repeated));
	TestEqual(TEXT("caller source time is preserved"), Repeated.Identity.SourceTime, 14.0);
	TestEqual(TEXT("caller clock epoch is preserved"), Repeated.Identity.ClockEpoch,
		static_cast<uint64>(8));

	FRenderFrame Preserved = Repeated;
	const TArray<double> WrongWidth = {0.0, 1.0};
	TestFalse(TEXT("wrong qpos width rejects"), Kinematics.FrameFromQpos(TEXT("scene-a"),
		WrongWidth, 7, 15.0, true, 8, Preserved, Error));
	TestTrue(TEXT("width failure leaves output frame unchanged"),
		RenderKinematicsSameFrame(Preserved, Repeated));
	const TArray<double> NonFinite = {std::numeric_limits<double>::quiet_NaN()};
	TestFalse(TEXT("non-finite qpos rejects"), Kinematics.FrameFromQpos(TEXT("scene-a"),
		NonFinite, 7, 15.0, true, 8, Preserved, Error));
	TestTrue(TEXT("finite failure leaves output frame unchanged"),
		RenderKinematicsSameFrame(Preserved, Repeated));
	TestFalse(TEXT("stale scene revision rejects"), Kinematics.FrameFromQpos(TEXT("stale"),
		Positive, 7, 15.0, true, 8, Preserved, Error));
	TestTrue(TEXT("revision failure leaves output frame unchanged"),
		RenderKinematicsSameFrame(Preserved, Repeated));
	FRenderScene PatchedScene = Scene;
	PatchedScene.SceneRevision = TEXT("scene-patched");
	PatchedScene.Cameras[0].Optics.VerticalFovDegrees = 55.0f;
	PatchedScene.Visualization.Features.Add(TEXT("com"));
	TestTrue(TEXT("qpos adapter accepts patched neutral configuration"),
		Kinematics.RebaseSceneRevision(TEXT("scene-a"), PatchedScene, Error));
	FRenderFrame PatchedFrame;
	TestTrue(TEXT("qpos after patch uses new local revision"),
		Kinematics.FrameFromQpos(TEXT("scene-patched"), Positive, 8, 16.0,
			true, 8, PatchedFrame, Error));
	TestEqual(TEXT("patched qpos frame revision"), PatchedFrame.Identity.SceneRevision,
		FString(TEXT("scene-patched")));
	TestTrue(TEXT("patched qpos emits model debug drawables"),
		PatchedFrame.DebugMeshes.Num() > 0);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderKinematicsTransactionalInstallTest,
	"URLab.RenderKinematics.TransactionalReplacement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderKinematicsTransactionalInstallTest::RunTest(const FString&)
{
	FString Error;
	FMjRenderKinematicsCandidate Initial;
	FMjRenderKinematicsCandidate ReusedRevision;
	FMjRenderKinematicsCandidate Replacement;
	if (!RenderKinematicsPrepare(TEXT("scene-a"), Initial, Error)
		|| !RenderKinematicsPrepare(TEXT("scene-a"), ReusedRevision, Error)
		|| !RenderKinematicsPrepare(TEXT("scene-b"), Replacement, Error))
	{
		AddError(Error);
		return false;
	}
	FMjRenderKinematics Kinematics;
	TestTrue(TEXT("initial commits"), Kinematics.Commit(MoveTemp(Initial), FString(), Error));
	TestFalse(TEXT("replacement cannot reuse the active scene revision"),
		Kinematics.Commit(MoveTemp(ReusedRevision), TEXT("scene-a"), Error));
	TestEqual(TEXT("reused revision rejection preserves active install"),
		Kinematics.GetSceneRevision(), FString(TEXT("scene-a")));
	TestFalse(TEXT("wrong expected revision preserves active install"),
		Kinematics.Commit(MoveTemp(Replacement), TEXT("wrong"), Error));
	TestEqual(TEXT("failed replacement preserves revision"), Kinematics.GetSceneRevision(),
		FString(TEXT("scene-a")));
	FRenderFrame StillActive;
	const TArray<double> Pose = {0.1};
	TestTrue(TEXT("failed replacement preserves active FK"), Kinematics.FrameFromQpos(
		TEXT("scene-a"), Pose, 1, 0.0, false, 0, StillActive, Error));
	TestTrue(TEXT("same prepared candidate can commit after stale rejection"),
		Kinematics.Commit(MoveTemp(Replacement), TEXT("scene-a"), Error));
	TestEqual(TEXT("replacement revision becomes active"), Kinematics.GetSceneRevision(),
		FString(TEXT("scene-b")));
	TestFalse(TEXT("old revision cannot produce frames"), Kinematics.FrameFromQpos(
		TEXT("scene-a"), Pose, 2, 0.0, false, 0, StillActive, Error));
	return !HasAnyErrors();
}
#endif // WITH_DEV_AUTOMATION_TESTS
