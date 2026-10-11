// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "CoreMinimal.h"
#include "Async/Async.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"
#include "Utils/CoordinateConversion.h"

#include <limits>

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
const char* DynamicXml = R"(
<mujoco model="independent_runtime">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="falling" pos="0 0 1">
      <freejoint name="root"/>
      <geom name="ball" type="sphere" size="0.05" mass="1"/>
      <site name="sensor_site"/>
      <camera name="eye"/>
    </body>
  </worldbody>
  <sensor><framepos name="position" objtype="site" objname="sensor_site"/></sensor>
</mujoco>)";

const char* AssetXml = R"(
<mujoco model="asset_runtime">
  <asset><mesh name="tetra" file="tetra.obj"/></asset>
  <worldbody>
    <body name="mesh_body" pos="0 0 1">
      <freejoint name="root"/>
      <geom name="shape" type="mesh" mesh="tetra" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";

const char* TetraObj = R"(v 0 0 0
v 0.1 0 0
v 0 0.1 0
v 0 0 0.1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)";

const char* ControlXml = R"(
<mujoco model="runtime_control">
  <compiler autolimits="true"/>
  <default><position ctrlrange="-2 2" ctrllimited="true"/></default>
  <worldbody>
    <body><joint name="orientation_joint" type="ball"/><geom type="sphere" size=".1"/></body>
    <body><joint name="drive_joint"/><geom type="sphere" size=".1"/></body>
  </worldbody>
  <actuator>
    <orientation name="orientation" joint="orientation_joint" kp="1"/>
    <motor name="drive" joint="drive_joint" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>)";

TArray<uint8> BytesOf(const char* Text)
{
	const int32 Length = FCStringAnsi::Strlen(Text);
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), Length);
	return Out;
}

mjModel* Compile(const char* Xml, FString& OutError)
{
	char Error[1024]{};
	mjSpec* Spec = mj_parseXMLString(Xml, nullptr, Error, sizeof(Error));
	if (!Spec)
	{
		OutError = UTF8_TO_TCHAR(Error);
		return nullptr;
	}
	mjModel* Model = mj_compile(Spec, nullptr);
	if (!Model) OutError = UTF8_TO_TCHAR(mjs_getError(Spec));
	mj_deleteSpec(Spec);
	return Model;
}

bool SaveMjb(const FMjOwnedModelCandidate& Candidate, TArray<uint8>& Out)
{
	return Candidate.VisitPrepared([&Out](const mjModel* Model, const mjData*)
	{
		const mjtSize Size = mj_sizeModel(Model);
		if (Size <= 0 || Size > MAX_int32) return false;
		Out.SetNumUninitialized(static_cast<int32>(Size));
		mj_saveModel(Model, nullptr, Out.GetData(), Out.Num());
		return true;
	});
}

bool SameValues(const TArray<double>& A, const TArray<double>& B, double Tolerance = 1e-12)
{
	if (A.Num() != B.Num()) return false;
	for (int32 Index = 0; Index < A.Num(); ++Index)
	{
		if (!FMath::IsNearlyEqual(A[Index], B[Index], Tolerance)) return false;
	}
	return true;
}

bool PrepareControlCandidate(FMjOwnedModelCandidate& Out, FString& OutError)
{
	return FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		BytesOf(ControlXml), {}, Out, OutError, TEXT("control fixture"));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjIndependentRuntimePrimarySourcesTest,
	"URLab.Runtime.Independent.PrimarySourcesAndOwnership",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjIndependentRuntimePrimarySourcesTest::RunTest(const FString&)
{
	FMjSimulationRuntime Runtime;
	FString Error;

	TMap<FString, TArray<uint8>> XmlAssets;
	XmlAssets.Add(TEXT("tetra.obj"), BytesOf(TetraObj));
	FMjOwnedModelCandidate XmlCandidate;
	if (!TestTrue(TEXT("XML plus referenced asset compiles directly"),
		FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
			BytesOf(AssetXml), XmlAssets, XmlCandidate, Error, TEXT("asset fixture"))))
	{
		AddError(Error);
		return false;
	}
	TestEqual(TEXT("XML source kind retained"), XmlCandidate.GetSourceInfo().Kind,
		EMjModelSourceKind::XmlWithAssets);
	TestEqual(TEXT("asset count retained"), XmlCandidate.GetSourceInfo().AssetCount, 1);
	TestTrue(TEXT("XML asset produced a native mesh"), XmlCandidate.VisitPrepared(
		[](const mjModel* Model, const mjData*) { return Model->nmesh == 1; }));

	TArray<uint8> AssetMjb;
	TestTrue(TEXT("candidate visitor can serialize the native MJB fixture"),
		SaveMjb(XmlCandidate, AssetMjb));
	const FMjSimulationInstallResult XmlInstall = Runtime.Install(MoveTemp(XmlCandidate));
	TestTrue(TEXT("XML candidate commits"), XmlInstall.bCommitted);
	TestEqual(TEXT("first generation"), XmlInstall.Generation, static_cast<uint64>(1));
	TestEqual(TEXT("first epoch"), XmlInstall.ClockEpoch, static_cast<uint64>(1));
	FMjModelObservation XmlObservation;
	TestTrue(TEXT("XML observation available"), Runtime.GetObservation(XmlObservation));
	TestEqual(TEXT("candidate preparation did not integrate"), XmlObservation.StepSequence,
		static_cast<uint64>(0));
	TestEqual(TEXT("candidate preparation kept initial time"), XmlObservation.Time, 0.0);

	FMjOwnedModelCandidate MjbCandidate;
	if (!TestTrue(TEXT("MJB loads directly"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::Mjb, AssetMjb, {}, MjbCandidate, Error, TEXT("mjb fixture"))))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationInstallResult MjbInstall = Runtime.Install(MoveTemp(MjbCandidate));
	TestTrue(TEXT("MJB candidate commits"), MjbInstall.bCommitted);
	TestTrue(TEXT("generation increases"), MjbInstall.Generation > XmlInstall.Generation);
	TestTrue(TEXT("epoch increases across replacement"), MjbInstall.ClockEpoch > XmlInstall.ClockEpoch);
	TestEqual(TEXT("MJB source kind retained"), MjbInstall.Source.Kind,
		EMjModelSourceKind::Mjb);
	TestEqual(TEXT("equivalent XML and MJB have the same canonical content hash"),
		MjbInstall.Identity.ContentSha256, XmlInstall.Identity.ContentSha256);
	TestEqual(TEXT("equivalent XML and MJB nq"), MjbInstall.Identity.Nq, XmlInstall.Identity.Nq);
	TestEqual(TEXT("equivalent XML and MJB nv"), MjbInstall.Identity.Nv, XmlInstall.Identity.Nv);
	TestEqual(TEXT("equivalent XML and MJB nu"), MjbInstall.Identity.Nu, XmlInstall.Identity.Nu);
	TestEqual(TEXT("equivalent XML and MJB sensor width"), MjbInstall.Identity.NSensorData,
		XmlInstall.Identity.NSensorData);

	mjModel* Original = Compile(DynamicXml, Error);
	if (!TestNotNull(TEXT("compiled fixture exists"), Original))
	{
		AddError(Error);
		return false;
	}
	FMjOwnedModelCandidate CopiedCandidate;
	const bool bCopied = FMjOwnedModelCandidate::CopyCompiled(Original, CopiedCandidate,
		Error, TEXT("compiled fixture"));
	mj_deleteModel(Original);
	if (!TestTrue(TEXT("compiled model is copied, not borrowed"), bCopied))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationInstallResult CopiedInstall = Runtime.Install(MoveTemp(CopiedCandidate));
	TestTrue(TEXT("copied model survives source destruction"), CopiedInstall.bCommitted);
	TestEqual(TEXT("compiled source kind retained"), CopiedInstall.Source.Kind,
		EMjModelSourceKind::CompiledModel);
	FMjModelDescriptor CopiedDescriptor;
	TestTrue(TEXT("copied descriptor available"), Runtime.GetDescriptor(CopiedDescriptor));
	TestEqual(TEXT("copied model camera retained"), CopiedDescriptor.Cameras.Num(), 1);

	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!TestTrue(TEXT("plugin source location available"), Plugin.IsValid())) return false;
	const FString MjzPath = FPaths::Combine(Plugin->GetBaseDir(),
		TEXT("third_party/MuJoCo/src/test/testdata/model.mjz"));
	TArray<uint8> MjzBytes;
	if (!TestTrue(TEXT("checked-in asset-bearing MJZ fixture is present"),
		FFileHelper::LoadFileToArray(MjzBytes, *MjzPath)))
	{
		AddError(MjzPath);
		return false;
	}
	FMjOwnedModelCandidate MjzCandidate;
	if (!TestTrue(TEXT("asset-bearing MJZ compiles directly"),
		FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::Mjz,
			MjzBytes, {}, MjzCandidate, Error, TEXT("native model.mjz"))))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("MJZ decoded its referenced mesh and texture assets"),
		MjzCandidate.VisitPrepared([](const mjModel* Model, const mjData*)
		{
			return Model->nmesh > 0 && Model->ntex > 0;
		}));
	const FMjSimulationInstallResult MjzInstall = Runtime.Install(MoveTemp(MjzCandidate));
	TestTrue(TEXT("MJZ candidate commits"), MjzInstall.bCommitted);
	TestEqual(TEXT("MJZ source kind retained"), MjzInstall.Source.Kind,
		EMjModelSourceKind::Mjz);
	TestTrue(TEXT("MJZ primary byte count retained"),
		MjzInstall.Source.PrimaryByteCount > 0);

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRuntimeRenderScratchTest,
	"URLab.Runtime.RenderScratch.PostStepPosePreservesLiveState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRuntimeRenderScratchTest::RunTest(const FString&)
{
	FString Error;
	FMjOwnedModelCandidate Candidate;
	if (!TestTrue(TEXT("dynamic model prepares"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, BytesOf(DynamicXml), {}, Candidate,
		Error, TEXT("render scratch"))))
	{ AddError(Error); return false; }
	FMjRenderSceneAdapter Adapter;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	const bool bExtracted = Candidate.VisitPrepared([&](const mjModel* Model, const mjData* Data)
	{ return Adapter.Extract(Model, Data, TEXT("runtime-render"), 7, -1,
		false, nullptr, Scene, Resources, Error); });
	if (!TestTrue(TEXT("render adapter extracts installed model"), bExtracted))
	{ AddError(Error); return false; }
	FMjSimulationRuntime Runtime;
	const FMjSimulationInstallResult Install = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("runtime commits"), Install.bCommitted))
	{ AddError(Install.Error); return false; }
	TestTrue(TEXT("simulator advances"), Runtime.Step(3, Error));
	FMjModelObservation Before;
	TestTrue(TEXT("observation before render"), Runtime.GetObservation(Before));
	const uint64 Revision = Runtime.GetStateRevision();
	FRenderFrame Frame;
	if (!TestTrue(TEXT("render scratch captures"), Runtime.CaptureRenderFrame(
		Adapter, Install.Generation, 4, Frame, Error)))
	{ AddError(Error); return false; }
	FMjSimulationCartesianSnapshot Cartesian;
	TestTrue(TEXT("reference post-step Cartesian captures"),
		Runtime.CaptureCartesianSnapshot(Cartesian) == EMjSimulationSnapshotError::None);
	if (Frame.BodySceneTransforms.IsValidIndex(1) && Cartesian.BodyPositions.Num() >= 6)
		TestTrue(TEXT("render pose matches post-step Cartesian reference"),
			Frame.BodySceneTransforms[1].GetTranslation().Equals(
				URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(
					Cartesian.BodyPositions.GetData() + 3), 1.e-5));
	FMjModelObservation After;
	TestTrue(TEXT("observation after render"), Runtime.GetObservation(After));
	TestTrue(TEXT("render leaves qpos unchanged"), SameValues(Before.Qpos, After.Qpos));
	TestTrue(TEXT("render leaves qvel unchanged"), SameValues(Before.Qvel, After.Qvel));
	TestEqual(TEXT("render leaves time unchanged"), After.Time, Before.Time);
	TestEqual(TEXT("render leaves state revision unchanged"), Runtime.GetStateRevision(), Revision);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjIndependentRuntimeTransactionStateTest,
	"URLab.Runtime.Independent.TransactionAndStateContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjIndependentRuntimeTransactionStateTest::RunTest(const FString&)
{
	FString Error;
	mjModel* Original = Compile(DynamicXml, Error);
	if (!TestNotNull(TEXT("dynamic model compiles"), Original))
	{
		AddError(Error);
		return false;
	}
	FMjOwnedModelCandidate Candidate;
	const bool bCopied = FMjOwnedModelCandidate::CopyCompiled(Original, Candidate, Error);
	mj_deleteModel(Original);
	if (!TestTrue(TEXT("owned candidate prepared"), bCopied))
	{
		AddError(Error);
		return false;
	}

	FMjSimulationRuntime Runtime;
	const FMjSimulationInstallResult Installed = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("runtime install committed"), Installed.bCommitted))
	{
		AddError(Installed.Error);
		return false;
	}
	TestTrue(TEXT("three explicit steps succeed"), Runtime.Step(3, Error));
	FMjModelState Checkpoint;
	TestTrue(TEXT("full checkpoint captured"), Runtime.CaptureState(mjSTATE_INTEGRATION,
		Checkpoint) == EMjModelStateError::None);
	TestEqual(TEXT("checkpoint carries current epoch"), Checkpoint.ClockEpoch,
		Runtime.GetClockEpoch());

	TestTrue(TEXT("two continuation steps succeed"), Runtime.Step(2, Error));
	FMjModelObservation Reference;
	TestTrue(TEXT("reference observation captured"), Runtime.GetObservation(Reference));
	const uint64 EpochBeforeRestore = Runtime.GetClockEpoch();
	TestTrue(TEXT("checkpoint restores"), Runtime.ApplyState(Checkpoint) == EMjModelStateError::None);
	TestTrue(TEXT("restore advances epoch"), Runtime.GetClockEpoch() > EpochBeforeRestore);
	TestEqual(TEXT("restore resets sequence"), Runtime.GetStepSequence(), static_cast<uint64>(0));
	TestTrue(TEXT("replayed two steps succeed"), Runtime.Step(2, Error));
	FMjModelObservation Replayed;
	TestTrue(TEXT("replayed observation captured"), Runtime.GetObservation(Replayed));
	TestTrue(TEXT("checkpoint continuation qpos matches"), SameValues(Reference.Qpos, Replayed.Qpos));
	TestTrue(TEXT("checkpoint continuation qvel matches"), SameValues(Reference.Qvel, Replayed.Qvel));
	TestTrue(TEXT("checkpoint continuation sensors match"),
		SameValues(Reference.SensorData, Replayed.SensorData));
	TestEqual(TEXT("checkpoint continuation time matches"), Replayed.Time, Reference.Time);

	FMjModelState Wrong = Checkpoint;
	Wrong.Identity.Generation++;
	FMjModelObservation BeforeRejectedState;
	Runtime.GetObservation(BeforeRejectedState);
	TestTrue(TEXT("wrong generation state rejected"), Runtime.ApplyState(Wrong)
		== EMjModelStateError::WrongModel);
	FMjModelObservation AfterRejectedState;
	Runtime.GetObservation(AfterRejectedState);
	TestEqual(TEXT("rejected state preserves epoch"), AfterRejectedState.ClockEpoch,
		BeforeRejectedState.ClockEpoch);
	TestEqual(TEXT("rejected state preserves sequence"), AfterRejectedState.StepSequence,
		BeforeRejectedState.StepSequence);
	TestEqual(TEXT("rejected state preserves time"), AfterRejectedState.Time,
		BeforeRejectedState.Time);

	FMjOwnedModelCandidate Invalid;
	const FMjSimulationInstallResult RejectedInstall = Runtime.Install(MoveTemp(Invalid));
	TestFalse(TEXT("invalid install rejected"), RejectedInstall.bCommitted);
	FMjModelObservation AfterRejectedInstall;
	Runtime.GetObservation(AfterRejectedInstall);
	TestEqual(TEXT("failed install preserves generation"), AfterRejectedInstall.Identity.Generation,
		Installed.Generation);
	TestEqual(TEXT("failed install preserves state"), AfterRejectedInstall.Time,
		BeforeRejectedState.Time);

	const uint64 EpochBeforeReset = Runtime.GetClockEpoch();
	TestTrue(TEXT("reset succeeds"), Runtime.Reset(Error));
	TestTrue(TEXT("reset advances epoch"), Runtime.GetClockEpoch() > EpochBeforeReset);
	TestEqual(TEXT("reset clears sequence"), Runtime.GetStepSequence(), static_cast<uint64>(0));
	FMjModelObservation ResetObservation;
	Runtime.GetObservation(ResetObservation);
	TestEqual(TEXT("reset returns to time zero without integration"), ResetObservation.Time, 0.0);

	TArray<TFuture<bool>> Readers;
	for (int32 ReaderIndex = 0; ReaderIndex < 3; ++ReaderIndex)
	{
		Readers.Add(Async(EAsyncExecution::ThreadPool, [&Runtime]()
		{
			for (int32 ReadIndex = 0; ReadIndex < 50; ++ReadIndex)
			{
				FMjModelObservation Observation;
				if (!Runtime.GetObservation(Observation)
					|| Observation.Qpos.Num() != Observation.Identity.Nq
					|| Observation.Qvel.Num() != Observation.Identity.Nv
					|| Observation.Ctrl.Num() != Observation.Identity.Nu
					|| Observation.SensorData.Num() != Observation.Identity.NSensorData)
				{
					return false;
				}
			}
			return true;
		}));
	}
	TestTrue(TEXT("concurrent integration succeeds"), Runtime.Step(10, Error));
	for (TFuture<bool>& Reader : Readers)
	{
		TestTrue(TEXT("concurrent reads remain generation-consistent"), Reader.Get());
	}
	TestEqual(TEXT("explicit step count is exact"), Runtime.GetStepSequence(),
		static_cast<uint64>(10));

	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjIndependentRuntimeRejectsAmbiguousSourceTest,
	"URLab.Runtime.Independent.RejectsAmbiguousSourceBytes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjIndependentRuntimeRejectsAmbiguousSourceTest::RunTest(const FString&)
{
	FString Error;
	mjModel* Original = Compile(DynamicXml, Error);
	if (!Original) return false;
	FMjOwnedModelCandidate Preserved;
	const bool bCopied = FMjOwnedModelCandidate::CopyCompiled(Original, Preserved, Error);
	mj_deleteModel(Original);
	if (!bCopied) return false;

	TMap<FString, TArray<uint8>> BadAssets;
	BadAssets.Add(FString(), BytesOf(TetraObj));
	TestFalse(TEXT("empty asset name rejected"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, BytesOf(AssetXml), BadAssets,
		Preserved, Error));
	TestTrue(TEXT("failed factory preserves prior candidate"), Preserved.IsValid());
	BadAssets.Reset();
	BadAssets.Add(TEXT("tetra.obj"), TArray<uint8>());
	TestFalse(TEXT("empty supplied asset data rejected"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, BytesOf(AssetXml), BadAssets,
		Preserved, Error));
	TestTrue(TEXT("empty-asset rejection preserves prior candidate"), Preserved.IsValid());

	TArray<uint8> Truncated = BytesOf(DynamicXml);
	Truncated.Insert(0, Truncated.Num() / 2);
	TestFalse(TEXT("embedded NUL XML rejected"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, Truncated, {}, Preserved, Error));
	TestTrue(TEXT("second failed factory still preserves prior candidate"), Preserved.IsValid());

	FMjSimulationRuntime Runtime;
	TestTrue(TEXT("preserved candidate remains installable"),
		Runtime.Install(MoveTemp(Preserved)).bCommitted);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjIndependentRuntimePrecommitFinalizerTest,
	"URLab.Runtime.Independent.PrecommitFinalizerIsTransactional",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjIndependentRuntimePrecommitFinalizerTest::RunTest(const FString&)
{
	FString Error;
	FMjOwnedModelCandidate Initial;
	if (!TestTrue(TEXT("initial candidate prepares"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, BytesOf(DynamicXml), {}, Initial, Error)))
	{
		AddError(Error);
		return false;
	}
	FMjSimulationRuntime Runtime;
	const FMjSimulationInstallResult First = Runtime.Install(MoveTemp(Initial));
	if (!TestTrue(TEXT("initial candidate commits"), First.bCommitted)) return false;

	FMjOwnedModelCandidate Rejected;
	TMap<FString, TArray<uint8>> RejectedAssets;
	RejectedAssets.Add(TEXT("tetra.obj"), BytesOf(TetraObj));
	FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		BytesOf(AssetXml), RejectedAssets, Rejected, Error);
	FMjSimulationInstallResult Offered;
	const FMjSimulationInstallResult Failed = Runtime.Install(MoveTemp(Rejected),
		[&](const FMjSimulationInstallResult& Proposed, FString& OutError)
		{
			Offered = Proposed;
			OutError = TEXT("hidden scene rejected exact identity");
			return false;
		});
	TestFalse(TEXT("finalizer rejection does not commit"), Failed.bCommitted);
	TestEqual(TEXT("finalizer sees proposed generation"), Offered.Generation,
		static_cast<uint64>(2));
	TestEqual(TEXT("finalizer sees proposed epoch"), Offered.ClockEpoch,
		static_cast<uint64>(2));
	TestEqual(TEXT("rejection error is retained"), Failed.Error,
		FString(TEXT("hidden scene rejected exact identity")));
	TestEqual(TEXT("rejection preserves generation"), Runtime.GetModelGeneration(),
		First.Generation);
	TestEqual(TEXT("rejection preserves epoch"), Runtime.GetClockEpoch(), First.ClockEpoch);

	FMjOwnedModelCandidate Accepted;
	TMap<FString, TArray<uint8>> AcceptedAssets;
	AcceptedAssets.Add(TEXT("tetra.obj"), BytesOf(TetraObj));
	FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		BytesOf(AssetXml), AcceptedAssets, Accepted, Error);
	FMjSimulationInstallResult AcceptedOffer;
	const FMjSimulationInstallResult Second = Runtime.Install(MoveTemp(Accepted),
		[&](const FMjSimulationInstallResult& Proposed, FString&)
		{
			AcceptedOffer = Proposed;
			return true;
		});
	TestTrue(TEXT("accepted finalizer commits"), Second.bCommitted);
	TestEqual(TEXT("rejected reservation was not consumed"), Second.Generation,
		static_cast<uint64>(2));
	TestEqual(TEXT("installed generation matches exact offer"), Second.Generation,
		AcceptedOffer.Generation);
	TestEqual(TEXT("installed epoch matches exact offer"), Second.ClockEpoch,
		AcceptedOffer.ClockEpoch);
	TestEqual(TEXT("installed identity matches exact offer"), Second.Identity.ContentSha256,
		AcceptedOffer.Identity.ContentSha256);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationControlAuthorityTest,
	"URLab.Runtime.Control.AuthorityLifetimeAndReload",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationControlAuthorityTest::RunTest(const FString&)
{
	double SessionNow = 0.0;
	const auto SessionClock = FMjSimulationControlClock::ForTest(
		[&SessionNow]() { return SessionNow; });
	FString SessionError;
	auto BoundarySession = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("boundary"), SessionClock, SessionError);
	if (!TestTrue(TEXT("boundary session created"), BoundarySession.IsValid())) return false;
	SessionNow = 100.0;
	TestTrue(TEXT("session touch extends its independent lifetime"), BoundarySession->Touch());
	SessionNow = 50.0;
	TestTrue(TEXT("backward overlapping touch cannot shorten session lifetime"),
		BoundarySession->Touch());
	SessionNow = 399.0;
	TestTrue(TEXT("session remains active before extended deadline"),
		BoundarySession->IsActiveAt(SessionNow));
	SessionNow = 400.0;
	TestFalse(TEXT("session expires exactly at its deadline"),
		BoundarySession->IsActiveAt(SessionNow));
	auto LocalBoundary = FMjSimulationControlSessionLifetime::CreateLocal(
		TEXT("local-boundary"), SessionClock, SessionError);
	if (!TestTrue(TEXT("local session created"), LocalBoundary.IsValid())) return false;
	TestFalse(TEXT("nonfinite clock is inactive even for a local session"),
		LocalBoundary->IsActiveAt(std::numeric_limits<double>::quiet_NaN()));

	double Now = 100.0;
	const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock =
		FMjSimulationControlClock::ForTest([&Now]() { return Now; });
	FMjSimulationRuntime Runtime(Clock);
	FString Error;
	auto Session = FMjSimulationControlSessionLifetime::CreateRemote(TEXT("remote-a"), Clock, Error);
	if (!TestTrue(TEXT("remote session created"), Session.IsValid())) return false;

	FMjSimulationControlClaimRequest Request;
	Request.Session = Session;
	Request.ExpectedModelGeneration = 1;
	Request.TtlSeconds = 10.0;
	TestEqual(TEXT("claim without model is rejected"), Runtime.ClaimControl(Request).Error,
		EMjSimulationControlError::NoModel);

	FMjOwnedModelCandidate Candidate;
	if (!TestTrue(TEXT("control candidate prepares"), PrepareControlCandidate(Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FMjSimulationInstallResult First = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("control model installs"), First.bCommitted)) return false;
	Request.ExpectedModelGeneration = First.Generation;

	const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> OtherClock =
		FMjSimulationControlClock::ForTest([&Now]() { return Now; });
	auto WrongClockSession = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("wrong-clock"), OtherClock, Error);
	FMjSimulationControlClaimRequest WrongClockRequest = Request;
	WrongClockRequest.Session = WrongClockSession;
	TestEqual(TEXT("session and runtime must share one clock"),
		Runtime.ClaimControl(WrongClockRequest).Error, EMjSimulationControlError::InvalidSession);

	auto Closed = FMjSimulationControlSessionLifetime::CreateRemote(TEXT("closed"), Clock, Error);
	Closed->Deactivate();
	FMjSimulationControlClaimRequest ClosedRequest = Request;
	ClosedRequest.Session = Closed;
	TestEqual(TEXT("queued claim cannot revive a closed session"),
		Runtime.ClaimControl(ClosedRequest).Error, EMjSimulationControlError::InactiveSession);

	const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(Request);
	if (!TestTrue(TEXT("first live source claims control"), Claim.IsOk())) return false;
	TestTrue(TEXT("claim returns opaque proof"), Claim.Handle.IsValid());
	TestTrue(TEXT("reader status reports a claim"), Claim.Status.bClaimed);
	TestEqual(TEXT("claim TTL is retained"), Claim.Status.TtlSeconds, 10.0);

	Now = 105.0;
	TestTrue(TEXT("transport activity can refresh the session"), Session->Touch());
	FMjSimulationControlHandle Forged = Claim.Handle;
	Forged.ClaimToken.Append(TEXT("x"));
	const TArray<double> ZeroControls({0.0, 0.0, 0.0, 0.0});
	const FMjSimulationMutationResult ForgedWrite = Runtime.SetControlVector(
		Forged, ZeroControls);
	TestEqual(TEXT("forged token is rejected"), ForgedWrite.Error,
		EMjSimulationControlError::InvalidToken);
	TestEqual(TEXT("rejected write does not refresh TTL"),
		ForgedWrite.Status.SecondsRemaining, 5.0);

	auto Competitor = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("remote-b"), Clock, Error);
	FMjSimulationControlClaimRequest CompetingRequest = Request;
	CompetingRequest.Session = Competitor;
	TestEqual(TEXT("only one source may hold authority"),
		Runtime.ClaimControl(CompetingRequest).Error, EMjSimulationControlError::AlreadyClaimed);
	const FMjSimulationControlLeaseResult BadRenew = Runtime.RenewControl(Claim.Handle, 0.0);
	TestEqual(TEXT("remote zero TTL is rejected"), BadRenew.Error,
		EMjSimulationControlError::InvalidTtl);
	TestEqual(TEXT("rejected renewal preserves deadline"), BadRenew.Status.SecondsRemaining, 5.0);

	Now = 110.0;
	const FMjSimulationMutationResult Expired = Runtime.AuthorizedForward(Claim.Handle);
	TestEqual(TEXT("lease expires exactly at its deadline"), Expired.Error,
		EMjSimulationControlError::Expired);
	TestFalse(TEXT("expired lease is no longer published"), Expired.Status.bClaimed);

	CompetingRequest.TtlSeconds = 30.0;
	const FMjSimulationControlClaimResult SecondClaim = Runtime.ClaimControl(CompetingRequest);
	if (!TestTrue(TEXT("new source can claim after expiry"), SecondClaim.IsOk())) return false;
	Now = 115.0;
	const FMjSimulationControlLeaseResult Renewed =
		Runtime.RenewControl(SecondClaim.Handle, 20.0);
	TestTrue(TEXT("explicit renewal succeeds"), Renewed.bChanged);
	TestEqual(TEXT("renewal replaces and refreshes TTL"), Renewed.Status.SecondsRemaining, 20.0);
	TestEqual(TEXT("wrong release token is rejected"), Runtime.ReleaseControl(Forged).Error,
		EMjSimulationControlError::InvalidToken);
	TestTrue(TEXT("authoritative release succeeds"),
		Runtime.ReleaseControl(SecondClaim.Handle).bChanged);
	TestEqual(TEXT("released handle cannot mutate"),
		Runtime.AuthorizedForward(SecondClaim.Handle).Error,
		EMjSimulationControlError::NotClaimed);

	auto RevokedSession = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("remote-c"), Clock, Error);
	FMjSimulationControlClaimRequest RevokeRequest = CompetingRequest;
	RevokeRequest.Session = RevokedSession;
	const FMjSimulationControlClaimResult RevokedClaim = Runtime.ClaimControl(RevokeRequest);
	if (!TestTrue(TEXT("revocation fixture claims"), RevokedClaim.IsOk())) return false;
	RevokedSession->Deactivate();
	TestTrue(TEXT("disconnect revocation clears matching authority"),
		Runtime.RevokeControlForSession(RevokedSession.ToSharedRef()).bChanged);
	TestEqual(TEXT("deactivated session cannot reclaim"), Runtime.ClaimControl(RevokeRequest).Error,
		EMjSimulationControlError::InactiveSession);

	auto ReloadSession = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("remote-d"), Clock, Error);
	FMjSimulationControlClaimRequest ReloadRequest = CompetingRequest;
	ReloadRequest.Session = ReloadSession;
	const FMjSimulationControlClaimResult ReloadClaim = Runtime.ClaimControl(ReloadRequest);
	if (!TestTrue(TEXT("reload fixture claims"), ReloadClaim.IsOk())) return false;
	FMjOwnedModelCandidate Replacement;
	PrepareControlCandidate(Replacement, Error);
	const FMjSimulationInstallResult Reload = Runtime.Install(MoveTemp(Replacement));
	TestTrue(TEXT("replacement installs"), Reload.bCommitted);
	TestFalse(TEXT("reload clears authority"), Runtime.GetControlStatus().bClaimed);
	TestEqual(TEXT("old handle is explicitly stale after reload"),
		Runtime.AuthorizedForward(ReloadClaim.Handle).Error,
		EMjSimulationControlError::WrongModelGeneration);

	const auto SystemA = FMjSimulationControlClock::System();
	const auto SystemB = FMjSimulationControlClock::System();
	TestTrue(TEXT("default clocks share one process identity"), &SystemA.Get() == &SystemB.Get());
	double OldNow = 10.0;
	double NewNow = 1000.0;
	const auto OldClock = FMjSimulationControlClock::ForTest([&OldNow]() { return OldNow; });
	const auto NewClock = FMjSimulationControlClock::ForTest([&NewNow]() { return NewNow; });
	auto OldScope = FMjSimulationControlSessionLifetime::CreateRemote(
		TEXT("rotated"), OldClock, Error);
	OldNow = 110.0;
	TestTrue(TEXT("activity refreshes the old absolute session deadline"), OldScope->Touch());
	OldNow = 120.0;
	OldScope->Deactivate();
	auto Successor = OldScope->CreateSuccessorScope(NewClock, Error);
	if (!TestTrue(TEXT("detach creates an active successor scope"), Successor.IsValid())) return false;
	NewNow = 1289.999;
	TestTrue(TEXT("clock rotation preserves remaining session lifetime"),
		Successor->IsActiveAt(NewNow));
	NewNow = 1290.0;
	TestFalse(TEXT("successor expires at the preserved boundary"),
		Successor->IsActiveAt(NewNow));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationControlWriteTest,
	"URLab.Runtime.Control.FullAndNamedWritesAreAtomic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationControlWriteTest::RunTest(const FString&)
{
	double Now = 20.0;
	const auto Clock = FMjSimulationControlClock::ForTest([&Now]() { return Now; });
	FMjSimulationRuntime Runtime(Clock);
	FString Error;
	FMjOwnedModelCandidate Candidate;
	if (!PrepareControlCandidate(Candidate, Error)) return false;
	const FMjSimulationInstallResult Install = Runtime.Install(MoveTemp(Candidate));
	if (!TestTrue(TEXT("control model installed"), Install.bCommitted)) return false;
	FMjModelDescriptor Descriptor;
	Runtime.GetDescriptor(Descriptor);
	TestEqual(TEXT("fixture has two native actuators"), Descriptor.Actuators.Num(), 2);
	TestEqual(TEXT("multi-input actuator makes nu differ from nactuator"),
		Descriptor.Identity.Nu, 4);
	TestEqual(TEXT("orientation native control width is three"),
		Descriptor.Actuators[0].ControlWidth, 3);

	auto Session = FMjSimulationControlSessionLifetime::CreateRemote(TEXT("writer"), Clock, Error);
	FMjSimulationControlClaimRequest Request;
	Request.Session = Session;
	Request.ExpectedModelGeneration = Install.Generation;
	Request.TtlSeconds = 20.0;
	const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(Request);
	if (!TestTrue(TEXT("writer claims control"), Claim.IsOk())) return false;
	Now = 21.0;

	FMjModelObservation Initial;
	Runtime.GetObservation(Initial);
	const TArray<double> TooShort({0.0, 0.0});
	const FMjSimulationMutationResult WrongWidth = Runtime.SetControlVector(
		Claim.Handle, TooShort);
	TestEqual(TEXT("full vector requires exact nu"), WrongWidth.Error,
		EMjSimulationControlError::WrongControlWidth);
	TestEqual(TEXT("wrong width does not refresh TTL"), WrongWidth.Status.SecondsRemaining, 19.0);

	TArray<double> NonFinite({0.0, 0.0, 0.0, 0.0});
	NonFinite[1] = std::numeric_limits<double>::quiet_NaN();
	TestEqual(TEXT("nonfinite scalar is rejected"),
		Runtime.SetControlVector(Claim.Handle, NonFinite).Error,
		EMjSimulationControlError::NonFinite);
	TArray<double> OutOfRange({0.0, 0.0, 0.0, 2.0});
	TestEqual(TEXT("limited scalar is rejected instead of clamped"),
		Runtime.SetControlVector(Claim.Handle, OutOfRange).Error,
		EMjSimulationControlError::OutOfRange);

	TMap<FString, TArray<double>> PartialBad;
	PartialBad.Add(TEXT("orientation"), TArray<double>({0.2, 0.3, 0.4}));
	PartialBad.Add(TEXT("drive"), TArray<double>({2.0}));
	const FMjSimulationMutationResult AtomicReject =
		Runtime.SetNamedActuatorControls(Claim.Handle, PartialBad);
	TestEqual(TEXT("one invalid named span rejects the whole write"), AtomicReject.Error,
		EMjSimulationControlError::OutOfRange);
	FMjModelObservation AfterReject;
	Runtime.GetObservation(AfterReject);
	TestTrue(TEXT("rejected writes preserve every control"),
		SameValues(Initial.Ctrl, AfterReject.Ctrl));

	TMap<FString, TArray<double>> WrongNamedWidth;
	WrongNamedWidth.Add(TEXT("orientation"), TArray<double>({0.1, 0.2}));
	TestEqual(TEXT("named actuator requires exact native span width"),
		Runtime.SetNamedActuatorControls(Claim.Handle, WrongNamedWidth).Error,
		EMjSimulationControlError::WrongControlWidth);
	TMap<FString, TArray<double>> EmptyName;
	EmptyName.Add(FString(), TArray<double>({0.0}));
	TestEqual(TEXT("empty actuator name is never an alias"),
		Runtime.SetNamedActuatorControls(Claim.Handle, EmptyName).Error,
		EMjSimulationControlError::UnknownActuator);
	TMap<FString, TArray<double>> Named;
	Named.Add(TEXT("orientation"), TArray<double>({0.2, 0.3, 0.4}));
	Named.Add(TEXT("drive"), TArray<double>({0.75}));
	const FMjSimulationMutationResult NamedWrite =
		Runtime.SetNamedActuatorControls(Claim.Handle, Named);
	TestTrue(TEXT("exact named spans commit"), NamedWrite.IsOk());
	TestTrue(TEXT("named result owns committed controls"),
		SameValues(NamedWrite.Observation.Ctrl, TArray<double>({0.2, 0.3, 0.4, 0.75})));
	const TArray<double> FullValues({-0.2, -0.3, -0.4, -0.75});
	const FMjSimulationMutationResult FullWrite =
		Runtime.SetControlVector(Claim.Handle, FullValues);
	TestTrue(TEXT("exact full-nu vector commits"), FullWrite.IsOk());
	TestTrue(TEXT("full vector result owns committed controls"),
		SameValues(FullWrite.Observation.Ctrl, FullValues));
	TMap<FString, TArray<double>> DriveOnly;
	DriveOnly.Add(TEXT("drive"), TArray<double>({0.25}));
	const FMjSimulationMutationResult PartialWrite =
		Runtime.SetNamedActuatorControls(Claim.Handle, DriveOnly);
	TestTrue(TEXT("one named span commits independently"), PartialWrite.IsOk());
	TestTrue(TEXT("named write preserves untouched native spans"),
		SameValues(PartialWrite.Observation.Ctrl,
			TArray<double>({-0.2, -0.3, -0.4, 0.25})));

	const uint64 SequenceBeforeReject = Runtime.GetStepSequence();
	TestEqual(TEXT("authorized integration is bounded"),
		Runtime.AuthorizedStep(Claim.Handle,
			FMjSimulationRuntime::MaxAuthorizedStepCount + 1).Error,
		EMjSimulationControlError::InvalidStepCount);
	TestEqual(TEXT("rejected step preserves sequence"), Runtime.GetStepSequence(),
		SequenceBeforeReject);
	const uint64 EpochBeforeForward = Runtime.GetClockEpoch();
	const uint64 SequenceBeforeForward = Runtime.GetStepSequence();
	const FMjSimulationMutationResult Forward = Runtime.AuthorizedForward(Claim.Handle);
	TestTrue(TEXT("authorized forward commits"), Forward.IsOk());
	TestEqual(TEXT("forward preserves epoch"), Forward.Observation.ClockEpoch, EpochBeforeForward);
	TestEqual(TEXT("forward does not integrate"), Forward.Observation.StepSequence,
		SequenceBeforeForward);
	const FMjSimulationMutationResult Step = Runtime.AuthorizedStep(Claim.Handle, 2);
	TestTrue(TEXT("authorized bounded step commits"), Step.IsOk());
	TestEqual(TEXT("step result is sampled under the commit lock"), Step.Observation.StepSequence,
		SequenceBeforeForward + 2);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimulationControlStateTest,
	"URLab.Runtime.Control.StateResetAndStagedControlContinuity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSimulationControlStateTest::RunTest(const FString&)
{
	double Now = 1.0;
	const auto Clock = FMjSimulationControlClock::ForTest([&Now]() { return Now; });
	FMjSimulationRuntime Runtime(Clock);
	FString Error;
	FMjOwnedModelCandidate Candidate;
	if (!PrepareControlCandidate(Candidate, Error)) return false;
	const FMjSimulationInstallResult Install = Runtime.Install(MoveTemp(Candidate));
	auto Session = FMjSimulationControlSessionLifetime::CreateRemote(TEXT("state"), Clock, Error);
	FMjSimulationControlClaimRequest Request;
	Request.Session = Session;
	Request.ExpectedModelGeneration = Install.Generation;
	const FMjSimulationControlClaimResult Claim = Runtime.ClaimControl(Request);
	if (!TestTrue(TEXT("state source claims"), Claim.IsOk())) return false;

	const TArray<double> SavedControls({0.4, -0.3, 0.2, 0.5});
	TestTrue(TEXT("checkpoint controls stage"),
		Runtime.SetControlVector(Claim.Handle, SavedControls).IsOk());
	FMjModelState CtrlState;
	TestEqual(TEXT("CTRL state captures"), Runtime.CaptureState(mjSTATE_CTRL, CtrlState),
		EMjModelStateError::None);
	const TArray<double> ReplacementControls({-0.4, 0.3, -0.2, -0.5});
	TestTrue(TEXT("replacement controls stage"),
		Runtime.SetControlVector(Claim.Handle, ReplacementControls).IsOk());
	const FMjSimulationMutationResult Restore =
		Runtime.AuthorizedApplyState(Claim.Handle, CtrlState);
	TestTrue(TEXT("authorized CTRL restore commits"), Restore.IsOk());
	TestTrue(TEXT("CTRL restore replaces staged controls"),
		SameValues(Restore.Observation.Ctrl, SavedControls));
	const FMjSimulationMutationResult AfterRestoreStep = Runtime.AuthorizedStep(Claim.Handle, 1);
	TestTrue(TEXT("restored controls survive the next integration"),
		SameValues(AfterRestoreStep.Observation.Ctrl, SavedControls));

	FMjModelState TimeState;
	TestEqual(TEXT("time-only state captures"), Runtime.CaptureState(mjSTATE_TIME, TimeState),
		EMjModelStateError::None);
	TestTrue(TEXT("new controls stage before time restore"),
		Runtime.SetControlVector(Claim.Handle, ReplacementControls).IsOk());
	const uint64 RevisionBeforeTimeRestore = Runtime.GetControlStatus().ControlRevision;
	const FMjSimulationMutationResult TimeRestore =
		Runtime.AuthorizedApplyState(Claim.Handle, TimeState);
	TestTrue(TEXT("state without CTRL commits"), TimeRestore.IsOk());
	TestTrue(TEXT("state without CTRL preserves staged controls"),
		SameValues(TimeRestore.Observation.Ctrl, ReplacementControls));
	TestEqual(TEXT("state without CTRL preserves control revision"),
		TimeRestore.Status.ControlRevision, RevisionBeforeTimeRestore);

	FMjModelState WrongState = TimeState;
	WrongState.Identity.Generation++;
	const double RemainingBeforeReject = Runtime.GetControlStatus().SecondsRemaining;
	const FMjSimulationMutationResult Rejected =
		Runtime.AuthorizedApplyState(Claim.Handle, WrongState);
	TestEqual(TEXT("wrong-model state is rejected"), Rejected.Error,
		EMjSimulationControlError::InvalidState);
	TestFalse(TEXT("rejected state is not reported committed"), Rejected.bCommitted);
	TestEqual(TEXT("rejected state does not refresh control TTL"),
		Rejected.Status.SecondsRemaining, RemainingBeforeReject);

	const uint64 EpochBeforeReset = Runtime.GetClockEpoch();
	const FMjSimulationMutationResult Reset = Runtime.AuthorizedReset(Claim.Handle);
	TestTrue(TEXT("authorized reset commits"), Reset.IsOk());
	TestTrue(TEXT("reset advances epoch"), Reset.Observation.ClockEpoch > EpochBeforeReset);
	TestEqual(TEXT("reset clears step sequence"), Reset.Observation.StepSequence,
		static_cast<uint64>(0));
	TestTrue(TEXT("reset clears staged and native controls"),
		SameValues(Reset.Observation.Ctrl, TArray<double>({0.0, 0.0, 0.0, 0.0})));
	TestTrue(TEXT("reset preserves authority"), Reset.Status.bClaimed);
	TestTrue(TEXT("preserved reset authority remains usable"),
		Runtime.SetControlVector(Claim.Handle, SavedControls).IsOk());
	return !HasAnyErrors();
}
