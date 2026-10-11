// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/ScopeExit.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/URLabRenderScene.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderWorldCoordinatorTest
{
struct FIsolatedRenderWorld
{
	UWorld* World = nullptr;

	bool Create()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		return true;
	}

	~FIsolatedRenderWorld()
	{
		if (!World) return;
		World->DestroyWorld(false);
		if (GEngine) GEngine->DestroyWorldContext(World);
	}
};

void MakeOneBodyScene(const FString& Revision, double X, FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FRenderFrame& OutFrame)
{
	OutScene.SceneRevision = Revision;
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("neutral_body");
	OutResources.SceneRevision = Revision;
	OutFrame.Identity.SceneRevision = Revision;
	OutFrame.BodySceneTransforms.Add(FTransform(FVector(X, 0.0, 0.0)));
}

bool StageOneBody(UURLabRenderWorldSubsystem& Coordinator,
	const FURLabRenderProducerRegistration& Registration, const FString& Baseline,
	const FString& Revision, double X, FURLabRenderSceneCandidate& OutCandidate,
	FString& OutError)
{
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Frame;
	MakeOneBodyScene(Revision, X, Scene, Resources, Frame);
	FRenderSceneInstallOptions Options;
	return Coordinator.StageScene(Registration, Baseline, FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(Frame)), Options, OutCandidate, OutError);
}

AURLabRenderScene* FindScene(UWorld& World, const FString& Revision)
{
	for (TActorIterator<AURLabRenderScene> It(&World); It; ++It)
	{
		if (It->GetScene().SceneRevision == Revision) return *It;
	}
	return nullptr;
}

int32 CountSceneActors(UWorld& World)
{
	int32 Count = 0;
	for (TActorIterator<AURLabRenderScene> It(&World); It; ++It) ++Count;
	return Count;
}

class FCountingSceneLease final : public IRenderSceneResourceLifetimeLease
{
public:
	explicit FCountingSceneLease(int32& InLive) : Live(InLive) { ++Live; }
	~FCountingSceneLease() override { --Live; }
private:
	int32& Live;
};

class FOrderCheckingSceneLease final : public IRenderSceneResourceLifetimeLease
{
public:
	explicit FOrderCheckingSceneLease(TFunction<void()> InOnDestroy)
		: OnDestroy(MoveTemp(InOnDestroy)) {}
	~FOrderCheckingSceneLease() override { OnDestroy(); }
private:
	TFunction<void()> OnDestroy;
};

FRenderSceneInstallPayload MakeLeasedOneBody(const FString& Revision, int32& Live)
{
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Frame;
	MakeOneBodyScene(Revision, 0.0, Scene, Resources, Frame);
	return FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), MoveTemp(Frame),
		MakeShared<FCountingSceneLease, ESPMode::ThreadSafe>(Live));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderWorldAtomicCommitTest,
	"URLab.Render.WorldCoordinator.AtomicRelatedStateCommit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderWorldAtomicCommitTest::RunTest(const FString&)
{
	using namespace URLabRenderWorldCoordinatorTest;
	FIsolatedRenderWorld Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("world owns neutral render coordinator"), Coordinator)) return false;

	FString Error;
	FURLabRenderProducerRegistration Registration;
	TestEqual(TEXT("registration starts model-free"), CountSceneActors(*Fixture.World), 0);
	if (!TestTrue(TEXT("sole producer registers"),
		Coordinator->RegisterProducer(Registration, Error)))
	{
		AddError(Error);
		return false;
	}
	TestEqual(TEXT("registration creates no scene actor"), CountSceneActors(*Fixture.World), 0);
	FURLabRenderProducerRegistration RefusedRegistration;
	TestFalse(TEXT("second producer is refused"),
		Coordinator->RegisterProducer(RefusedRegistration, Error));

	FURLabRenderSceneCandidate FirstCandidate;
	if (!TestTrue(TEXT("initial scene stages hidden"), StageOneBody(*Coordinator,
		Registration, TEXT(""), TEXT("neutral-a"), 10.0, FirstCandidate, Error)))
	{
		AddError(Error);
		return false;
	}
	AURLabRenderScene* FirstActor = FindScene(*Fixture.World, TEXT("neutral-a"));
	if (!TestNotNull(TEXT("staged actor exists"), FirstActor)) return false;
	TestNull(TEXT("staging does not publish a scene"),
		Coordinator->ResolveActiveScene(Registration));
	TestTrue(TEXT("staged actor is hidden"), FirstActor->IsHidden());
	FURLabRenderSceneCandidate OtherCandidate;
	if (!TestTrue(TEXT("another candidate stages against the same baseline"), StageOneBody(
		*Coordinator, Registration, TEXT(""), TEXT("neutral-other"), 15.0,
		OtherCandidate, Error)))
	{
		AddError(Error);
		return false;
	}
	AURLabRenderScene* OtherActor = FindScene(*Fixture.World, TEXT("neutral-other"));
	if (!TestNotNull(TEXT("other staged actor exists"), OtherActor)) return false;

	FRenderFrame FinalizedFirst;
	FinalizedFirst.Identity.SceneRevision = TEXT("neutral-a");
	FinalizedFirst.BodySceneTransforms.Add(FTransform(FVector(25.0, 0.0, 0.0)));
	FRenderFrame OtherFrame;
	OtherFrame.Identity.SceneRevision = TEXT("neutral-other");
	OtherFrame.BodySceneTransforms.Add(FTransform(FVector(35.0, 0.0, 0.0)));
	bool bCallbackEntered = false;
	FURLabRenderWorldCommitResult Rejected = Coordinator->CommitCandidate(
		Registration, FirstCandidate, [&](FString& OutError)
		{
			bCallbackEntered = true;
			FURLabRenderProducerRegistration DuringCommit;
			FString GuardError;
			TestFalse(TEXT("registration mutation is guarded"),
				Coordinator->RegisterProducer(DuringCommit, GuardError));
			TestFalse(TEXT("release is guarded"),
				Coordinator->ReleaseProducer(Registration, GuardError));
			TestFalse(TEXT("discard is guarded"), Coordinator->DiscardCandidate(
				Registration, FirstCandidate, GuardError));
			TestFalse(TEXT("another candidate cannot finalize in callback"),
				Coordinator->ApplyCandidateFrame(Registration, OtherCandidate,
					OtherFrame, GuardError));
			TestTrue(TEXT("exact candidate frame finalizes inside callback"),
				Coordinator->ApplyCandidateFrame(Registration, FirstCandidate,
					FinalizedFirst, GuardError));
			OutError = TEXT("related-state fixture rejected commit");
			return false;
		});
	TestTrue(TEXT("related-state callback ran"), bCallbackEntered);
	TestFalse(TEXT("related-state rejection prevents publication"), Rejected.bCommitted);
	TestEqual(TEXT("related-state error is retained"), Rejected.Error,
		FString(TEXT("related-state fixture rejected commit")));
	TestNull(TEXT("failed callback preserves model-free active scene"),
		Coordinator->ResolveActiveScene(Registration));
	TestFalse(TEXT("failed callback does not destroy candidate"),
		FirstActor->IsActorBeingDestroyed());

	FURLabRenderWorldCommitResult FirstCommit = Coordinator->CommitCandidate(
		Registration, FirstCandidate);
	TestTrue(TEXT("retained candidate commits"), FirstCommit.bCommitted);
	TestEqual(TEXT("first generation"), FirstCommit.SceneGeneration, uint64(1));
	TestEqual(TEXT("first revision"), FirstCommit.SceneRevision, FString(TEXT("neutral-a")));
	TestTrue(TEXT("first scene is current"),
		Coordinator->ResolveActiveScene(Registration) == FirstActor);
	TestFalse(TEXT("committed scene is exposed"), FirstActor->IsHidden());
	TestTrue(TEXT("callback-finalized frame is published"),
		FirstActor->GetAppliedFrame().BodySceneTransforms[0].GetTranslation().Equals(
			FVector(25.0, 0.0, 0.0)));
	TestTrue(TEXT("successful commit retires alternate staged candidate"),
		OtherActor->IsActorBeingDestroyed());

	FRenderScene InvalidScene;
	FRenderSceneResources InvalidResources;
	FRenderFrame InvalidFrame;
	MakeOneBodyScene(TEXT("invalid-b"), 80.0, InvalidScene, InvalidResources, InvalidFrame);
	InvalidFrame.Identity.SceneRevision = TEXT("wrong-revision");
	FURLabRenderSceneCandidate InvalidCandidate;
	TestFalse(TEXT("fallible staging failure is rejected"), Coordinator->StageScene(
		Registration, TEXT("neutral-a"), FRenderSceneInstallPayload(MoveTemp(InvalidScene), MoveTemp(InvalidResources), FRenderFrame(InvalidFrame)), FRenderSceneInstallOptions(), InvalidCandidate, Error));
	TestTrue(TEXT("staging failure preserves active actor"),
		Coordinator->ResolveActiveScene(Registration) == FirstActor);

	FURLabRenderSceneCandidate SecondCandidate;
	if (!TestTrue(TEXT("replacement stages"), StageOneBody(*Coordinator, Registration,
		TEXT("neutral-a"), TEXT("neutral-b"), 40.0, SecondCandidate, Error)))
	{
		AddError(Error);
		return false;
	}
	AURLabRenderScene* SecondActor = FindScene(*Fixture.World, TEXT("neutral-b"));
	if (!TestNotNull(TEXT("replacement actor exists"), SecondActor)) return false;
	TestTrue(TEXT("replacement remains hidden before commit"), SecondActor->IsHidden());
	TestTrue(TEXT("replacement staging preserves first actor"),
		Coordinator->ResolveActiveScene(Registration) == FirstActor);
	bool bDestroyNotificationObserved = false;
	bool bReentrantStageAccepted = false;
	FURLabRenderSceneCandidate ReentrantCandidate;
	FString ReentrantError;
	const FDelegateHandle DestroyHandle = Fixture.World->AddOnActorDestroyedHandler(
		FOnActorDestroyed::FDelegate::CreateLambda([&](AActor* DestroyedActor)
		{
			if (DestroyedActor != FirstActor || bDestroyNotificationObserved) return;
			bDestroyNotificationObserved = true;
			bReentrantStageAccepted = StageOneBody(*Coordinator, Registration,
				TEXT("neutral-b"), TEXT("reentrant"), 50.0, ReentrantCandidate,
				ReentrantError);
		}));
	ON_SCOPE_EXIT
	{
		if (Fixture.World) Fixture.World->RemoveOnActorDestroyedHandler(DestroyHandle);
	};

	FRenderFrame WrongTarget = FinalizedFirst;
	WrongTarget.BodySceneTransforms[0].SetTranslation(FVector(999.0, 0.0, 0.0));
	TestFalse(TEXT("wrong active revision rejects frame"), Coordinator->ApplyFrame(
		Registration, TEXT("neutral-b"), WrongTarget, Error));
	TestTrue(TEXT("rejected active frame preserves transform"),
		FirstActor->GetAppliedFrame().BodySceneTransforms[0].GetTranslation().Equals(
			FVector(25.0, 0.0, 0.0)));

	FURLabRenderWorldCommitResult SecondCommit = Coordinator->CommitCandidate(
		Registration, SecondCandidate);
	TestTrue(TEXT("replacement commits"), SecondCommit.bCommitted);
	TestEqual(TEXT("replacement generation"), SecondCommit.SceneGeneration, uint64(2));
	TestTrue(TEXT("replacement is current"),
		Coordinator->ResolveActiveScene(Registration) == SecondActor);
	TestTrue(TEXT("old actor retires after swap"), FirstActor->IsActorBeingDestroyed());
	TestTrue(TEXT("old actor destruction notification ran"), bDestroyNotificationObserved);
	TestFalse(TEXT("actor destruction cannot reenter scene staging"),
		bReentrantStageAccepted);
	TestFalse(TEXT("reentrant staging did not mint a candidate"),
		ReentrantCandidate.IsValid());
	TestFalse(TEXT("consumed candidate cannot commit twice"),
		Coordinator->CommitCandidate(Registration, SecondCandidate).bCommitted);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderWorldStaleCleanupTest,
	"URLab.Render.WorldCoordinator.StaleTokensAndOwnedCleanup",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderWorldStaleCleanupTest::RunTest(const FString&)
{
	using namespace URLabRenderWorldCoordinatorTest;
	FIsolatedRenderWorld Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("world owns neutral render coordinator"), Coordinator)) return false;

	FString Error;
	FURLabRenderProducerRegistration FirstRegistration;
	if (!TestTrue(TEXT("first producer registers"),
		Coordinator->RegisterProducer(FirstRegistration, Error))) return false;
	FURLabRenderSceneCandidate Winner;
	FURLabRenderSceneCandidate Superseded;
	TestTrue(TEXT("winner stages"), StageOneBody(*Coordinator, FirstRegistration,
		TEXT(""), TEXT("winner"), 1.0, Winner, Error));
	TestTrue(TEXT("parallel candidate stages against same baseline"), StageOneBody(
		*Coordinator, FirstRegistration, TEXT(""), TEXT("superseded"), 2.0,
		Superseded, Error));
	FURLabRenderSceneCandidate Overflow;
	TestFalse(TEXT("third staged candidate is rejected before allocation"), StageOneBody(
		*Coordinator, FirstRegistration, TEXT(""), TEXT("overflow"), 3.0,
		Overflow, Error));
	TestFalse(TEXT("overflow does not mint a candidate"), Overflow.IsValid());
	TestEqual(TEXT("overflow preserves the two staged actors"),
		CountSceneActors(*Fixture.World), 2);
	AURLabRenderScene* SupersededActor = FindScene(*Fixture.World, TEXT("superseded"));
	if (!TestNotNull(TEXT("parallel staged actor exists"), SupersededActor)) return false;
	TestTrue(TEXT("winner commits"),
		Coordinator->CommitCandidate(FirstRegistration, Winner).bCommitted);
	TestTrue(TEXT("commit cleans other staged actor"),
		SupersededActor->IsActorBeingDestroyed());
	TestFalse(TEXT("cleaned candidate is stale"),
		Coordinator->CommitCandidate(FirstRegistration, Superseded).bCommitted);

	AURLabRenderScene* FirstActive = Coordinator->ResolveActiveScene(FirstRegistration);
	if (!TestNotNull(TEXT("first active scene exists"), FirstActive)) return false;
	TestTrue(TEXT("exact release succeeds"),
		Coordinator->ReleaseProducer(FirstRegistration, Error));
	TestTrue(TEXT("release retires active scene"), FirstActive->IsActorBeingDestroyed());
	TestFalse(TEXT("released registration is stale"),
		Coordinator->IsRegistrationCurrent(FirstRegistration));

	FURLabRenderProducerRegistration SuccessorRegistration;
	TestTrue(TEXT("successor producer registers"),
		Coordinator->RegisterProducer(SuccessorRegistration, Error));
	TestFalse(TEXT("stale release cannot detach successor"),
		Coordinator->ReleaseProducer(FirstRegistration, Error));
	TestTrue(TEXT("successor remains current"),
		Coordinator->IsRegistrationCurrent(SuccessorRegistration));

	FURLabRenderSceneCandidate Discarded;
	TestTrue(TEXT("successor stages discardable scene"), StageOneBody(*Coordinator,
		SuccessorRegistration, TEXT(""), TEXT("discarded"), 3.0, Discarded, Error));
	AURLabRenderScene* DiscardedActor = FindScene(*Fixture.World, TEXT("discarded"));
	if (!TestNotNull(TEXT("discardable actor exists"), DiscardedActor)) return false;
	TestTrue(TEXT("exact discard succeeds"), Coordinator->DiscardCandidate(
		SuccessorRegistration, Discarded, Error));
	TestTrue(TEXT("discard retires candidate actor"), DiscardedActor->IsActorBeingDestroyed());
	TestFalse(TEXT("discarded candidate cannot commit"),
		Coordinator->CommitCandidate(SuccessorRegistration, Discarded).bCommitted);

	FURLabRenderSceneCandidate SuccessorCandidate;
	TestTrue(TEXT("successor stages active scene"), StageOneBody(*Coordinator,
		SuccessorRegistration, TEXT(""), TEXT("successor"), 4.0,
		SuccessorCandidate, Error));
	TestTrue(TEXT("successor scene commits"),
		Coordinator->CommitCandidate(SuccessorRegistration, SuccessorCandidate).bCommitted);
	AURLabRenderScene* SuccessorActive =
		Coordinator->ResolveActiveScene(SuccessorRegistration);
	if (!TestNotNull(TEXT("successor active scene exists"), SuccessorActive)) return false;
	TestTrue(TEXT("successor release succeeds"),
		Coordinator->ReleaseProducer(SuccessorRegistration, Error));
	TestTrue(TEXT("successor release cleans active actor"),
		SuccessorActive->IsActorBeingDestroyed());
	TestNull(TEXT("stale registration resolves no active scene"),
		Coordinator->ResolveActiveScene(SuccessorRegistration));

	Coordinator->Deinitialize();
	FURLabRenderProducerRegistration AfterShutdown;
	TestFalse(TEXT("held subsystem pointer cannot register after deinitialize"),
		Coordinator->RegisterProducer(AfterShutdown, Error));
	TestFalse(TEXT("deinitialized coordinator returns no registration"),
		AfterShutdown.IsValid());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderWorldDeferredDeinitializeTest,
	"URLab.Render.WorldCoordinator.DeferredDeinitializeDuringRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderWorldDeferredDeinitializeTest::RunTest(const FString&)
{
	using namespace URLabRenderWorldCoordinatorTest;
	FIsolatedRenderWorld Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("world owns neutral render coordinator"), Coordinator)) return false;

	FString Error;
	FURLabRenderProducerRegistration Registration;
	TestTrue(TEXT("producer registers"), Coordinator->RegisterProducer(Registration, Error));
	FURLabRenderSceneCandidate InitialCandidate;
	TestTrue(TEXT("initial scene stages"), StageOneBody(*Coordinator, Registration,
		TEXT(""), TEXT("deinit-a"), 1.0, InitialCandidate, Error));
	TestTrue(TEXT("initial scene commits"),
		Coordinator->CommitCandidate(Registration, InitialCandidate).bCommitted);
	AURLabRenderScene* InitialActor =
		Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("initial actor is active"), InitialActor)) return false;

	FURLabRenderSceneCandidate ReplacementCandidate;
	TestTrue(TEXT("replacement scene stages"), StageOneBody(*Coordinator, Registration,
		TEXT("deinit-a"), TEXT("deinit-b"), 2.0, ReplacementCandidate, Error));
	AURLabRenderScene* ReplacementActor = FindScene(*Fixture.World, TEXT("deinit-b"));
	if (!TestNotNull(TEXT("replacement actor exists"), ReplacementActor)) return false;
	bool bDestroyCallbackRan = false;
	const FDelegateHandle DestroyHandle = Fixture.World->AddOnActorDestroyedHandler(
		FOnActorDestroyed::FDelegate::CreateLambda([&](AActor* DestroyedActor)
		{
			if (DestroyedActor != InitialActor || bDestroyCallbackRan) return;
			bDestroyCallbackRan = true;
			Coordinator->Deinitialize();
		}));
	const FURLabRenderWorldCommitResult Replacement = Coordinator->CommitCandidate(
		Registration, ReplacementCandidate);
	Fixture.World->RemoveOnActorDestroyedHandler(DestroyHandle);

	TestTrue(TEXT("accepted replacement finishes before deferred shutdown"),
		Replacement.bCommitted);
	TestTrue(TEXT("retirement callback requested deinitialize"), bDestroyCallbackRan);
	TestTrue(TEXT("deferred shutdown retires the just-committed actor"),
		ReplacementActor->IsActorBeingDestroyed());
	TestFalse(TEXT("registration is stale after deferred shutdown"),
		Coordinator->IsRegistrationCurrent(Registration));
	FURLabRenderProducerRegistration AfterShutdown;
	TestFalse(TEXT("deinitialized coordinator refuses a new producer"),
		Coordinator->RegisterProducer(AfterShutdown, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderWorldSceneLeaseLifetimeTest,
	"URLab.Render.WorldCoordinator.SceneLeaseLifetime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderWorldSceneLeaseLifetimeTest::RunTest(const FString&)
{
	using namespace URLabRenderWorldCoordinatorTest;
	int32 AssignedFromLive = 0;
	int32 AssignedToLive = 0;
	bool bOldArraysFreedBeforeLease = false;
	{
		FRenderSceneInstallPayload AssignedFrom =
			MakeLeasedOneBody(TEXT("assigned-from"), AssignedFromLive);
		AssignedFrom.LifetimeLease = MakeShared<FOrderCheckingSceneLease, ESPMode::ThreadSafe>(
			[&AssignedFrom, &bOldArraysFreedBeforeLease]()
			{
				bOldArraysFreedBeforeLease = AssignedFrom.Scene.Bodies.IsEmpty()
					&& AssignedFrom.Resources.Geoms.IsEmpty()
					&& AssignedFrom.InitialFrame.BodySceneTransforms.IsEmpty();
			});
		AssignedFromLive = 0;
		FRenderSceneInstallPayload AssignedTo =
			MakeLeasedOneBody(TEXT("assigned-to"), AssignedToLive);
		AssignedFrom = MoveTemp(AssignedTo);
		TestEqual(TEXT("move assignment retires old arrays before old lease"),
			AssignedFromLive, 0);
		TestTrue(TEXT("move assignment frees arrays before releasing lease"),
			bOldArraysFreedBeforeLease);
		TestEqual(TEXT("move assignment retains incoming lease"), AssignedToLive, 1);
	}
	TestEqual(TEXT("assigned payload destruction releases incoming lease"), AssignedToLive, 0);

	FIsolatedRenderWorld Fixture;
	if (!TestTrue(TEXT("isolated game world creates"), Fixture.Create())) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		Fixture.World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("coordinator exists"), Coordinator)) return false;
	FString Error;
	FURLabRenderProducerRegistration Registration;
	if (!TestTrue(TEXT("producer registers"),
		Coordinator->RegisterProducer(Registration, Error))) return false;

	int32 FailedLive = 0;
	FRenderSceneInstallPayload Invalid = MakeLeasedOneBody(TEXT("bad"), FailedLive);
	Invalid.InitialFrame.Identity.SceneRevision = TEXT("wrong");
	FURLabRenderSceneCandidate Candidate;
	TestFalse(TEXT("invalid staged payload rejects"), Coordinator->StageScene(Registration,
		TEXT(""), MoveTemp(Invalid), FRenderSceneInstallOptions(), Candidate, Error));
	TestEqual(TEXT("failed staging releases arrays and lease"), FailedLive, 0);
	int32 DiscardedLive = 0;
	TestTrue(TEXT("discard fixture stages"), Coordinator->StageScene(Registration, TEXT(""),
		MakeLeasedOneBody(TEXT("discarded"), DiscardedLive), FRenderSceneInstallOptions(),
		Candidate, Error));
	TestEqual(TEXT("staged discard fixture retains lease"), DiscardedLive, 1);
	TestTrue(TEXT("candidate discard succeeds"),
		Coordinator->DiscardCandidate(Registration, Candidate, Error));
	TestEqual(TEXT("candidate discard releases lease"), DiscardedLive, 0);

	int32 FirstLive = 0;
	TestTrue(TEXT("first payload stages"), Coordinator->StageScene(Registration, TEXT(""),
		MakeLeasedOneBody(TEXT("first"), FirstLive), FRenderSceneInstallOptions(),
		Candidate, Error));
	TestEqual(TEXT("staged actor retains lease"), FirstLive, 1);
	TestTrue(TEXT("first payload commits"),
		Coordinator->CommitCandidate(Registration, Candidate).bCommitted);
	TestEqual(TEXT("active actor retains lease"), FirstLive, 1);

	int32 ReplacementLive = 0;
	FURLabRenderSceneCandidate Replacement;
	TestTrue(TEXT("replacement stages"), Coordinator->StageScene(Registration, TEXT("first"),
		MakeLeasedOneBody(TEXT("replacement"), ReplacementLive),
		FRenderSceneInstallOptions(), Replacement, Error));
	TestEqual(TEXT("replacement staged lease retained"), ReplacementLive, 1);
	TestTrue(TEXT("replacement commits"),
		Coordinator->CommitCandidate(Registration, Replacement).bCommitted);
	TestEqual(TEXT("replacement retires prior lease"), FirstLive, 0);
	TestEqual(TEXT("replacement owns its lease"), ReplacementLive, 1);

	TestTrue(TEXT("producer release succeeds"), Coordinator->ReleaseProducer(Registration, Error));
	TestEqual(TEXT("active retirement releases replacement lease"), ReplacementLive, 0);
	return !HasAnyErrors();
}

#endif
