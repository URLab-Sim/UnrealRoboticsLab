// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Render/RenderWorldSubsystem.h"

#include "Engine/World.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/URLabRenderScene.h"

namespace
{
bool RenderWorldGameThread(FString& OutError)
{
	if (IsInGameThread()) return true;
	OutError = TEXT("render world scene operations must run on the game thread");
	return false;
}
}

bool FURLabRenderProducerRegistration::IsValid() const
{
	return Coordinator.IsValid() && WorldLifetime.IsValid() && ProducerSerial > 0;
}

bool FURLabRenderSceneCandidate::IsValid() const
{
	return Coordinator.IsValid() && WorldLifetime.IsValid()
		&& ProducerSerial > 0 && CandidateSerial > 0;
}

bool FURLabRenderPresentationHold::IsValid() const
{
	return Coordinator.IsValid() && WorldLifetime.IsValid() && ProducerSerial > 0
		&& SceneGeneration > 0 && !SceneRevision.IsEmpty() && HoldSerial > 0;
}

UURLabRenderWorldSubsystem::FMutationScope::FMutationScope(
	UURLabRenderWorldSubsystem& InOwner, EMutationPhase Phase)
	: Owner(InOwner), PreviousPhase(InOwner.MutationPhase)
{
	Owner.MutationPhase = Phase;
}

UURLabRenderWorldSubsystem::FMutationScope::~FMutationScope()
{
	Owner.MutationPhase = PreviousPhase;
	if (PreviousPhase == EMutationPhase::None && Owner.bShutdownRequested
		&& !Owner.bDeinitialized)
	{
		Owner.ShutdownInternal();
	}
}

bool UURLabRenderWorldSubsystem::RegisterProducer(
	FURLabRenderProducerRegistration& OutRegistration, FString& OutError)
{
	OutRegistration = {};
	OutError.Reset();
	if (!RenderWorldGameThread(OutError)) return false;
	if (bDeinitialized || bShutdownRequested)
	{
		OutError = TEXT("render coordinator is shut down");
		return false;
	}
	if (MutationPhase != EMutationPhase::None)
	{
		OutError = TEXT("render coordinator registration cannot change during another scene mutation");
		return false;
	}
	FMutationScope MutationGuard(*this, EMutationPhase::Registering);
	UWorld* World = GetWorld();
	if (!World)
	{
		OutError = TEXT("render coordinator has no world");
		return false;
	}
	if (CurrentProducerSerial != 0)
	{
		OutError = TEXT("render coordinator already has a registered producer");
		return false;
	}
	if (NextProducerSerial == 0 || NextProducerSerial == MAX_uint64)
	{
		OutError = TEXT("render producer identity space is exhausted");
		return false;
	}
	if (!WorldLifetime.IsValid()) WorldLifetime = FGuid::NewGuid();
	CurrentProducerSerial = NextProducerSerial++;
	OutRegistration.Coordinator = this;
	OutRegistration.WorldLifetime = WorldLifetime;
	OutRegistration.ProducerSerial = CurrentProducerSerial;
	return true;
}

bool UURLabRenderWorldSubsystem::ValidateRegistration(
	const FURLabRenderProducerRegistration& Registration, FString& OutError) const
{
	OutError.Reset();
	if (!RenderWorldGameThread(OutError)) return false;
	if (bDeinitialized)
	{
		OutError = TEXT("render coordinator is shut down");
		return false;
	}
	if (Registration.Coordinator.Get() != this
		|| Registration.WorldLifetime != WorldLifetime
		|| Registration.ProducerSerial == 0
		|| Registration.ProducerSerial != CurrentProducerSerial)
	{
		OutError = TEXT("render producer registration is stale or belongs to another world");
		return false;
	}
	return true;
}

bool UURLabRenderWorldSubsystem::IsRegistrationCurrent(
	const FURLabRenderProducerRegistration& Registration) const
{
	FString Error;
	return ValidateRegistration(Registration, Error);
}

bool UURLabRenderWorldSubsystem::StageScene(
	const FURLabRenderProducerRegistration& Registration,
	const FString& ExpectedCurrentRevision, FRenderSceneInstallPayload&& Payload,
	const FRenderSceneInstallOptions& Options, FURLabRenderSceneCandidate& OutCandidate,
	FString& OutError)
{
	FRenderSceneInstallPayload OwnedPayload = MoveTemp(Payload);
	OutCandidate = {};
	if (!ValidateRegistration(Registration, OutError)) return false;
	if (MutationPhase != EMutationPhase::None)
	{
		OutError = TEXT("render scene cannot be staged during another scene mutation");
		return false;
	}
	const FString CurrentRevision = ActiveScene ? ActiveScene->GetScene().SceneRevision : FString();
	if (ExpectedCurrentRevision != CurrentRevision)
	{
		OutError = TEXT("render scene changed before candidate staging");
		return false;
	}
	const FString NewRevision = OwnedPayload.Scene.SceneRevision;
	if (NewRevision.IsEmpty() || NewRevision == CurrentRevision)
	{
		OutError = TEXT("staged render scene requires a fresh nonempty revision");
		return false;
	}
	if (ActiveSceneGeneration == MAX_uint64)
	{
		OutError = TEXT("render scene generation space is exhausted");
		return false;
	}
	if (NextCandidateSerial == 0 || NextCandidateSerial == MAX_uint64)
	{
		OutError = TEXT("render candidate identity space is exhausted");
		return false;
	}
	for (const TPair<uint64, FStagedScene>& Entry : Candidates)
	{
		if (Entry.Value.SceneRevision == NewRevision)
		{
			OutError = TEXT("render scene revision is already staged");
			return false;
		}
	}
	if (Candidates.Num() >= MaxStagedCandidates)
	{
		OutError = TEXT("render coordinator staged-candidate limit is reached");
		return false;
	}

	FMutationScope MutationGuard(*this, EMutationPhase::Staging);
	UWorld* World = GetWorld();
	if (!World)
	{
		OutError = TEXT("render coordinator has no world");
		return false;
	}
	FActorSpawnParameters Spawn;
	if (!Options.bPersistentEditorActors) Spawn.ObjectFlags |= RF_Transient;
	AURLabRenderScene* CandidateActor = World->SpawnActor<AURLabRenderScene>(
		AURLabRenderScene::StaticClass(), FTransform::Identity, Spawn);
	if (!CandidateActor)
	{
		OutError = TEXT("render coordinator could not spawn a staged scene actor");
		return false;
	}
	FRenderSceneInstallOptions HiddenOptions = Options;
	HiddenOptions.bInitiallyVisible = false;
	if (!CandidateActor->InstallScene(MoveTemp(OwnedPayload), HiddenOptions, OutError))
	{
		CandidateActor->Destroy();
		return false;
	}
	CandidateActor->SetActorHiddenInGame(true);

	const uint64 CandidateSerial = NextCandidateSerial++;
	FStagedScene Record;
	Record.Actor = CandidateActor;
	Record.ExpectedCurrentRevision = ExpectedCurrentRevision;
	Record.SceneRevision = NewRevision;
	Record.ProposedGeneration = ActiveSceneGeneration + 1;
	Record.bInitiallyVisible = Options.bInitiallyVisible;
	Candidates.Add(CandidateSerial, MoveTemp(Record));
	StagedSceneActors.Add(CandidateActor);
	OutCandidate.Coordinator = this;
	OutCandidate.WorldLifetime = WorldLifetime;
	OutCandidate.ProducerSerial = CurrentProducerSerial;
	OutCandidate.CandidateSerial = CandidateSerial;
	return true;
}

UURLabRenderWorldSubsystem::FStagedScene* UURLabRenderWorldSubsystem::FindCandidate(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderSceneCandidate& Candidate, FString& OutError)
{
	if (!ValidateRegistration(Registration, OutError)) return nullptr;
	if (Candidate.Coordinator.Get() != this
		|| Candidate.WorldLifetime != WorldLifetime
		|| Candidate.ProducerSerial != CurrentProducerSerial
		|| Candidate.ProducerSerial != Registration.ProducerSerial
		|| Candidate.CandidateSerial == 0)
	{
		OutError = TEXT("render scene candidate is stale or belongs to another producer");
		return nullptr;
	}
	FStagedScene* Record = Candidates.Find(Candidate.CandidateSerial);
	if (!Record || !Record->Actor.IsValid())
	{
		OutError = TEXT("render scene candidate is no longer staged");
		return nullptr;
	}
	return Record;
}

bool UURLabRenderWorldSubsystem::ApplyCandidateFrame(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderSceneCandidate& Candidate, const FRenderFrame& Frame,
	FString& OutError)
{
	FStagedScene* Record = FindCandidate(Registration, Candidate, OutError);
	if (!Record) return false;
	if (MutationPhase == EMutationPhase::RelatedCommit
		&& Candidate.CandidateSerial != TransactionCandidateSerial)
	{
		OutError = TEXT("only the committing candidate may be finalized during scene commit");
		return false;
	}
	if (MutationPhase != EMutationPhase::None
		&& MutationPhase != EMutationPhase::RelatedCommit)
	{
		OutError = TEXT("render candidate frame cannot change during another scene mutation");
		return false;
	}
	const FString CurrentRevision = ActiveScene ? ActiveScene->GetScene().SceneRevision : FString();
	if (CurrentRevision != Record->ExpectedCurrentRevision)
	{
		OutError = TEXT("render scene changed before candidate finalization");
		return false;
	}
	FMutationScope MutationGuard(*this, EMutationPhase::CandidateFrame);
	return Record->Actor->ApplyFrame(Frame, OutError);
}

FURLabRenderWorldCommitResult UURLabRenderWorldSubsystem::CommitCandidate(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderSceneCandidate& Candidate,
	TFunctionRef<bool(FString& OutError)> CommitRelatedState)
{
	FURLabRenderWorldCommitResult Result;
	if (MutationPhase != EMutationPhase::None)
	{
		Result.Error = TEXT("another render scene mutation is already in progress");
		return Result;
	}
	FString Error;
	FStagedScene* Record = FindCandidate(Registration, Candidate, Error);
	if (!Record)
	{
		Result.Error = MoveTemp(Error);
		return Result;
	}
	const FString CurrentRevision = ActiveScene ? ActiveScene->GetScene().SceneRevision : FString();
	if (CurrentRevision != Record->ExpectedCurrentRevision)
	{
		Result.Error = TEXT("render scene changed before candidate commit");
		return Result;
	}
	FMutationScope MutationGuard(*this, EMutationPhase::PrecommitCancellation);
	// Admission and publication for the scene being replaced stop before the
	// related native commit. Cancellation only marks shared terminal state here;
	// workflow notification is deferred to the camera scheduler tick.
	if (!CancelSceneCaptureJobs(true, Error))
	{
		Result.Error = MoveTemp(Error);
		return Result;
	}
	if (bShutdownRequested)
	{
		Result.Error = TEXT("render coordinator shutdown was requested during precommit cancellation");
		return Result;
	}
	MutationPhase = EMutationPhase::RelatedCommit;
	TGuardValue<uint64> CandidateGuard(TransactionCandidateSerial,
		Candidate.CandidateSerial);
	if (!CommitRelatedState(Error))
	{
		Result.Error = Error.IsEmpty()
			? TEXT("related state refused the render scene commit") : MoveTemp(Error);
		return Result;
	}
	// The related callback and any native lock it held are gone. Keep mutations
	// guarded while publishing and retiring actors, whose notifications may reenter.
	MutationPhase = EMutationPhase::CommitRetirement;
	AURLabRenderScene* CandidateActor = Record->Actor.Get();
	const FString Revision = Record->SceneRevision;
	const uint64 Generation = Record->ProposedGeneration;
	const bool bInitiallyVisible = Record->bInitiallyVisible;
	TArray<TObjectPtr<AURLabRenderScene>> RetiredCandidates = MoveTemp(StagedSceneActors);
	RetiredCandidates.RemoveSingleSwap(CandidateActor);
	Candidates.Reset();
	AURLabRenderScene* RetiredScene = ActiveScene;
	ActiveScene = CandidateActor;
	ActiveSceneGeneration = Generation;
	if (RetiredScene) RetiredScene->SetSceneVisible(false);
	ActiveScene->SetActorHiddenInGame(false);
	ActiveScene->SetSceneVisible(bInitiallyVisible);
	if (RetiredScene) RetiredScene->Destroy();
	for (AURLabRenderScene* Actor : RetiredCandidates)
		if (Actor) Actor->Destroy();

	Result.bCommitted = true;
	Result.SceneGeneration = Generation;
	Result.SceneRevision = Revision;
	return Result;
}

FURLabRenderWorldCommitResult UURLabRenderWorldSubsystem::CommitCandidate(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderSceneCandidate& Candidate)
{
	return CommitCandidate(Registration, Candidate, [](FString&) { return true; });
}

bool UURLabRenderWorldSubsystem::ValidateActiveFrame(
	const FString& ExpectedSceneRevision, const FRenderFrame& Frame,
	FString& OutError) const
{
	if (!ActiveScene || ActiveScene->GetScene().SceneRevision != ExpectedSceneRevision)
	{
		OutError = TEXT("active render scene revision does not match the frame target");
		return false;
	}
	const FRenderScene& Scene = ActiveScene->GetScene();
	return Frame.IsValid(Scene.Bodies.Num(), Scene.Cameras.Num(),
		Scene.UserCamera.IsSet(), Scene.Lights.Num(), ExpectedSceneRevision, OutError);
}

FURLabRenderFramePublicationResult UURLabRenderWorldSubsystem::PublishAuthoritativeFrame(
	const FURLabRenderProducerRegistration& Registration,
	const FString& ExpectedSceneRevision, const FRenderFrame& Frame)
{
	FURLabRenderFramePublicationResult Result;
	if (!ValidateRegistration(Registration, Result.Error)) return Result;
	if (MutationPhase != EMutationPhase::None)
	{
		Result.Error = TEXT("active render frame cannot change during another scene mutation");
		return Result;
	}
	if (!ValidateActiveFrame(ExpectedSceneRevision, Frame, Result.Error)) return Result;
	if (PresentationHold.IsSet())
	{
		if (PresentationHold->ProducerSerial != CurrentProducerSerial
			|| PresentationHold->SceneGeneration != ActiveSceneGeneration
			|| PresentationHold->SceneRevision != ExpectedSceneRevision)
		{
			Result.Error = TEXT("active presentation hold belongs to stale scene state");
			return Result;
		}
		PresentationHold->CoalescedAuthoritativeFrame = Frame;
		Result.bAccepted = true;
		Result.Disposition = EURLabRenderFramePublication::Coalesced;
		return Result;
	}
	FMutationScope MutationGuard(*this, EMutationPhase::ActiveFrame);
	Result.bAccepted = ActiveScene->ApplyFrame(Frame, Result.Error);
	Result.Disposition = EURLabRenderFramePublication::Applied;
	return Result;
}

bool UURLabRenderWorldSubsystem::AcquirePresentationHold(
	const FURLabRenderProducerRegistration& Registration,
	const FString& ExpectedSceneRevision, const FRenderFrame& JobFrame,
	FURLabRenderPresentationHold& OutHold, FString& OutError)
{
	OutHold = {};
	if (!ValidateRegistration(Registration, OutError)) return false;
	if (MutationPhase != EMutationPhase::None)
	{
		OutError = TEXT("presentation cannot be held during another scene mutation");
		return false;
	}
	if (PresentationHold.IsSet())
	{
		OutError = TEXT("render scene already has an active presentation hold");
		return false;
	}
	if (NextHoldSerial == 0 || NextHoldSerial == MAX_uint64)
	{
		OutError = TEXT("render presentation hold identity space is exhausted");
		return false;
	}
	if (!ValidateActiveFrame(ExpectedSceneRevision, JobFrame, OutError)) return false;

	FMutationScope MutationGuard(*this, EMutationPhase::PresentationHold);
	FActivePresentationHold NewHold;
	NewHold.HoldSerial = NextHoldSerial;
	NewHold.ProducerSerial = CurrentProducerSerial;
	NewHold.SceneGeneration = ActiveSceneGeneration;
	NewHold.SceneRevision = ExpectedSceneRevision;
	NewHold.SavedAuthoritativeFrame = ActiveScene->GetAppliedFrame();
	if (!ActiveScene->ApplyFrame(JobFrame, OutError)) return false;
	++NextHoldSerial;
	PresentationHold = MoveTemp(NewHold);

	OutHold.Coordinator = this;
	OutHold.WorldLifetime = WorldLifetime;
	OutHold.ProducerSerial = CurrentProducerSerial;
	OutHold.SceneGeneration = ActiveSceneGeneration;
	OutHold.SceneRevision = ExpectedSceneRevision;
	OutHold.HoldSerial = PresentationHold->HoldSerial;
	return true;
}

bool UURLabRenderWorldSubsystem::ValidatePresentationHold(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderPresentationHold& Hold, FString& OutError) const
{
	if (!ValidateRegistration(Registration, OutError)) return false;
	if (!PresentationHold.IsSet() || Hold.Coordinator.Get() != this
		|| Hold.WorldLifetime != WorldLifetime
		|| Hold.ProducerSerial != CurrentProducerSerial
		|| Hold.SceneGeneration != ActiveSceneGeneration
		|| Hold.SceneRevision != ResolveActiveSceneRevision(Registration)
		|| Hold.HoldSerial == 0 || Hold.HoldSerial != PresentationHold->HoldSerial)
	{
		OutError = TEXT("render presentation hold is stale or belongs to another scene");
		return false;
	}
	return true;
}

bool UURLabRenderWorldSubsystem::ReleasePresentationHold(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderPresentationHold& Hold, FString& OutError)
{
	if (MutationPhase != EMutationPhase::None
		&& MutationPhase != EMutationPhase::PrecommitCancellation)
	{
		OutError = TEXT("presentation hold cannot be released during another scene mutation");
		return false;
	}
	if (!ValidatePresentationHold(Registration, Hold, OutError)) return false;
	FMutationScope MutationGuard(*this, EMutationPhase::PresentationHold);
	const FRenderFrame RestoreFrame = PresentationHold->CoalescedAuthoritativeFrame.IsSet()
		? PresentationHold->CoalescedAuthoritativeFrame.GetValue()
		: PresentationHold->SavedAuthoritativeFrame;
	if (!ActiveScene->ApplyFrame(RestoreFrame, OutError)) return false;
	PresentationHold.Reset();
	return true;
}

bool UURLabRenderWorldSubsystem::ApplyFrame(
	const FURLabRenderProducerRegistration& Registration,
	const FString& ExpectedSceneRevision, const FRenderFrame& Frame, FString& OutError)
{
	FURLabRenderFramePublicationResult Result = PublishAuthoritativeFrame(
		Registration, ExpectedSceneRevision, Frame);
	OutError = MoveTemp(Result.Error);
	return Result.bAccepted;
}

bool UURLabRenderWorldSubsystem::DiscardCandidate(
	const FURLabRenderProducerRegistration& Registration,
	const FURLabRenderSceneCandidate& Candidate, FString& OutError)
{
	if (MutationPhase != EMutationPhase::None)
	{
		OutError = TEXT("render candidate cannot be discarded during another scene mutation");
		return false;
	}
	if (!FindCandidate(Registration, Candidate, OutError)) return false;
	FMutationScope MutationGuard(*this, EMutationPhase::Discarding);
	DestroyCandidate(Candidate.CandidateSerial);
	OutError.Reset();
	return true;
}

bool UURLabRenderWorldSubsystem::ReleaseProducer(
	const FURLabRenderProducerRegistration& Registration, FString& OutError)
{
	if (!ValidateRegistration(Registration, OutError)) return false;
	if (MutationPhase != EMutationPhase::None)
	{
		OutError = TEXT("render producer cannot be released during another scene mutation");
		return false;
	}
	CancelSceneCaptureJobs(false, OutError);
	FMutationScope MutationGuard(*this, EMutationPhase::Releasing);
	TArray<TObjectPtr<AURLabRenderScene>> RetiredCandidates = MoveTemp(StagedSceneActors);
	Candidates.Reset();
	AURLabRenderScene* RetiredScene = ActiveScene;
	ActiveScene = nullptr;
	PresentationHold.Reset();
	ActiveSceneGeneration = 0;
	CurrentProducerSerial = 0;
	TransactionCandidateSerial = 0;
	for (AURLabRenderScene* Actor : RetiredCandidates)
		if (Actor) Actor->Destroy();
	if (RetiredScene) RetiredScene->Destroy();
	OutError.Reset();
	return true;
}

bool UURLabRenderWorldSubsystem::CancelSceneCaptureJobs(
	bool bRestorePresentation, FString& OutError)
{
	OutError.Reset();
	if (!ActiveScene || CurrentProducerSerial == 0) return true;
	if (UWorld* World = GetWorld())
	{
		if (URenderCameraSubsystem* Scheduler = World->GetSubsystem<URenderCameraSubsystem>())
		{
			Scheduler->CancelJobsForSceneMutation(this, CurrentProducerSerial,
				ActiveSceneGeneration, ActiveScene->GetScene().SceneRevision,
				bRestorePresentation);
		}
	}
	// A hold can be acquired directly by an internal workflow before it has a
	// scheduler record. Restore or retire that orphan here as part of the same
	// precommit cancellation boundary.
	if (PresentationHold.IsSet())
	{
		if (bRestorePresentation)
		{
			const FRenderFrame RestoreFrame =
				PresentationHold->CoalescedAuthoritativeFrame.IsSet()
				? PresentationHold->CoalescedAuthoritativeFrame.GetValue()
				: PresentationHold->SavedAuthoritativeFrame;
			if (!ActiveScene->ApplyFrame(RestoreFrame, OutError)) return false;
		}
		CancelPresentationHoldForRetirement();
	}
	return true;
}

void UURLabRenderWorldSubsystem::CancelPresentationHoldForRetirement()
{
	PresentationHold.Reset();
}

AURLabRenderScene* UURLabRenderWorldSubsystem::ResolveActiveScene(
	const FURLabRenderProducerRegistration& Registration) const
{
	FString Error;
	return ValidateRegistration(Registration, Error) ? ActiveScene.Get() : nullptr;
}

FString UURLabRenderWorldSubsystem::ResolveActiveSceneRevision(
	const FURLabRenderProducerRegistration& Registration) const
{
	AURLabRenderScene* Scene = ResolveActiveScene(Registration);
	return Scene ? Scene->GetScene().SceneRevision : FString();
}

uint64 UURLabRenderWorldSubsystem::ResolveActiveSceneGeneration(
	const FURLabRenderProducerRegistration& Registration) const
{
	FString Error;
	return ValidateRegistration(Registration, Error) ? ActiveSceneGeneration : 0;
}

void UURLabRenderWorldSubsystem::DestroyCandidate(uint64 CandidateSerial)
{
	FStagedScene Record;
	if (!Candidates.RemoveAndCopyValue(CandidateSerial, Record)) return;
	if (AURLabRenderScene* Actor = Record.Actor.Get())
	{
		StagedSceneActors.RemoveSingleSwap(Actor);
		// Remove lookup/ownership state before actor destruction broadcasts.
		Actor->Destroy();
	}
}

void UURLabRenderWorldSubsystem::DestroyAllCandidates()
{
	TArray<TObjectPtr<AURLabRenderScene>> RetiredCandidates = MoveTemp(StagedSceneActors);
	Candidates.Reset();
	for (AURLabRenderScene* Actor : RetiredCandidates)
		if (Actor) Actor->Destroy();
}

void UURLabRenderWorldSubsystem::ShutdownInternal()
{
	if (bDeinitialized) return;
	FString CancellationError;
	CancelSceneCaptureJobs(false, CancellationError);
	bShutdownRequested = false;
	bDeinitialized = true;
	MutationPhase = EMutationPhase::Shutdown;
	TransactionCandidateSerial = 0;
	TArray<TObjectPtr<AURLabRenderScene>> RetiredCandidates = MoveTemp(StagedSceneActors);
	Candidates.Reset();
	AURLabRenderScene* RetiredScene = ActiveScene;
	ActiveScene = nullptr;
	PresentationHold.Reset();
	ActiveSceneGeneration = 0;
	CurrentProducerSerial = 0;
	WorldLifetime = FGuid::NewGuid();
	for (AURLabRenderScene* Actor : RetiredCandidates)
		if (Actor) Actor->Destroy();
	if (RetiredScene) RetiredScene->Destroy();
}

bool UURLabRenderWorldSubsystem::DoesSupportWorldType(EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE
		|| WorldType == EWorldType::Editor;
}

void UURLabRenderWorldSubsystem::Deinitialize()
{
	if (!bDeinitialized)
	{
		bShutdownRequested = true;
		if (IsInGameThread() && MutationPhase == EMutationPhase::None)
		{
			ShutdownInternal();
		}
		else if (!IsInGameThread())
		{
			bDeinitialized = true;
			MutationPhase = EMutationPhase::Shutdown;
		}
	}
	if (!bSuperDeinitialized)
	{
		bSuperDeinitialized = true;
		Super::Deinitialize();
	}
}
