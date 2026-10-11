#include "Workflows/RenderServerWorldSubsystem.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RenderServiceBinding.h"
#include "Bridge/RpcDispatcher.h"

#include "Engine/World.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Render/MjRenderKinematics.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/RenderCameraSubsystem.h"
#include "Render/CameraStreamController.h"
#include "Render/URLabRenderScene.h"

struct FURLabRenderWorldServiceBinding
{
	TWeakObjectPtr<UURLabBridgeServer> Bridge;
	FURLabRpcDispatcher::FRenderServiceRegistration Registration;
};
#include "Utils/URLabLogging.h"

struct UURLabRenderServerWorldSubsystem::FCaptureJobRecord
{
	FURLabRenderOwnerProof Owner;
	FString SceneRevision;
	FRenderFrameIdentity SubmittedFrame;
	FURLabRenderCaptureJob SchedulerJob;
	FRenderCameraCaptureLifetimeLeasePtr ResultLease;
};

bool FURLabRenderServerCaptureHandle::IsValid() const
{
	return Workflow.IsValid() && WorkflowLifetime.IsValid() && JobSerial != 0
		&& !SubmittedFrame.SceneRevision.IsEmpty();
}

namespace
{
FURLabRenderServerInstallResult InstallRejected(FString Error)
{
	FURLabRenderServerInstallResult Result;
	Result.Error = MoveTemp(Error);
	return Result;
}

FURLabRenderFramePublicationResult PublishRejected(FString Error)
{
	FURLabRenderFramePublicationResult Result;
	Result.Error = MoveTemp(Error);
	return Result;
}

template <typename TResult>
TResult WorkflowUnavailable(const TCHAR* Message,
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::Unavailable)
{
	TResult Result;
	Result.Error = Error;
	Result.Status.bAvailable = false;
	Result.Message = Message;
	return Result;
}
}

void UURLabRenderServerWorldSubsystem::FKinematicsDeleter::operator()(
	FMjRenderKinematics* Value) const { delete Value; }
void UURLabRenderServerWorldSubsystem::FCaptureJobRecordDeleter::operator()(
	FCaptureJobRecord* Value) const { delete Value; }

UURLabRenderServerWorldSubsystem::UURLabRenderServerWorldSubsystem()
	: WorkflowLifetime(FGuid::NewGuid())
{
}
UURLabRenderServerWorldSubsystem::~UURLabRenderServerWorldSubsystem() = default;

bool UURLabRenderServerWorldSubsystem::DoesSupportWorldType(EWorldType::Type Type) const
{
	return Type == EWorldType::Game || Type == EWorldType::PIE || Type == EWorldType::Editor;
}

bool UURLabRenderServerWorldSubsystem::Activate(FString& OutError)
{
	OutError.Reset();
	check(IsInGameThread());
	if (bShutdownPermanent)
	{ OutError = TEXT("render-server workflow has shut down for this world lifetime"); return false; }
	if (bMutationInProgress || bShutdownRequested || bShutdownInProgress)
	{ OutError = TEXT("render-server workflow cannot activate during install or shutdown"); return false; }
	if (bActive)
	{
		if (UURLabRenderWorldSubsystem* Current = Coordinator.Get();
			Current && Current->IsRegistrationCurrent(Producer)) return true;
		OutError = TEXT("render-server producer registration is no longer current");
		return false;
	}
	UWorld* World = GetWorld();
	if (!World || !DoesSupportWorldType(World->WorldType))
	{ OutError = TEXT("render-server workflow requires a supported world"); return false; }
	UURLabRenderWorldSubsystem* Candidate = World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!Candidate) { OutError = TEXT("world has no neutral render coordinator"); return false; }
	FURLabRenderProducerRegistration Registration;
	if (!Candidate->RegisterProducer(Registration, OutError)) return false;
	Coordinator = Candidate;
	Producer = Registration;
	bActive = true;
	return true;
}

bool UURLabRenderServerWorldSubsystem::IsActive() const
{
	const UURLabRenderWorldSubsystem* Current = Coordinator.Get();
	return bActive && Current && Current->IsRegistrationCurrent(Producer);
}

bool UURLabRenderServerWorldSubsystem::AttachBridge(FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (!IsActive() || bMutationInProgress || bShutdownRequested || bShutdownInProgress)
	{
		OutError = TEXT("render workflow must be active and idle before bridge attachment");
		return false;
	}
	if (ServiceBinding.IsValid())
	{
		UURLabBridgeServer* Existing = ServiceBinding->Bridge.Get();
		FURLabRpcDispatcher* Dispatcher = Existing ? Existing->GetDispatcher() : nullptr;
		if (Existing && Existing->IsRunning() && Dispatcher
			&& Dispatcher->IsRenderServiceRegistrationActive(ServiceBinding->Registration))
			return true;
		ServiceBinding.Reset();
	}
	UWorld* World = GetWorld();
	UURLabBridgeServer* Bridge = World ? UURLabBridgeSubsystem::EnsureFor(World) : nullptr;
	FURLabRpcDispatcher* Dispatcher = Bridge ? Bridge->GetDispatcher() : nullptr;
	if (!Bridge || !Dispatcher)
	{
		OutError = TEXT("render world bridge did not provide a running dispatcher");
		return false;
	}
	TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Opaque =
		FURLabRenderServiceBinding::Create(*this, OutError);
	if (!Opaque.IsValid()) return false;
	auto Candidate = MakeShared<FURLabRenderWorldServiceBinding, ESPMode::ThreadSafe>();
	Candidate->Bridge = Bridge;
	if (!Dispatcher->AttachRenderService(Opaque.ToSharedRef(), Candidate->Registration, OutError))
		return false;
	ServiceBinding = MoveTemp(Candidate);
	return true;
}

void UURLabRenderServerWorldSubsystem::DetachBridge()
{
	check(IsInGameThread());
	if (!ServiceBinding.IsValid()) return;
	if (UURLabBridgeServer* Bridge = ServiceBinding->Bridge.Get())
		if (FURLabRpcDispatcher* Dispatcher = Bridge->GetDispatcher())
			Dispatcher->DetachRenderService(ServiceBinding->Registration);
	ServiceBinding.Reset();
}

bool UURLabRenderServerWorldSubsystem::IsBridgeAttached() const
{
	if (!ServiceBinding.IsValid() || !IsActive()) return false;
	UURLabBridgeServer* Bridge = ServiceBinding->Bridge.Get();
	FURLabRpcDispatcher* Dispatcher = Bridge ? Bridge->GetDispatcher() : nullptr;
	return Bridge && Bridge->IsRunning() && Dispatcher
		&& Dispatcher->IsRenderServiceRegistrationActive(ServiceBinding->Registration);
}

FURLabRenderOwnerStatusResult UURLabRenderServerWorldSubsystem::OwnerStatus()
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerStatusResult>(
		TEXT("render-server workflow is not active"), bShutdownPermanent
			? EURLabRenderOwnerAuthorityError::ShuttingDown
			: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return Authority.Status();
}

FURLabRenderOwnerAttachResult UURLabRenderServerWorldSubsystem::AttachOwner(
	const FURLabRenderOwnerSessionScopePtr& Scope, double TtlSeconds)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerAttachResult>(
		TEXT("render-server workflow is not active"), bShutdownPermanent
			? EURLabRenderOwnerAuthorityError::ShuttingDown
			: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FURLabRenderOwnerAttachResult Result = Authority.Attach(Scope, TtlSeconds);
	if (Result.IsOk()) OwnerScope = Scope;
	return Result;
}

FURLabRenderOwnerAuthorizationResult UURLabRenderServerWorldSubsystem::AuthorizeOwner(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerAuthorizationResult>(
		TEXT("render-server workflow is not active"), bShutdownPermanent
			? EURLabRenderOwnerAuthorityError::ShuttingDown
			: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return Authority.Authorize(Scope, Generation, Token);
}

FURLabRenderOwnerLeaseResult UURLabRenderServerWorldSubsystem::RenewOwner(
	const FURLabRenderOwnerProof& Proof, double TtlSeconds)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerLeaseResult>(
		TEXT("render-server workflow is not active"), bShutdownPermanent
			? EURLabRenderOwnerAuthorityError::ShuttingDown
			: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return Authority.Renew(Proof, TtlSeconds);
}

bool UURLabRenderServerWorldSubsystem::IsBoundOwner(const FURLabRenderOwnerProof& Proof) const
{
	return Source.IsSet()
		&& Source->Owner.GetGeneration() == Proof.GetGeneration()
		&& Source->Owner.GetOpaqueToken() == Proof.GetOpaqueToken();
}

void UURLabRenderServerWorldSubsystem::RevokeSource() { Source.Reset(); }

FURLabRenderOwnerLeaseResult UURLabRenderServerWorldSubsystem::CompleteRetirement(
	FURLabRenderOwnerRetirementResult&& Begun)
{
	if (!Begun.IsOk())
	{
		FURLabRenderOwnerLeaseResult Result;
		Result.Error = Begun.Error;
		Result.Status = Begun.Status;
		Result.Message = MoveTemp(Begun.Message);
		return Result;
	}
	const bool bOwnMutation = !bMutationInProgress;
	if (bOwnMutation) bMutationInProgress = true;
	ON_SCOPE_EXIT { if (bOwnMutation) FinishMutation(); };
	// One authority slot exists, and predecessor records are removed before a
	// successor can attach, so every retained record belongs to this retirement.
	RetireCaptureJobs(nullptr);
	RevokeSource();
	OwnerScope.Reset();
	return Authority.FinishRetirement(Begun.Token);
}

void UURLabRenderServerWorldSubsystem::RetireCaptureJobs(
	const FURLabRenderOwnerProof* ExactOwner)
{
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	TArray<uint64> RetiredSerials;
	for (const TPair<uint64, FCaptureJobRecordPtr>& Entry : CaptureJobs)
	{
		const FCaptureJobRecord& Record = *Entry.Value;
		if (ExactOwner && (Record.Owner.GetGeneration() != ExactOwner->GetGeneration()
			|| Record.Owner.GetOpaqueToken() != ExactOwner->GetOpaqueToken())) continue;
		if (Scheduler && !Record.SchedulerJob.PollStatus().IsTerminal())
		{
			FString Ignored;
			Scheduler->CancelExactCapture(Record.SchedulerJob, Ignored);
		}
		RetiredSerials.Add(Entry.Key);
	}
	for (uint64 Serial : RetiredSerials)
	{
		FCaptureJobRecordPtr Retired;
		CaptureJobs.RemoveAndCopyValue(Serial, Retired);
		if (Retired) PendingRetiredCaptureJobs.Add(MoveTemp(Retired));
	}
}

FURLabRenderOwnerLeaseResult UURLabRenderServerWorldSubsystem::ReleaseOwner(
	const FURLabRenderOwnerProof& Proof)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerLeaseResult>(
		TEXT("render-server workflow is not active"), bShutdownPermanent
			? EURLabRenderOwnerAuthorityError::ShuttingDown
			: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return CompleteRetirement(Authority.BeginRelease(Proof));
}

FURLabRenderOwnerLeaseResult UURLabRenderServerWorldSubsystem::ReleaseOwnerCredentials(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation,
	const FString& Token)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress)
		return WorkflowUnavailable<FURLabRenderOwnerLeaseResult>(
			TEXT("render-server workflow is not active"), bShutdownPermanent
				? EURLabRenderOwnerAuthorityError::ShuttingDown
				: EURLabRenderOwnerAuthorityError::Unavailable);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	const FURLabRenderOwnerAuthorizationResult Authorized =
		Authority.AuthorizeForRelease(Scope, Generation, Token);
	if (!Authorized.IsOk())
	{
		FURLabRenderOwnerLeaseResult Result;
		Result.Error = Authorized.Error;
		Result.Status = Authorized.Status;
		Result.Message = Authorized.Message;
		return Result;
	}
	return CompleteRetirement(Authority.BeginRelease(Authorized.Proof));
}

FURLabRenderOwnerLeaseResult UURLabRenderServerWorldSubsystem::RetireOwnerScope(
	const FURLabRenderOwnerSessionScopePtr& Scope)
{
	check(IsInGameThread());
	if (bMutationInProgress) return WorkflowUnavailable<FURLabRenderOwnerLeaseResult>(
		TEXT("render-server workflow is mutating"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return CompleteRetirement(Authority.BeginLifecycleRetirement(Scope));
}

bool UURLabRenderServerWorldSubsystem::ValidateMutation(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision, FString& OutError)
{
	if (!IsInGameThread()) { OutError = TEXT("render-server mutation requires the game thread"); return false; }
	if (!IsActive() || bShutdownRequested || bShutdownInProgress)
	{ OutError = TEXT("render-server workflow is not active"); return false; }
	const FURLabRenderOwnerAdmissionResult Admission = Authority.Validate(Proof);
	if (!Admission.IsOk()) { OutError = Admission.Message; return false; }
	if (Coordinator->ResolveActiveSceneRevision(Producer) != ExpectedRevision)
	{ OutError = TEXT("expected render scene revision is not active"); return false; }
	return true;
}

FURLabRenderServerInstallResult UURLabRenderServerWorldSubsystem::InstallNeutralScene(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options)
{
	check(IsInGameThread());
	FRenderSceneInstallPayload OwnedPayload = MoveTemp(Payload);
	if (bMutationInProgress)
		return InstallRejected(TEXT("a render-server mutation is already in progress"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return InstallPrepared(Proof, ExpectedRevision, MoveTemp(OwnedPayload), Options, FKinematicsPtr());
}

FURLabRenderServerInstallResult UURLabRenderServerWorldSubsystem::InstallKinematicsCandidate(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FMjRenderKinematicsCandidate&& Candidate, const FRenderSceneInstallOptions& Options)
{
	check(IsInGameThread());
	if (bMutationInProgress)
		return InstallRejected(TEXT("a render-server mutation is already in progress"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame InitialFrame;
	if (!Candidate.TakeNeutralScene(Scene, Resources, InitialFrame, Error))
		return InstallRejected(MoveTemp(Error));
	FKinematicsPtr Prepared(new FMjRenderKinematics());
	if (!Prepared->Commit(MoveTemp(Candidate), FString(), Error))
		return InstallRejected(MoveTemp(Error));
	FRenderSceneInstallPayload Payload(MoveTemp(Scene), MoveTemp(Resources),
		MoveTemp(InitialFrame));
	return InstallPrepared(Proof, ExpectedRevision, MoveTemp(Payload), Options, MoveTemp(Prepared));
}

FURLabRenderServerInstallResult UURLabRenderServerWorldSubsystem::InstallPrepared(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options,
	FKinematicsPtr&& Kinematics)
{
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	FURLabRenderSceneCandidate Staged;
	const FString NewRevision = Payload.InitialFrame.Identity.SceneRevision;
	if (!Coordinator->StageScene(Producer, ExpectedRevision, MoveTemp(Payload),
		Options, Staged, Error))
		return InstallRejected(MoveTemp(Error));
	TOptional<FSourceBinding> RetiredSource;
	FURLabRenderWorldCommitResult Committed = Coordinator->CommitCandidate(
		Producer, Staged, [&](FString& RelatedError)
		{
			if (!ValidateMutation(Proof, ExpectedRevision, RelatedError)) return false;
			FSourceBinding Replacement;
			Replacement.Owner = Proof;
			Replacement.SceneRevision = NewRevision;
			Replacement.Kinematics = MoveTemp(Kinematics);
			if (Source.IsSet()) RetiredSource.Emplace(MoveTemp(Source.GetValue()));
			Source.Emplace(MoveTemp(Replacement));
			return true;
		});
	if (!Committed.bCommitted)
	{
		FString Ignored;
		Coordinator->DiscardCandidate(Producer, Staged, Ignored);
		return InstallRejected(MoveTemp(Committed.Error));
	}
	FURLabRenderServerInstallResult Result;
	Result.bCommitted = true;
	Result.SceneGeneration = Committed.SceneGeneration;
	Result.SceneRevision = MoveTemp(Committed.SceneRevision);
	return Result;
}

FURLabRenderServerInstallResult UURLabRenderServerWorldSubsystem::PatchScene(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const FURLabRenderScenePatch& Patch)
{
	check(IsInGameThread());
	if (bMutationInProgress) return InstallRejected(TEXT("a render-server mutation is already in progress"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	AURLabRenderScene* Active = Coordinator->ResolveActiveScene(Producer);
	if (!Source.IsSet() || Source->SceneRevision != ExpectedRevision || !Active)
		return InstallRejected(TEXT("owner has no matching installed render source"));

	FRenderSceneInstallPayload Payload;
	FRenderSceneInstallOptions Options;
	if (!Active->CopyInstalledPayload(Payload, Options, Error))
		return InstallRejected(MoveTemp(Error));
	const FString NewRevision = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
	if (!Patch.Apply(Payload.Scene, Payload.Resources, Options.VisibleGroupMask, Error))
		return InstallRejected(MoveTemp(Error));
	Payload.Scene.SceneRevision = NewRevision;
	Payload.Resources.SceneRevision = NewRevision;
	Payload.InitialFrame.Identity.SceneRevision = NewRevision;
	if (!Payload.InitialFrame.IsValid(Payload.Scene, Error))
		return InstallRejected(FString::Printf(TEXT("patched initial frame is invalid: %s"), *Error));
	const FRenderScene PatchedScene = Payload.Scene;

	FURLabRenderSceneCandidate Staged;
	if (!Coordinator->StageScene(Producer, ExpectedRevision, MoveTemp(Payload),
		Options, Staged, Error)) return InstallRejected(MoveTemp(Error));
	TOptional<FSourceBinding> RetiredSource;
	FURLabRenderWorldCommitResult Committed = Coordinator->CommitCandidate(
		Producer, Staged, [&](FString& RelatedError)
		{
			if (!ValidateMutation(Proof, ExpectedRevision, RelatedError)) return false;
			if (Source->Kinematics.IsValid()
				&& !Source->Kinematics->RebaseSceneRevision(ExpectedRevision, PatchedScene, RelatedError))
				return false;
			FSourceBinding Replacement;
			Replacement.Owner = Proof;
			Replacement.SceneRevision = NewRevision;
			Replacement.Kinematics = MoveTemp(Source->Kinematics);
			RetiredSource.Emplace(MoveTemp(Source.GetValue()));
			Source.Emplace(MoveTemp(Replacement));
			return true;
		});
	if (!Committed.bCommitted)
	{
		FString Ignored;
		Coordinator->DiscardCandidate(Producer, Staged, Ignored);
		return InstallRejected(MoveTemp(Committed.Error));
	}
	FURLabRenderServerInstallResult Result;
	Result.bCommitted = true;
	Result.SceneGeneration = Committed.SceneGeneration;
	Result.SceneRevision = MoveTemp(Committed.SceneRevision);
	return Result;
}

bool UURLabRenderServerWorldSubsystem::DescribeSource(
	const FURLabRenderOwnerProof& Proof, FURLabRenderSourceDescription& OutDescription,
	FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (bMutationInProgress)
	{
		OutError = TEXT("a render-server mutation is already in progress");
		return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!IsActive() || bShutdownRequested || bShutdownInProgress)
	{
		OutError = TEXT("render-server workflow is not active");
		return false;
	}
	const FURLabRenderOwnerAdmissionResult Admission = Authority.Validate(Proof);
	if (!Admission.IsOk()) { OutError = Admission.Message; return false; }
	if (!IsBoundOwner(Proof))
	{
		OutError = TEXT("owner has no installed render source");
		return false;
	}
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Producer);
	if (!Actor || Actor->GetScene().SceneRevision != Source->SceneRevision)
	{
		OutError = TEXT("installed render source is not the active scene");
		return false;
	}
	URenderCameraRig* Rig = Actor->GetCameraRig();
	if (!Rig) { OutError = TEXT("active render scene has no camera rig"); return false; }

	FURLabRenderSourceDescription Description;
	Description.SceneRevision = Source->SceneRevision;
	Description.SceneGeneration = Coordinator->ResolveActiveSceneGeneration(Producer);
	if (Description.SceneGeneration == 0)
	{
		OutError = TEXT("active render scene generation is invalid");
		return false;
	}
	for (const FRenderBodyDesc& Body : Actor->GetScene().Bodies)
		Description.BodyNames.Add(Body.Name);
	Description.GeomCount = Actor->GetScene().Geoms.Num();
	Description.Deformables = Actor->GetScene().Deformables;
	Description.LightNames.Reserve(Actor->GetScene().Lights.Num());
	for (const FRenderLightDesc& Light : Actor->GetScene().Lights)
		Description.LightNames.Add(Light.Name);
	Description.ModelCameras.Reserve(Rig->NumCameras());
	for (int32 Index = 0; Index < Rig->NumCameras(); ++Index)
	{
		URenderCameraComponent* Camera = Rig->GetCamera(Index);
		FRenderCameraEffectiveConfiguration Snapshot;
		if (!Camera || !Camera->ResolveCaptureConfiguration({}, Snapshot, OutError)) return false;
		Description.ModelCameras.Add(MoveTemp(Snapshot));
	}
	if (URenderCameraComponent* Camera = Rig->GetUserCamera())
	{
		FRenderCameraEffectiveConfiguration Snapshot;
		if (!Camera->ResolveCaptureConfiguration({}, Snapshot, OutError)) return false;
		Description.UserCamera = MoveTemp(Snapshot);
	}
	Description.bSupportsQpos = Source->Kinematics.IsValid();
	Description.QposWidth = Description.bSupportsQpos
		? Source->Kinematics->GetQposWidth() : 0;
	OutDescription = MoveTemp(Description);
	return true;
}

bool UURLabRenderServerWorldSubsystem::ConfigureCameraStream(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FURLabCameraStreamController& Controller, const FString& CameraName,
	bool bEnabled, TOptional<double> RateHz,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (bMutationInProgress)
	{
		OutError = TEXT("a render-server mutation is already in progress"); return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision)
	{
		if (OutError.IsEmpty()) OutError = TEXT("owner has no installed render source");
		return false;
	}
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Producer);
	if (!Scene || Scene->GetScene().SceneRevision != ExpectedRevision)
	{
		OutError = TEXT("installed render source is not the active scene"); return false;
	}
	return Controller.Configure(*Scene, ExpectedRevision, CameraName, bEnabled,
		RateHz, OutDescriptor, OutError);
}

FURLabRenderFramePublicationResult UURLabRenderServerWorldSubsystem::PublishCartesian(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const FRenderFrame& Frame)
{
	check(IsInGameThread());
	FString Error;
	if (bMutationInProgress) return PublishRejected(TEXT("a render-server mutation is already in progress"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, Error) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision)
		return PublishRejected(Error.IsEmpty() ? TEXT("owner has no installed source") : MoveTemp(Error));
	return Coordinator->PublishAuthoritativeFrame(Producer, ExpectedRevision, Frame);
}

FURLabRenderFramePublicationResult UURLabRenderServerWorldSubsystem::PublishQpos(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	TConstArrayView<double> Qpos, uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch)
{
	check(IsInGameThread());
	FString Error;
	if (bMutationInProgress) return PublishRejected(TEXT("a render-server mutation is already in progress"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, Error) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision || !Source->Kinematics)
		return PublishRejected(Error.IsEmpty() ? TEXT("owner has no installed qpos source") : MoveTemp(Error));
	FRenderFrame Frame;
	if (!Source->Kinematics->FrameFromQpos(ExpectedRevision, Qpos, FrameId,
		SourceTime, bHasSourceTime, ClockEpoch, Frame, Error))
		return PublishRejected(MoveTemp(Error));
	if (!ValidateMutation(Proof, ExpectedRevision, Error) || !IsBoundOwner(Proof))
		return PublishRejected(Error.IsEmpty() ? TEXT("source authority changed during conversion") : MoveTemp(Error));
	return Coordinator->PublishAuthoritativeFrame(Producer, ExpectedRevision, Frame);
}

FURLabRenderServerCaptureAdmission
UURLabRenderServerWorldSubsystem::SubmitCartesianCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderFrame&& Frame, FURLabRenderExactCapturePlan Plan,
	FRenderCameraCaptureLifetimeLeasePtr PixelLease,
	FRenderCameraCaptureLifetimeLeasePtr ResultLease, double TimeoutSeconds)
{
	check(IsInGameThread());
	FURLabRenderServerCaptureAdmission Rejected;
	if (bMutationInProgress)
	{
		Rejected.Error = TEXT("a render-server mutation is already in progress");
		return Rejected;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	return SubmitPreparedCapture(Proof, ExpectedRevision, MoveTemp(Frame), MoveTemp(Plan),
		MoveTemp(PixelLease), MoveTemp(ResultLease), TimeoutSeconds);
}

bool UURLabRenderServerWorldSubsystem::PlanCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const TArray<FString>& CameraNames, FURLabRenderExactCapturePlan& OutPlan,
	FString& OutError)
{
	check(IsInGameThread());
	OutPlan = {};
	if (bMutationInProgress)
	{
		OutError = TEXT("a render-server mutation is already in progress");
		return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision)
	{
		if (OutError.IsEmpty()) OutError = TEXT("owner has no installed render source");
		return false;
	}
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	if (!Scheduler)
	{
		OutError = TEXT("world has no render camera scheduler");
		return false;
	}
	if (!Scheduler->PlanExactCapture(
			Producer, ExpectedRevision, CameraNames, OutPlan, OutError)) return false;
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Producer);
	if (!Scene || !Scene->PlanGeomPoseOverrides({}, OutPlan.GeomPoseOverrides,
			OutPlan.GeometryConfigurationId, OutError))
	{
		OutPlan = {}; return false;
	}
	return true;
}

bool UURLabRenderServerWorldSubsystem::PlanCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const TArray<FRenderCameraCaptureSelection>& Cameras,
	FURLabRenderExactCapturePlan& OutPlan, FString& OutError)
{
	return PlanCapture(Proof, ExpectedRevision, Cameras, {}, OutPlan, OutError);
}

bool UURLabRenderServerWorldSubsystem::PlanCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const TArray<FRenderCameraCaptureSelection>& Cameras,
	const TArray<FRenderGeomPoseOverride>& GeomPoseOverrides,
	FURLabRenderExactCapturePlan& OutPlan, FString& OutError)
{
	check(IsInGameThread());
	OutPlan = {};
	if (bMutationInProgress)
	{
		OutError = TEXT("a render-server mutation is already in progress");
		return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision)
	{
		if (OutError.IsEmpty()) OutError = TEXT("owner has no matching installed source");
		return false;
	}
	URenderCameraSubsystem* Scheduler = GetWorld()->GetSubsystem<URenderCameraSubsystem>();
	if (!Scheduler) { OutError = TEXT("world has no render camera scheduler"); return false; }
	if (!Scheduler->PlanExactCapture(Producer, ExpectedRevision, Cameras, OutPlan, OutError))
		return false;
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Producer);
	if (!Scene || !Scene->PlanGeomPoseOverrides(GeomPoseOverrides, OutPlan.GeomPoseOverrides,
			OutPlan.GeometryConfigurationId, OutError))
	{
		OutPlan = {}; return false;
	}
	return true;
}

FURLabRenderServerCaptureAdmission UURLabRenderServerWorldSubsystem::SubmitQposCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	TConstArrayView<double> Qpos, uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch, FURLabRenderExactCapturePlan Plan,
	FRenderCameraCaptureLifetimeLeasePtr PixelLease,
	FRenderCameraCaptureLifetimeLeasePtr ResultLease, double TimeoutSeconds)
{
	check(IsInGameThread());
	FURLabRenderServerCaptureAdmission Rejected;
	if (bMutationInProgress)
	{
		Rejected.Error = TEXT("a render-server mutation is already in progress");
		return Rejected;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision || !Source->Kinematics)
	{
		Rejected.Error = Error.IsEmpty()
			? TEXT("owner has no installed qpos source") : MoveTemp(Error);
		return Rejected;
	}
	FRenderFrame Frame;
	if (!Source->Kinematics->FrameFromQpos(ExpectedRevision, Qpos, FrameId,
		SourceTime, bHasSourceTime, ClockEpoch, Frame, Error))
	{
		Rejected.Error = MoveTemp(Error);
		return Rejected;
	}
	return SubmitPreparedCapture(Proof, ExpectedRevision, MoveTemp(Frame), MoveTemp(Plan),
		MoveTemp(PixelLease), MoveTemp(ResultLease), TimeoutSeconds);
}

FURLabRenderServerCaptureAdmission UURLabRenderServerWorldSubsystem::SubmitPreparedCapture(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderFrame&& Frame, FURLabRenderExactCapturePlan&& Plan,
	FRenderCameraCaptureLifetimeLeasePtr&& PixelLease,
	FRenderCameraCaptureLifetimeLeasePtr&& ResultLease, double TimeoutSeconds)
{
	FURLabRenderServerCaptureAdmission Result;
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error) || !IsBoundOwner(Proof)
		|| Source->SceneRevision != ExpectedRevision)
	{
		Result.Error = Error.IsEmpty()
			? TEXT("owner has no installed render source") : MoveTemp(Error);
		return Result;
	}
	if (!PixelLease || !ResultLease)
	{
		Result.Error = TEXT("capture admission requires pixel and result capacity leases");
		return Result;
	}
	if (CaptureJobs.Num() >= MaxRetainedCaptureJobs)
	{
		Result.Error = TEXT("render-server retained capture capacity is exhausted");
		return Result;
	}
	if (NextCaptureJobSerial == MAX_uint64)
	{
		Result.Error = TEXT("render-server capture job identity is exhausted");
		return Result;
	}
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	if (!Scheduler)
	{
		Result.Error = TEXT("world has no render camera scheduler");
		return Result;
	}

	const uint64 Serial = NextCaptureJobSerial++;
	FCaptureJobRecordPtr Record(new FCaptureJobRecord());
	Record->Owner = Proof;
	Record->SceneRevision = ExpectedRevision;
	const FRenderFrameIdentity SubmittedIdentity = Frame.Identity;
	Record->SubmittedFrame = SubmittedIdentity;
	Record->ResultLease = MoveTemp(ResultLease);
	CaptureJobs.Add(Serial, MoveTemp(Record));

	FURLabRenderCaptureJobRequest Request;
	Request.Producer = Producer;
	Request.ExpectedSceneRevision = ExpectedRevision;
	Request.PresentationFrame = MoveTemp(Frame);
	Request.Plan = MoveTemp(Plan);
	Request.TimeoutSeconds = TimeoutSeconds;
	Request.LifetimeLease = MoveTemp(PixelLease);
	FURLabRenderCaptureJobAdmission SchedulerAdmission =
		Scheduler->SubmitExactCapture(MoveTemp(Request));
	FCaptureJobRecordPtr* Installed = CaptureJobs.Find(Serial);
	if (!SchedulerAdmission.bAccepted || !Installed)
	{
		FCaptureJobRecordPtr Retired;
		CaptureJobs.RemoveAndCopyValue(Serial, Retired);
		if (Retired) PendingRetiredCaptureJobs.Add(MoveTemp(Retired));
		Result.Error = SchedulerAdmission.Error.IsEmpty()
			? TEXT("render camera scheduler refused capture")
			: MoveTemp(SchedulerAdmission.Error);
		return Result;
	}
	(*Installed)->SchedulerJob = SchedulerAdmission.Job;
	Result.bAccepted = true;
	Result.Handle.Workflow = this;
	Result.Handle.WorkflowLifetime = WorkflowLifetime;
	Result.Handle.JobSerial = Serial;
	Result.Handle.SubmittedFrame = SubmittedIdentity;
	return Result;
}

FURLabRenderServerCaptureAdmission UURLabRenderServerWorldSubsystem::QueryCaptureHistory(
	const FURLabRenderOwnerProof& Proof, const FURLabRenderHistoryQuery& Query,
	FRenderCameraCaptureLifetimeLeasePtr ResultLease)
{
	FURLabRenderServerCaptureAdmission Result; FString Error;
	if (!ValidateMutation(Proof, Query.ExpectedSceneRevision, Error) || !IsBoundOwner(Proof))
	{
		Result.Error = Error.IsEmpty() ? TEXT("render owner is invalid") : MoveTemp(Error); return Result;
	}
	if (!ResultLease) { Result.Error = TEXT("history query requires result capacity lease"); return Result; }
	if (CaptureJobs.Num() >= MaxRetainedCaptureJobs)
	{
		Result.Error = TEXT("render-server retained capture capacity is exhausted"); return Result;
	}
	if (NextCaptureJobSerial == MAX_uint64)
	{
		Result.Error = TEXT("render-server capture job identity is exhausted"); return Result;
	}
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	if (!Scheduler) { Result.Error = TEXT("world has no render camera scheduler"); return Result; }
	FURLabRenderCaptureJobAdmission Admission = Scheduler->QueryExactCaptureHistory(Query);
	if (!Admission.bAccepted) { Result.Error = MoveTemp(Admission.Error); return Result; }
	const FURLabRenderCaptureJobSnapshot Snapshot = Admission.Job.Poll();
	if (Snapshot.Frames.IsEmpty()) { Result.Error = TEXT("history query returned no frames"); return Result; }
	const uint64 Serial = NextCaptureJobSerial++;
	FCaptureJobRecordPtr Record(new FCaptureJobRecord());
	Record->Owner = Proof; Record->SceneRevision = Query.ExpectedSceneRevision;
	Record->SubmittedFrame = Snapshot.Frames[0]->Identity;
	Record->SchedulerJob = Admission.Job; Record->ResultLease = MoveTemp(ResultLease);
	CaptureJobs.Add(Serial, MoveTemp(Record));
	Result.bAccepted = true; Result.Handle.Workflow = this;
	Result.Handle.WorkflowLifetime = WorkflowLifetime; Result.Handle.JobSerial = Serial;
	Result.Handle.SubmittedFrame = Snapshot.Frames[0]->Identity; return Result;
}

void UURLabRenderServerWorldSubsystem::RetireCaptureHistory(const FGuid& RetentionContext)
{
	check(IsInGameThread());
	if (URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr)
		Scheduler->RetireExactCaptureHistory(RetentionContext);
}

bool UURLabRenderServerWorldSubsystem::ResolveCaptureHistory(
	const FURLabRenderOwnerProof& Proof, const FURLabRenderHistoryQuery& Query,
	FURLabRenderExactCapturePlan& OutPlan, FRenderFrameIdentity& OutIdentity, FString& OutError)
{
	if (!ValidateMutation(Proof, Query.ExpectedSceneRevision, OutError) || !IsBoundOwner(Proof)) return false;
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	if (!Scheduler) { OutError = TEXT("world has no render camera scheduler"); return false; }
	return Scheduler->ResolveExactCaptureHistory(Query, OutPlan, OutIdentity, OutError);
}

bool UURLabRenderServerWorldSubsystem::ValidateJobOwner(
	const FURLabRenderOwnerProof& Proof,
	const FURLabRenderServerCaptureHandle& Handle,
	FCaptureJobRecord*& OutRecord, FString& OutError)
{
	OutRecord = nullptr;
	if (!IsActive() || bShutdownRequested || bShutdownInProgress)
	{
		OutError = TEXT("render-server workflow is not active");
		return false;
	}
	const FURLabRenderOwnerAdmissionResult Admission = Authority.Validate(Proof);
	if (!Admission.IsOk())
	{
		OutError = Admission.Message;
		return false;
	}
	if (!Handle.IsValid() || Handle.Workflow.Get() != this
		|| Handle.WorkflowLifetime != WorkflowLifetime)
	{
		OutError = TEXT("capture handle is stale or belongs to another workflow");
		return false;
	}
	FCaptureJobRecordPtr* Found = CaptureJobs.Find(Handle.JobSerial);
	if (!Found || !Found->IsValid())
	{
		OutError = TEXT("capture job is no longer retained");
		return false;
	}
	if ((*Found)->Owner.GetGeneration() != Proof.GetGeneration()
		|| (*Found)->Owner.GetOpaqueToken() != Proof.GetOpaqueToken())
	{
		OutError = TEXT("capture job belongs to another render owner");
		return false;
	}
	OutRecord = Found->Get();
	OutError.Reset();
	return true;
}

bool UURLabRenderServerWorldSubsystem::PollCapture(
	const FURLabRenderOwnerProof& Proof, const FURLabRenderServerCaptureHandle& Handle,
	FURLabRenderCaptureJobSnapshot& OutSnapshot, FString& OutError)
{
	check(IsInGameThread());
	OutSnapshot = {};
	if (bMutationInProgress) { OutError = TEXT("a render-server mutation is already in progress"); return false; }
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FCaptureJobRecord* Record = nullptr;
	if (!ValidateJobOwner(Proof, Handle, Record, OutError)) return false;
	OutSnapshot = Record->SchedulerJob.Poll();
	return true;
}

bool UURLabRenderServerWorldSubsystem::CancelCapture(
	const FURLabRenderOwnerProof& Proof, const FURLabRenderServerCaptureHandle& Handle,
	FString& OutError)
{
	check(IsInGameThread());
	if (bMutationInProgress) { OutError = TEXT("a render-server mutation is already in progress"); return false; }
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FCaptureJobRecord* Record = nullptr;
	if (!ValidateJobOwner(Proof, Handle, Record, OutError)) return false;
	URenderCameraSubsystem* Scheduler = GetWorld()
		? GetWorld()->GetSubsystem<URenderCameraSubsystem>() : nullptr;
	if (!Scheduler) { OutError = TEXT("world has no render camera scheduler"); return false; }
	return Scheduler->CancelExactCapture(Record->SchedulerJob, OutError);
}

bool UURLabRenderServerWorldSubsystem::ReleaseCapture(
	const FURLabRenderOwnerProof& Proof, const FURLabRenderServerCaptureHandle& Handle,
	FString& OutError)
{
	check(IsInGameThread());
	if (bMutationInProgress) { OutError = TEXT("a render-server mutation is already in progress"); return false; }
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FCaptureJobRecord* Record = nullptr;
	if (!ValidateJobOwner(Proof, Handle, Record, OutError)) return false;
	if (!Record->SchedulerJob.Poll().IsTerminal())
	{
		OutError = TEXT("capture job must be terminal before release");
		return false;
	}
	FCaptureJobRecordPtr Retired;
	CaptureJobs.RemoveAndCopyValue(Handle.JobSerial, Retired);
	if (Retired) PendingRetiredCaptureJobs.Add(MoveTemp(Retired));
	OutError.Reset();
	return true;
}

void UURLabRenderServerWorldSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (bShutdownRequested && !bMutationInProgress && !bShutdownInProgress)
	{
		ShutdownNow();
		return;
	}
	if (bActive && !IsActive())
	{
		ShutdownNow();
		return;
	}
	if (!bActive || bMutationInProgress || bShutdownInProgress || !OwnerScope) return;
	const FURLabRenderOwnerStatusResult Status = OwnerStatus();
	if (!IsActive()) return;
	if (Status.IsOk() && Status.Status.bStale)
		RetireOwnerScope(OwnerScope);
}

TStatId UURLabRenderServerWorldSubsystem::GetStatId() const
{ RETURN_QUICK_DECLARE_CYCLE_STAT(UURLabRenderServerWorldSubsystem, STATGROUP_Tickables); }

void UURLabRenderServerWorldSubsystem::ShutdownWorkflow()
{
	check(IsInGameThread());
	if (bMutationInProgress) { bShutdownRequested = true; return; }
	ShutdownNow();
}

void UURLabRenderServerWorldSubsystem::FinishMutation()
{
	bMutationInProgress = false;
	TArray<FCaptureJobRecordPtr> Retired = MoveTemp(PendingRetiredCaptureJobs);
	Retired.Reset();
	if (bShutdownRequested && !bShutdownInProgress) ShutdownNow();
}

void UURLabRenderServerWorldSubsystem::ShutdownNow()
{
	if (bShutdownInProgress) return;
	TGuardValue<bool> Guard(bShutdownInProgress, true);
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	bShutdownRequested = false;
	DetachBridge();
	FURLabRenderOwnerRetirementResult Begun = Authority.BeginShutdownRetirement();
	RetireCaptureJobs(nullptr);
	RevokeSource();
	OwnerScope.Reset();
	if (Begun.Token.IsValid()) Authority.FinishRetirement(Begun.Token);
	if (UURLabRenderWorldSubsystem* Current = Coordinator.Get())
	{
		FString ReleaseError;
		if (Current->IsRegistrationCurrent(Producer)
			&& !Current->ReleaseProducer(Producer, ReleaseError))
		{
			bShutdownRequested = true;
			UE_LOG(LogURLab, Warning, TEXT("[URLabRenderServer] producer release deferred: %s"),
				*ReleaseError);
			return;
		}
	}
	Producer = {};
	Coordinator.Reset();
	bActive = false;
	bShutdownPermanent = true;
}

void UURLabRenderServerWorldSubsystem::Deinitialize()
{
	ShutdownWorkflow();
	Super::Deinitialize();
}
