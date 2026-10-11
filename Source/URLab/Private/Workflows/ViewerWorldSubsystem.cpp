#include "Workflows/ViewerWorldSubsystem.h"
#include "Bridge/RenderPoseIngress.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/ViewerServiceBinding.h"
#include "Bridge/CameraStreamServiceBinding.h"
#include "MuJoCo/Render/MjLauncherFlags.h"
#include "HAL/PlatformProcess.h"

#include "Engine/World.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Render/MjRenderKinematics.h"
#include "Render/CameraStreamController.h"
#include "Render/URLabRenderScene.h"
#include "Utils/URLabLogging.h"
#include "Transport/PoseStreamFrameCodec.h"
#include "Utils/CoordinateConversion.h"

struct FURLabViewerWorldServiceBinding
{
	TWeakObjectPtr<UURLabBridgeServer> Bridge;
	FURLabRpcDispatcher::FViewerServiceRegistration Registration;
	FURLabRpcDispatcher::FCameraStreamServiceRegistration CameraRegistration;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> CameraBinding;
};

namespace
{
FURLabViewerInstallResult InstallRejected(FString Error)
{
	FURLabViewerInstallResult Result;
	Result.Error = MoveTemp(Error);
	return Result;
}

template <typename TResult>
TResult Unavailable(const TCHAR* Message,
	EURLabRenderOwnerAuthorityError Error = EURLabRenderOwnerAuthorityError::Unavailable)
{
	TResult Result;
	Result.Error = Error;
	Result.Status.bAvailable = false;
	Result.Message = Message;
	return Result;
}

bool SameOwner(const FURLabRenderOwnerProof& A, const FURLabRenderOwnerProof& B)
{
	return A.GetGeneration() == B.GetGeneration()
		&& A.GetOpaqueToken() == B.GetOpaqueToken();
}
}

bool FURLabViewerStreamDescriptor::IsValid(FString& OutError) const
{
	OutError.Reset();
	if (ProducerBootId.IsEmpty() || StreamId.IsEmpty() || SourceRevision.IsEmpty())
	{ OutError = TEXT("viewer stream identity and source revision must be non-empty"); return false; }
	if (StreamGeneration == 0)
	{ OutError = TEXT("viewer stream generation must be positive"); return false; }
	if (SchemaVersion != 2 || PayloadFormat != TEXT("dense_rh_m_wxyz_f64le_dynamic_mesh_v1"))
	{ OutError = TEXT("viewer stream schema or payload format is unsupported"); return false; }
	return true;
}

void UURLabViewerWorldSubsystem::FKinematicsDeleter::operator()(FMjRenderKinematics* Value) const
{ delete Value; }

UURLabViewerWorldSubsystem::UURLabViewerWorldSubsystem() = default;
UURLabViewerWorldSubsystem::~UURLabViewerWorldSubsystem() = default;

bool UURLabViewerWorldSubsystem::DoesSupportWorldType(EWorldType::Type Type) const
{ return Type == EWorldType::Game || Type == EWorldType::PIE || Type == EWorldType::Editor; }

bool UURLabViewerWorldSubsystem::Activate(FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (bShutdownPermanent) { OutError = TEXT("viewer workflow has shut down"); return false; }
	if (bMutationInProgress || bShutdownRequested || bShutdownInProgress)
	{ OutError = TEXT("viewer workflow is busy"); return false; }
	if (bActive) return IsActive();
	UWorld* World = GetWorld();
	UURLabRenderWorldSubsystem* Candidate = World ? World->GetSubsystem<UURLabRenderWorldSubsystem>() : nullptr;
	if (!World || !DoesSupportWorldType(World->WorldType) || !Candidate)
	{ OutError = TEXT("viewer workflow requires a supported render world"); return false; }
	if (!Candidate->RegisterProducer(Producer, OutError)) return false;
	Coordinator = Candidate;
	bActive = true;
	return true;
}

bool UURLabViewerWorldSubsystem::IsActive() const
{
	const UURLabRenderWorldSubsystem* Current = Coordinator.Get();
	return bActive && Current && Current->IsRegistrationCurrent(Producer);
}

bool UURLabViewerWorldSubsystem::AttachBridge(FString& OutError)
{
	check(IsInGameThread());OutError.Reset();
	if(!IsActive()||bMutationInProgress||bShutdownRequested||bShutdownInProgress){OutError=TEXT("viewer workflow must be active and idle before bridge attachment");return false;}
	if(ServiceBinding)
	{
		auto* Bridge=ServiceBinding->Bridge.Get();auto* Dispatcher=Bridge?Bridge->GetDispatcher():nullptr;
		if(Bridge&&Bridge->IsRunning()&&Dispatcher&&Dispatcher->IsViewerServiceRegistrationActive(ServiceBinding->Registration)
			&&(!ServiceBinding->CameraRegistration.IsValid()||Dispatcher->IsCameraStreamServiceRegistrationActive(ServiceBinding->CameraRegistration)))return true;
		ServiceBinding.Reset();
	}
	auto* Bridge=UURLabBridgeSubsystem::EnsureFor(GetWorld());auto* Dispatcher=Bridge?Bridge->GetDispatcher():nullptr;
	if(!Bridge||!Dispatcher){OutError=TEXT("viewer world bridge did not provide a running dispatcher");return false;}
	auto Opaque=FURLabViewerServiceBinding::Create(*this,OutError);if(!Opaque)return false;
	auto Candidate=MakeShared<FURLabViewerWorldServiceBinding,ESPMode::ThreadSafe>();Candidate->Bridge=Bridge;
	if(!Dispatcher->AttachViewerService(Opaque.ToSharedRef(),Candidate->Registration,OutError))return false;
	if (URLabLauncherFlags::ParseCaps().Has(EMjCapability::CameraFrames))
	{
		const auto& Config = Bridge->ResolveInstanceConfig();
		FURLabCameraStreamProducerConfig CameraConfig;
		CameraConfig.BindEndpoint = FString::Printf(TEXT("tcp://%s:%d"), *Config.BindAddress, Config.CamBasePort);
		const FString DialHost = Config.BindAddress == TEXT("0.0.0.0")
			? FString(FPlatformProcess::ComputerName()) : Config.BindAddress;
		CameraConfig.AdvertisedEndpoint = FString::Printf(TEXT("tcp://%s:%d"), *DialHost, Config.CamBasePort);
		CameraConfig.ShmRegion = Dispatcher->GetShmRegionForLocalProducer();
		Candidate->CameraBinding = FURLabCameraStreamServiceBinding::Create(*this, CameraConfig, OutError);
		if (!Candidate->CameraBinding || !Dispatcher->AttachCameraStreamService(
			Candidate->CameraBinding.ToSharedRef(), Candidate->CameraRegistration, OutError))
		{
			if (Candidate->CameraBinding) Candidate->CameraBinding->Deactivate();
			Dispatcher->DetachViewerService(Candidate->Registration);
			return false;
		}
	}
	ServiceBinding=MoveTemp(Candidate);return true;
}

void UURLabViewerWorldSubsystem::DetachBridge()
{
	check(IsInGameThread());if(!ServiceBinding)return;
	if(auto* Bridge=ServiceBinding->Bridge.Get())if(auto* Dispatcher=Bridge->GetDispatcher())
	{
		if(ServiceBinding->CameraRegistration.IsValid()) Dispatcher->DetachCameraStreamService(ServiceBinding->CameraRegistration);
		Dispatcher->DetachViewerService(ServiceBinding->Registration);
	}
	if(ServiceBinding->CameraBinding) ServiceBinding->CameraBinding->Deactivate();
	ServiceBinding.Reset();
}

bool UURLabViewerWorldSubsystem::IsBridgeAttached() const
{
	if(!ServiceBinding||!IsActive())return false;auto* Bridge=ServiceBinding->Bridge.Get();auto* Dispatcher=Bridge?Bridge->GetDispatcher():nullptr;
	return Bridge&&Bridge->IsRunning()&&Dispatcher&&Dispatcher->IsViewerServiceRegistrationActive(ServiceBinding->Registration);
}

FURLabViewerStatus UURLabViewerWorldSubsystem::Status()
{
	check(IsInGameThread());
	FURLabViewerStatus Result;
	const FURLabRenderOwnerStatusResult OwnerStatus = Authority.Status();
	Result.Owner = OwnerStatus.Status;
	Result.bHasInstalledSource = Source.IsSet();
	Result.bStreamBound = Stream.IsSet();
	Result.bDisplayStale = bDisplayStale;
	if (Source) Result.SceneRevision = Source->SceneRevision;
	return Result;
}

FURLabRenderOwnerAttachResult UURLabViewerWorldSubsystem::AttachOwner(
	const FURLabRenderOwnerSessionScopePtr& Scope, double TtlSeconds)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return Unavailable<FURLabRenderOwnerAttachResult>(TEXT("viewer workflow is unavailable"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	FURLabRenderOwnerAttachResult Result = Authority.Attach(Scope, TtlSeconds);
	if (Result.IsOk()) OwnerScope = Scope;
	return Result;
}

FURLabRenderOwnerAuthorizationResult UURLabViewerWorldSubsystem::AuthorizeOwner(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return Unavailable<FURLabRenderOwnerAuthorizationResult>(TEXT("viewer workflow is unavailable"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	return Authority.Authorize(Scope, Generation, Token);
}

FURLabRenderOwnerLeaseResult UURLabViewerWorldSubsystem::RenewOwner(
	const FURLabRenderOwnerProof& Proof, double TtlSeconds)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return Unavailable<FURLabRenderOwnerLeaseResult>(TEXT("viewer workflow is unavailable"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	return Authority.Renew(Proof, TtlSeconds);
}

FURLabRenderOwnerLeaseResult UURLabViewerWorldSubsystem::CompleteRetirement(
	FURLabRenderOwnerRetirementResult&& Begun)
{
	if (!Begun.IsOk())
	{
		FURLabRenderOwnerLeaseResult Result;
		Result.Error = Begun.Error; Result.Status = Begun.Status; Result.Message = Begun.Message;
		return Result;
	}
	ClearStreamBinding();
	OwnerScope.Reset();
	return Authority.FinishRetirement(Begun.Token);
}

FURLabRenderOwnerLeaseResult UURLabViewerWorldSubsystem::ReleaseOwnerCredentials(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 Generation, const FString& Token)
{
	check(IsInGameThread());
	if (!IsActive() || bMutationInProgress) return Unavailable<FURLabRenderOwnerLeaseResult>(TEXT("viewer workflow is unavailable"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	const FURLabRenderOwnerAuthorizationResult Authorized = Authority.AuthorizeForRelease(Scope, Generation, Token);
	if (!Authorized.IsOk())
	{
		FURLabRenderOwnerLeaseResult Result;
		Result.Error=Authorized.Error; Result.Status=Authorized.Status; Result.Message=Authorized.Message;
		return Result;
	}
	return CompleteRetirement(Authority.BeginRelease(Authorized.Proof));
}

FURLabRenderOwnerLeaseResult UURLabViewerWorldSubsystem::RetireOwnerScope(
	const FURLabRenderOwnerSessionScopePtr& Scope)
{
	check(IsInGameThread());
	if (bMutationInProgress) return Unavailable<FURLabRenderOwnerLeaseResult>(TEXT("viewer workflow is mutating"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	return CompleteRetirement(Authority.BeginLifecycleRetirement(Scope));
}

bool UURLabViewerWorldSubsystem::ValidateMutation(const FURLabRenderOwnerProof& Proof,
	const FString& ExpectedRevision, FString& OutError)
{
	if (!IsActive() || bShutdownRequested || bShutdownInProgress)
	{ OutError = TEXT("viewer workflow is unavailable"); return false; }
	const FURLabRenderOwnerAdmissionResult Admission = Authority.Validate(Proof);
	if (!Admission.IsOk()) { OutError = Admission.Message; return false; }
	if (Coordinator->ResolveActiveSceneRevision(Producer) != ExpectedRevision)
	{ OutError = TEXT("expected viewer scene revision is not active"); return false; }
	return true;
}

FURLabViewerInstallResult UURLabViewerWorldSubsystem::InstallNeutralScene(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options)
{
	check(IsInGameThread());
	FRenderSceneInstallPayload Owned = MoveTemp(Payload);
	if (bMutationInProgress) return InstallRejected(TEXT("viewer workflow is mutating"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	return InstallPrepared(Proof, ExpectedRevision, MoveTemp(Owned), Options, FKinematicsPtr());
}

FURLabViewerInstallResult UURLabViewerWorldSubsystem::InstallKinematicsCandidate(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FMjRenderKinematicsCandidate&& Candidate, const FRenderSceneInstallOptions& Options)
{
	check(IsInGameThread());
	if (bMutationInProgress) return InstallRejected(TEXT("viewer workflow is mutating"));
	bMutationInProgress = true; ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	FRenderScene Scene; FRenderSceneResources Resources; FRenderFrame Initial;
	if (!Candidate.TakeNeutralScene(Scene, Resources, Initial, Error)) return InstallRejected(MoveTemp(Error));
	FKinematicsPtr Prepared(new FMjRenderKinematics());
	if (!Prepared->Commit(MoveTemp(Candidate), FString(), Error)) return InstallRejected(MoveTemp(Error));
	FRenderSceneInstallPayload Payload(MoveTemp(Scene), MoveTemp(Resources), MoveTemp(Initial));
	return InstallPrepared(Proof, ExpectedRevision, MoveTemp(Payload), Options, MoveTemp(Prepared));
}

FURLabViewerInstallResult UURLabViewerWorldSubsystem::InstallPrepared(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FRenderSceneInstallPayload&& Payload, const FRenderSceneInstallOptions& Options,
	FKinematicsPtr&& Kinematics)
{
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	const FString NewRevision = Payload.InitialFrame.Identity.SceneRevision;
	const int32 NewBodyCount=Payload.Scene.Bodies.Num();
	const int32 NewCameraCount=Payload.Scene.Cameras.Num();
	const int32 NewLightCount=Payload.Scene.Lights.Num();
	const bool bNewUserCamera=Payload.Scene.UserCamera.IsSet();
	const TArray<FRenderDeformableDesc> NewDeformables=Payload.Scene.Deformables;
	FURLabRenderSceneCandidate Staged;
	if (!Coordinator->StageScene(Producer, ExpectedRevision, MoveTemp(Payload), Options, Staged, Error))
		return InstallRejected(MoveTemp(Error));
	TOptional<FSourceState> Retired;
	FURLabRenderWorldCommitResult Committed = Coordinator->CommitCandidate(Producer, Staged,
		[&](FString& RelatedError)
		{
			if (!ValidateMutation(Proof, ExpectedRevision, RelatedError)) return false;
			ClearStreamBinding();
			FSourceState Replacement; Replacement.SceneRevision = NewRevision;
			Replacement.Kinematics = MoveTemp(Kinematics); Replacement.BodyCount=NewBodyCount; Replacement.ModelCameraCount=NewCameraCount; Replacement.LightCount=NewLightCount; Replacement.bHasUserCamera=bNewUserCamera;Replacement.Deformables=NewDeformables;
			if (Source) Retired.Emplace(MoveTemp(Source.GetValue()));
			VisibleFrameLease.Reset();
			Source.Emplace(MoveTemp(Replacement));
			bDisplayStale = true;
			return true;
		});
	if (!Committed.bCommitted)
	{
		FString Ignored; Coordinator->DiscardCandidate(Producer, Staged, Ignored);
		return InstallRejected(MoveTemp(Committed.Error));
	}
	FURLabViewerInstallResult Result;
	Result.bCommitted=true; Result.SceneGeneration=Committed.SceneGeneration;
	Result.SceneRevision=MoveTemp(Committed.SceneRevision);
	return Result;
}

FURLabViewerInstallResult UURLabViewerWorldSubsystem::PatchScene(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const FURLabRenderScenePatch& Patch)
{
	check(IsInGameThread());
	if (bMutationInProgress) return InstallRejected(TEXT("viewer workflow is mutating"));
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!ValidateMutation(Proof, ExpectedRevision, Error)) return InstallRejected(MoveTemp(Error));
	if (!Source || Source->SceneRevision != ExpectedRevision)
		return InstallRejected(TEXT("viewer has no matching installed scene"));
	AURLabRenderScene* Active = Coordinator->ResolveActiveScene(Producer);
	if (!Active) return InstallRejected(TEXT("viewer active scene is unavailable"));
	FRenderSceneInstallPayload Payload;
	FRenderSceneInstallOptions Options;
	if (!Active->CopyInstalledPayload(Payload, Options, Error))
		return InstallRejected(MoveTemp(Error));
	const FString NewRevision=FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
	if (!Patch.Apply(Payload.Scene,Payload.Resources,Options.VisibleGroupMask,Error))
		return InstallRejected(MoveTemp(Error));
	Payload.Scene.SceneRevision=NewRevision;
	Payload.Resources.SceneRevision=NewRevision;
	Payload.InitialFrame.Identity.SceneRevision=NewRevision;
	if(!Payload.InitialFrame.IsValid(Payload.Scene,Error))
		return InstallRejected(FString::Printf(TEXT("patched initial frame is invalid: %s"),*Error));
	const FRenderScene PatchedScene=Payload.Scene;
	FURLabRenderSceneCandidate Staged;
	if(!Coordinator->StageScene(Producer,ExpectedRevision,MoveTemp(Payload),Options,Staged,Error))
		return InstallRejected(MoveTemp(Error));
	TOptional<FSourceState> Retired;
	FURLabRenderWorldCommitResult Committed=Coordinator->CommitCandidate(Producer,Staged,
		[&](FString& RelatedError)
		{
			if(!ValidateMutation(Proof,ExpectedRevision,RelatedError))return false;
			if(Source->Kinematics.IsValid()
				&&!Source->Kinematics->RebaseSceneRevision(ExpectedRevision,PatchedScene,RelatedError))
				return false;
			FSourceState Replacement;Replacement.SceneRevision=NewRevision;
			Replacement.Kinematics=MoveTemp(Source->Kinematics);
			Replacement.BodyCount=Source->BodyCount;Replacement.ModelCameraCount=Source->ModelCameraCount;
			Replacement.LightCount=Source->LightCount;Replacement.bHasUserCamera=Source->bHasUserCamera;
			Replacement.Deformables=Source->Deformables;
			if(Source)Retired.Emplace(MoveTemp(Source.GetValue()));
			Source.Emplace(MoveTemp(Replacement));
			if(Stream)Stream->LocalSceneRevision=NewRevision;
			return true;
		});
	if(!Committed.bCommitted)
	{
		FString Ignored;Coordinator->DiscardCandidate(Producer,Staged,Ignored);
		return InstallRejected(MoveTemp(Committed.Error));
	}
	FURLabViewerInstallResult Result;Result.bCommitted=true;
	Result.SceneGeneration=Committed.SceneGeneration;Result.SceneRevision=MoveTemp(Committed.SceneRevision);
	return Result;
}

bool UURLabViewerWorldSubsystem::BindStream(const FURLabRenderOwnerProof& Proof,
	const FString& ExpectedRevision, const FURLabViewerStreamDescriptor& Descriptor,
	FString& OutError)
{
	check(IsInGameThread());
	if (bMutationInProgress) { OutError=TEXT("viewer workflow is mutating"); return false; }
	bMutationInProgress=true; ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError)) return false;
	if (!Source || Source->SceneRevision != ExpectedRevision)
	{ OutError=TEXT("viewer has no matching installed source"); return false; }
	if (!Descriptor.IsValid(OutError)) return false;
	FStreamBinding Replacement;
	Replacement.Owner=Proof; Replacement.Descriptor=Descriptor;
	Replacement.LocalSceneRevision=ExpectedRevision;
	Stream.Emplace(MoveTemp(Replacement));
	bDisplayStale=true;
	return true;
}

bool UURLabViewerWorldSubsystem::BindPreparedStream(const FURLabRenderOwnerProof& Proof,const FString& ExpectedRevision,const FURLabViewerStreamDescriptor& Descriptor,FURLabViewerStreamConnectionPtr Connection,TSharedPtr<FURLabPayloadStore,ESPMode::ThreadSafe> Store,const FURLabPayloadOwner& PayloadOwner,FString& OutError)
{
	check(IsInGameThread()); if(!Connection||!Store||!PayloadOwner.Session){OutError=TEXT("viewer stream connection or payload owner is unavailable");return false;}
	if(!BindStream(Proof,ExpectedRevision,Descriptor,OutError))return false;
	Stream->Connection=MoveTemp(Connection);Stream->Store=MoveTemp(Store);Stream->PayloadOwner=PayloadOwner;return true;
}

bool UURLabViewerWorldSubsystem::ConfigureCameraStream(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	FURLabCameraStreamController& Controller, const FString& CameraName,
	bool bEnabled, TOptional<double> RateHz,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (bMutationInProgress)
	{
		OutError = TEXT("viewer workflow is mutating");
		return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError)) return false;
	if (!Source || Source->SceneRevision != ExpectedRevision)
	{
		OutError = TEXT("viewer has no matching installed source");
		return false;
	}
	AURLabRenderScene* Scene = Coordinator->ResolveActiveScene(Producer);
	if (!Scene || Scene->GetScene().SceneRevision != ExpectedRevision)
	{
		OutError = TEXT("installed viewer source is not the active scene");
		return false;
	}
	return Controller.Configure(*Scene, ExpectedRevision, CameraName, bEnabled,
		RateHz, OutDescriptor, OutError);
}

bool UURLabViewerWorldSubsystem::ForwardInteraction(
	const FURLabRenderOwnerProof& Proof, const FString& ExpectedRevision,
	const FURLabInteractionSourceIdentity& ProducerSource, FURLabWireValue Body,
	int32 TimeoutMs,
	FURLabViewerStreamConnection::FInteractionCallback&& Completion,
	FString& OutError)
{
	check(IsInGameThread());
	if (bMutationInProgress)
	{
		OutError = TEXT("viewer workflow is mutating");
		return false;
	}
	bMutationInProgress = true;
	ON_SCOPE_EXIT { FinishMutation(); };
	if (!ValidateMutation(Proof, ExpectedRevision, OutError)) return false;
	if (!Stream || !Source || Source->SceneRevision != ExpectedRevision
		|| Stream->LocalSceneRevision != ExpectedRevision
		|| !SameOwner(Proof, Stream->Owner) || !Stream->Connection)
	{
		OutError = TEXT("viewer has no matching prepared producer binding");
		return false;
	}
	const FURLabViewerStreamDescriptor& Bound = Stream->Descriptor;
	if (!ProducerSource.IsValid()
		|| ProducerSource.ProducerBootId != Bound.ProducerBootId
		|| ProducerSource.StreamId != Bound.StreamId
		|| ProducerSource.StreamGeneration != Bound.StreamGeneration
		|| ProducerSource.SourceRevision != Bound.SourceRevision)
	{
		OutError = TEXT("interaction grant source does not match the bound producer");
		return false;
	}
	return Stream->Connection->ForwardInteraction(MoveTemp(Body), TimeoutMs,
		MoveTemp(Completion), OutError);
}

FURLabRenderFramePublicationResult UURLabViewerWorldSubsystem::ApplyStreamFrame(
	const FURLabRenderOwnerProof& Proof, const FURLabViewerStreamFrameHeader& Header,
	const FRenderFrame& Frame)
{
	check(IsInGameThread());
	FURLabRenderFramePublicationResult Rejected;
	if (bMutationInProgress) { Rejected.Error=TEXT("viewer workflow is mutating"); return Rejected; }
	bMutationInProgress=true; ON_SCOPE_EXIT { FinishMutation(); };
	FString Error;
	if (!Stream || !ValidateMutation(Proof, Stream ? Stream->LocalSceneRevision : FString(), Error))
	{ Rejected.Error=Stream ? MoveTemp(Error) : TEXT("viewer stream is not bound"); return Rejected; }
	if (!SameOwner(Proof, Stream->Owner)) { Rejected.Error=TEXT("viewer stream belongs to another owner"); return Rejected; }
	const FURLabViewerStreamDescriptor& D=Stream->Descriptor;
	if (Header.ProducerBootId!=D.ProducerBootId || Header.StreamId!=D.StreamId
		|| Header.StreamGeneration!=D.StreamGeneration || Header.SourceRevision!=D.SourceRevision)
	{
		// A larger generation from the exact bound producer/stream is authoritative
		// evidence that the source was replaced. Retire only this viewer binding;
		// unrelated, malformed and old/out-of-order identities remain harmless drops.
		if (Header.ProducerBootId==D.ProducerBootId && Header.StreamId==D.StreamId
			&& Header.StreamGeneration>D.StreamGeneration)
		{
			ClearStreamBinding();
		}
		Rejected.Error=TEXT("viewer frame producer identity or source revision is stale"); return Rejected;
	}
	if (Header.Sequence==0 || Header.Sequence<=Stream->LastSequence)
	{ Rejected.Error=TEXT("viewer frame sequence is not newer"); return Rejected; }
	if (Header.bHasSourceTime && !FMath::IsFinite(Header.SourceTime))
	{ Rejected.Error=TEXT("viewer frame source time is invalid"); return Rejected; }
	if (Frame.Identity.SceneRevision!=Stream->LocalSceneRevision
		|| Frame.Identity.FrameId!=Header.FrameId || Frame.Identity.ClockEpoch!=Header.ClockEpoch
		|| Frame.Identity.bHasSourceTime!=Header.bHasSourceTime
		|| (Header.bHasSourceTime && Frame.Identity.SourceTime!=Header.SourceTime))
	{ Rejected.Error=TEXT("decoded viewer frame identity does not match its stream header"); return Rejected; }
	FURLabRenderFramePublicationResult Result = Coordinator->PublishAuthoritativeFrame(
		Producer, Stream->LocalSceneRevision, Frame);
	if (Result.bAccepted) { Stream->LastSequence=Header.Sequence; bDisplayStale=false; }
	return Result;
}

void UURLabViewerWorldSubsystem::ClearStreamBinding()
{
	Stream.Reset();
	if (ServiceBinding && ServiceBinding->CameraBinding)
		ServiceBinding->CameraBinding->DisableStream();
	bDisplayStale=true;
}

void UURLabViewerWorldSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	for(int32 Index=RetiringConnections.Num()-1;Index>=0;--Index)
	{
		if(RetiringConnections[Index]&&RetiringConnections[Index]->CanFinalizeRetirement())
		{RetiringConnections[Index]->FinalizeRetirement();RetiringConnections.RemoveAtSwap(Index);}
	}
	if (bShutdownRequested && !bMutationInProgress) { ShutdownNow(); return; }
	if (bActive && !IsActive()) { ShutdownNow(); return; }
	if (!bActive || bMutationInProgress || !OwnerScope) return;
	if(Stream && Stream->Connection && Source)
	{
		if(Stream->Connection->GetHealth()!=FURLabViewerStreamConnection::EHealth::Healthy)
		{
			FURLabViewerStreamConnectionPtr Retiring=MoveTemp(Stream->Connection);
			Retiring->BeginRetirement();RetiringConnections.Add(MoveTemp(Retiring));
			ClearStreamBinding();
		}
		if(Stream&&Stream->Connection)
		{
		TArray<uint8> Bytes; if(Stream->Connection->PopLatest(Bytes))
		{
			FURLabPoseStreamFrame Wire; FString DecodeError; const int32 Count=Source->BodyCount+Source->ModelCameraCount+(Source->bHasUserCamera?1:0)+Source->LightCount;
			if(FURLabPoseStreamFrameCodec::Decode(Bytes,Wire,DecodeError,int64(Count)*7*8))
			{
				FRenderCameraCaptureLifetimeLeasePtr PresentationLease;
				if(FURLabRenderPoseIngress::ReserveCartesianPresentation(Stream->Store,Stream->PayloadOwner,Source->BodyCount,Source->ModelCameraCount,Source->bHasUserCamera,Source->LightCount,PresentationLease,DecodeError))
				{
				FRenderFrame Frame;Frame.Identity.SceneRevision=Source->SceneRevision;Frame.Identity.FrameId=Wire.FrameId;Frame.Identity.ClockEpoch=Wire.ClockEpoch;Frame.Identity.bHasSourceTime=Wire.bHasSourceTime;Frame.Identity.SourceTime=Wire.SourceTime;
				auto Read=[&](int32 Index,FTransform& Out,bool bCamera)->bool{double V[7];FMemory::Memcpy(V,Wire.Payload.GetData()+Index*56,56);for(double X:V)if(!FMath::IsFinite(X))return false;const double N=V[3]*V[3]+V[4]*V[4]+V[5]*V[5]+V[6]*V[6];if(FMath::Abs(N-1.0)>1e-6)return false;double P[3]={V[0],V[1],V[2]},Q[4]={V[3],V[4],V[5],V[6]};Out=bCamera?URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(P,Q):FTransform(URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Q),URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(P));return !Out.ContainsNaN();};
				bool Ok=Wire.Deformables.Num()==Source->Deformables.Num();int32 I=0;Frame.BodySceneTransforms.SetNum(Source->BodyCount);for(auto& T:Frame.BodySceneTransforms)Ok&=Read(I++,T,false);Frame.CameraSceneTransforms.SetNum(Source->ModelCameraCount);for(auto&T:Frame.CameraSceneTransforms)Ok&=Read(I++,T,true);if(Source->bHasUserCamera){FTransform T;Ok&=Read(I++,T,true);Frame.UserCameraSceneTransform=T;}Frame.LightSceneTransforms.SetNum(Source->LightCount);for(auto&T:Frame.LightSceneTransforms)Ok&=Read(I++,T,false);Frame.Deformables=MoveTemp(Wire.Deformables);Frame.DebugMeshes=MoveTemp(Wire.DebugMeshes);
					if(Ok){FURLabViewerStreamFrameHeader H;H.ProducerBootId=Wire.ProducerBootId;H.StreamId=Wire.StreamId;H.StreamGeneration=Wire.StreamGeneration;H.SourceRevision=Wire.SourceRevision;H.Sequence=Wire.Sequence;H.FrameId=Wire.FrameId;H.bHasSourceTime=Wire.bHasSourceTime;H.SourceTime=Wire.SourceTime;H.ClockEpoch=Wire.ClockEpoch;const auto Result=ApplyStreamFrame(Stream->Owner,H,Frame);if(Result.bAccepted)VisibleFrameLease=MoveTemp(PresentationLease);}
				}
			}
		}
		}
	}
	const FURLabRenderOwnerStatusResult OwnerStatus=Authority.Status();
	if (OwnerStatus.IsOk() && OwnerStatus.Status.bStale) RetireOwnerScope(OwnerScope);
}

TStatId UURLabViewerWorldSubsystem::GetStatId() const
{ RETURN_QUICK_DECLARE_CYCLE_STAT(UURLabViewerWorldSubsystem, STATGROUP_Tickables); }

void UURLabViewerWorldSubsystem::FinishMutation()
{
	bMutationInProgress=false;
	if (bShutdownRequested && !bShutdownInProgress) ShutdownNow();
}

void UURLabViewerWorldSubsystem::ShutdownWorkflow()
{
	check(IsInGameThread());
	if (bMutationInProgress) { bShutdownRequested=true; return; }
	ShutdownNow();
}

void UURLabViewerWorldSubsystem::ShutdownNow()
{
	if (bShutdownInProgress) return;
	TGuardValue<bool> Guard(bShutdownInProgress,true);
	bMutationInProgress=true; ON_SCOPE_EXIT { FinishMutation(); };
	bShutdownRequested=false;
	DetachBridge();
	FURLabRenderOwnerRetirementResult Begun=Authority.BeginShutdownRetirement();
	ClearStreamBinding(); OwnerScope.Reset();
	if (Begun.Token.IsValid()) Authority.FinishRetirement(Begun.Token);
	Source.Reset();
	if (UURLabRenderWorldSubsystem* Current=Coordinator.Get())
	{
		FString Error;
		if (Current->IsRegistrationCurrent(Producer) && !Current->ReleaseProducer(Producer,Error))
		{ bShutdownRequested=true; UE_LOG(LogURLab,Warning,TEXT("viewer producer release deferred: %s"),*Error); return; }
	}
	VisibleFrameLease.Reset();
	Producer={}; Coordinator.Reset(); bActive=false; bShutdownPermanent=true;
}

void UURLabViewerWorldSubsystem::Deinitialize()
{ ShutdownWorkflow(); Super::Deinitialize(); }
