// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Runtime/MjSimulationWorldSubsystem.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/PoseStreamServiceBinding.h"
#include "Bridge/CameraStreamServiceBinding.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Guid.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"
#include "MuJoCo/Runtime/MjSimulationSnapshot.h"
#include "MuJoCo/Render/MjLauncherFlags.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/CameraStreamController.h"
#include "Render/URLabRenderScene.h"
#include "Transport/PoseStreamProducer.h"
#include "Utils/URLabLogging.h"

struct FMjSimulationWorldServiceBinding
{
	TWeakObjectPtr<UURLabBridgeServer> Bridge;
	FURLabRpcDispatcher::FSimulationRuntimeRegistration Registration;
	FURLabRpcDispatcher::FPoseStreamServiceRegistration StreamRegistration;
	TSharedPtr<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> StreamBinding;
	TSharedPtr<FURLabPoseStreamProducer, ESPMode::ThreadSafe> StreamProducer;
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> InteractionOwner;
	FURLabRpcDispatcher::FCameraStreamServiceRegistration CameraRegistration;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> CameraBinding;
};

namespace
{
FMjSimulationWorldInstallResult Rejected(FString Error)
{
	FMjSimulationWorldInstallResult Result;
	Result.Error = MoveTemp(Error);
	return Result;
}

FMjSimulationWorldInstallResult WorldResult(const FMjSimulationInstallResult& RuntimeResult,
	const FString& SceneRevision)
{
	FMjSimulationWorldInstallResult Result;
	Result.bCommitted = RuntimeResult.bCommitted;
	Result.Generation = RuntimeResult.Generation;
	Result.ClockEpoch = RuntimeResult.ClockEpoch;
	Result.Source = RuntimeResult.Source;
	Result.Error = RuntimeResult.Error;
	if (Result.bCommitted) Result.SceneRevision = SceneRevision;
	return Result;
}
}

bool UMjSimulationWorldSubsystem::ActivateRuntime(FString& OutError)
{
	OutError.Reset();
	if (!IsInGameThread())
	{
		OutError = TEXT("simulation runtime must be activated on the game thread");
		return false;
	}
	if (bInstallInProgress || bShutdownInProgress || bShutdownRequested)
	{
		OutError = TEXT("simulation lifecycle cannot activate during install or shutdown");
		return false;
	}
	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		OutError = TEXT("simulation runtime requires a game world");
		return false;
	}
	if (Runtime.IsValid())
	{
		UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get();
		if (!Coordinator || !Coordinator->IsRegistrationCurrent(RenderRegistration))
		{
			OutError = TEXT("simulation render producer registration is no longer current");
			return false;
		}
		return true;
	}

	UURLabRenderWorldSubsystem* Coordinator =
		World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!Coordinator)
	{
		OutError = TEXT("world did not provide a neutral render coordinator");
		return false;
	}
	FURLabRenderProducerRegistration Registration;
	if (!Coordinator->RegisterProducer(Registration, OutError)) return false;
	Runtime = MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
	RenderCoordinator = Coordinator;
	RenderRegistration = Registration;
	ResetMonotonicBaseline();
	return true;
}

bool UMjSimulationWorldSubsystem::AttachBridge(FString& OutError)
{
	OutError.Reset();
	if (!IsInGameThread())
	{
		OutError = TEXT("simulation bridge must be attached on the game thread");
		return false;
	}
	if (bInstallInProgress || bShutdownInProgress || bShutdownRequested)
	{
		OutError = TEXT("simulation bridge cannot attach during install or shutdown");
		return false;
	}
	if (!Runtime.IsValid())
	{
		OutError = TEXT("simulation runtime is not active");
		return false;
	}
	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		OutError = TEXT("simulation bridge requires a game world");
		return false;
	}

	if (ServiceBinding.IsValid())
	{
		UURLabBridgeServer* ExistingBridge = ServiceBinding->Bridge.Get();
		FURLabRpcDispatcher* ExistingDispatcher = ExistingBridge
			? ExistingBridge->GetDispatcher() : nullptr;
		if (ExistingBridge && ExistingBridge->IsRunning() && ExistingDispatcher
			&& ExistingDispatcher->IsSimulationRuntimeRegistrationActive(
				ServiceBinding->Registration)
			&& (!ServiceBinding->StreamRegistration.IsValid()
				|| ExistingDispatcher->IsPoseStreamServiceRegistrationActive(
					ServiceBinding->StreamRegistration))
			&& (!ServiceBinding->CameraRegistration.IsValid()
				|| ExistingDispatcher->IsCameraStreamServiceRegistrationActive(
					ServiceBinding->CameraRegistration)))
		{
			return true;
		}
		// A stopped/replaced dispatcher cannot be dereferenced. Its opaque token is
		// intentionally useless to a replacement; retain the runtime and reattach.
		ServiceBinding.Reset();
	}

	UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::EnsureFor(World);
	FURLabRpcDispatcher* Dispatcher = Bridge ? Bridge->GetDispatcher() : nullptr;
	if (!Bridge || !Dispatcher)
	{
		OutError = TEXT("world bridge did not provide a running dispatcher");
		return false;
	}
	TSharedPtr<FMjSimulationWorldServiceBinding, ESPMode::ThreadSafe> Candidate =
		MakeShared<FMjSimulationWorldServiceBinding, ESPMode::ThreadSafe>();
	Candidate->Bridge = Bridge;
	if (!Dispatcher->AttachSimulationRuntime(Runtime.ToSharedRef(), Candidate->Registration, OutError))
	{
		return false;
	}
	const FURLabBridgeServerConfig& Config = Bridge->ResolveInstanceConfig();
	if (URLabLauncherFlags::ParseCaps().Has(EMjCapability::CameraFrames))
	{
		FURLabCameraStreamProducerConfig CameraConfig;
		CameraConfig.BindEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*Config.BindAddress, Config.CamBasePort);
		const FString DialHost = Config.BindAddress == TEXT("0.0.0.0")
			? FString(FPlatformProcess::ComputerName()) : Config.BindAddress;
		CameraConfig.AdvertisedEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*DialHost, Config.CamBasePort);
		CameraConfig.ShmRegion = Dispatcher->GetShmRegionForLocalProducer();
		Candidate->CameraBinding = FURLabCameraStreamServiceBinding::Create(
			*this, CameraConfig, OutError);
		if (!Candidate->CameraBinding || !Dispatcher->AttachCameraStreamService(
			Candidate->CameraBinding.ToSharedRef(), Candidate->CameraRegistration, OutError))
		{
			if (Candidate->CameraBinding) Candidate->CameraBinding->Deactivate();
			Dispatcher->DetachSimulationRuntime(Candidate->Registration);
			return false;
		}
	}
	if (Config.bBroadcastOnRenderBus)
	{
		FURLabPoseStreamProducerConfig ProducerConfig;
		ProducerConfig.BindEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*Config.BindAddress, Config.RenderBusPort);
		const FString DialHost = Config.BindAddress == TEXT("0.0.0.0")
			? FString(FPlatformProcess::ComputerName()) : Config.BindAddress;
		ProducerConfig.AdvertisedEndpoint = FString::Printf(TEXT("tcp://%s:%d"),
			*DialHost, Config.RenderBusPort);
		ProducerConfig.ShmRegion = Dispatcher->GetShmRegionForLocalProducer();
		TUniquePtr<FURLabPoseStreamProducer> Created =
			FURLabPoseStreamProducer::Create(ProducerConfig, OutError);
		if (!Created)
		{
			if (Candidate->CameraRegistration.IsValid())
				Dispatcher->DetachCameraStreamService(Candidate->CameraRegistration);
			Dispatcher->DetachSimulationRuntime(Candidate->Registration);
			return false;
		}
		Candidate->StreamProducer = MakeShareable(Created.Release());
		Candidate->InteractionOwner = MakeShared<FMjSimulationInteractionOwner,
			ESPMode::ThreadSafe>();
		Runtime->SetInteractionOwner(Candidate->InteractionOwner);
		Candidate->StreamBinding = MakeShared<FURLabPoseStreamServiceBinding,
			ESPMode::ThreadSafe>(Candidate->StreamProducer.ToSharedRef(),
			Candidate->InteractionOwner);
		if (!Dispatcher->AttachPoseStreamService(Candidate->StreamBinding.ToSharedRef(),
			Candidate->StreamRegistration, OutError))
		{
			Runtime->SetInteractionOwner(nullptr);
			Candidate->StreamBinding->Deactivate();
			Candidate->StreamProducer->Stop();
			if (Candidate->CameraRegistration.IsValid())
				Dispatcher->DetachCameraStreamService(Candidate->CameraRegistration);
			Dispatcher->DetachSimulationRuntime(Candidate->Registration);
			return false;
		}
	}
	ServiceBinding = MoveTemp(Candidate);
	return true;
}

void UMjSimulationWorldSubsystem::DetachBridge()
{
	if (ServiceBinding.IsValid())
	{
		// The editor bridge may outlive or replace this world's dispatcher. Calling
		// only through the weak bridge and using the opaque token makes either case safe.
		if (UURLabBridgeServer* Bridge = ServiceBinding->Bridge.Get())
		{
			if (FURLabRpcDispatcher* Dispatcher = Bridge->GetDispatcher())
			{
				if (ServiceBinding->StreamRegistration.IsValid())
					Dispatcher->DetachPoseStreamService(ServiceBinding->StreamRegistration);
				if (ServiceBinding->CameraRegistration.IsValid())
					Dispatcher->DetachCameraStreamService(ServiceBinding->CameraRegistration);
				Dispatcher->DetachSimulationRuntime(ServiceBinding->Registration);
			}
		}
		if (ServiceBinding->StreamBinding) ServiceBinding->StreamBinding->Deactivate();
		if (ServiceBinding->CameraBinding) ServiceBinding->CameraBinding->Deactivate();
		if (ServiceBinding->InteractionOwner) ServiceBinding->InteractionOwner->RevokeSource();
		if (Runtime) Runtime->SetInteractionOwner(nullptr);
		if (ServiceBinding->StreamProducer) ServiceBinding->StreamProducer->Stop();
		ServiceBinding.Reset();
	}
}

void UMjSimulationWorldSubsystem::ShutdownService()
{
	if (bInstallInProgress)
	{
		bShutdownRequested = true;
		return;
	}
	ShutdownServiceNow();
}

void UMjSimulationWorldSubsystem::ShutdownServiceNow()
{
	if (bShutdownInProgress) return;
	TGuardValue<bool> ShutdownGuard(bShutdownInProgress, true);
	bShutdownRequested = false;
	DetachBridge();
	if (UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get())
	{
		FString ReleaseError;
		if (Coordinator->IsRegistrationCurrent(RenderRegistration)
			&& !Coordinator->ReleaseProducer(RenderRegistration, ReleaseError))
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[URLabSimulation] neutral render producer release failed: %s"),
				*ReleaseError);
		}
	}
	RenderRegistration = {};
	RenderCoordinator.Reset();
	ActiveAdapter.Reset();
	ActiveModelGeneration = 0;
	LastPublishedStateRevision = 0;
	LastPublicationFailureRevision = 0;
	NextSceneFrameId = 1;
	LastAdvanceError = EMjSimulationClockError::None;
	Runtime.Reset();
	RequestedClockMode = EMjSimulationClockMode::Stepped;
	bHasObservedClockMode = false;
	LastObservedClockEpoch = 0;
	ResetMonotonicBaseline();
}

AURLabRenderScene* UMjSimulationWorldSubsystem::GetActiveScene() const
{
	UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get();
	return Coordinator ? Coordinator->ResolveActiveScene(RenderRegistration) : nullptr;
}

FString UMjSimulationWorldSubsystem::GetActiveSceneRevision() const
{
	UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get();
	return Coordinator
		? Coordinator->ResolveActiveSceneRevision(RenderRegistration) : FString();
}

bool UMjSimulationWorldSubsystem::ConfigureCameraStream(
	const FMjSimulationControlHandle& Control,
	FURLabCameraStreamController& Controller, const FString& CameraName,
	bool bEnabled, TOptional<double> RateHz,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	check(IsInGameThread());
	OutError.Reset();
	if (!Runtime || !RenderRegistration.IsValid())
	{
		OutError = TEXT("simulation camera stream requires an installed model"); return false;
	}
	if (Runtime->AuthorizeControl(Control, OutError) != EMjSimulationControlError::None)
		return false;
	if (Control.ModelGeneration != ActiveModelGeneration)
	{
		OutError = TEXT("simulation camera stream model generation is stale"); return false;
	}
	AURLabRenderScene* Scene = GetActiveScene();
	const FString Revision = GetActiveSceneRevision();
	if (!Scene || Revision.IsEmpty() || Scene->GetScene().SceneRevision != Revision)
	{
		OutError = TEXT("simulation render scene is unavailable"); return false;
	}
	return Controller.Configure(*Scene, Revision, CameraName, bEnabled,
		RateHz, OutDescriptor, OutError);
}

bool UMjSimulationWorldSubsystem::IsBridgeAttached() const
{
	if (!ServiceBinding.IsValid() || !Runtime.IsValid()) return false;
	UURLabBridgeServer* Bridge = ServiceBinding->Bridge.Get();
	FURLabRpcDispatcher* Dispatcher = Bridge ? Bridge->GetDispatcher() : nullptr;
	return Bridge && Bridge->IsRunning() && Dispatcher
		&& Dispatcher->IsSimulationRuntimeRegistrationActive(ServiceBinding->Registration);
}

bool UMjSimulationWorldSubsystem::ConfigureLocalClockMode(
	EMjSimulationClockMode Mode, FString& OutError)
{
	OutError.Reset();
	if (!IsInGameThread())
	{
		OutError = TEXT("local simulation clock must be configured on the game thread");
		return false;
	}
	if (bInstallInProgress || bShutdownInProgress || bShutdownRequested)
	{
		OutError = TEXT("local simulation clock cannot change during install or shutdown");
		return false;
	}
	if (!Runtime.IsValid())
	{
		OutError = TEXT("simulation runtime is not active");
		return false;
	}
	if (Mode != EMjSimulationClockMode::Stepped && Mode != EMjSimulationClockMode::FreeRun)
	{
		OutError = TEXT("local simulation clock mode is invalid");
		return false;
	}
	if (Runtime->HasModel())
	{
		const FMjSimulationClockChangeResult Changed = Runtime->ConfigureLocalClockMode(
			Mode, Runtime->GetModelGeneration());
		if (!Changed.IsOk())
		{
			OutError = Changed.Message.IsEmpty()
				? TEXT("runtime refused the local simulation clock mode") : Changed.Message;
			return false;
		}
	}
	RequestedClockMode = Mode;
	LastObservedClockMode = Mode;
	bHasObservedClockMode = true;
	LastAdvanceError = EMjSimulationClockError::None;
	ResetMonotonicBaseline();
	return true;
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::InstallSource(
	FMjModelSourcePayload&& Source, const FMjSimulationPresentation& Presentation)
{
	return InstallSourceInternal(MoveTemp(Source), Presentation, false);
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::ReplaceSource(
	FMjModelSourcePayload&& Source, const FMjSimulationPresentation& Presentation)
{
	return InstallSourceInternal(MoveTemp(Source), Presentation, true);
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::InstallCandidate(
	FMjOwnedModelCandidate&& Candidate, const FMjSimulationPresentation& Presentation)
{
	return InstallCandidateInternal(MoveTemp(Candidate), Presentation, false);
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::ReplaceCandidate(
	FMjOwnedModelCandidate&& Candidate, const FMjSimulationPresentation& Presentation)
{
	return InstallCandidateInternal(MoveTemp(Candidate), Presentation, true);
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::InstallSourceInternal(
	FMjModelSourcePayload&& Source, const FMjSimulationPresentation& Presentation,
	bool bReplacement)
{
	if (!IsInGameThread())
		return Rejected(TEXT("simulation source installation must run on the game thread"));
	if (bShutdownInProgress || bShutdownRequested)
		return Rejected(TEXT("simulation source installation cannot start during shutdown"));
	if (!Runtime.IsValid()) return Rejected(TEXT("simulation runtime is not active"));
	if (bReplacement != Runtime->HasModel())
	{
		return Rejected(bReplacement
			? TEXT("no simulation model is installed; use InstallSource for the initial source")
			: TEXT("a simulation model is already installed; use ReplaceSource for explicit replacement"));
	}

	FMjOwnedModelCandidate Candidate;
	FString Error;
	if (!FMjOwnedModelCandidate::FromBytes(Source.Kind, Source.PrimaryBytes, Source.Assets,
		Candidate, Error, Source.Label))
	{
		return Rejected(MoveTemp(Error));
	}
	return InstallCandidateInternal(MoveTemp(Candidate), Presentation, bReplacement);
}

FMjSimulationWorldInstallResult UMjSimulationWorldSubsystem::InstallCandidateInternal(
	FMjOwnedModelCandidate&& Candidate, const FMjSimulationPresentation& Presentation,
	bool bReplacement)
{
	if (!IsInGameThread())
		return Rejected(TEXT("simulation model installation must run on the game thread"));
	if (bShutdownInProgress || bShutdownRequested)
		return Rejected(TEXT("simulation model installation cannot start during shutdown"));
	if (!Runtime.IsValid()) return Rejected(TEXT("simulation runtime is not active"));
	if (bInstallInProgress) return Rejected(TEXT("a simulation model installation is already in progress"));
	if (bReplacement != Runtime->HasModel())
	{
		return Rejected(bReplacement
			? TEXT("no simulation model is installed; use InstallCandidate for the initial candidate")
			: TEXT("a simulation model is already installed; use ReplaceCandidate for explicit replacement"));
	}
	if (!Candidate.IsValid()) return Rejected(TEXT("candidate is not fully prepared"));
	bInstallInProgress = true;
	ON_SCOPE_EXIT
	{
		bInstallInProgress = false;
		if (bShutdownRequested) ShutdownServiceNow();
	};

	UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get();
	if (!Coordinator || !Coordinator->IsRegistrationCurrent(RenderRegistration))
	{
		return Rejected(TEXT("simulation render producer registration is no longer current"));
	}
	const FString Revision = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphensLower);
	TUniquePtr<FMjRenderSceneAdapter> StagedAdapter = MakeUnique<FMjRenderSceneAdapter>();
	FRenderScene StagedScene;
	FRenderSceneResources StagedResources;
	FRenderFrame StagedFrame;
	FString StageError;
	const bool bExtracted = Candidate.VisitPrepared(
		[&](const mjModel* Model, const mjData* Data)
		{
			return StagedAdapter->Extract(Model, Data, Revision,
				Presentation.SceneOptions.VisibleGroupMask, Presentation.CameraMaxHeight,
				Presentation.bIncludeUserCamera, &Presentation.CameraOverrides,
				StagedScene, StagedResources, StageError)
				&& StagedAdapter->FrameFromData(Data, /*FrameId=*/0, StagedFrame, StageError);
		});
	if (!bExtracted)
	{
		return Rejected(StageError.IsEmpty()
			? TEXT("candidate could not produce a neutral scene and initial frame")
			: MoveTemp(StageError));
	}

	FURLabRenderSceneCandidate SceneCandidate;
	const FString ExpectedRevision =
		Coordinator->ResolveActiveSceneRevision(RenderRegistration);
	FRenderFrame InstalledFrame = StagedFrame;
	FRenderSceneInstallPayload RenderPayload(MoveTemp(StagedScene),
		MoveTemp(StagedResources), MoveTemp(InstalledFrame));
	if (!Coordinator->StageScene(RenderRegistration, ExpectedRevision,
		MoveTemp(RenderPayload),
		Presentation.SceneOptions, SceneCandidate, StageError))
	{
		return Rejected(MoveTemp(StageError));
	}

	FMjSimulationInstallResult RuntimeResult;
	const FURLabRenderWorldCommitResult SceneResult = Coordinator->CommitCandidate(
		RenderRegistration, SceneCandidate, [&](FString& OutRelatedError)
		{
			RuntimeResult = Runtime->Install(MoveTemp(Candidate),
				[&](const FMjSimulationInstallResult& Proposed, FString& OutFinalizeError)
				{
					StagedFrame.Identity.ClockEpoch = Proposed.ClockEpoch;
					return Coordinator->ApplyCandidateFrame(RenderRegistration,
						SceneCandidate, StagedFrame, OutFinalizeError);
				});
			if (RuntimeResult.bCommitted)
			{
				// Publish simulation-owned adapter/counters before actor visibility or
				// retirement notifications can observe the newly committed runtime.
				ActiveAdapter = MoveTemp(StagedAdapter);
				ActiveModelGeneration = RuntimeResult.Generation;
				LastObservedClockEpoch = RuntimeResult.ClockEpoch;
				LastPublishedStateRevision = RuntimeResult.StateRevision;
				LastPublicationFailureRevision = 0;
				NextSceneFrameId = 1;
				return true;
			}
			OutRelatedError = RuntimeResult.Error.IsEmpty()
				? TEXT("simulation runtime refused the prepared model") : RuntimeResult.Error;
			return false;
		});
	if (!SceneResult.bCommitted)
	{
		FString DiscardError;
		Coordinator->DiscardCandidate(RenderRegistration, SceneCandidate, DiscardError);
		if (!RuntimeResult.bCommitted && !RuntimeResult.Error.IsEmpty())
			return WorldResult(RuntimeResult, FString());
		return Rejected(SceneResult.Error.IsEmpty()
			? TEXT("neutral render coordinator refused the prepared scene") : SceneResult.Error);
	}
	if (ServiceBinding && ServiceBinding->StreamProducer)
	{
		FURLabPoseStreamDescriptor Stream;
		FString StreamError;
		if (!ServiceBinding->StreamProducer->CommitSource(SceneResult.SceneRevision,
			Stream, StreamError))
		{
			if (ServiceBinding->InteractionOwner)
				ServiceBinding->InteractionOwner->RevokeSource();
			UE_LOG(LogURLabNet, Error,
				TEXT("[URLabSimulation] pose stream source commit failed: %s"), *StreamError);
		}
		else if (ServiceBinding->InteractionOwner
			&& !ServiceBinding->InteractionOwner->BindSource(Stream, StreamError))
		{
			ServiceBinding->InteractionOwner->RevokeSource();
			UE_LOG(LogURLabNet, Error,
				TEXT("[URLabSimulation] interaction source bind failed: %s"), *StreamError);
		}
	}

	// The callback has committed the native model and the coordinator has completed
	// its infallible scene publication before returning here.
	ResetMonotonicBaseline();
	FMjSimulationWorldInstallResult Result = WorldResult(RuntimeResult,
		SceneResult.SceneRevision);
	if (!bReplacement)
	{
		const FMjSimulationClockChangeResult Clock = Runtime->ConfigureLocalClockMode(
			RequestedClockMode, RuntimeResult.Generation);
		if (!Clock.IsOk())
		{
			Result.Error = FString::Printf(
				TEXT("model and scene committed, but local clock configuration failed: %s"),
				Clock.Message.IsEmpty() ? TEXT("runtime rejected the requested mode") : *Clock.Message);
		}
	}
	return Result;
}

void UMjSimulationWorldSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!Runtime.IsValid())
	{
		ResetMonotonicBaseline();
		return;
	}
	const FMjSimulationClockStatus Clock = Runtime->GetClockStatus();
	const uint64 ClockEpoch = Runtime->GetClockEpoch();
	if (!bHasObservedClockMode || Clock.Mode != LastObservedClockMode
		|| (LastObservedClockEpoch != 0 && ClockEpoch != LastObservedClockEpoch))
	{
		LastObservedClockMode = Clock.Mode;
		bHasObservedClockMode = true;
		LastAdvanceError = EMjSimulationClockError::None;
		ResetMonotonicBaseline();
	}
	LastObservedClockEpoch = ClockEpoch;

	const double Now = FPlatformTime::Seconds();
	double Elapsed = 0.0;
	if (bHasMonotonicBaseline && FMath::IsFinite(Now) && Now >= LastMonotonicSeconds)
	{
		Elapsed = Now - LastMonotonicSeconds;
	}
	LastMonotonicSeconds = Now;
	bHasMonotonicBaseline = FMath::IsFinite(Now);

	if (Elapsed > 0.0 && Clock.Mode == EMjSimulationClockMode::FreeRun)
	{
		const FMjSimulationAdvanceResult Advanced = Runtime->AdvanceFreeRun(Elapsed);
		if (!Advanced.IsOk())
		{
			if (LastAdvanceError != Advanced.Error)
			{
				UE_LOG(LogURLab, Warning, TEXT("[URLabSimulation] free-run advance failed: %s"),
					Advanced.Message.IsEmpty() ? TEXT("runtime rejected elapsed time") : *Advanced.Message);
			}
			LastAdvanceError = Advanced.Error;
		}
		else
		{
			LastAdvanceError = EMjSimulationClockError::None;
		}
	}
	PublishLatestSnapshot();
}

TStatId UMjSimulationWorldSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UMjSimulationWorldSubsystem, STATGROUP_Tickables);
}

void UMjSimulationWorldSubsystem::PublishLatestSnapshot()
{
	if (!Runtime.IsValid() || !ActiveAdapter || !Runtime->HasModel()) return;
	UURLabRenderWorldSubsystem* Coordinator = RenderCoordinator.Get();
	if (!Coordinator || !Coordinator->IsRegistrationCurrent(RenderRegistration)) return;
	const FString SceneRevision =
		Coordinator->ResolveActiveSceneRevision(RenderRegistration);
	if (SceneRevision.IsEmpty()) return;
	const uint64 AvailableRevision = Runtime->GetStateRevision();
	if (AvailableRevision == LastPublishedStateRevision) return;

	FMjSimulationCartesianSnapshot Snapshot;
	const EMjSimulationSnapshotError CaptureError = Runtime->CaptureCartesianSnapshot(Snapshot);
	if (CaptureError != EMjSimulationSnapshotError::None)
	{
		if (LastPublicationFailureRevision != AvailableRevision)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[URLabSimulation] Cartesian snapshot capture failed for state revision %llu (error %d)"),
				static_cast<unsigned long long>(AvailableRevision), static_cast<int32>(CaptureError));
		}
		LastPublicationFailureRevision = AvailableRevision;
		return;
	}
	if (Snapshot.Identity.StateRevision == LastPublishedStateRevision) return;

	FRenderFrame Frame;
	FString Error;
	if (!Runtime->CaptureRenderFrame(*ActiveAdapter, ActiveModelGeneration,
			NextSceneFrameId, Frame, Error))
	{
		if (LastPublicationFailureRevision != Snapshot.Identity.StateRevision)
		{
			UE_LOG(LogURLab, Warning, TEXT("[URLabSimulation] snapshot publication rejected: %s"),
				*Error);
		}
		LastPublicationFailureRevision = Snapshot.Identity.StateRevision;
		return;
	}
	const FURLabRenderFramePublicationResult Publication =
		Coordinator->PublishAuthoritativeFrame(RenderRegistration, SceneRevision, Frame);
	if (!Publication.bAccepted)
	{
		if (LastPublicationFailureRevision != Snapshot.Identity.StateRevision)
		{
			UE_LOG(LogURLab, Warning, TEXT("[URLabSimulation] neutral frame apply failed: %s"),
				*Publication.Error);
		}
		LastPublicationFailureRevision = Snapshot.Identity.StateRevision;
		return;
	}
	if (ServiceBinding && ServiceBinding->StreamProducer)
	{
		const int32 BodyCount = Snapshot.BodyPositions.Num() / 3;
		const int32 CameraCount = Snapshot.CameraPositions.Num() / 3;
		TArray<double> Pose;
		Pose.Reserve((BodyCount + CameraCount) * 7);
		auto AppendTransforms = [&Pose](const TArray<double>& Positions,
			const TArray<double>& Quaternions)
		{
			const int32 Count = Positions.Num() / 3;
			for (int32 Index = 0; Index < Count; ++Index)
			{
				Pose.Append(Positions.GetData() + Index * 3, 3);
				Pose.Append(Quaternions.GetData() + Index * 4, 4);
			}
		};
		AppendTransforms(Snapshot.BodyPositions, Snapshot.BodyQuaternions);
		AppendTransforms(Snapshot.CameraPositions, Snapshot.CameraQuaternions);
		FString StreamError;
		if (!ServiceBinding->StreamProducer->Publish(Frame.Identity.FrameId,
			Frame.Identity.SourceTime, Frame.Identity.bHasSourceTime,
			Frame.Identity.ClockEpoch, Pose, Frame, StreamError))
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("[URLabSimulation] pose stream publication rejected: %s"), *StreamError);
		}
	}
	LastPublishedStateRevision = Snapshot.Identity.StateRevision;
	LastPublicationFailureRevision = 0;
	++NextSceneFrameId;
}

void UMjSimulationWorldSubsystem::ResetMonotonicBaseline()
{
	LastMonotonicSeconds = 0.0;
	bHasMonotonicBaseline = false;
}

void UMjSimulationWorldSubsystem::Deinitialize()
{
	ShutdownService();
	Super::Deinitialize();
}
