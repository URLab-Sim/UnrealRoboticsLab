// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Runtime/MjSimulationPresentation.h"
#include "MuJoCo/Core/MjModelSourcePayload.h"
#include "MuJoCo/Runtime/MjSimulationClock.h"
#include "Render/RenderWorldSubsystem.h"
#include "Subsystems/WorldSubsystem.h"
#include "MjSimulationWorldSubsystem.generated.h"

class AURLabRenderScene;
class FURLabCameraStreamController;
class FMjSimulationRuntime;
struct FMjSimulationWorldServiceBinding;
struct FMjSimulationControlHandle;

/** Result of a coordinated runtime and neutral-scene installation. */
struct URLAB_API FMjSimulationWorldInstallResult
{
	bool bCommitted = false;
	uint64 Generation = 0;
	uint64 ClockEpoch = 0;
	FString SceneRevision;
	FMjModelSourceInfo Source;
	FString Error;
};

/**
 * Explicit world-scoped owner of one manager-free simulation service.
 * Merely creating a world or resolving this subsystem starts nothing.
 */
UCLASS()
class URLAB_API UMjSimulationWorldSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** Activate this world's sole runtime without requiring or starting RPC. */
	bool ActivateRuntime(FString& OutError);
	/** Attach the active runtime to this world's running bridge using an opaque binding. */
	bool AttachBridge(FString& OutError);
	/** Revoke the exact bridge binding while preserving runtime, clock and scene. */
	void DetachBridge();
	/** Exact detach followed by neutral-scene/runtime retirement. Idempotent. */
	void ShutdownService();
	/** Store/apply the local boot clock choice; model-free activation retains the intent. */
	bool ConfigureLocalClockMode(EMjSimulationClockMode Mode, FString& OutError);

	/** Initial installation. Refuses to overwrite an already installed model. */
	FMjSimulationWorldInstallResult InstallSource(FMjModelSourcePayload&& Source,
		const FMjSimulationPresentation& Presentation);
	FMjSimulationWorldInstallResult InstallCandidate(FMjOwnedModelCandidate&& Candidate,
		const FMjSimulationPresentation& Presentation);

	/** Deliberate transactional replacement. Refuses when no initial model exists. */
	FMjSimulationWorldInstallResult ReplaceSource(FMjModelSourcePayload&& Source,
		const FMjSimulationPresentation& Presentation);
	FMjSimulationWorldInstallResult ReplaceCandidate(FMjOwnedModelCandidate&& Candidate,
		const FMjSimulationPresentation& Presentation);

	bool IsRuntimeActive() const { return Runtime.IsValid(); }
	bool IsBridgeAttached() const;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> GetRuntime() const { return Runtime; }
	AURLabRenderScene* GetActiveScene() const;
	const FMjRenderSceneAdapter* GetActiveAdapter() const { return ActiveAdapter.Get(); }
	FString GetActiveSceneRevision() const;
	bool ConfigureCameraStream(const FMjSimulationControlHandle& Control,
		FURLabCameraStreamController& Controller, const FString& CameraName,
		bool bEnabled, TOptional<double> RateHz,
		struct FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);

	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual void Deinitialize() override;

private:
	FMjSimulationWorldInstallResult InstallSourceInternal(FMjModelSourcePayload&& Source,
		const FMjSimulationPresentation& Presentation, bool bReplacement);
	FMjSimulationWorldInstallResult InstallCandidateInternal(FMjOwnedModelCandidate&& Candidate,
		const FMjSimulationPresentation& Presentation, bool bReplacement);
	void ShutdownServiceNow();
	void PublishLatestSnapshot();
	void ResetMonotonicBaseline();

	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	TSharedPtr<FMjSimulationWorldServiceBinding, ESPMode::ThreadSafe> ServiceBinding;
	TUniquePtr<FMjRenderSceneAdapter> ActiveAdapter;
	TWeakObjectPtr<UURLabRenderWorldSubsystem> RenderCoordinator;
	FURLabRenderProducerRegistration RenderRegistration;
	EMjSimulationClockMode RequestedClockMode = EMjSimulationClockMode::Stepped;
	uint64 ActiveModelGeneration = 0;
	uint64 LastPublishedStateRevision = 0;
	uint64 LastPublicationFailureRevision = 0;
	uint64 NextSceneFrameId = 1;
	EMjSimulationClockError LastAdvanceError = EMjSimulationClockError::None;
	double LastMonotonicSeconds = 0.0;
	bool bHasMonotonicBaseline = false;
	EMjSimulationClockMode LastObservedClockMode = EMjSimulationClockMode::Stepped;
	bool bHasObservedClockMode = false;
	uint64 LastObservedClockEpoch = 0;
	bool bInstallInProgress = false;
	bool bShutdownRequested = false;
	bool bShutdownInProgress = false;
};
