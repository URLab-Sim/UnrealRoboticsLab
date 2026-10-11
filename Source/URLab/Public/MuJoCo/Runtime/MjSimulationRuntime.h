// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "MuJoCo/Core/MjModelRuntime.h"
#include "MuJoCo/Runtime/MjSimulationClock.h"
#include "MuJoCo/Runtime/MjSimulationControl.h"
#include "MuJoCo/Runtime/MjSimulationSnapshot.h"
#include "Workflows/InteractionInbox.h"

#include "MuJoCo/Core/MjOwnedModel.h"

class FMjRenderSceneAdapter;
struct FRenderFrame;

struct URLAB_API FMjSimulationInstallResult
{
	bool bCommitted = false;
	uint64 Generation = 0;
	uint64 ClockEpoch = 0;
	uint64 StateRevision = 0;
	FMjModelIdentity Identity;
	FMjModelSourceInfo Source;
	FString Error;
};

class FMjSimulationInteractionOwner;

/**
 * Thread-safe, manager-free owner of one installed MuJoCo simulation state.
 *
 * Returned descriptors, observations, states and source information own their
 * memory. No model/data pointer escapes the runtime lock. This bounded runtime
 * integrates only through an explicit Step call and has no worker or callbacks.
 */
class URLAB_API FMjSimulationRuntime
{
public:
	FMjSimulationRuntime();
	explicit FMjSimulationRuntime(
		TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> InControlClock);
	~FMjSimulationRuntime();

	FMjSimulationRuntime(const FMjSimulationRuntime&) = delete;
	FMjSimulationRuntime& operator=(const FMjSimulationRuntime&) = delete;

	FMjSimulationInstallResult Install(FMjOwnedModelCandidate&& Candidate);
	/**
	 * Finalize an already-prepared external transaction against the exact identity
	 * this install will receive. Finalize runs synchronously under the runtime lock,
	 * after all candidate allocation and validation and before counters or installed
	 * pointers change. It receives no native pointers. Returning false preserves the
	 * current runtime and its counters.
	 *
	 * Finalize must not call back into this runtime, publish the staged replacement,
	 * or destroy the currently published external state. Its intended use is a short,
	 * failure-aware identity stamp on hidden, fully staged resources.
	 */
	FMjSimulationInstallResult Install(FMjOwnedModelCandidate&& Candidate,
		TFunctionRef<bool(const FMjSimulationInstallResult& Proposed, FString& OutError)> Finalize);

	bool HasModel() const;
	bool GetSourceInfo(FMjModelSourceInfo& OutSource) const;
	bool GetDescriptor(FMjModelDescriptor& OutDescriptor) const;
	bool GetObservation(FMjModelObservation& OutObservation) const;
	EMjModelStateError CaptureState(uint32 StateMask, FMjModelState& OutState) const;
	FMjSimulationControlStatus GetControlStatus() const;
	FMjSimulationClockStatus GetClockStatus() const;
	FMjSimulationClockChangeResult ConfigureLocalClockMode(
		EMjSimulationClockMode Mode, uint64 ExpectedModelGeneration);
	FMjSimulationClockChangeResult AuthorizedSetClockMode(
		const FMjSimulationControlHandle& Handle, EMjSimulationClockMode Mode);
	FMjSimulationAdvanceResult AdvanceFreeRun(double ElapsedSeconds);
	EMjSimulationSnapshotError CaptureCartesianSnapshot(
		FMjSimulationCartesianSnapshot& OutSnapshot) const;
	/** Adapter-only extraction while the live mjData remains protected by the runtime lock. */
	bool CaptureRenderFrame(FMjRenderSceneAdapter& Adapter, uint64 ExpectedGeneration,
		uint64 FrameId, FRenderFrame& OutFrame, FString& OutError) const;
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> GetControlClock() const
	{
		return ControlClock;
	}

	FMjSimulationControlClaimResult ClaimControl(
		const FMjSimulationControlClaimRequest& Request);
	FMjSimulationControlLeaseResult RenewControl(
		const FMjSimulationControlHandle& Handle, double TtlSeconds);
	FMjSimulationControlLeaseResult ReleaseControl(
		const FMjSimulationControlHandle& Handle);
	/** Validate exact live control credentials without renewing or mutating them. */
	EMjSimulationControlError AuthorizeControl(
		const FMjSimulationControlHandle& Handle, FString& OutMessage) const;
	/** Atomically validates control and stamps the opaque interaction context. */
	bool AuthorizeInteractionContext(const FMjSimulationControlHandle& Handle,
		const FURLabInteractionSourceIdentity& Source,
		FURLabInteractionOwnerContext& OutContext, FString& OutMessage) const;
	/** Installs or removes the one optional owner-thread interaction adapter. */
	void SetInteractionOwner(
		TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner);
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>
		GetInteractionOwner() const;
	FMjSimulationControlLeaseResult RevokeControlForSession(
		const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& Session);

	FMjSimulationMutationResult SetControlVector(
		const FMjSimulationControlHandle& Handle, TConstArrayView<double> Values);
	FMjSimulationMutationResult SetNamedActuatorControls(
		const FMjSimulationControlHandle& Handle,
		const TMap<FString, TArray<double>>& ValuesByActuator);
	FMjSimulationMutationResult AuthorizedStep(
		const FMjSimulationControlHandle& Handle, uint32 Count);
	/** Refreshes derived MuJoCo state without integration or clock-counter changes. */
	FMjSimulationMutationResult AuthorizedForward(
		const FMjSimulationControlHandle& Handle);
	FMjSimulationMutationResult AuthorizedReset(
		const FMjSimulationControlHandle& Handle);
	FMjSimulationMutationResult AuthorizedApplyState(
		const FMjSimulationControlHandle& Handle, const FMjModelState& State);

	static constexpr double DefaultRemoteControlTtlSeconds = 30.0;
	static constexpr double MaxRemoteControlTtlSeconds = 300.0;
	static constexpr uint32 MaxAuthorizedStepCount = 1024;
	static constexpr uint32 MaxFreeRunStepsPerAdvance = 64;
	static constexpr double MaxFreeRunRetainedDebtSeconds = 0.25;

	/** Applies state without forward/integration, then advances epoch and resets sequence. */
	EMjModelStateError ApplyState(const FMjModelState& State);
	/** Resets data, refreshes derived state, advances epoch, and resets sequence. */
	bool Reset(FString& OutError);
	/** Performs exactly Count mj_step calls under the runtime lock. */
	bool Step(uint32 Count, FString& OutError);

	uint64 GetModelGeneration() const;
	uint64 GetClockEpoch() const;
	uint64 GetStepSequence() const;
	uint64 GetStateRevision() const;

private:
	struct FInstalledModel;
	double ControlNowLocked() const;
	bool ExpireControlLocked(FInstalledModel& Model, double Now) const;
	FMjSimulationControlStatus ControlStatusLocked(FInstalledModel& Model, double Now) const;
	EMjSimulationControlError ValidateControlLocked(FInstalledModel& Model,
		const FMjSimulationControlHandle& Handle, double Now, FString& OutMessage) const;
	bool ValidateControlTtl(double TtlSeconds, EMjSimulationControlSourceKind SourceKind,
		FString& OutMessage) const;
	bool CaptureMutationObservationLocked(FInstalledModel& Model,
		double Now, FMjSimulationMutationResult& OutResult) const;
	EMjModelStateError ApplyStateLocked(FInstalledModel& Model, const FMjModelState& State);
	bool ResetLocked(FInstalledModel& Model, FString& OutError);
	bool StepLocked(FInstalledModel& Model, uint32 Count, FString& OutError);
	FMjSimulationClockStatus ClockStatusLocked(const FInstalledModel* Model) const;
	bool IsValidClockMode(EMjSimulationClockMode Mode) const;
	bool CanAdvanceStateRevisionLocked(FString& OutError) const;
	void CommitStateRevisionLocked(FInstalledModel& Model);
	void ClearClockDebtLocked(FInstalledModel& Model);
	void RefreshControlLocked(FInstalledModel& Model, double Now);
	FURLabInteractionControlIdentity InteractionControlLocked(
		const FInstalledModel& Model) const;
	mutable FCriticalSection Mutex;
	TUniquePtr<FInstalledModel> Installed;
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> ControlClock;
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> InteractionOwner;
	uint64 NextGeneration = 1;
	uint64 NextClockEpoch = 1;
	uint64 NextStateRevision = 1;
	EMjSimulationClockMode ClockMode = EMjSimulationClockMode::Stepped;
};
