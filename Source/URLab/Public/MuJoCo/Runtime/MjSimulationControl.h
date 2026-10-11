// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjModelRuntime.h"

#include <atomic>

enum class EMjSimulationControlSourceKind : uint8
{
	Local,
	Remote
};

enum class EMjSimulationControlError : uint8
{
	None,
	NoModel,
	InvalidSession,
	InactiveSession,
	InvalidToken,
	WrongModelGeneration,
	AlreadyClaimed,
	NotClaimed,
	Expired,
	InvalidTtl,
	WrongControlWidth,
	UnknownActuator,
	NonFinite,
	OutOfRange,
	InvalidStepCount,
	WrongClockMode,
	CounterExhausted,
	InvalidState,
	ObservationUnavailable
};

/**
 * Shared time basis for session lifetime and runtime authority. ReadSeconds must
 * return finite seconds from one monotonic domain; production uses one process singleton.
 */
class URLAB_API FMjSimulationControlClock
{
public:
	static TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> System();
	static TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> ForTest(
		TFunction<double()> ReadSeconds);

	double NowSeconds() const;

private:
	explicit FMjSimulationControlClock(TFunction<double()> InReadSeconds);
	TFunction<double()> ReadSeconds;
};

/**
 * Transport-independent session lifetime. A dispatcher deactivates this before
 * revoking authority, so a claim queued behind disconnect cannot become live.
 */
class URLAB_API FMjSimulationControlSessionLifetime
{
public:
	static constexpr double RemoteLifetimeSeconds = 300.0;

	static TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> CreateRemote(
		FString Id, TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock,
		FString& OutError);
	static TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> CreateLocal(
		FString Id, TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock,
		FString& OutError);

	const FString& GetId() const { return Id; }
	bool IsExpiring() const { return bExpires; }
	bool UsesClock(const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe>& Other) const;
	bool IsActiveAt(double NowSeconds) const;
	/** Per-request session activity. This never refreshes the control lease. */
	bool Touch();
	/**
	 * Creates a fresh active control scope without extending this session's
	 * remaining lifetime. The retired scope may already be deactivated.
	 */
	TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> CreateSuccessorScope(
		TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> NewClock,
		FString& OutError) const;
	void Deactivate();

private:
	FMjSimulationControlSessionLifetime(FString InId,
		TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> InClock, bool bInExpires);

	const FString Id;
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock;
	const bool bExpires;
	mutable std::atomic<bool> bActive{true};
	std::atomic<double> DeadlineSeconds{0.0};
};

struct URLAB_API FMjSimulationControlClaimRequest
{
	TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> Session;
	EMjSimulationControlSourceKind SourceKind = EMjSimulationControlSourceKind::Remote;
	double TtlSeconds = 30.0;
	uint64 ExpectedModelGeneration = 0;
};

/** Claim proof returned only to the controlling session. */
struct URLAB_API FMjSimulationControlHandle
{
	FString SessionId;
	FString ClaimToken;
	uint64 ModelGeneration = 0;

	bool IsValid() const
	{
		return !SessionId.IsEmpty() && !ClaimToken.IsEmpty() && ModelGeneration != 0;
	}
};

/** Reader-safe authority metadata. It deliberately omits session and claim token. */
struct URLAB_API FMjSimulationControlStatus
{
	bool bClaimed = false;
	uint64 ModelGeneration = 0;
	EMjSimulationControlSourceKind SourceKind = EMjSimulationControlSourceKind::Remote;
	double TtlSeconds = 0.0;
	bool bExpires = false;
	double SecondsRemaining = 0.0;
	uint64 ControlRevision = 0;
};

struct URLAB_API FMjSimulationControlClaimResult
{
	EMjSimulationControlError Error = EMjSimulationControlError::None;
	FMjSimulationControlHandle Handle;
	FMjSimulationControlStatus Status;
	FString Message;

	bool IsOk() const { return Error == EMjSimulationControlError::None; }
};

struct URLAB_API FMjSimulationControlLeaseResult
{
	EMjSimulationControlError Error = EMjSimulationControlError::None;
	bool bChanged = false;
	FMjSimulationControlStatus Status;
	FString Message;

	bool IsOk() const { return Error == EMjSimulationControlError::None; }
};

/** Atomic mutation outcome, including the observation sampled before unlocking. */
struct URLAB_API FMjSimulationMutationResult
{
	bool bCommitted = false;
	EMjSimulationControlError Error = EMjSimulationControlError::None;
	EMjModelStateError StateError = EMjModelStateError::None;
	FMjModelObservation Observation;
	FMjSimulationControlStatus Status;
	FString Message;

	bool IsOk() const { return bCommitted && Error == EMjSimulationControlError::None; }
};
