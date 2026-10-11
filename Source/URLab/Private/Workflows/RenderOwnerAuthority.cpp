// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Workflows/RenderOwnerAuthority.h"

#include "HAL/PlatformTime.h"

#include <limits>

namespace
{
FString MintOpaqueToken()
{
	const FGuid Identity = FGuid::NewGuid();
	return Identity.IsValid() ? Identity.ToString(EGuidFormats::Digits) : FString();
}

template <typename ResultType>
ResultType OwnerFailure(EURLabRenderOwnerAuthorityError Error,
	FURLabRenderOwnerStatus Status, FString Message)
{
	ResultType Result;
	Result.Error = Error;
	Result.Status = MoveTemp(Status);
	Result.Message = MoveTemp(Message);
	return Result;
}
}

FURLabRenderOwnerClock::FURLabRenderOwnerClock(TFunction<double()> InReadSeconds)
	: ReadSeconds(MoveTemp(InReadSeconds))
{
}

TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe>
FURLabRenderOwnerClock::System()
{
	static TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> Clock =
		MakeShareable(new FURLabRenderOwnerClock(
			[]() { return FPlatformTime::Seconds(); }));
	return Clock;
}

TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe>
FURLabRenderOwnerClock::ForTest(TFunction<double()> ReadSeconds)
{
	return MakeShareable(new FURLabRenderOwnerClock(MoveTemp(ReadSeconds)));
}

double FURLabRenderOwnerClock::NowSeconds() const
{
	return ReadSeconds ? ReadSeconds() : std::numeric_limits<double>::quiet_NaN();
}

FURLabRenderOwnerSessionScope::FURLabRenderOwnerSessionScope(
	FString InExternalSessionLabel, FGuid InIdentity)
	: ExternalSessionLabel(MoveTemp(InExternalSessionLabel))
	, Identity(InIdentity)
{
}

FURLabRenderOwnerSessionScopePtr FURLabRenderOwnerSessionScope::Create(
	FString ExternalSessionLabel, FString& OutError)
{
	const FGuid Identity = FGuid::NewGuid();
	if (ExternalSessionLabel.IsEmpty() || !Identity.IsValid())
	{
		OutError = ExternalSessionLabel.IsEmpty()
			? TEXT("render-owner session label is empty")
			: TEXT("render-owner session identity could not be minted");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FURLabRenderOwnerSessionScope(
		MoveTemp(ExternalSessionLabel), Identity));
}

bool FURLabRenderOwnerProof::IsValid() const
{
	return AuthorityIdentity.IsValid() && Scope.IsValid() && ScopeIdentity.IsValid()
		&& AttachmentGeneration != 0 && !OpaqueToken.IsEmpty();
}

bool FURLabRenderOwnerRetirementToken::IsValid() const
{
	return AuthorityIdentity.IsValid() && ScopeIdentity.IsValid()
		&& AttachmentGeneration != 0 && !OwnerToken.IsEmpty()
		&& RetirementIdentity.IsValid();
}

FURLabRenderOwnerAuthority::FURLabRenderOwnerAuthority(
	TSharedRef<FURLabRenderOwnerClock, ESPMode::ThreadSafe> InClock)
	: Clock(MoveTemp(InClock))
	, AuthorityIdentity(FGuid::NewGuid())
{
}

bool FURLabRenderOwnerAuthority::ReadTimedNow(double& OutNow,
	EURLabRenderOwnerAuthorityError& OutError, FString& OutMessage)
{
	OutNow = Clock->NowSeconds();
	if (!FMath::IsFinite(OutNow))
	{
		OutError = EURLabRenderOwnerAuthorityError::InvalidClock;
		OutMessage = TEXT("render-owner monotonic clock is not finite");
		return false;
	}
	if (bHasClockFloor && OutNow < LastValidClockSeconds)
	{
		OutError = EURLabRenderOwnerAuthorityError::ClockMovedBackward;
		OutMessage = TEXT("render-owner monotonic clock moved backward");
		return false;
	}
	bHasClockFloor = true;
	LastValidClockSeconds = OutNow;
	OutError = EURLabRenderOwnerAuthorityError::None;
	OutMessage.Reset();
	return true;
}

bool FURLabRenderOwnerAuthority::IsExactScope(
	const FURLabRenderOwnerSessionScopePtr& Scope) const
{
	return Scope.IsValid() && ActiveScope.IsValid() && Scope == ActiveScope
		&& Scope->Identity == ActiveScope->Identity;
}

bool FURLabRenderOwnerAuthority::IsExactProof(
	const FURLabRenderOwnerProof& Proof) const
{
	return Proof.IsValid() && AuthorityIdentity.IsValid()
		&& Proof.AuthorityIdentity == AuthorityIdentity
		&& IsExactScope(Proof.Scope)
		&& Proof.ScopeIdentity == ActiveScope->Identity
		&& Proof.AttachmentGeneration == AttachmentGeneration
		&& Proof.OpaqueToken == ActiveOpaqueToken;
}

bool FURLabRenderOwnerAuthority::IsExpiredAt(double NowSeconds) const
{
	return Phase == ESlotPhase::Active && NowSeconds >= LeaseDeadlineSeconds;
}

FURLabRenderOwnerProof FURLabRenderOwnerAuthority::MakeProof() const
{
	FURLabRenderOwnerProof Proof;
	Proof.AuthorityIdentity = AuthorityIdentity;
	Proof.Scope = ActiveScope;
	Proof.ScopeIdentity = ActiveScope.IsValid() ? ActiveScope->Identity : FGuid();
	Proof.AttachmentGeneration = AttachmentGeneration;
	Proof.OpaqueToken = ActiveOpaqueToken;
	return Proof;
}

FURLabRenderOwnerStatus FURLabRenderOwnerAuthority::BuildStatusWithoutClock() const
{
	FURLabRenderOwnerStatus Result;
	Result.AttachmentGeneration = AttachmentGeneration;
	if (Phase == ESlotPhase::Available)
	{
		Result.bAvailable = !bShutdown;
		return Result;
	}
	Result.bAvailable = false;
	if (Phase == ESlotPhase::Retiring)
	{
		Result.bRetiring = true;
		return Result;
	}
	if (!ActiveScope.IsValid() || !ActiveScope->IsActive())
	{
		Result.bStale = true;
	}
	return Result;
}

FURLabRenderOwnerStatus FURLabRenderOwnerAuthority::BuildStatusAt(
	double NowSeconds) const
{
	FURLabRenderOwnerStatus Result = BuildStatusWithoutClock();
	if (Phase != ESlotPhase::Active || Result.bStale)
	{
		return Result;
	}
	if (IsExpiredAt(NowSeconds))
	{
		Result.bStale = true;
		return Result;
	}
	Result.bActive = true;
	Result.SecondsRemaining = FMath::Max(0.0, LeaseDeadlineSeconds - NowSeconds);
	return Result;
}

FURLabRenderOwnerStatusResult FURLabRenderOwnerAuthority::Status()
{
	check(IsInGameThread());
	FURLabRenderOwnerStatusResult Result;
	Result.Status = BuildStatusWithoutClock();
	if (Phase != ESlotPhase::Active || Result.Status.bStale)
	{
		return Result;
	}
	double Now = 0.0;
	if (!ReadTimedNow(Now, Result.Error, Result.Message))
	{
		return Result;
	}
	Result.Status = BuildStatusAt(Now);
	return Result;
}

FURLabRenderOwnerAttachResult FURLabRenderOwnerAuthority::Attach(
	const FURLabRenderOwnerSessionScopePtr& Scope, double TtlSeconds)
{
	check(IsInGameThread());
	if (bShutdown)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::ShuttingDown, BuildStatusWithoutClock(),
			TEXT("render-owner authority is shutting down"));
	}
	if (Phase == ESlotPhase::Retiring)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::Retiring, BuildStatusWithoutClock(),
			TEXT("render-owner slot is awaiting retirement completion"));
	}
	if (Phase == ESlotPhase::Active)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::Occupied, BuildStatusWithoutClock(),
			TEXT("render-owner slot is already occupied"));
	}
	if (!Scope.IsValid())
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::InvalidScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is invalid"));
	}
	if (!Scope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is inactive"));
	}
	double Now = 0.0;
	EURLabRenderOwnerAuthorityError ClockError;
	FString ClockMessage;
	if (!ReadTimedNow(Now, ClockError, ClockMessage))
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			ClockError, BuildStatusWithoutClock(), MoveTemp(ClockMessage));
	}
	if (!FMath::IsFinite(TtlSeconds) || TtlSeconds <= 0.0
		|| TtlSeconds > MaxLeaseSeconds)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::InvalidTtl, BuildStatusWithoutClock(),
			TEXT("render-owner lease TTL must be finite and in (0, 30] seconds"));
	}
	if (!AuthorityIdentity.IsValid() || AttachmentGeneration == MAX_uint64)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::CounterExhausted,
			BuildStatusWithoutClock(), TEXT("render-owner identity space is exhausted"));
	}
	const FString NewToken = MintOpaqueToken();
	const double NewDeadline = Now + TtlSeconds;
	if (NewToken.IsEmpty())
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::CounterExhausted,
			BuildStatusWithoutClock(), TEXT("render-owner identity space is exhausted"));
	}
	if (!FMath::IsFinite(NewDeadline) || NewDeadline <= Now)
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::InvalidClock,
			BuildStatusWithoutClock(),
			TEXT("render-owner lease deadline is not representable on this clock"));
	}
	if (!Scope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAttachResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope deactivated during attachment"));
	}
	++AttachmentGeneration;
	ActiveScope = Scope;
	ActiveOpaqueToken = NewToken;
	LeaseDeadlineSeconds = NewDeadline;
	ActiveRetirement = {};
	Phase = ESlotPhase::Active;

	FURLabRenderOwnerAttachResult Result;
	Result.Proof = MakeProof();
	Result.Status = BuildStatusAt(Now);
	return Result;
}

FURLabRenderOwnerAuthorizationResult FURLabRenderOwnerAuthority::Authorize(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 InAttachmentGeneration,
	const FString& OpaqueToken)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring)
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::Retiring, BuildStatusWithoutClock(),
			TEXT("render-owner slot is retiring"));
	}
	if (Phase != ESlotPhase::Active || !IsExactScope(Scope)
		|| InAttachmentGeneration != AttachmentGeneration
		|| OpaqueToken.IsEmpty() || OpaqueToken != ActiveOpaqueToken)
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::InvalidProof, BuildStatusWithoutClock(),
			TEXT("render-owner credentials do not match the active attachment"));
	}
	if (!Scope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is inactive"));
	}
	double Now = 0.0;
	EURLabRenderOwnerAuthorityError ClockError;
	FString ClockMessage;
	if (!ReadTimedNow(Now, ClockError, ClockMessage))
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			ClockError, BuildStatusWithoutClock(), MoveTemp(ClockMessage));
	}
	if (!Scope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope deactivated during authorization"));
	}
	if (IsExpiredAt(Now))
	{
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::Expired, BuildStatusAt(Now),
			TEXT("render-owner lease has expired"));
	}
	FURLabRenderOwnerAuthorizationResult Result;
	Result.Proof = MakeProof();
	Result.Status = BuildStatusAt(Now);
	return Result;
}

FURLabRenderOwnerAuthorizationResult FURLabRenderOwnerAuthority::AuthorizeForRelease(
	const FURLabRenderOwnerSessionScopePtr& Scope, uint64 InAttachmentGeneration,
	const FString& OpaqueToken)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring)
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::Retiring, BuildStatusWithoutClock(),
			TEXT("render-owner slot is retiring"));
	if (Phase != ESlotPhase::Active || !IsExactScope(Scope)
		|| InAttachmentGeneration != AttachmentGeneration || OpaqueToken.IsEmpty()
		|| OpaqueToken != ActiveOpaqueToken)
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::InvalidProof, BuildStatusWithoutClock(),
			TEXT("render-owner credentials do not match the active attachment"));
	if (!Scope->IsActive())
		return OwnerFailure<FURLabRenderOwnerAuthorizationResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is inactive"));
	FURLabRenderOwnerAuthorizationResult Result;
	Result.Proof = MakeProof();
	Result.Status = BuildStatusWithoutClock();
	return Result;
}

FURLabRenderOwnerAdmissionResult FURLabRenderOwnerAuthority::Validate(
	const FURLabRenderOwnerProof& Proof)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring)
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			EURLabRenderOwnerAuthorityError::Retiring, BuildStatusWithoutClock(),
			TEXT("render-owner slot is retiring"));
	}
	if (Phase != ESlotPhase::Active || !IsExactProof(Proof))
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			EURLabRenderOwnerAuthorityError::InvalidProof, BuildStatusWithoutClock(),
			TEXT("render-owner proof does not match the active attachment"));
	}
	if (!ActiveScope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is inactive"));
	}
	double Now = 0.0;
	EURLabRenderOwnerAuthorityError ClockError;
	FString ClockMessage;
	if (!ReadTimedNow(Now, ClockError, ClockMessage))
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			ClockError, BuildStatusWithoutClock(), MoveTemp(ClockMessage));
	}
	if (!ActiveScope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope deactivated during validation"));
	}
	if (IsExpiredAt(Now))
	{
		return OwnerFailure<FURLabRenderOwnerAdmissionResult>(
			EURLabRenderOwnerAuthorityError::Expired, BuildStatusAt(Now),
			TEXT("render-owner lease has expired"));
	}
	FURLabRenderOwnerAdmissionResult Result;
	Result.bAuthorized = true;
	Result.Status = BuildStatusAt(Now);
	return Result;
}

FURLabRenderOwnerLeaseResult FURLabRenderOwnerAuthority::Renew(
	const FURLabRenderOwnerProof& Proof, double TtlSeconds)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring)
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::Retiring, BuildStatusWithoutClock(),
			TEXT("render-owner slot is retiring"));
	}
	if (Phase != ESlotPhase::Active || !IsExactProof(Proof))
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InvalidProof, BuildStatusWithoutClock(),
			TEXT("render-owner proof does not match the active attachment"));
	}
	if (!ActiveScope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope is inactive"));
	}
	double Now = 0.0;
	EURLabRenderOwnerAuthorityError ClockError;
	FString ClockMessage;
	if (!ReadTimedNow(Now, ClockError, ClockMessage))
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			ClockError, BuildStatusWithoutClock(), MoveTemp(ClockMessage));
	}
	if (!ActiveScope->IsActive())
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InactiveScope, BuildStatusWithoutClock(),
			TEXT("render-owner session scope deactivated during renewal"));
	}
	if (IsExpiredAt(Now))
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::Expired, BuildStatusAt(Now),
			TEXT("render-owner lease has expired"));
	}
	if (!FMath::IsFinite(TtlSeconds) || TtlSeconds <= 0.0
		|| TtlSeconds > MaxLeaseSeconds)
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InvalidTtl, BuildStatusAt(Now),
			TEXT("render-owner lease TTL must be finite and in (0, 30] seconds"));
	}
	const double NewDeadline = Now + TtlSeconds;
	if (!FMath::IsFinite(NewDeadline) || NewDeadline <= Now)
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InvalidClock, BuildStatusAt(Now),
			TEXT("render-owner lease deadline is not representable on this clock"));
	}
	FURLabRenderOwnerLeaseResult Result;
	Result.bChanged = NewDeadline != LeaseDeadlineSeconds;
	LeaseDeadlineSeconds = NewDeadline;
	Result.Status = BuildStatusAt(Now);
	return Result;
}

FURLabRenderOwnerRetirementResult FURLabRenderOwnerAuthority::RetiringResult(
	bool bChanged) const
{
	FURLabRenderOwnerRetirementResult Result;
	Result.bChanged = bChanged;
	Result.Token = ActiveRetirement;
	Result.Status = BuildStatusWithoutClock();
	return Result;
}

FURLabRenderOwnerRetirementResult FURLabRenderOwnerAuthority::BeginRetirement()
{
	const FGuid RetirementIdentity = FGuid::NewGuid();
	if (!RetirementIdentity.IsValid())
	{
		return OwnerFailure<FURLabRenderOwnerRetirementResult>(
			EURLabRenderOwnerAuthorityError::CounterExhausted,
			BuildStatusWithoutClock(), TEXT("render-owner retirement identity could not be minted"));
	}
	ActiveRetirement = {};
	ActiveRetirement.AuthorityIdentity = AuthorityIdentity;
	ActiveRetirement.ScopeIdentity = ActiveScope->Identity;
	ActiveRetirement.AttachmentGeneration = AttachmentGeneration;
	ActiveRetirement.OwnerToken = ActiveOpaqueToken;
	ActiveRetirement.RetirementIdentity = RetirementIdentity;
	Phase = ESlotPhase::Retiring;
	return RetiringResult(true);
}

FURLabRenderOwnerRetirementResult FURLabRenderOwnerAuthority::BeginRelease(
	const FURLabRenderOwnerProof& Proof)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring && IsExactProof(Proof))
	{
		return RetiringResult(false);
	}
	if (Phase != ESlotPhase::Active || !IsExactProof(Proof))
	{
		return OwnerFailure<FURLabRenderOwnerRetirementResult>(
			EURLabRenderOwnerAuthorityError::InvalidProof, BuildStatusWithoutClock(),
			TEXT("render-owner proof does not match the retiring attachment"));
	}
	return BeginRetirement();
}

FURLabRenderOwnerRetirementResult
FURLabRenderOwnerAuthority::BeginLifecycleRetirement(
	const FURLabRenderOwnerSessionScopePtr& Scope)
{
	check(IsInGameThread());
	if (Phase == ESlotPhase::Retiring && IsExactScope(Scope))
	{
		return RetiringResult(false);
	}
	if (Phase != ESlotPhase::Active || !IsExactScope(Scope))
	{
		return OwnerFailure<FURLabRenderOwnerRetirementResult>(
			EURLabRenderOwnerAuthorityError::InvalidScope, BuildStatusWithoutClock(),
			TEXT("render-owner lifecycle scope does not match the active attachment"));
	}
	if (Scope->IsActive())
	{
		double Now = 0.0;
		EURLabRenderOwnerAuthorityError ClockError;
		FString ClockMessage;
		if (!ReadTimedNow(Now, ClockError, ClockMessage))
		{
			return OwnerFailure<FURLabRenderOwnerRetirementResult>(
				ClockError, BuildStatusWithoutClock(), MoveTemp(ClockMessage));
		}
		if (!IsExpiredAt(Now))
		{
			return OwnerFailure<FURLabRenderOwnerRetirementResult>(
				EURLabRenderOwnerAuthorityError::OwnerStillActive,
				BuildStatusAt(Now), TEXT("render-owner lifecycle scope is still active"));
		}
	}
	return BeginRetirement();
}

FURLabRenderOwnerRetirementResult
FURLabRenderOwnerAuthority::BeginShutdownRetirement()
{
	check(IsInGameThread());
	const bool bChangedShutdown = !bShutdown;
	bShutdown = true;
	if (Phase == ESlotPhase::Retiring)
	{
		return RetiringResult(bChangedShutdown);
	}
	if (Phase == ESlotPhase::Available)
	{
		FURLabRenderOwnerRetirementResult Result;
		Result.bChanged = bChangedShutdown;
		Result.Status = BuildStatusWithoutClock();
		return Result;
	}
	return BeginRetirement();
}

FURLabRenderOwnerLeaseResult FURLabRenderOwnerAuthority::FinishRetirement(
	const FURLabRenderOwnerRetirementToken& Token)
{
	check(IsInGameThread());
	if (Phase != ESlotPhase::Retiring || !Token.IsValid()
		|| Token.AuthorityIdentity != AuthorityIdentity
		|| !ActiveScope.IsValid() || Token.ScopeIdentity != ActiveScope->Identity
		|| Token.AttachmentGeneration != AttachmentGeneration
		|| Token.OwnerToken != ActiveOpaqueToken
		|| Token.RetirementIdentity != ActiveRetirement.RetirementIdentity)
	{
		return OwnerFailure<FURLabRenderOwnerLeaseResult>(
			EURLabRenderOwnerAuthorityError::InvalidRetirement,
			BuildStatusWithoutClock(), TEXT("render-owner retirement token is stale or foreign"));
	}
	Phase = ESlotPhase::Available;
	ActiveScope.Reset();
	ActiveOpaqueToken.Reset();
	LeaseDeadlineSeconds = 0.0;
	ActiveRetirement = {};
	FURLabRenderOwnerLeaseResult Result;
	Result.bChanged = true;
	Result.Status = BuildStatusWithoutClock();
	return Result;
}
