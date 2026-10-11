// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Runtime/MjSimulationControl.h"

#include "HAL/PlatformTime.h"

FMjSimulationControlClock::FMjSimulationControlClock(TFunction<double()> InReadSeconds)
	: ReadSeconds(MoveTemp(InReadSeconds))
{
}

TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> FMjSimulationControlClock::System()
{
	static TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock =
		MakeShareable(new FMjSimulationControlClock(
			[]() { return FPlatformTime::Seconds(); }));
	return Clock;
}

TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> FMjSimulationControlClock::ForTest(
	TFunction<double()> ReadSeconds)
{
	return MakeShareable(new FMjSimulationControlClock(MoveTemp(ReadSeconds)));
}

double FMjSimulationControlClock::NowSeconds() const
{
	return ReadSeconds ? ReadSeconds() : 0.0;
}

FMjSimulationControlSessionLifetime::FMjSimulationControlSessionLifetime(FString InId,
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> InClock, bool bInExpires)
	: Id(MoveTemp(InId)), Clock(MoveTemp(InClock)), bExpires(bInExpires)
{
	if (bExpires)
	{
		DeadlineSeconds.store(Clock->NowSeconds() + RemoteLifetimeSeconds,
			std::memory_order_release);
	}
}

TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>
FMjSimulationControlSessionLifetime::CreateRemote(FString Id,
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock, FString& OutError)
{
	if (Id.IsEmpty())
	{
		OutError = TEXT("control session id is empty");
		return nullptr;
	}
	const double Now = Clock->NowSeconds();
	if (!FMath::IsFinite(Now))
	{
		OutError = TEXT("control session clock is not finite");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FMjSimulationControlSessionLifetime(
		MoveTemp(Id), MoveTemp(Clock), true));
}

TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>
FMjSimulationControlSessionLifetime::CreateLocal(FString Id,
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock, FString& OutError)
{
	if (Id.IsEmpty())
	{
		OutError = TEXT("control session id is empty");
		return nullptr;
	}
	OutError.Reset();
	return MakeShareable(new FMjSimulationControlSessionLifetime(
		MoveTemp(Id), MoveTemp(Clock), false));
}

bool FMjSimulationControlSessionLifetime::UsesClock(
	const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe>& Other) const
{
	return &Clock.Get() == &Other.Get();
}

bool FMjSimulationControlSessionLifetime::IsActiveAt(double NowSeconds) const
{
	if (!FMath::IsFinite(NowSeconds)
		|| !bActive.load(std::memory_order_acquire)) return false;
	if (!bExpires) return true;
	if (NowSeconds < DeadlineSeconds.load(std::memory_order_acquire)) return true;
	bActive.store(false, std::memory_order_release);
	return false;
}

bool FMjSimulationControlSessionLifetime::Touch()
{
	const double Now = Clock->NowSeconds();
	if (!FMath::IsFinite(Now) || !IsActiveAt(Now)) return false;
	if (bExpires)
	{
		const double Proposed = Now + RemoteLifetimeSeconds;
		double Current = DeadlineSeconds.load(std::memory_order_acquire);
		while (Current < Proposed
			&& !DeadlineSeconds.compare_exchange_weak(Current, Proposed,
				std::memory_order_release, std::memory_order_acquire))
		{
		}
	}
	return bActive.load(std::memory_order_acquire);
}

TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>
FMjSimulationControlSessionLifetime::CreateSuccessorScope(
	TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> NewClock,
	FString& OutError) const
{
	const double OldNow = Clock->NowSeconds();
	const double NewNow = NewClock->NowSeconds();
	if (!FMath::IsFinite(OldNow) || !FMath::IsFinite(NewNow))
	{
		OutError = TEXT("control session clock is not finite");
		return nullptr;
	}
	if (!bExpires)
	{
		OutError.Reset();
		return MakeShareable(new FMjSimulationControlSessionLifetime(
			Id, MoveTemp(NewClock), false));
	}

	const double OldDeadline = DeadlineSeconds.load(std::memory_order_acquire);
	const double Remaining = OldDeadline - OldNow;
	if (!FMath::IsFinite(Remaining) || Remaining <= 0.0)
	{
		OutError = TEXT("control session has expired");
		return nullptr;
	}
	const bool bSameClock = &Clock.Get() == &NewClock.Get();
	const double NewDeadline = bSameClock ? OldDeadline : NewNow + Remaining;
	if (!FMath::IsFinite(NewDeadline))
	{
		OutError = TEXT("control session deadline is not finite");
		return nullptr;
	}
	TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> Successor =
		MakeShareable(new FMjSimulationControlSessionLifetime(Id, MoveTemp(NewClock), true));
	Successor->DeadlineSeconds.store(NewDeadline, std::memory_order_release);
	OutError.Reset();
	return Successor;
}

void FMjSimulationControlSessionLifetime::Deactivate()
{
	bActive.store(false, std::memory_order_release);
}
