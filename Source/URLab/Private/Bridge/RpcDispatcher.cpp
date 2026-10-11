// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Bridge/RpcDispatcher.h"
#include "Containers/Ticker.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Bridge/BridgeServer.h"
#include "Wire/UrlabWire.gen.h"
#include "HAL/PlatformTime.h"  // session LRU stamps
#include "Misc/Guid.h"
#include "Utils/URLabLogging.h"

FURLabRpcDispatcher::FURLabRpcDispatcher()
	: BootId(FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower())
	, RequestLedger(MakeShared<FURLabRequestLedger, ESPMode::ThreadSafe>())
	, SimulationCommandQueue(MakeShared<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>())
	, RenderCommandQueue(MakeShared<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>())
	, PayloadCommandQueue(MakeShared<FURLabSerialCommandQueue, ESPMode::ThreadSafe>())
	, PayloadStore(MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(BootId))
	, SessionClockOverrideForTest(MakeShared<std::atomic<double>, ESPMode::ThreadSafe>(-1.0))
{
	const FURLabShmExactStatus ShmStatus =
		FURLabShmExactRegion::CreateForBoot(BootId, ShmRegion);
	if (!ShmStatus)
	{
		UE_LOG(LogURLabNet, Log, TEXT("Protocol-v2 exact SHM data unavailable: %s"),
			*ShmStatus.Message);
	}
}
FURLabRpcDispatcher::~FURLabRpcDispatcher()
{
	Shutdown();
}

bool FURLabRpcDispatcher::AttachSimulationRuntime(
	TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime,
	FSimulationRuntimeRegistration& OutRegistration, FString& OutError)
{
	FRetiredSessionScopes ClosedScopes;
	{
		FScopeLock Lock(&DispatchMutex);
		if (bDraining.load(std::memory_order_acquire))
		{
			OutError = TEXT("cannot attach a simulation runtime while the dispatcher is draining");
			return false;
		}
		if (SimulationRuntime.IsValid())
		{
			OutError = TEXT("a simulation runtime is already attached");
			return false;
		}
		if (NextSimulationRegistration == 0)
		{
			OutError = TEXT("simulation runtime registration tokens are exhausted");
			return false;
		}
		const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> RuntimeClock =
			Runtime->GetControlClock();
		for (int32 Index = ActiveSessions.Num() - 1; Index >= 0; --Index)
		{
			FSession& Session = ActiveSessions[Index];
			if (Session.ControlSession->UsesClock(RuntimeClock)) continue;
			Session.ControlSession->Deactivate();
			FString ScopeError;
			const TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> Successor =
				Session.ControlSession->CreateSuccessorScope(RuntimeClock, ScopeError);
			if (!Successor)
			{
				DeactivateSessionLocked(Session, ClosedScopes);
				ActiveSessions.RemoveAt(Index);
				continue;
			}
			Session.ControlSession = Successor.ToSharedRef();
		}

		FSimulationRuntimeRegistration Registration;
		Registration.DispatcherIdentity = SimulationRegistrationDispatcherId;
		Registration.Serial = NextSimulationRegistration++;
		SimulationRuntime = Runtime;
		ActiveSimulationRegistration = Registration.Serial;
		OutRegistration = Registration;
		OutError.Reset();
	}
	RevokeSessionScopes(Runtime, PayloadStore, ClosedScopes);
	return true;
}

bool FURLabRpcDispatcher::AttachRenderService(
	TSharedRef<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding,
	FRenderServiceRegistration& OutRegistration, FString& OutError)
{
	FScopeLock Lock(&DispatchMutex);
	if (bDraining.load(std::memory_order_acquire))
	{
		OutError = TEXT("cannot attach a render service while the dispatcher is draining");
		return false;
	}
	if (RenderService)
	{
		OutError = TEXT("a render service is already attached");
		return false;
	}
	if (!Binding->IsActive() || NextRenderRegistration == 0)
	{
		OutError = TEXT("render service binding is inactive or tokens are exhausted");
		return false;
	}
	FRenderServiceRegistration Registration;
	Registration.DispatcherIdentity = RenderRegistrationDispatcherId;
	Registration.Serial = NextRenderRegistration++;
	RenderService = Binding;
	ActiveRenderRegistration = Registration.Serial;
	OutRegistration = Registration;
	OutError.Reset();
	return true;
}

bool FURLabRpcDispatcher::IsRenderServiceRegistrationActive(
	FRenderServiceRegistration Registration) const
{
	FScopeLock Lock(&DispatchMutex);
	return Registration.IsValid()
		&& Registration.DispatcherIdentity == RenderRegistrationDispatcherId
		&& Registration.Serial == ActiveRenderRegistration && RenderService
		&& RenderService->IsActive();
}

bool FURLabRpcDispatcher::DetachRenderService(FRenderServiceRegistration Registration)
{
	TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Retired;
	TArray<FURLabRenderOwnerSessionScopePtr> Scopes;
	{
		FScopeLock Lock(&DispatchMutex);
		if (!Registration.IsValid()
			|| Registration.DispatcherIdentity != RenderRegistrationDispatcherId
			|| Registration.Serial != ActiveRenderRegistration || !RenderService)
			return false;
		Retired = MoveTemp(RenderService);
		ActiveRenderRegistration = 0;
		Retired->Deactivate();
		for (FSession& Session : ActiveSessions)
		{
			if (!Session.RenderOwnerScope) continue;
			Scopes.Add(Session.RenderOwnerScope);
			Session.RenderOwnerScope->Deactivate();
			FString ScopeError;
			Session.RenderOwnerScope = FURLabRenderOwnerSessionScope::Create(
				Session.Id, ScopeError);
			if (!Session.RenderOwnerScope)
			{
				// A session without an exact render scope cannot issue render commands.
				UE_LOG(LogURLabNet, Error, TEXT("Failed to replace render session scope: %s"),
					*ScopeError);
			}
		}
	}
	FRetiredSessionScopes Retirement;
	for (const FURLabRenderOwnerSessionScopePtr& Scope : Scopes)
	{
		FRetiredSessionScopes::FRender& Entry = Retirement.Render.AddDefaulted_GetRef();
		Entry.Binding = Retired;
		Entry.Scope = Scope;
	}
	RevokeSessionScopes(nullptr, PayloadStore, Retirement);
	return true;
}

bool FURLabRpcDispatcher::AttachViewerService(
	TSharedRef<FURLabViewerServiceBinding, ESPMode::ThreadSafe> Binding,
	FViewerServiceRegistration& OutRegistration, FString& OutError)
{
	FScopeLock Lock(&DispatchMutex);
	if (bDraining.load(std::memory_order_acquire) || ViewerService || !Binding->IsActive()
		|| NextViewerRegistration == 0)
	{
		OutError = TEXT("viewer service is unavailable or already attached");
		return false;
	}
	FViewerServiceRegistration Registration;
	Registration.DispatcherIdentity = ViewerRegistrationDispatcherId;
	Registration.Serial = NextViewerRegistration++;
	ViewerService = Binding; ActiveViewerRegistration = Registration.Serial;
	OutRegistration = Registration; OutError.Reset(); return true;
}

bool FURLabRpcDispatcher::DetachViewerService(FViewerServiceRegistration Registration)
{
	TSharedPtr<FURLabViewerServiceBinding, ESPMode::ThreadSafe> Retired;
	FRetiredSessionScopes Retirement;
	{
		FScopeLock Lock(&DispatchMutex);
		if (!Registration.IsValid() || Registration.DispatcherIdentity != ViewerRegistrationDispatcherId
			|| Registration.Serial != ActiveViewerRegistration || !ViewerService) return false;
		Retired = MoveTemp(ViewerService); ActiveViewerRegistration = 0; Retired->Deactivate();
		for (FSession& Session : ActiveSessions)
		{
			if (!Session.ViewerOwnerScope) continue;
			Session.ViewerOwnerScope->Deactivate();
			auto& Entry = Retirement.Viewer.AddDefaulted_GetRef();
			Entry.Binding = Retired; Entry.Scope = Session.ViewerOwnerScope;
			FString Error; Session.ViewerOwnerScope = FURLabRenderOwnerSessionScope::Create(Session.Id, Error);
		}
	}
	RevokeSessionScopes(nullptr, PayloadStore, Retirement); return true;
}

bool FURLabRpcDispatcher::IsViewerServiceRegistrationActive(
	FViewerServiceRegistration Registration) const
{
	FScopeLock Lock(&DispatchMutex);
	return Registration.IsValid()
		&& Registration.DispatcherIdentity == ViewerRegistrationDispatcherId
		&& Registration.Serial == ActiveViewerRegistration && ViewerService
		&& ViewerService->IsActive();
}

bool FURLabRpcDispatcher::AttachPoseStreamService(
	TSharedRef<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> Binding,
	FPoseStreamServiceRegistration& OutRegistration, FString& OutError)
{
	FScopeLock Lock(&DispatchMutex);
	if (bDraining.load(std::memory_order_acquire) || PoseStreamService
		|| !Binding->IsActive() || NextPoseStreamRegistration == 0)
	{
		OutError = TEXT("pose stream service is unavailable or already attached");
		return false;
	}
	OutRegistration.DispatcherIdentity = PoseStreamRegistrationDispatcherId;
	OutRegistration.Serial = NextPoseStreamRegistration++;
	PoseStreamService = Binding;
	ActivePoseStreamRegistration = OutRegistration.Serial;
	OutError.Reset();
	return true;
}

bool FURLabRpcDispatcher::DetachPoseStreamService(FPoseStreamServiceRegistration Registration)
{
	TSharedPtr<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> Retired;
	{
		FScopeLock Lock(&DispatchMutex);
		if (!Registration.IsValid()
			|| Registration.DispatcherIdentity != PoseStreamRegistrationDispatcherId
			|| Registration.Serial != ActivePoseStreamRegistration || !PoseStreamService)
			return false;
		Retired = MoveTemp(PoseStreamService);
		ActivePoseStreamRegistration = 0;
		Retired->Deactivate();
	}
	return true;
}

bool FURLabRpcDispatcher::IsPoseStreamServiceRegistrationActive(
	FPoseStreamServiceRegistration Registration) const
{
	FScopeLock Lock(&DispatchMutex);
	return Registration.IsValid()
		&& Registration.DispatcherIdentity == PoseStreamRegistrationDispatcherId
		&& Registration.Serial == ActivePoseStreamRegistration && PoseStreamService
		&& PoseStreamService->IsActive();
}

bool FURLabRpcDispatcher::AttachCameraStreamService(
	TSharedRef<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Binding,
	FCameraStreamServiceRegistration& OutRegistration, FString& OutError)
{
	FScopeLock Lock(&DispatchMutex);
	if (bDraining.load(std::memory_order_acquire) || CameraStreamService
		|| !Binding->IsActive() || NextCameraStreamRegistration == 0)
	{
		OutError = TEXT("camera stream service is unavailable or already attached"); return false;
	}
	OutRegistration.DispatcherIdentity = CameraStreamRegistrationDispatcherId;
	OutRegistration.Serial = NextCameraStreamRegistration++;
	CameraStreamService = Binding;
	ActiveCameraStreamRegistration = OutRegistration.Serial;
	OutError.Reset(); return true;
}

bool FURLabRpcDispatcher::DetachCameraStreamService(FCameraStreamServiceRegistration Registration)
{
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Retired;
	{
		FScopeLock Lock(&DispatchMutex);
		if (!Registration.IsValid()
			|| Registration.DispatcherIdentity != CameraStreamRegistrationDispatcherId
			|| Registration.Serial != ActiveCameraStreamRegistration || !CameraStreamService)
			return false;
		Retired = MoveTemp(CameraStreamService);
		ActiveCameraStreamRegistration = 0;
	}
	Retired->Deactivate();
	return true;
}

bool FURLabRpcDispatcher::IsCameraStreamServiceRegistrationActive(
	FCameraStreamServiceRegistration Registration) const
{
	FScopeLock Lock(&DispatchMutex);
	return Registration.IsValid()
		&& Registration.DispatcherIdentity == CameraStreamRegistrationDispatcherId
		&& Registration.Serial == ActiveCameraStreamRegistration && CameraStreamService
		&& CameraStreamService->IsActive();
}

bool FURLabRpcDispatcher::DetachSimulationRuntime(
	FSimulationRuntimeRegistration Registration)
{
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Retired;
	TArray<TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>> RetiredScopes;
	FRetiredSessionScopes ClosedScopes;
	{
		FScopeLock Lock(&DispatchMutex);
		if (!Registration.IsValid() ||
			Registration.DispatcherIdentity != SimulationRegistrationDispatcherId ||
			Registration.Serial != ActiveSimulationRegistration || !SimulationRuntime.IsValid())
		{
			return false;
		}
		Retired = MoveTemp(SimulationRuntime);
		ActiveSimulationRegistration = 0;
		for (int32 Index = ActiveSessions.Num() - 1; Index >= 0; --Index)
		{
			FSession& Session = ActiveSessions[Index];
			RetiredScopes.Add(Session.ControlSession);
			Session.ControlSession->Deactivate();
			FString ScopeError;
			const TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> Successor =
				Session.ControlSession->CreateSuccessorScope(Retired->GetControlClock(), ScopeError);
			if (!Successor)
			{
				DeactivateSessionLocked(Session, ClosedScopes);
				ActiveSessions.RemoveAt(Index);
				continue;
			}
			Session.ControlSession = Successor.ToSharedRef();
		}
	}
	RevokeControlSessions(Retired, RetiredScopes);
	RevokeSessionScopes(nullptr, PayloadStore, ClosedScopes);
	// Runtime destruction and revocation can touch native state; both stay outside
	// the short admission lock. Accepted work holds its exact shared reference.
	Retired.Reset();
	return true;
}

bool FURLabRpcDispatcher::IsSimulationRuntimeRegistrationActive(
	FSimulationRuntimeRegistration Registration) const
{
	FScopeLock Lock(&DispatchMutex);
	return Registration.IsValid()
		&& Registration.DispatcherIdentity == SimulationRegistrationDispatcherId
		&& Registration.Serial == ActiveSimulationRegistration
		&& SimulationRuntime.IsValid();
}

bool FURLabRpcDispatcher::HasSimulationRuntime() const
{
	FScopeLock Lock(&DispatchMutex);
	return SimulationRuntime.IsValid();
}

bool FURLabRpcDispatcher::HasRenderService() const
{
	FScopeLock Lock(&DispatchMutex);
	return RenderService && RenderService->IsActive();
}

bool FURLabRpcDispatcher::HasViewerService() const
{
	FScopeLock Lock(&DispatchMutex);
	return ViewerService && ViewerService->IsActive();
}

bool FURLabRpcDispatcher::HasPoseStreamService() const
{
	FScopeLock Lock(&DispatchMutex);
	return PoseStreamService && PoseStreamService->IsActive();
}

void FURLabRpcDispatcher::Shutdown()
{
	bDraining.store(true, std::memory_order_release);
	SimulationCommandQueue->CloseAndCancel();
	RenderCommandQueue->CloseAndCancel();
	PayloadCommandQueue->CloseAndCancel();
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> RetiredRuntime;
	TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> RetiredRender;
	TSharedPtr<FURLabViewerServiceBinding, ESPMode::ThreadSafe> RetiredViewer;
	TSharedPtr<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> RetiredStreams;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> RetiredCameraStreams;
	FRetiredSessionScopes RetiredScopes;
	{
		FScopeLock Lock(&DispatchMutex);
		RetiredRuntime = MoveTemp(SimulationRuntime);
		RetiredRender = RenderService;
		RetiredViewer = ViewerService;
		RetiredStreams = PoseStreamService;
		RetiredCameraStreams = CameraStreamService;
		if (RetiredCameraStreams) RetiredCameraStreams->Deactivate();
		if (RetiredStreams) RetiredStreams->Deactivate();
		if (RetiredViewer) RetiredViewer->Deactivate();
		if (RetiredRender) RetiredRender->Deactivate();
		ActiveRenderRegistration = 0;
		ActiveViewerRegistration = 0;
		ActiveSimulationRegistration = 0;
		ActivePoseStreamRegistration = 0;
		ActiveCameraStreamRegistration = 0;
		for (FSession& Session : ActiveSessions)
		{
			DeactivateSessionLocked(Session, RetiredScopes);
		}
		RenderService.Reset();
		ViewerService.Reset();
		PoseStreamService.Reset();
		CameraStreamService.Reset();
		ActiveSessions.Reset();
	}
	RevokeSessionScopes(RetiredRuntime, PayloadStore, RetiredScopes);
	RetiredRuntime.Reset();
}

bool FURLabRpcDispatcher::ValidateSession(const FString& ClientSessionId) const
{
	if (ClientSessionId.IsEmpty())
	{
		return false;
	}
	FScopeLock Lock(&DispatchMutex);
	const double Now = SessionNow();
	for (const FSession& S : ActiveSessions)
	{
		if (S.Id.Equals(ClientSessionId) && S.Adapter == TEXT("native") &&
			Now - S.LastSeenSeconds < kSessionIdleTimeoutSeconds)
		{
			return true;
		}
	}
	return false;
}

bool FURLabRpcDispatcher::ValidateAndTouchSession(const FString& ClientSessionId)
{
	if (ClientSessionId.IsEmpty())
	{
		return false;
	}
	FRetiredSessionScopes RetiredScopes;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	bool bFound = false;
	double Now = 0.0;
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		Now = SessionNow();
		ActiveSessions.RemoveAll([this, Now, &RetiredScopes](const FSession& S) {
			if (Now - S.LastSeenSeconds < kSessionIdleTimeoutSeconds) return false;
			DeactivateSessionLocked(S, RetiredScopes);
			return true;
		});
		for (int32 Index = 0; Index < ActiveSessions.Num(); ++Index)
		{
			FSession& S = ActiveSessions[Index];
			if (!S.Id.Equals(ClientSessionId) || S.Adapter != TEXT("native")) continue;
			if (!S.ControlSession->Touch())
			{
				DeactivateSessionLocked(S, RetiredScopes);
				ActiveSessions.RemoveAt(Index);
				break;
			}
			S.LastSeenSeconds = Now;
			bFound = true;
			break;
		}
	}
	RevokeSessionScopes(Runtime, PayloadStore, RetiredScopes);
	return bFound;
}

FString FURLabRpcDispatcher::GetActiveSessionId() const
{
	FScopeLock Lock(&DispatchMutex);
	const double Now = SessionNow();
	const FSession* Best = nullptr;
	for (const FSession& S : ActiveSessions)
	{
		if (Now - S.LastSeenSeconds < kSessionIdleTimeoutSeconds &&
			(Best == nullptr || S.LastSeenSeconds > Best->LastSeenSeconds))
		{
			Best = &S;
		}
	}
	return Best != nullptr ? Best->Id : FString();
}

void FURLabRpcDispatcher::SetActiveSessionIdForTest(const FString& Id)
{
	FRetiredSessionScopes RetiredScopes;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		for (const FSession& Session : ActiveSessions)
		{
			DeactivateSessionLocked(Session, RetiredScopes);
		}
		ActiveSessions.Reset();
	}
	// Reclaim the old exact scope before the store may mint the same textual ID.
	RevokeSessionScopes(Runtime, PayloadStore, RetiredScopes);
	if (Id.IsEmpty()) return;

	FURLabPayloadSessionPtr PayloadSession;
	const FURLabPayloadStoreStatus PayloadStatus =
		PayloadStore->OpenSession(BootId, Id, PayloadSession);
	checkf(PayloadStatus.IsOk(), TEXT("test payload session failed: %s"),
		*PayloadStatus.Message);
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock = Runtime
			? Runtime->GetControlClock() : FMjSimulationControlClock::System();
		FString Error;
		const TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> ControlSession =
			FMjSimulationControlSessionLifetime::CreateRemote(Id, Clock, Error);
		checkf(ControlSession.IsValid(), TEXT("test session control scope failed: %s"), *Error);
		const FURLabRenderOwnerSessionScopePtr RenderScope =
			FURLabRenderOwnerSessionScope::Create(Id, Error);
		checkf(RenderScope.IsValid(), TEXT("test render scope failed: %s"), *Error);
		const auto ViewerScope = FURLabRenderOwnerSessionScope::Create(Id, Error);
		checkf(ViewerScope.IsValid(), TEXT("test viewer scope failed: %s"), *Error);
		ActiveSessions.Emplace(Id, TEXT("native"), SessionNow(),
			RequestLedger->OpenSession(Id), ControlSession.ToSharedRef(), RenderScope, ViewerScope, PayloadSession,
			nullptr);
	}
}

bool FURLabRpcDispatcher::RegisterNewSession(const FString& Adapter,
	EURLabDataPolicy DataPolicy, FString& OutSessionId,
	FURLabDataSessionSnapshot& OutData, FString& OutErrorCode, FString& OutError)
{
	OutSessionId.Reset();
	OutData = {};
	OutErrorCode.Reset();
	OutError.Reset();
	const double ConnectStarted = SessionNow();
	if (!FMath::IsFinite(ConnectStarted))
	{
		OutErrorCode = TEXT("not_ready");
		OutError = TEXT("session clock is invalid");
		return false;
	}

	FRetiredSessionScopes RetiredScopes;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	bool bHasProtocolCapacity = false;
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		ActiveSessions.RemoveAll([this, ConnectStarted, &RetiredScopes](const FSession& S)
		{
			if (ConnectStarted - S.LastSeenSeconds < kSessionIdleTimeoutSeconds) return false;
			DeactivateSessionLocked(S, RetiredScopes);
			return true;
		});
		bHasProtocolCapacity = ActiveSessions.Num() < kMaxActiveSessions;
	}
	RevokeSessionScopes(Runtime, PayloadStore, RetiredScopes);
	if (!bHasProtocolCapacity || IsDraining())
	{
		OutErrorCode = TEXT("busy");
		OutError = TEXT("Session capacity reached");
		return false;
	}

	const bool bAdapterCanShm = Adapter == TEXT("zmq") && ShmRegion.IsValid();
	if (DataPolicy == EURLabDataPolicy::Shm && !bAdapterCanShm)
	{
		OutErrorCode = TEXT("shm_unavailable");
		OutError = TEXT("Required SHM data is unavailable on this command adapter");
		return false;
	}
	const FString CandidateId =
		FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
	FURLabPayloadSessionPtr PayloadSession;
	const FURLabPayloadStoreStatus OpenStatus =
		PayloadStore->OpenSession(BootId, CandidateId, PayloadSession);
	if (!OpenStatus)
	{
		OutErrorCode = TEXT("busy");
		OutError = OpenStatus.Message;
		return false;
	}
	const FURLabPayloadOwner Owner = FURLabPayloadOwner::FromSession(PayloadSession);
	FURLabDataSessionPtr DataSession;
	FURLabShmExactChallengePtr Challenge;
	FString DataError;
	if (bAdapterCanShm && DataPolicy != EURLabDataPolicy::Network)
	{
		const FURLabShmExactStatus ChallengeStatus =
			ShmRegion->CreateChallenge(*PayloadStore, Owner, Challenge);
		if (ChallengeStatus)
		{
			DataSession = FURLabDataSession::CreateProvisionalShm(Adapter, DataPolicy,
				Owner, ShmRegion, Challenge,
				ConnectStarted + URLabWire::Limits::DataNegotiationTimeoutS, DataError);
		}
		else if (DataPolicy == EURLabDataPolicy::Shm)
		{
			DataError = ChallengeStatus.Message;
		}
	}
	if (!DataSession && DataPolicy != EURLabDataPolicy::Shm)
	{
		DataSession = FURLabDataSession::CreateNetwork(Adapter, DataPolicy, Owner, DataError);
	}
	if (!DataSession)
	{
		PayloadSession->Deactivate();
		if (Challenge && ShmRegion) ShmRegion->RetireChallenge(Owner, Challenge);
		int32 RemovedEntries = 0;
		PayloadStore->RevokeSession(Owner, RemovedEntries);
		OutErrorCode = DataPolicy == EURLabDataPolicy::Shm
			? TEXT("shm_unavailable") : TEXT("not_ready");
		OutError = DataError.IsEmpty() ? TEXT("Data session could not be prepared") : DataError;
		return false;
	}

	bool bInstalled = false;
	bool bPreparedChallengeExpired = false;
	bool bAdmissionClockInvalid = false;
	double AdmissionNow = ConnectStarted;
	FURLabDataSessionSnapshot AdmissionData;
	{
		FScopeLock Lock(&DispatchMutex);
		AdmissionNow = SessionNow();
		bAdmissionClockInvalid = !FMath::IsFinite(AdmissionNow);
		if (!bAdmissionClockInvalid)
		{
			AdmissionData = DataSession->Snapshot(AdmissionNow);
			bPreparedChallengeExpired =
				AdmissionData.Transfer == EURLabDataTransfer::Provisional
				&& AdmissionData.ExpiresInSeconds <= 0.0;
		}
		if (!bAdmissionClockInvalid && !bPreparedChallengeExpired
			&& !IsDraining() && ActiveSessions.Num() < kMaxActiveSessions)
		{
			Runtime = SimulationRuntime;
			const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock = Runtime
				? Runtime->GetControlClock() : FMjSimulationControlClock::System();
			FString ControlError;
			const TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> ControlSession =
				FMjSimulationControlSessionLifetime::CreateRemote(CandidateId, Clock, ControlError);
			checkf(ControlSession.IsValid(), TEXT("session control scope failed: %s"), *ControlError);
			const FURLabRenderOwnerSessionScopePtr RenderScope =
				FURLabRenderOwnerSessionScope::Create(CandidateId, ControlError);
			checkf(RenderScope.IsValid(), TEXT("render scope failed: %s"), *ControlError);
			const auto ViewerScope = FURLabRenderOwnerSessionScope::Create(CandidateId, ControlError);
			checkf(ViewerScope.IsValid(), TEXT("viewer scope failed: %s"), *ControlError);
			ActiveSessions.Emplace(CandidateId, Adapter, AdmissionNow,
				RequestLedger->OpenSession(CandidateId), ControlSession.ToSharedRef(), RenderScope, ViewerScope,
				PayloadSession, DataSession);
			bInstalled = true;
		}
	}
	if (bInstalled)
	{
		OutSessionId = CandidateId;
		OutData = MoveTemp(AdmissionData);
		return true;
	}

	FURLabShmExactChallengePtr ChallengeToRetire;
	DataSession->Deactivate(ChallengeToRetire);
	if (ChallengeToRetire && ShmRegion)
		ShmRegion->RetireChallenge(Owner, ChallengeToRetire);
	if (bPreparedChallengeExpired && DataPolicy == EURLabDataPolicy::Auto
		&& FMath::IsFinite(AdmissionNow) && !IsDraining())
	{
		DataSession = FURLabDataSession::CreateNetwork(Adapter, DataPolicy, Owner, DataError);
		if (DataSession)
		{
			FScopeLock Lock(&DispatchMutex);
			AdmissionNow = SessionNow();
			if (FMath::IsFinite(AdmissionNow) && !IsDraining()
				&& ActiveSessions.Num() < kMaxActiveSessions)
			{
				Runtime = SimulationRuntime;
				const TSharedRef<FMjSimulationControlClock, ESPMode::ThreadSafe> Clock = Runtime
					? Runtime->GetControlClock() : FMjSimulationControlClock::System();
				FString ControlError;
				const TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> ControlSession =
					FMjSimulationControlSessionLifetime::CreateRemote(CandidateId, Clock, ControlError);
				checkf(ControlSession.IsValid(), TEXT("session control scope failed: %s"), *ControlError);
				const FURLabRenderOwnerSessionScopePtr RenderScope =
					FURLabRenderOwnerSessionScope::Create(CandidateId, ControlError);
				checkf(RenderScope.IsValid(), TEXT("render scope failed: %s"), *ControlError);
				const auto ViewerScope = FURLabRenderOwnerSessionScope::Create(CandidateId, ControlError);
				checkf(ViewerScope.IsValid(), TEXT("viewer scope failed: %s"), *ControlError);
				ActiveSessions.Emplace(CandidateId, Adapter, AdmissionNow,
					RequestLedger->OpenSession(CandidateId), ControlSession.ToSharedRef(), RenderScope, ViewerScope,
					PayloadSession, DataSession);
				OutSessionId = CandidateId;
				OutData = DataSession->Snapshot(AdmissionNow);
				return true;
			}
		}
	}
	PayloadSession->Deactivate();
	int32 RemovedEntries = 0;
	PayloadStore->RevokeSession(Owner, RemovedEntries);
	OutErrorCode = bPreparedChallengeExpired && DataPolicy == EURLabDataPolicy::Shm
		? TEXT("shm_unavailable")
		: (bAdmissionClockInvalid ? TEXT("not_ready") : TEXT("busy"));
	OutError = bAdmissionClockInvalid
		? TEXT("session clock is invalid")
		: (bPreparedChallengeExpired
		? TEXT("SHM challenge expired before session admission")
		: TEXT("Session capacity reached"));
	return false;
}

double FURLabRpcDispatcher::SessionNow() const
{
	const double Override = SessionClockOverrideForTest->load(std::memory_order_acquire);
	return Override >= 0.0
		? Override
		: FPlatformTime::Seconds();
}

void FURLabRpcDispatcher::DeactivateSessionLocked(const FSession& Session,
	FRetiredSessionScopes& OutScopes)
{
	Session.ControlSession->Deactivate();
	OutScopes.Control.Add(Session.ControlSession);
	if (Session.RenderOwnerScope)
	{
		Session.RenderOwnerScope->Deactivate();
		FRetiredSessionScopes::FRender& Retired = OutScopes.Render.AddDefaulted_GetRef();
		Retired.Binding = RenderService;
		Retired.Scope = Session.RenderOwnerScope;
	}
	if (Session.ViewerOwnerScope)
	{
		Session.ViewerOwnerScope->Deactivate();
		auto& Retired = OutScopes.Viewer.AddDefaulted_GetRef();
		Retired.Binding = ViewerService; Retired.Scope = Session.ViewerOwnerScope;
	}
	if (Session.PayloadSession)
	{
		Session.PayloadSession->Deactivate();
		OutScopes.Payload.Add(FURLabPayloadOwner::FromSession(Session.PayloadSession));
	}
	if (Session.DataSession)
	{
		FRetiredDataScope& Retired = OutScopes.Data.AddDefaulted_GetRef();
		Retired.Session = Session.DataSession;
		Session.DataSession->Deactivate(Retired.Challenge);
	}
	RequestLedger->CloseSession(Session.LedgerSession);
}

void FURLabRpcDispatcher::RevokeSessionScopes(
	const TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
	const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FRetiredSessionScopes& Scopes)
{
	RevokeControlSessions(Runtime, Scopes.Control);
	for (const FRetiredDataScope& Retired : Scopes.Data)
	{
		if (Retired.Session && Retired.Challenge && Retired.Session->GetRegion())
		{
			Retired.Session->GetRegion()->RetireChallenge(
				Retired.Session->GetOwner(), Retired.Challenge);
		}
	}
	auto RevokePayloads = [Store, Owners = Scopes.Payload]()
	{
		for (const FURLabPayloadOwner& Owner : Owners)
		{
			int32 RemovedEntries = 0;
			Store->RevokeSession(Owner, RemovedEntries);
		}
	};
	if (Scopes.Render.IsEmpty() && Scopes.Viewer.IsEmpty())
	{
		RevokePayloads();
		return;
	}
	const TSharedRef<TArray<FRetiredSessionScopes::FRender>, ESPMode::ThreadSafe> Pending =
		MakeShared<TArray<FRetiredSessionScopes::FRender>, ESPMode::ThreadSafe>(Scopes.Render);
	const TSharedRef<TArray<FRetiredSessionScopes::FViewer>, ESPMode::ThreadSafe> PendingViewer =
		MakeShared<TArray<FRetiredSessionScopes::FViewer>, ESPMode::ThreadSafe>(Scopes.Viewer);
	const TSharedRef<TFunction<bool(float)>, ESPMode::ThreadSafe> Retry =
		MakeShared<TFunction<bool(float)>, ESPMode::ThreadSafe>();
	*Retry = [Pending, PendingViewer, RevokePayloads = MoveTemp(RevokePayloads)](float) mutable
	{
		check(IsInGameThread());
		for (int32 Index = Pending->Num() - 1; Index >= 0; --Index)
		{
			const FRetiredSessionScopes::FRender& Entry = (*Pending)[Index];
			if (!Entry.Binding || !Entry.Scope)
			{
				Pending->RemoveAtSwap(Index);
				continue;
			}
			const FURLabRenderOwnerLeaseResult Result = Entry.Binding->RetireScope(Entry.Scope);
			if (Result.Error == EURLabRenderOwnerAuthorityError::Unavailable) continue;
			Pending->RemoveAtSwap(Index);
		}
		for (int32 Index = PendingViewer->Num() - 1; Index >= 0; --Index)
		{
			const auto& Entry = (*PendingViewer)[Index];
			if (!Entry.Binding || !Entry.Scope) { PendingViewer->RemoveAtSwap(Index); continue; }
			const auto Result = Entry.Binding->RetireScope(Entry.Scope);
			if (Result.Error == EURLabRenderOwnerAuthorityError::Unavailable) continue;
			PendingViewer->RemoveAtSwap(Index);
		}
		if (!Pending->IsEmpty() || !PendingViewer->IsEmpty()) return true;
		RevokePayloads();
		return false;
	};
	auto Start = [Retry]()
	{
		if (!(*Retry)(0.0f)) return;
		FTSTicker::GetCoreTicker().AddTicker(
			FTickerDelegate::CreateLambda([Retry](float Delta) { return (*Retry)(Delta); }));
	};
	if (IsInGameThread()) Start();
	else AsyncTask(ENamedThreads::GameThread, MoveTemp(Start));
}

void FURLabRpcDispatcher::RevokeControlSessions(
	const TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
	const TArray<TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>>& Sessions)
{
	if (!Runtime) return;
	for (const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& Session : Sessions)
	{
		Runtime->RevokeControlForSession(Session);
	}
}

bool FURLabRpcDispatcher::RemoveSession(const FString& Id, const FString& Adapter)
{
	FRetiredSessionScopes RetiredScopes;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	int32 Removed = 0;
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		Removed = ActiveSessions.RemoveAll(
			[this, &Id, &Adapter, &RetiredScopes](const FSession& S) {
				if (S.Id != Id || S.Adapter != Adapter) return false;
				DeactivateSessionLocked(S, RetiredScopes);
				return true;
			});
	}
	RevokeSessionScopes(Runtime, PayloadStore, RetiredScopes);
	return Removed > 0;
}

bool FURLabRpcDispatcher::FindAndTouchSession(const FString& Id,
	const FString& Adapter,
	TSharedPtr<FURLabRequestLedger::FSession, ESPMode::ThreadSafe>& OutSession,
	TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& OutControlSession,
	FURLabRenderOwnerSessionScopePtr& OutRenderOwnerScope,
	FURLabRenderOwnerSessionScopePtr& OutViewerOwnerScope,
	FURLabPayloadSessionPtr& OutPayloadSession, FURLabDataSessionPtr& OutDataSession)
{
	FRetiredSessionScopes RetiredScopes;
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime;
	bool bFound = false;
	{
		FScopeLock Lock(&DispatchMutex);
		Runtime = SimulationRuntime;
		const double Now = SessionNow();
		ActiveSessions.RemoveAll([this, Now, &RetiredScopes](const FSession& S) {
			if (Now - S.LastSeenSeconds < kSessionIdleTimeoutSeconds) return false;
			DeactivateSessionLocked(S, RetiredScopes);
			return true;
		});
		for (int32 Index = 0; Index < ActiveSessions.Num(); ++Index)
		{
			FSession& Session = ActiveSessions[Index];
			if (Session.Id != Id || Session.Adapter != Adapter) continue;
			if (!Session.ControlSession->Touch() || !Session.PayloadSession
				|| !Session.PayloadSession->IsActive() || !Session.DataSession)
			{
				DeactivateSessionLocked(Session, RetiredScopes);
				ActiveSessions.RemoveAt(Index);
				break;
			}
			Session.LastSeenSeconds = Now;
			OutSession = Session.LedgerSession;
			OutControlSession = Session.ControlSession;
			OutRenderOwnerScope = Session.RenderOwnerScope;
			OutViewerOwnerScope = Session.ViewerOwnerScope;
			OutPayloadSession = Session.PayloadSession;
			OutDataSession = Session.DataSession;
			bFound = true;
			break;
		}
	}
	RevokeSessionScopes(Runtime, PayloadStore, RetiredScopes);
	if (bFound && OutDataSession)
	{
		FURLabShmExactChallengePtr ExpiredChallenge;
		OutDataSession->ExpireChallenge(SessionNow(), ExpiredChallenge);
		if (ExpiredChallenge && OutDataSession->GetRegion())
		{
			const FURLabShmExactStatus Retire =
				OutDataSession->GetRegion()->RetireChallenge(
					OutDataSession->GetOwner(), ExpiredChallenge);
			if (!Retire)
			{
				UE_LOG(LogURLabNet, Warning,
					TEXT("Expired protocol-v2 SHM challenge cleanup failed: %s"),
					*Retire.Message);
			}
		}
	}
	return bFound;
}

// Out-of-line so the TWeakObjectPtr assignment sees the full UURLabBridgeServer
// type (the header only forward-declares it to avoid an include cycle).
void FURLabRpcDispatcher::SetOwningBridge(UURLabBridgeServer* InBridge)
{
	OwningBridge = InBridge;
}

bool FURLabRpcDispatcher::GrantsPermission(EMjCapability Cap) const
{
	if (UURLabBridgeServer* Bridge = OwningBridge.Get())
	{
		return Bridge->GrantsPermission(Cap);
	}
	// No bridge means no wire, so nothing has reached this dispatcher from outside the
	// process and there is no authority to withhold.
	return true;
}
