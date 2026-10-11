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

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjCapabilities.h"
#include "Bridge/RequestLedger.h"
#include "Bridge/DataSession.h"
#include "Bridge/RenderServiceBinding.h"
#include "Bridge/ViewerServiceBinding.h"
#include "Bridge/PoseStreamServiceBinding.h"
#include "Bridge/CameraStreamServiceBinding.h"
#include "Bridge/SerialCommandQueue.h"
#include "Bridge/GameThreadCommandQueue.h"
#include "Bridge/URLabPayloadStore.h"
#include "Wire/RequestEnvelope.h"
#include <atomic>

class UURLabBridgeServer;
class FMjSimulationRuntime;
class FMjSimulationControlSessionLifetime;

/**
 * @brief Bridge-owned command dispatcher and protocol-v2 admission core.
 *
 * Owns typed sessions, request admission, and service bindings. ZMQ and the
 * render-only gRPC extension enter as
 * command adapters; negotiated network/exact-SHM data transfers retain their
 * own session resources. The dispatcher does not own adapter handles.
 *
 * Owned by `UURLabBridgeServer`, so every command adapter shares one session
 * and request ledger. Protocol-v2 simulation work captures an independently
 * owned `FMjSimulationRuntime` at admission.
 */
class URLAB_API FURLabRpcDispatcher
{
public:
	FURLabRpcDispatcher();
	~FURLabRpcDispatcher();

	/** Opaque ownership proof for one independent simulation-service binding. */
	class FSimulationRuntimeRegistration
	{
	public:
		bool IsValid() const { return DispatcherIdentity.IsValid() && Serial != 0; }

	private:
		friend class FURLabRpcDispatcher;
		FGuid DispatcherIdentity;
		uint64 Serial = 0;
	};

	/**
	 * Register the independent simulation runtime served by typed protocol-v2
	 * operations. A second live registration is rejected. OutRegistration is
	 * changed only on success.
	 */
	bool AttachSimulationRuntime(
		TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime,
		FSimulationRuntimeRegistration& OutRegistration, FString& OutError);

	/** True only while this dispatcher owns the exact identity-and-serial token. */
	bool IsSimulationRuntimeRegistrationActive(
		FSimulationRuntimeRegistration Registration) const;

	/** Clear the exact registration represented by Registration. */
	bool DetachSimulationRuntime(FSimulationRuntimeRegistration Registration);

	class FRenderServiceRegistration
	{
	public:
		bool IsValid() const { return DispatcherIdentity.IsValid() && Serial != 0; }
	private:
		friend class FURLabRpcDispatcher;
		FGuid DispatcherIdentity;
		uint64 Serial = 0;
	};
	bool AttachRenderService(
		TSharedRef<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding,
		FRenderServiceRegistration& OutRegistration, FString& OutError);
	bool IsRenderServiceRegistrationActive(FRenderServiceRegistration Registration) const;
	bool DetachRenderService(FRenderServiceRegistration Registration);
	class FViewerServiceRegistration
	{
	public: bool IsValid() const { return DispatcherIdentity.IsValid() && Serial != 0; }
	private: friend class FURLabRpcDispatcher; FGuid DispatcherIdentity; uint64 Serial=0;
	};
	bool AttachViewerService(TSharedRef<FURLabViewerServiceBinding,ESPMode::ThreadSafe> Binding,
		FViewerServiceRegistration& OutRegistration,FString& OutError);
	bool IsViewerServiceRegistrationActive(FViewerServiceRegistration Registration) const;
	bool DetachViewerService(FViewerServiceRegistration Registration);
	class FPoseStreamServiceRegistration
	{
	public: bool IsValid() const { return DispatcherIdentity.IsValid() && Serial != 0; }
	private: friend class FURLabRpcDispatcher; FGuid DispatcherIdentity; uint64 Serial = 0;
	};
	bool AttachPoseStreamService(
		TSharedRef<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> Binding,
		FPoseStreamServiceRegistration& OutRegistration, FString& OutError);
	bool DetachPoseStreamService(FPoseStreamServiceRegistration Registration);
	bool IsPoseStreamServiceRegistrationActive(FPoseStreamServiceRegistration Registration) const;
	class FCameraStreamServiceRegistration
	{
	public: bool IsValid() const { return DispatcherIdentity.IsValid() && Serial != 0; }
	private: friend class FURLabRpcDispatcher; FGuid DispatcherIdentity; uint64 Serial = 0;
	};
	bool AttachCameraStreamService(
		TSharedRef<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> Binding,
		FCameraStreamServiceRegistration& OutRegistration, FString& OutError);
	bool DetachCameraStreamService(FCameraStreamServiceRegistration Registration);
	bool IsCameraStreamServiceRegistrationActive(FCameraStreamServiceRegistration Registration) const;
	FURLabShmExactRegionPtr GetShmRegionForLocalProducer() const { return ShmRegion; }

	/** Back-pointer to the owning bridge server for permission and endpoint metadata. */
	void SetOwningBridge(UURLabBridgeServer* InBridge);

	/** Whether this instance grants peers `Cap`. Asked of the owning bridge, which is
	 *  what a peer reaches and what holds the resolved grants; a dispatcher with no
	 *  bridge (a bare test double) grants everything, because there is no peer. */
	bool GrantsPermission(EMjCapability Cap) const;

	/** Full bridge-server teardown. */
	void Shutdown();

	/** Cooperative shutdown signal. A fresh dispatcher starts open; Shutdown
	 *  closes admission permanently for that dispatcher lifetime. */
	void SetDraining(bool bIn) { bDraining.store(bIn, std::memory_order_release); }
	bool IsDraining() const { return bDraining.load(std::memory_order_acquire); }

	/** Wire format echoed to the client as the urlab plugin version. */
	FString URLabVersion = TEXT("urlab/0.2");

	/** Strict protocol-v2 MessagePack entry point used by command transports. */
	bool DispatchBytes(TConstArrayView<uint8> RequestBytes, const FString& Adapter,
		TArray<uint8>& OutReplyBytes, const FString* OuterOp = nullptr);

	// --- Session introspection (used by transports and tests) ---

	/** True when ClientSessionId names a live session. Read-only: prefer
	 *  ValidateAndTouchSession on the dispatch path so the session's
	 *  least-recently-used ranking stays honest. */
	bool ValidateSession(const FString& ClientSessionId) const;
	/** Validate ClientSessionId and stamp it as most-recently-used. */
	bool ValidateAndTouchSession(const FString& ClientSessionId);
	/** The most recently active session id, for the transports that synthesise a
	 *  request on a connected client's behalf. Empty when nothing is connected. */
	FString GetActiveSessionId() const;
	/** Make Id the one and only live session. Test seam. */
	void SetActiveSessionIdForTest(const FString& Id);
	void SetSessionClockForTest(double NowSeconds)
	{
		SessionClockOverrideForTest->store(NowSeconds, std::memory_order_release);
	}
	TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> GetRequestLedgerForTest() const
	{
		return RequestLedger;
	}
	TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> GetPayloadStoreForTest() const
	{
		return PayloadStore;
	}
	void SetPayloadQueueAutoDrainForTest(bool bEnabled)
	{
		bPayloadQueueAutoDrainForTest.store(bEnabled, std::memory_order_release);
	}
	TSharedRef<FURLabSerialCommandQueue, ESPMode::ThreadSafe>
	GetPayloadCommandQueueForTest() const
	{
		return PayloadCommandQueue;
	}

private:
	/** Owning bridge server (set at construction). */
	TWeakObjectPtr<UURLabBridgeServer> OwningBridge;

	/** Independent runtime service and its owner proof. Guarded by DispatchMutex. */
	TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe> SimulationRuntime;
	const FGuid SimulationRegistrationDispatcherId = FGuid::NewGuid();
	uint64 ActiveSimulationRegistration = 0;
	uint64 NextSimulationRegistration = 1;
	TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> RenderService;
	const FGuid RenderRegistrationDispatcherId = FGuid::NewGuid();
	uint64 ActiveRenderRegistration = 0;
	uint64 NextRenderRegistration = 1;
	TSharedPtr<FURLabViewerServiceBinding,ESPMode::ThreadSafe> ViewerService;
	const FGuid ViewerRegistrationDispatcherId=FGuid::NewGuid();
	uint64 ActiveViewerRegistration=0;
	uint64 NextViewerRegistration=1;
	TSharedPtr<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> PoseStreamService;
	TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> CameraStreamService;
	const FGuid CameraStreamRegistrationDispatcherId = FGuid::NewGuid();
	uint64 ActiveCameraStreamRegistration = 0;
	uint64 NextCameraStreamRegistration = 1;
	const FGuid PoseStreamRegistrationDispatcherId = FGuid::NewGuid();
	uint64 ActivePoseStreamRegistration = 0;
	uint64 NextPoseStreamRegistration = 1;

	/** Guards service bindings and active sessions. */
	mutable FCriticalSection DispatchMutex;

	/** One live client session. */
	struct FSession
	{
		FString Id;
		FString Adapter;
		/** FPlatformTime::Seconds() when a request last carried this id. */
		double LastSeenSeconds = 0.0;
		FURLabRequestLedger::FSessionRef LedgerSession;
		TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> ControlSession;
		FURLabRenderOwnerSessionScopePtr RenderOwnerScope;
		FURLabRenderOwnerSessionScopePtr ViewerOwnerScope;
		FURLabPayloadSessionPtr PayloadSession;
		FURLabDataSessionPtr DataSession;

		FSession(const FString& InId, const FString& InAdapter, double InLastSeen,
			FURLabRequestLedger::FSessionRef InLedgerSession,
			TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> InControlSession,
			FURLabRenderOwnerSessionScopePtr InRenderOwnerScope,
			FURLabRenderOwnerSessionScopePtr InViewerOwnerScope,
			FURLabPayloadSessionPtr InPayloadSession, FURLabDataSessionPtr InDataSession)
			: Id(InId), Adapter(InAdapter), LastSeenSeconds(InLastSeen),
			  LedgerSession(MoveTemp(InLedgerSession)), ControlSession(MoveTemp(InControlSession)),
			  RenderOwnerScope(MoveTemp(InRenderOwnerScope)),
			  ViewerOwnerScope(MoveTemp(InViewerOwnerScope)),
			  PayloadSession(MoveTemp(InPayloadSession)), DataSession(MoveTemp(InDataSession)) {}
	};

	/** Every live session, keyed by id. A single slot used to be overwritten by
	 *  each `hello`, and a pool's discovery probe is indistinguishable from a real
	 *  client on the wire -- so probing a busy instance evicted the working client
	 *  before it could take the lease that exists to protect it. Sessions are
	 *  additive instead: a second `hello` mints its own id and leaves the first
	 *  valid. */
	TArray<FSession> ActiveSessions;
	/** Fresh for each dispatcher/bridge-server lifetime; stable across PIE. */
	FString BootId;
	TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> RequestLedger;
	TSharedRef<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe> SimulationCommandQueue;
	TSharedRef<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe> RenderCommandQueue;
	TSharedRef<FURLabSerialCommandQueue, ESPMode::ThreadSafe> PayloadCommandQueue;
	TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> PayloadStore;
	FURLabShmExactRegionPtr ShmRegion;

	/** Ceiling on retained sessions. Idle sessions expire; when every slot is
	 *  active, new connections are rejected instead of evicting a live client. */
	static constexpr int32 kMaxActiveSessions = 16;

	static constexpr double kSessionIdleTimeoutSeconds = 300.0;
	TSharedRef<std::atomic<double>, ESPMode::ThreadSafe> SessionClockOverrideForTest;
	double SessionNow() const;
	struct FRetiredDataScope
	{
		FURLabDataSessionPtr Session;
		FURLabShmExactChallengePtr Challenge;
	};
	struct FRetiredSessionScopes
	{
		TArray<TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>> Control;
		TArray<FURLabPayloadOwner> Payload;
		TArray<FRetiredDataScope> Data;
		struct FRender
		{
			TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding;
			FURLabRenderOwnerSessionScopePtr Scope;
		};
		TArray<FRender> Render;
		struct FViewer { TSharedPtr<FURLabViewerServiceBinding,ESPMode::ThreadSafe> Binding; FURLabRenderOwnerSessionScopePtr Scope; };
		TArray<FViewer> Viewer;
	};
	void DeactivateSessionLocked(const FSession& Session, FRetiredSessionScopes& OutScopes);
	static void RevokeSessionScopes(
		const TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
		const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
		const FRetiredSessionScopes& Scopes);
	static void RevokeControlSessions(
		const TSharedPtr<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
		const TArray<TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>>& Sessions);
	bool RemoveSession(const FString& Id, const FString& Adapter);
	bool FindAndTouchSession(const FString& Id,
		const FString& Adapter,
		TSharedPtr<FURLabRequestLedger::FSession, ESPMode::ThreadSafe>& OutSession,
		TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& OutControlSession,
		FURLabRenderOwnerSessionScopePtr& OutRenderOwnerScope,
		FURLabRenderOwnerSessionScopePtr& OutViewerOwnerScope,
		FURLabPayloadSessionPtr& OutPayloadSession, FURLabDataSessionPtr& OutDataSession);
	bool HasSimulationRuntime() const;
	bool HasRenderService() const;
	bool HasViewerService() const;
	bool HasPoseStreamService() const;
	bool HasEditorService() const;

	/** Mint and register a fresh session id, or return empty at active capacity. */
	bool RegisterNewSession(const FString& Adapter, EURLabDataPolicy DataPolicy,
		FString& OutSessionId, FURLabDataSessionSnapshot& OutData,
		FString& OutErrorCode, FString& OutError);
	bool DispatchTypedSimulation(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		const TSharedRef<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe>& ControlSession,
		TArray<uint8>& OutReplyBytes);
	bool DispatchTypedPayload(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		const FURLabDataSessionPtr& DataSession,
		TArray<uint8>& OutReplyBytes);
	bool DispatchTypedRender(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		const FURLabRenderOwnerSessionScopePtr& OwnerScope,
		const FURLabDataSessionPtr& DataSession,
		TArray<uint8>& OutReplyBytes);
	bool DispatchTypedViewer(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		const FURLabRenderOwnerSessionScopePtr& OwnerScope,
		const FURLabDataSessionPtr& DataSession,TArray<uint8>& OutReplyBytes);
	bool DispatchTypedStreams(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		const FURLabDataSessionPtr& DataSession, TArray<uint8>& OutReplyBytes);
	bool DispatchTypedEditor(const FURLabRequestEnvelope& Request,
		const FURLabRequestDigest& Digest,
		const FURLabRequestLedger::FSessionRef& Session,
		TArray<uint8>& OutReplyBytes);

	/** Set by BridgeServer::Stop() so blocking handlers exit early. */
	std::atomic<bool> bDraining{false};
	std::atomic<bool> bPayloadQueueAutoDrainForTest{true};
};
