// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include <atomic>
#include "ViewerSubscribeTransport.generated.h"

class AAMjManager;
class UMjPhysicsEngine;
class FRunnable;
class FRunnableThread;

/**
 * @class UURLabViewerSubscribeTransport
 * @brief Read-only viewer input: a ZMQ SUB that CONNECTS to an owner's viewer
 *        bus (the puppet client or a direct/live UE owner), decodes each raw
 *        {t, qpos, qvel} frame, and drives this process's own render.
 *
 * The owner is the single physics authority; a viewer never steps and never
 * sends state back. On each received frame the worker thread writes qpos/qvel
 * into this process's mjData, runs mj_forward, and pushes a render snapshot --
 * the exact apply path puppet mode uses, minus the RPC reply. Its own worker
 * thread (not the physics step loop, which is paused on a viewer) so the render
 * cadence follows the owner's broadcast rate.
 *
 * Wire format matches URLab_Bridge's client PUB and the owner-side UE PUB:
 * topic "viewer", msgpack map {"t": f64, "qpos": [f64], "qvel": [f64]}.
 */
UCLASS()
class URLAB_API UURLabViewerSubscribeTransport : public UObject
{
	GENERATED_BODY()

public:
	/** Owner viewer-bus endpoint to connect to, e.g. "tcp://127.0.0.1:5560". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Viewer")
	FString SourceEndpoint;

	/** Topic filter; must match the owner's publish topic. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Viewer")
	FString Topic = TEXT("viewer");

	void SetOwningManager(AAMjManager* InMgr);

	/** Connect the SUB and spin up the receive thread. */
	bool TransportInit();
	/** Stop the receive thread and close the socket. Idempotent. Must run
	 *  before the physics engine it applies into is torn down. */
	void TransportShutdown();

	// Called by the worker thread; public so the FRunnable can reach it.
	void RunReceiveLoop();

	std::atomic<bool> bStop{false};

private:
	TWeakObjectPtr<AAMjManager> OwningManager;
	// Cached at init on the game thread; the engine outlives this transport
	// (shut down first in AAMjManager::EndPlay). Accessed under its CallbackMutex.
	UMjPhysicsEngine* Engine = nullptr;

	void* ZmqContext = nullptr;
	void* Subscriber = nullptr;
	FRunnable* WorkerRunnable = nullptr;
	FRunnableThread* WorkerThread = nullptr;
	bool bIsInitialized = false;
	// Throttles the model-mismatch warning to once per mismatch episode (re-armed
	// after any frame that applies), so a wrong-scene viewer is diagnosable
	// without spamming the log every frame. Worker-thread only.
	bool bWarnedMismatch = false;

	// Applies one decoded frame into (m,d) + mj_forward + PushRenderState.
	void ApplyFrame(double Time, const TArray<double>& QPos, const TArray<double>& QVel);
};
