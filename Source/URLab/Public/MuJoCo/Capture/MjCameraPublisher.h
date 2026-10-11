// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Render/RenderCameraTypes.h"
#include "Transport/ShmSession.h"
#include <atomic>
#include "MjCameraPublisher.generated.h"

class URenderCameraComponent;
class UURLabCameraPublishTransport;

/** Process-wide host publisher pause used by direct and puppet step bursts. */
class FCameraZmqWorker
{
public:
	static URLAB_API std::atomic<bool> bPublishersPaused;
};

/** MuJoCo host-side identity and image transport fan-out for one neutral camera. */
UCLASS()
class URLAB_API UMjCameraPublisher : public UObject
{
	GENERATED_BODY()
public:
	void Configure(URenderCameraComponent* InCamera, int32 InChannelIndex,
		const FString& InEndpoint, const FURLabShmSession& InSession);
	void SetBroadcastEnabled(bool bZmq, bool bShm);
	void Shutdown();
	FString GetCanonicalName() const;
	FString GetActualZmqEndpoint() const;
	URenderCameraComponent* GetCamera() const { return Camera.Get(); }
	bool IsZmqEnabled() const { return bEnableZmq; }
	bool IsShmEnabled() const { return bEnableShm; }
private:
	void HandleFrame(const FRenderCameraFrame& Frame);
	void Open();
	TWeakObjectPtr<URenderCameraComponent> Camera;
	int32 ChannelIndex = 0;
	FString Endpoint;
	FURLabShmSession Session;
	bool bEnableZmq = false;
	bool bEnableShm = false;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UURLabCameraPublishTransport>> Publishers;
};
