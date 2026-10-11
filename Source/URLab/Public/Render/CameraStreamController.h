#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraTypes.h"
#include "Transport/CameraStreamProducer.h"

class AURLabRenderScene;
class URenderCameraComponent;

/**
 * Explicit one-camera live capture controller. Authority remains in its caller;
 * this class only selects an installed baseline camera, consumes its completed
 * immutable frames, and hands owned wire bytes to the transport producer.
 */
class URLAB_API FURLabCameraStreamController
{
public:
	static TUniquePtr<FURLabCameraStreamController> Create(
		const FURLabCameraStreamProducerConfig& Config, FString& OutError);
	~FURLabCameraStreamController();

	bool Configure(AURLabRenderScene& Scene, const FString& SourceRevision,
		const FString& CameraName, bool bEnabled, TOptional<double> RateHz,
		FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);
	bool Describe(const FString& CameraName, bool bShmEligible,
		FURLabCameraStreamDescriptor& OutDescriptor,
		FURLabCameraStreamDelivery& OutDelivery, FString& OutError) const;
	void Disable();
	bool IsEnabled() const { return Camera.IsValid(); }
	const FString& GetCameraName() const { return ActiveCameraName; }

private:
	explicit FURLabCameraStreamController(TUniquePtr<FURLabCameraStreamProducer>&& InProducer);
	void HandleFrame(const FRenderCameraFramePtr& Frame);

	TUniquePtr<FURLabCameraStreamProducer> Producer;
	TWeakObjectPtr<URenderCameraComponent> Camera;
	FDelegateHandle FrameDelegate;
	FString ActiveCameraName;
	FString SourceRevision;
	uint64 ConfigurationRevision = 0;
	bool bWasStreaming = false;
	bool bWasContinuous = false;
	bool bWasManualCaptureOnly = false;
	bool bBaselineOnStateChange = true;
	float BaselineMaxFps = 0.0f;
	bool bRateChanged = false;
	FString LastPublishError;
};
