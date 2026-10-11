// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Gen/Elements/Cameras/MjCamera.gen.h"
#include "Render/RenderCameraTypes.h"
#include "MjCamera.generated.h"

/** MJCF camera authoring definition. Runtime rendering lives in URenderCameraComponent. */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjCamera : public UMjCameraBase
{
	GENERATED_BODY()
public:
	UMjCamera();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	ERenderCameraMode CaptureMode = ERenderCameraMode::Real;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "0.1"))
	float DepthNearCm = 10.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "1.0"))
	float DepthFarCm = 10000.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	bool bCaptureOnStateChange = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "0.0"))
	float CaptureMaxFps = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	bool bManualCaptureOnly = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "1", ClampMax = "64"))
	int32 HistoryCapacity = 8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "0.0"))
	float RequestActiveTtlSeconds = 2.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	bool bRenderInMainRenderer = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float StreamingBoost = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	bool bEnableZmqBroadcast = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	FString ZmqEndpoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	bool bEnableShmBroadcast = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay", meta = (ClampMin = "0.0"))
	float DelaySeconds = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay", meta = (ClampMin = "0.0"))
	float DelayJitterSeconds = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay")
	bool bDelayUseWallClock = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float CustomNearClipCm = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float CustomFarClipCm = 0.0f;

	FRenderCameraCaptureSettings ExportCaptureSettings() const;
};
