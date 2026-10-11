// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Render/RenderFrame.h"
#include "Render/RenderScene.h"
#include "RenderCameraRig.generated.h"

class USceneComponent;
class URenderCameraComponent;

/** Owns model cameras and the separately indexed operator camera for one installed scene. */
UCLASS()
class URLAB_API URenderCameraRig : public UObject
{
	GENERATED_BODY()
public:
	bool Build(USceneComponent* AttachRoot, const FRenderScene& Scene,
		const FRenderFrame& InitialFrame, bool bInitiallyVisible, FString& OutError);
	bool ApplyFrame(const FRenderFrame& Frame, FString& OutError);
	void SetVisible(bool bVisible);
	void Teardown();
	int32 SetCaptureMode(ERenderCameraMode Mode);
	int32 SetCaptureModeFor(const TArray<URenderCameraComponent*>& Subset, ERenderCameraMode Mode);
	URenderCameraComponent* FindByCanonicalName(const FString& Name) const;
	void GetAllCameras(TArray<URenderCameraComponent*>& Out) const;
	URenderCameraComponent* GetCamera(int32 Index) const;
	URenderCameraComponent* GetUserCamera() const { return UserCamera.Get(); }
	int32 NumCameras() const { return Cameras.Num(); }
	void ClearHistory();
private:
	bool BuildOne(USceneComponent* AttachRoot, const FRenderCameraDesc& Desc,
		const FTransform& InitialTransform, bool bInitiallyVisible,
		URenderCameraComponent*& OutCamera, FString& OutError);
	FString SceneRevision;
	int32 ExpectedBodies = 0;
	int32 ExpectedLights = 0;
	FRenderFrameIdentity AppliedIdentity;
	UPROPERTY(Transient)
	TArray<TObjectPtr<URenderCameraComponent>> Cameras;
	UPROPERTY(Transient)
	TObjectPtr<URenderCameraComponent> UserCamera = nullptr;
};
