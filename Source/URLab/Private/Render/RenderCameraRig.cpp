// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Render/RenderCameraRig.h"
#include "Render/RenderCameraComponent.h"
#include "Components/SceneComponent.h"

bool URenderCameraRig::BuildOne(USceneComponent* AttachRoot, const FRenderCameraDesc& Desc,
	const FTransform& InitialTransform, bool bInitiallyVisible,
	URenderCameraComponent*& OutCamera, FString& OutError)
{
	OutCamera = NewObject<URenderCameraComponent>(AttachRoot ? AttachRoot->GetOwner() : nullptr);
	if (!OutCamera || !OutCamera->Configure(Desc, OutError)) return false;
	OutCamera->SetupAttachment(AttachRoot);
	OutCamera->SetRelativeTransform(InitialTransform);
	OutCamera->SetSceneVisibility(bInitiallyVisible);
	OutCamera->RegisterComponent();
	return true;
}

bool URenderCameraRig::Build(USceneComponent* AttachRoot, const FRenderScene& Scene,
	const FRenderFrame& InitialFrame, bool bInitiallyVisible, FString& OutError)
{
	OutError.Reset();
	if (!AttachRoot || !InitialFrame.IsValid(Scene.Bodies.Num(), Scene.Cameras.Num(),
		Scene.UserCamera.IsSet(), Scene.Lights.Num(), Scene.SceneRevision, OutError)) return false;
	TArray<TObjectPtr<URenderCameraComponent>> Candidate;
	Candidate.SetNum(Scene.Cameras.Num());
	TObjectPtr<URenderCameraComponent> CandidateUser = nullptr;
	for (int32 Index = 0; Index < Scene.Cameras.Num(); ++Index)
	{
		URenderCameraComponent* Camera = nullptr;
		if (!BuildOne(AttachRoot, Scene.Cameras[Index], InitialFrame.CameraSceneTransforms[Index],
			bInitiallyVisible, Camera, OutError))
		{
			for (URenderCameraComponent* Built : Candidate) if (Built) Built->DestroyComponent();
			return false;
		}
		Camera->SetAppliedFrameIdentity(InitialFrame.Identity);
		Candidate[Index] = Camera;
	}
	if (Scene.UserCamera.IsSet())
	{
		URenderCameraComponent* Camera = nullptr;
		if (!BuildOne(AttachRoot, Scene.UserCamera.GetValue(),
			InitialFrame.UserCameraSceneTransform.GetValue(), bInitiallyVisible, Camera, OutError))
		{
			for (URenderCameraComponent* Built : Candidate) if (Built) Built->DestroyComponent();
			return false;
		}
		Camera->SetAppliedFrameIdentity(InitialFrame.Identity);
		CandidateUser = Camera;
	}
	Teardown();
	SceneRevision = Scene.SceneRevision;
	ExpectedBodies = Scene.Bodies.Num();
	ExpectedLights = Scene.Lights.Num();
	AppliedIdentity = InitialFrame.Identity;
	Cameras = MoveTemp(Candidate);
	UserCamera = CandidateUser;
	return true;
}

bool URenderCameraRig::ApplyFrame(const FRenderFrame& Frame, FString& OutError)
{
	if (!Frame.IsValid(ExpectedBodies, Cameras.Num(), UserCamera != nullptr,
		ExpectedLights, SceneRevision, OutError)) return false;
	for (int32 Index = 0; Index < Cameras.Num(); ++Index)
	{
		Cameras[Index]->SetRelativeTransform(Frame.CameraSceneTransforms[Index]);
		Cameras[Index]->SetAppliedFrameIdentity(Frame.Identity);
	}
	if (UserCamera)
	{
		UserCamera->SetRelativeTransform(Frame.UserCameraSceneTransform.GetValue());
		UserCamera->SetAppliedFrameIdentity(Frame.Identity);
	}
	AppliedIdentity = Frame.Identity;
	OutError.Reset();
	return true;
}

void URenderCameraRig::SetVisible(bool bVisible)
{
	auto Apply = [bVisible](URenderCameraComponent* Camera)
	{
		if (!Camera) return;
		Camera->SetSceneVisibility(bVisible);
	};
	for (URenderCameraComponent* Camera : Cameras) Apply(Camera);
	Apply(UserCamera);
}

void URenderCameraRig::Teardown()
{
	for (URenderCameraComponent* Camera : Cameras)
	{
		if (Camera) { Camera->SetStreamingEnabled(false); Camera->DestroyComponent(); }
	}
	if (UserCamera) { UserCamera->SetStreamingEnabled(false); UserCamera->DestroyComponent(); }
	Cameras.Reset(); UserCamera = nullptr; SceneRevision.Reset(); ExpectedBodies = 0;
	ExpectedLights = 0;
	AppliedIdentity = {};
}

int32 URenderCameraRig::SetCaptureMode(ERenderCameraMode Mode)
{
	TArray<URenderCameraComponent*> All; GetAllCameras(All); return SetCaptureModeFor(All, Mode);
}

int32 URenderCameraRig::SetCaptureModeFor(const TArray<URenderCameraComponent*>& Subset,
	ERenderCameraMode Mode)
{
	int32 Changed = 0;
	for (URenderCameraComponent* Camera : Subset)
		if (Camera && Camera->ApplyCaptureMode(Mode)) ++Changed;
	return Changed;
}

URenderCameraComponent* URenderCameraRig::FindByCanonicalName(const FString& Name) const
{
	URenderCameraComponent* Match = nullptr;
	for (URenderCameraComponent* Camera : Cameras)
	{
		if (Camera && Camera->GetCanonicalName() == Name)
		{
			if (Match) return nullptr;
			Match = Camera;
		}
	}
	if (UserCamera && UserCamera->GetCanonicalName() == Name)
	{
		if (Match) return nullptr;
		Match = UserCamera;
	}
	return Match;
}

void URenderCameraRig::GetAllCameras(TArray<URenderCameraComponent*>& Out) const
{
	Out.Reset();
	for (URenderCameraComponent* Camera : Cameras) if (Camera) Out.Add(Camera);
	if (UserCamera) Out.Add(UserCamera);
}

URenderCameraComponent* URenderCameraRig::GetCamera(int32 Index) const
{
	return Cameras.IsValidIndex(Index) ? Cameras[Index].Get() : nullptr;
}

void URenderCameraRig::ClearHistory()
{
	for (URenderCameraComponent* Camera : Cameras) if (Camera) Camera->ClearHistory();
	if (UserCamera) UserCamera->ClearHistory();
}
