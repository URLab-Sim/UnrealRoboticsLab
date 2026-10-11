// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Render/MjRenderResourceShim.h"
#include "MuJoCo/Runtime/MjSimulationSnapshot.h"
#include "Render/RenderFrame.h"
#include "Render/RenderScene.h"

struct mjData_;
struct mjModel_;

struct FMjResolvedRenderCameraOverride
{
	FRenderCameraCaptureSettings Capture;
	bool bEnableZmq = false;
	bool bEnableShm = false;
	FString ZmqEndpoint;
	float CustomNearClipCm = 0.0f;
	float CustomFarClipCm = 0.0f;
};
using FMjResolvedRenderCameraOverrides = TMap<int32, FMjResolvedRenderCameraOverride>;

/** MuJoCo adapter for owned neutral rigid scene and Cartesian-frame extraction. */
class URLAB_API FMjRenderSceneAdapter
{
public:
	/** Extract resources and a finalized UE-space rigid scene without mutating Out on failure. */
	bool Extract(const mjModel_* Model, const mjData_* InitialData, const FString& SceneRevision,
		int32 VisibleGroupMask,
		int32 CameraMaxHeight, bool bIncludeUserCamera,
		const FMjResolvedRenderCameraOverrides* CameraOverrides,
		FRenderScene& OutScene,
		FRenderSceneResources& OutResources, FString& OutError);

	/** Copy a complete body pose from mjData into this adapter's scene revision. */
	bool FrameFromData(const mjData_* Data, uint64 FrameId, FRenderFrame& Out,
		FString& OutError) const;

	/** Convert exact-sized canonical RHS metre/+wxyz body arrays into a neutral frame. */
	bool FrameFromCartesian(TConstArrayView<double> BodyPositions,
		TConstArrayView<double> BodyQuaternions, TConstArrayView<double> LightPositions,
		TConstArrayView<double> LightDirections, uint64 FrameId, double SourceTime,
		bool bHasSourceTime, uint64 ClockEpoch, FRenderFrame& Out, FString& OutError) const;
	/** Explicit camera world poses for tracking cameras and remote producers. */
	bool FrameFromCartesianWithCameras(TConstArrayView<double> BodyPositions,
		TConstArrayView<double> BodyQuaternions, TConstArrayView<double> CameraPositions,
		TConstArrayView<double> CameraQuaternions, TConstArrayView<double> LightPositions,
		TConstArrayView<double> LightDirections, uint64 FrameId, double SourceTime,
		bool bHasSourceTime, uint64 ClockEpoch, FRenderFrame& Out, FString& OutError) const;
	/** Validate an owned runtime snapshot against the installed model generation and
	 * convert it without carrying native identity into the neutral render frame. */
	bool FrameFromSnapshot(const FMjSimulationCartesianSnapshot& Snapshot,
		uint64 ExpectedModelGeneration, uint64 FrameId, FRenderFrame& Out,
		FString& OutError) const;
	/** Convert a canonical URLab RHS metre eye/forward/up triple into a UE capture pose. */
	static bool UserCameraTransform(const double* Position, const double* Forward,
		const double* Up, FTransform& Out, FString& OutError);
	/** Change only the local scene revision while retaining the installed model mapping. */
	bool RebaseSceneRevision(const FString& ExpectedRevision, const FRenderScene& PatchedScene,
		FString& OutError);

	const FString& GetSceneRevision() const { return SceneRevision; }
	int32 GetNumBodies() const { return NumBodies; }

private:
	FString SceneRevision;
	int32 NumBodies = 0;
	struct FCameraMountPlan
	{
		int32 BodyIndex = INDEX_NONE;
		FTransform BodyLocalCaptureTransform = FTransform::Identity;
		bool bRequiresExplicitWorldPose = false;
	};
	TArray<FCameraMountPlan> CameraMountPlans;
	TArray<FRenderDeformableDesc> Deformables;
	/** Installed neutral feature selection, including owner-authorized patches. */
	FRenderVisualizationConfig Visualization;
	int32 NumLights = 0;
	bool bHasUserCamera = false;
	uint64 ModelSignature = 0;
	/** Borrowed from the model owner for adapter calculations only; never crosses the boundary. */
	const mjModel_* InstalledModel = nullptr;
};
