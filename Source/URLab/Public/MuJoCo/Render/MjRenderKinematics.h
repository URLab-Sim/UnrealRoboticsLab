// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "MuJoCo/Core/MjModelSourcePayload.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"

struct FMjRenderKinematicsOptions
{
	FMjResolvedRenderCameraOverrides CameraOverrides;
	int32 VisibleGroupMask = 0b0000111;
	int32 CameraMaxHeight = -1;
	bool bIncludeUserCamera = false;
};

/** Move-only prepared render model and its neutral scene inputs. */
class URLAB_API FMjRenderKinematicsCandidate
{
public:
	FMjRenderKinematicsCandidate();
	~FMjRenderKinematicsCandidate();
	FMjRenderKinematicsCandidate(FMjRenderKinematicsCandidate&& Other) noexcept;
	FMjRenderKinematicsCandidate& operator=(FMjRenderKinematicsCandidate&& Other) noexcept;

	FMjRenderKinematicsCandidate(const FMjRenderKinematicsCandidate&) = delete;
	FMjRenderKinematicsCandidate& operator=(const FMjRenderKinematicsCandidate&) = delete;

	static bool Prepare(FMjOwnedModelCandidate&& ModelCandidate,
		const FString& SceneRevision, const FMjRenderKinematicsOptions& Options,
		FMjRenderKinematicsCandidate& OutCandidate, FString& OutError);
	static bool Prepare(FMjModelSourcePayload&& Source,
		const FString& SceneRevision, const FMjRenderKinematicsOptions& Options,
		FMjRenderKinematicsCandidate& OutCandidate, FString& OutError);

	bool IsValid() const;
	const FString& GetSceneRevision() const { return SceneRevision; }
	const FMjModelSourceInfo& GetSourceInfo() const { return ModelCandidate.GetSourceInfo(); }

	/** Moves the heavyweight neutral scene package exactly once. */
	bool TakeNeutralScene(FRenderScene& OutScene, FRenderSceneResources& OutResources,
		FRenderFrame& OutInitialFrame, FString& OutError);

private:
	friend class FMjRenderKinematics;
	void Reset();
	FMjOwnedModelCandidate ModelCandidate;
	TUniquePtr<FMjRenderSceneAdapter> Adapter;
	mjData* ScratchData = nullptr;
	FString SceneRevision;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame InitialFrame;
	bool bNeutralSceneTaken = false;
};

/**
 * Thread-safe render-only MuJoCo FK adapter.
 *
 * Each qpos request starts from the installed model's initial mjData defaults,
 * replaces qpos, and runs mj_forward only. Controls, mocap, plugin state, and
 * other mutable data never carry over between requests; later explicit pose
 * contracts own those inputs.
 */
class URLAB_API FMjRenderKinematics
{
public:
	FMjRenderKinematics();
	~FMjRenderKinematics();
	FMjRenderKinematics(const FMjRenderKinematics&) = delete;
	FMjRenderKinematics& operator=(const FMjRenderKinematics&) = delete;

	/** Empty expected revision means initial install; replacement requires exact active revision. */
	bool Commit(FMjRenderKinematicsCandidate&& Candidate,
		const FString& ExpectedCurrentSceneRevision, FString& OutError);
	/** Preserve the installed MuJoCo model/FK mapping while committing a neutral
	 *  owner-authorized scene revision change. */
	bool RebaseSceneRevision(const FString& ExpectedRevision, const FRenderScene& PatchedScene,
		FString& OutError);

	bool FrameFromQpos(const FString& SceneRevision, TConstArrayView<double> Qpos,
		uint64 FrameId, double SourceTime, bool bHasSourceTime, uint64 ClockEpoch,
		FRenderFrame& OutFrame, FString& OutError);

	bool HasModel() const;
	FString GetSceneRevision() const;
	int32 GetQposWidth() const;
	bool GetSourceInfo(FMjModelSourceInfo& OutSource) const;

private:
	struct FInstalled;
	mutable FCriticalSection Mutex;
	TUniquePtr<FInstalled> Installed;
};
