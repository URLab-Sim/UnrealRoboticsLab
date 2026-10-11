// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjModelDescriptor.h"

struct mjModel_;
struct mjData_;
typedef mjModel_ mjModel;
typedef mjData_ mjData;

/** Primary representation from which an owned MuJoCo model was prepared. */
enum class EMjModelSourceKind : uint8
{
	Mjb,
	Mjz,
	XmlWithAssets,
	CompiledModel
};

/** Owned source metadata without source buffers or native pointers. */
struct URLAB_API FMjModelSourceInfo
{
	EMjModelSourceKind Kind = EMjModelSourceKind::Mjb;
	FString Label;
	int64 PrimaryByteCount = 0;
	int32 AssetCount = 0;
	int64 AssetByteCount = 0;
};

/**
 * Move-only prepared model/data pair shared by simulation and render workflows.
 *
 * Factories compile or copy into candidate-owned memory, allocate mjData, run one
 * initial mj_forward (never mj_step), and build an owned model descriptor. Failed
 * factories leave the caller's output candidate unchanged.
 */
class URLAB_API FMjOwnedModelCandidate
{
public:
	FMjOwnedModelCandidate();
	~FMjOwnedModelCandidate();
	FMjOwnedModelCandidate(FMjOwnedModelCandidate&& Other) noexcept;
	FMjOwnedModelCandidate& operator=(FMjOwnedModelCandidate&& Other) noexcept;

	FMjOwnedModelCandidate(const FMjOwnedModelCandidate&) = delete;
	FMjOwnedModelCandidate& operator=(const FMjOwnedModelCandidate&) = delete;

	static bool FromBytes(EMjModelSourceKind Kind,
		const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets,
		FMjOwnedModelCandidate& OutCandidate, FString& OutError,
		const FString& SourceLabel = FString());

	/** Copies Source with mj_copyModel; ownership of Source never transfers. */
	static bool CopyCompiled(const mjModel* Source, FMjOwnedModelCandidate& OutCandidate,
		FString& OutError, const FString& SourceLabel = FString());

	bool IsValid() const;
	const FMjModelSourceInfo& GetSourceInfo() const { return SourceInfo; }
	const FMjModelDescriptor& GetDescriptor() const { return Descriptor; }

	/** Synchronous candidate-only view. Native pointers cannot be retained. */
	bool VisitPrepared(TFunctionRef<bool(const mjModel*, const mjData*)> Visitor) const;

private:
	friend class FMjSimulationRuntime;
	friend class FMjRenderKinematicsCandidate;
	friend class FMjRenderKinematics;

	static bool PrepareOwned(mjModel* OwnedModel, FMjModelSourceInfo&& SourceInfo,
		FMjOwnedModelCandidate& OutCandidate, FString& OutError);
	void Reset();
	mjModel* Model = nullptr;
	mjData* Data = nullptr;
	FMjModelDescriptor Descriptor;
	FMjModelSourceInfo SourceInfo;
};
