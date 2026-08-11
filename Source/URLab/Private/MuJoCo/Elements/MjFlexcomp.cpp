// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjFlexcomp.h"

#include "Components/DynamicMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "StaticMeshResources.h"
#include "UDynamicMesh.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Spec/MjBinding.h"
#include "MuJoCo/Utils/URLabAxisConv.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
/** The name flex `Id` carries in the compiled model, or empty. */
FString FlexNameAt(const mjModel& Model, int32 Id)
{
	const char* Name = mj_id2name(&Model, mjOBJ_FLEX, Id);
	return Name != nullptr ? FString(UTF8_TO_TCHAR(Name)) : FString();
}

/** LOD 0 of a static mesh component, or null when it has no render data. */
const FStaticMeshLODResources* SourceLod(const UStaticMeshComponent& Component)
{
	const UStaticMesh* Asset = Component.GetStaticMesh();
	const FStaticMeshRenderData* Render = Asset != nullptr ? Asset->GetRenderData() : nullptr;
	if (Render == nullptr || Render->LODResources.Num() == 0)
	{
		return nullptr;
	}
	return &Render->LODResources[0];
}
} // namespace

UMjFlexcomp::UMjFlexcomp()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjFlexcomp::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	const mjModel* Model = Engine != nullptr ? Engine->GetModel() : nullptr;
	if (Model == nullptr)
	{
		// No model means the surface is driven by nothing; showing the last
		// frame it happened to hold would be a lie about a simulation that is
		// not running.
		ReleaseProceduralMesh();
		return;
	}
	if (!EnsureFlex(*Model))
	{
		return;
	}
	UpdateProceduralMesh(*Engine);
}

void UMjFlexcomp::OnComponentDestroyed(bool bDestroyingHierarchy)
{
	ReleaseProceduralMesh();
	Super::OnComponentDestroyed(bDestroyingHierarchy);
}

UStaticMeshComponent* UMjFlexcomp::FindSourceMesh() const
{
	TArray<USceneComponent*> Children;
	GetChildrenComponents(false, Children);
	for (USceneComponent* Child : Children)
	{
		UStaticMeshComponent* Mesh = Cast<UStaticMeshComponent>(Child);
		if (Mesh != nullptr && Mesh->GetStaticMesh() != nullptr)
		{
			return Mesh;
		}
	}
	return nullptr;
}

bool UMjFlexcomp::EnsureFlex(const mjModel& Model)
{
	if (FlexId < 0 || FlexId >= Model.nflex || FlexNameAt(Model, FlexId) != ResolvedFlexName)
	{
		ReleaseProceduralMesh();

		const FString Name = MjName.Get(FString());
		if (Name.IsEmpty())
		{
			return false;
		}

		// The expansion exists in the model and not in the tree, so this
		// element has no compiled id to index off. The flex carries the
		// flexcomp's name, under whatever prefix the scene gave this
		// participant when it attached it, which is the owning actor's.
		FMjBinding Index;
		Index.SetModel(&Model);

		const AActor* Owner = GetOwner();
		const FString Prefix = Owner != nullptr ? Owner->GetName() + TEXT("_") : FString();
		TArray<int32> Ids = Index.Find(mjOBJ_FLEX, Prefix + Name);
		if (Ids.Num() == 0)
		{
			Ids = Index.Find(mjOBJ_FLEX, Name);
		}
		if (Ids.Num() != 1)
		{
			return false;
		}

		FlexId = Ids[0];
		ResolvedFlexName = FlexNameAt(Model, FlexId);
	}

	FlexVertAdr = Model.flex_vertadr[FlexId];
	FlexVertNum = Model.flex_vertnum[FlexId];

	if (DynamicMesh == nullptr)
	{
		if (bSurfaceAttempted)
		{
			return false;
		}
		bSurfaceAttempted = true;
		if (!BuildWeldMap())
		{
			return false;
		}
		CreateProceduralMesh();
	}
	return DynamicMesh != nullptr;
}

bool UMjFlexcomp::BuildWeldMap()
{
	const UStaticMeshComponent* Source = FindSourceMesh();
	const FStaticMeshLODResources* Lod = Source != nullptr ? SourceLod(*Source) : nullptr;
	if (Lod == nullptr)
	{
		return false;
	}

	// Unreal splits one vertex into several wherever a normal or a UV differs
	// across a face; MuJoCo's flex has one vertex per position. Welding by
	// position recovers the correspondence, and the hash grid is what keeps it
	// linear instead of quadratic in vertex count.
	const FPositionVertexBuffer& Positions = Lod->VertexBuffers.PositionVertexBuffer;
	NumRenderVerts = static_cast<int32>(Positions.GetNumVertices());
	RawToWelded.SetNum(NumRenderVerts);

	constexpr float WeldTolerance = 1e-5f;
	constexpr int32 HashSize = 1 << 14;
	constexpr int32 HashMask = HashSize - 1;

	TArray<FVector3f> Welded;
	TArray<TArray<int32>> Buckets;
	Buckets.SetNum(HashSize);

	auto BucketOf = [](const FVector3f& P) {
		const uint32 H = static_cast<uint32>(P.X * 73856093.f)
					   ^ static_cast<uint32>(P.Y * 19349663.f)
					   ^ static_cast<uint32>(P.Z * 83492791.f);
		return static_cast<int32>(H & HashMask);
	};

	for (int32 Raw = 0; Raw < NumRenderVerts; ++Raw)
	{
		const FVector3f Position = Positions.VertexPosition(Raw);
		TArray<int32>& Bucket = Buckets[BucketOf(Position)];

		int32 Found = INDEX_NONE;
		for (const int32 Candidate : Bucket)
		{
			if (Welded[Candidate].Equals(Position, WeldTolerance))
			{
				Found = Candidate;
				break;
			}
		}
		if (Found == INDEX_NONE)
		{
			Found = Welded.Add(Position);
			Bucket.Add(Found);
		}
		RawToWelded[Raw] = Found;
	}
	return NumRenderVerts > 0;
}

void UMjFlexcomp::CreateProceduralMesh()
{
	AActor* Owner = GetOwner();
	UStaticMeshComponent* Source = FindSourceMesh();
	const FStaticMeshLODResources* Lod = Source != nullptr ? SourceLod(*Source) : nullptr;
	if (Owner == nullptr || Lod == nullptr || NumRenderVerts == 0)
	{
		return;
	}

	// Unique per element, because an actor can carry more than one flexcomp and
	// two components on one actor cannot share a name.
	const FName MeshName = MakeUniqueObjectName(
		Owner, UDynamicMeshComponent::StaticClass(), *(GetName() + TEXT("_FlexMesh")));

	DynamicMesh = NewObject<UDynamicMeshComponent>(Owner, MeshName);
	DynamicMesh->SetupAttachment(Owner->GetRootComponent());
	DynamicMesh->RegisterComponent();
	DynamicMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	// Positions move every frame and nothing recomputes tangents by hand, so
	// the component derives them itself through MikkTSpace.
	DynamicMesh->SetTangentsType(EDynamicMeshComponentTangentsMode::AutoCalculated);

	if (UMaterialInterface* SourceMaterial = Source->GetMaterial(0))
	{
		DynamicMesh->SetMaterial(0, SourceMaterial);
	}

	// The static mesh was only ever the rest pose; the surface is what the
	// viewer should see from here on.
	Source->SetVisibility(false);
	Source->SetHiddenInGame(true);

	const FStaticMeshVertexBuffer& Vertices = Lod->VertexBuffers.StaticMeshVertexBuffer;
	const FPositionVertexBuffer& Positions = Lod->VertexBuffers.PositionVertexBuffer;
	const FIndexArrayView Indices = Lod->IndexBuffer.GetArrayView();
	const bool bHasUVs = Vertices.GetNumTexCoords() > 0;

	DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh) {
		Mesh.Clear();
		Mesh.EnableAttributes();
		Mesh.Attributes()->SetNumNormalLayers(1);
		if (bHasUVs)
		{
			Mesh.Attributes()->SetNumUVLayers(1);
		}

		UE::Geometry::FDynamicMeshNormalOverlay* Normals = Mesh.Attributes()->PrimaryNormals();
		UE::Geometry::FDynamicMeshUVOverlay* UVs = bHasUVs ? Mesh.Attributes()->PrimaryUV() : nullptr;

		// Built at the rest pose, in the source mesh's local space. The first
		// writeback replaces every position; the normals and UVs stay.
		for (int32 Raw = 0; Raw < NumRenderVerts; ++Raw)
		{
			const FVector3f Position = Positions.VertexPosition(Raw);
			Mesh.AppendVertex(FVector3d(Position.X, Position.Y, Position.Z));

			const FVector4f Normal = Vertices.VertexTangentZ(Raw);
			Normals->AppendElement(FVector3f(Normal.X, Normal.Y, Normal.Z));
			if (UVs != nullptr)
			{
				const FVector2f UV = Vertices.GetVertexUV(Raw, 0);
				UVs->AppendElement(FVector2f(UV.X, UV.Y));
			}
		}

		// Raw Unreal indices, so the overlay elements added above are already
		// one per vertex and index identically. Degenerate triangles are
		// dropped: FDynamicMesh3 rejects them and would leave the overlays
		// pointing at a triangle that does not exist.
		for (int32 i = 0; i + 2 < Indices.Num(); i += 3)
		{
			const int32 A = Indices[i];
			const int32 B = Indices[i + 1];
			const int32 C = Indices[i + 2];
			if (A == B || B == C || A == C)
			{
				continue;
			}
			const int32 TriangleId = Mesh.AppendTriangle(A, B, C);
			if (TriangleId >= 0)
			{
				Normals->SetTriangle(TriangleId, UE::Geometry::FIndex3i(A, B, C));
				if (UVs != nullptr)
				{
					UVs->SetTriangle(TriangleId, UE::Geometry::FIndex3i(A, B, C));
				}
			}
		}
	},
		EDynamicMeshComponentRenderUpdateMode::FullUpdate);
}

void UMjFlexcomp::UpdateProceduralMesh(UMjPhysicsEngine& Engine)
{
	if (DynamicMesh == nullptr || NumRenderVerts == 0 || FlexVertNum <= 0)
	{
		return;
	}

	const USceneComponent* MeshParent = DynamicMesh->GetAttachParent();
	const FTransform ParentTransform =
		MeshParent != nullptr ? MeshParent->GetComponentTransform() : FTransform::Identity;

	TArray<FVector> WeldedPositions;
	WeldedPositions.SetNum(FlexVertNum);

	// The snapshot rather than mjData: it is one coherent physics frame for
	// every consumer in this Unreal frame, and reading it never blocks a step.
	bool bHavePositions = false;
	Engine.WithRenderState([&](const FMjRenderSnapshot& Snapshot) {
		const int32 NeededFloats = (FlexVertAdr + FlexVertNum) * 3;
		if (Snapshot.FlexvertXPos.Num() < NeededFloats)
		{
			return;
		}
		for (int32 Vert = 0; Vert < FlexVertNum; ++Vert)
		{
			const int32 Offset = (FlexVertAdr + Vert) * 3;
			const FVector WorldPosition = URLabAxisConv::MjPositionToUe(&Snapshot.FlexvertXPos[Offset]);
			WeldedPositions[Vert] = ParentTransform.InverseTransformPosition(WorldPosition);
		}
		bHavePositions = true;
	});
	if (!bHavePositions)
	{
		return;
	}

	DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh) {
		for (int32 Raw = 0; Raw < NumRenderVerts; ++Raw)
		{
			const int32 Welded = RawToWelded[Raw];
			const FVector Position = (Welded >= 0 && Welded < FlexVertNum)
									   ? WeldedPositions[Welded]
									   : FVector::ZeroVector;
			Mesh.SetVertex(Raw, FVector3d(Position.X, Position.Y, Position.Z));
		}
	},
		EDynamicMeshComponentRenderUpdateMode::NoUpdate);

	// Positions only: the topology, the UVs and the vertex colours are all
	// unchanged, and the tangent mode recomputes what depends on them.
	DynamicMesh->FastNotifyPositionsUpdated(/*bNormals=*/false, /*bColors=*/false, /*bUVs=*/false);
}

void UMjFlexcomp::ReleaseProceduralMesh()
{
	if (DynamicMesh != nullptr)
	{
		DynamicMesh->DestroyComponent();
		DynamicMesh = nullptr;

		// The source mesh gave up being the visible one in favour of the
		// surface; with the surface gone it takes the job back.
		if (UStaticMeshComponent* Source = FindSourceMesh())
		{
			Source->SetVisibility(true);
			Source->SetHiddenInGame(false);
		}
	}

	RawToWelded.Reset();
	NumRenderVerts = 0;
	FlexId = INDEX_NONE;
	FlexVertAdr = 0;
	FlexVertNum = 0;
	ResolvedFlexName.Reset();
	bSurfaceAttempted = false;
}
