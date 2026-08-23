// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjFlexcomp.h"

#include "CoreGlobals.h"
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

// --- Trilinear cage interpolation, inlined from MuJoCo -----------------------
// mju_interpolate3D is MJAPI but declared in a private header (engine_util_misc.h),
// so the ~15-line trilinear weights are reproduced here verbatim rather than
// linked (source-of-truth 8.5 / plan 9.6). flex_vert0 (the query point) is static,
// so this is a pure function of the streamed node cage.

/** 1-D Lagrange/serendipity basis, MuJoCo's phi (engine_util_misc.c:536). */
double FlexPhi(double s, int i, int order)
{
	if (order == 1)
	{
		return (i == 0) ? (1.0 - s) : s;
	}
	// order == 2
	switch (i)
	{
		case 0:
			return 2.0 * s * s - 3.0 * s + 1.0;
		case 1:
			return 4.0 * (s - s * s);
		case 2:
			return 2.0 * s * s - s;
		default:
			return 0.0;
	}
}

/** 3-D tensor-product basis, MuJoCo's mju_evalBasis (engine_util_misc.c:616). */
double FlexEvalBasis(const double x[3], int i, int order)
{
	if (order == 1)
	{
		return FlexPhi(x[2], i & 1, order) * FlexPhi(x[1], i & 2, order) * FlexPhi(x[0], i & 4, order);
	}
	// order == 2
	return FlexPhi(x[2], i % 3, order) * FlexPhi(x[1], (i / 3) % 3, order) * FlexPhi(x[0], i / 9, order);
}

/** res = sum_j coeff[j] * basis(x, j, order); mju_interpolate3D (engine_util_misc.c:616).
 *  coeff is the (order+1)^3 node world positions, x is the vertex's [0,1]^3 coords. */
void FlexInterpolate3D(double res[3], const double x[3], const double* coeff, int order)
{
	const int npoint = (order + 1) * (order + 1) * (order + 1);
	for (int j = 0; j < npoint; ++j)
	{
		const double w = FlexEvalBasis(x, j, order);
		res[0] += coeff[3 * j + 0] * w;
		res[1] += coeff[3 * j + 1] * w;
		res[2] += coeff[3 * j + 2] * w;
	}
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
		// No local engine. On a mirror (Drive=stream/push) the surface is driven
		// externally from the streamed per-body transforms via
		// UpdateFromBodyTransforms. Once that has ever happened this is a mirror
		// element: keep the surface and let the external driver refresh it (it
		// holds its last pose across stream gaps, exactly like the mirror's geoms),
		// so this component's own tick never fights the renderer-driven update.
		// A component that was never externally driven and has no engine is a
		// genuinely gone producer -- release it, as showing a stale frame would lie.
		if (LastExternalDriveFrame != 0)
		{
			return;
		}
		ReleaseProceduralMesh();
		return;
	}
	if (!EnsureFlex(*Model))
	{
		return;
	}
	UpdateProceduralMesh(*Engine);
}

void UMjFlexcomp::SetMirrorFlex(int32 InFlexId)
{
	MirrorFlexId = InFlexId;
	// Purely renderer-driven (AMjRenderer::UpdateMirrorFlex): a mirror has no
	// local engine to tick against, and the one thing the authored tick would do
	// -- release the surface while no engine resolves -- would fight the streamed
	// drive. Set before registration, so disabling the tick here sticks.
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;
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

		if (MirrorFlexId != INDEX_NONE)
		{
			// A renderer-created mirror element draws the flex index it was
			// assigned at creation (one element per compiled flex); there is no
			// authored name to resolve against the model.
			if (MirrorFlexId < 0 || MirrorFlexId >= Model.nflex)
			{
				return false;
			}
			FlexId = MirrorFlexId;
			ResolvedFlexName = FlexNameAt(Model, FlexId);
		}
		else
		{
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
	}

	FlexVertAdr = Model.flex_vertadr[FlexId];
	FlexVertNum = Model.flex_vertnum[FlexId];

	if (DynamicMesh == nullptr)
	{
		if (MirrorFlexId != INDEX_NONE)
		{
			// The ids above are resolved, but a renderer-created element has no
			// child static mesh: its surface is built from the model topology at
			// the first streamed frame's positions (BuildModelSurface, called by
			// UpdateFromBodyTransforms). No surface yet.
			return false;
		}
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

	// The same triangles in welded ids, one entry per drawn face, for the
	// per-frame normal recompute in ApplyFlexWorldPositions. Welded rather than
	// raw, so a vertex split at a UV seam still shades smoothly: every raw copy
	// reads the one normal accumulated for its welded MuJoCo vertex. Stored as
	// (A, C, B): a UE static mesh winds counter-clockwise in the left-handed
	// system, so its outward normal is Cross(C-A, B-A) (VectorUtil::Normal,
	// FStaticMeshOperations) -- the swap makes the recompute's shared
	// Cross(B-A, C-A) accumulation come out outward here too. Because a sign
	// slip here shades the whole surface black, the order is not trusted but
	// voted on against the asset's own authored normals, one whole-mesh flip by
	// majority (the MjRendererAssetBaker OrientVote pattern). Triangles that
	// collapse under the weld would contribute a zero cross product, so they
	// are dropped here instead of re-tested every frame.
	NormalTris.Reset();
	NormalTris.Reserve(Indices.Num() / 3);
	int64 OrientVote = 0;
	for (int32 i = 0; i + 2 < Indices.Num(); i += 3)
	{
		const int32 RawA = Indices[i];
		const int32 RawB = Indices[i + 1];
		const int32 RawC = Indices[i + 2];
		const int32 A = RawToWelded.IsValidIndex(RawA) ? RawToWelded[RawA] : INDEX_NONE;
		const int32 B = RawToWelded.IsValidIndex(RawB) ? RawToWelded[RawB] : INDEX_NONE;
		const int32 C = RawToWelded.IsValidIndex(RawC) ? RawToWelded[RawC] : INDEX_NONE;
		if (A < 0 || B < 0 || C < 0 || A == B || B == C || A == C)
		{
			continue;
		}
		NormalTris.Add(UE::Geometry::FIndex3i(A, C, B));

		// (A, C, B) through Cross(B-A, C-A) is the geometric normal below.
		const FVector3f P0 = Positions.VertexPosition(RawA);
		const FVector3f Geometric = FVector3f::CrossProduct(
			Positions.VertexPosition(RawC) - P0, Positions.VertexPosition(RawB) - P0);
		const FVector4f N0 = Vertices.VertexTangentZ(RawA);
		const FVector4f N1 = Vertices.VertexTangentZ(RawB);
		const FVector4f N2 = Vertices.VertexTangentZ(RawC);
		const FVector3f Authored(N0.X + N1.X + N2.X, N0.Y + N1.Y + N2.Y, N0.Z + N1.Z + N2.Z);
		const float Dot = FVector3f::DotProduct(Geometric, Authored);
		if (Dot != 0.0f)
		{
			OrientVote += (Dot > 0.0f) ? 1 : -1;
		}
	}
	if (OrientVote < 0)
	{
		for (UE::Geometry::FIndex3i& T : NormalTris)
		{
			Swap(T.B, T.C);
		}
	}
}

void UMjFlexcomp::UpdateProceduralMesh(UMjPhysicsEngine& Engine)
{
	if (DynamicMesh == nullptr || NumRenderVerts == 0 || FlexVertNum <= 0)
	{
		return;
	}

	TArray<FVector> WorldPositions;
	WorldPositions.SetNum(FlexVertNum);

	// The snapshot rather than mjData: it is one coherent physics frame for
	// every consumer in this Unreal frame, and reading it never blocks a step.
	// This is the producer (sim) fast path -- mj_flex already ran on the engine
	// and its flexvert_xpos was copied into the snapshot.
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
			WorldPositions[Vert] = URLabAxisConv::MjPositionToUe(&Snapshot.FlexvertXPos[Offset]);
		}
		bHavePositions = true;
	});
	if (!bHavePositions)
	{
		return;
	}

	ApplyFlexWorldPositions(WorldPositions);
}

void UMjFlexcomp::ApplyFlexWorldPositions(const TArray<FVector>& WorldPositions)
{
	if (DynamicMesh == nullptr || NumRenderVerts == 0 || FlexVertNum <= 0
		|| WorldPositions.Num() < FlexVertNum)
	{
		return;
	}

	const USceneComponent* MeshParent = DynamicMesh->GetAttachParent();
	const FTransform ParentTransform =
		MeshParent != nullptr ? MeshParent->GetComponentTransform() : FTransform::Identity;

	TArray<FVector> WeldedPositions;
	WeldedPositions.SetNum(FlexVertNum);
	for (int32 Vert = 0; Vert < FlexVertNum; ++Vert)
	{
		WeldedPositions[Vert] = ParentTransform.InverseTransformPosition(WorldPositions[Vert]);
	}

	// Area-weighted vertex normals at this deformed pose, over the same
	// one-winding welded triangles the surface's normals were built from
	// (NormalTris), in the same mesh-local space the positions above are in --
	// so the shading follows the deformation instead of freezing at the build
	// pose. Same accumulation as BuildModelSurface: N = (B-A) x (C-A), summed
	// unnormalized so triangle area is the weight. Indices are validated per
	// frame because a model recompile can resize the flex under a stored list.
	TArray<FVector> WeldedNormals;
	if (NormalTris.Num() > 0)
	{
		WeldedNormals.Init(FVector::ZeroVector, FlexVertNum);
		for (const UE::Geometry::FIndex3i& T : NormalTris)
		{
			if (T.A < 0 || T.A >= FlexVertNum || T.B < 0 || T.B >= FlexVertNum
				|| T.C < 0 || T.C >= FlexVertNum)
			{
				continue;
			}
			const FVector N = FVector::CrossProduct(
				WeldedPositions[T.B] - WeldedPositions[T.A],
				WeldedPositions[T.C] - WeldedPositions[T.A]);
			WeldedNormals[T.A] += N;
			WeldedNormals[T.B] += N;
			WeldedNormals[T.C] += N;
		}
	}

	bool bWroteNormals = false;
	DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh) {
		for (int32 Raw = 0; Raw < NumRenderVerts; ++Raw)
		{
			const int32 Welded = RawToWelded[Raw];
			const FVector Position = (Welded >= 0 && Welded < FlexVertNum)
									   ? WeldedPositions[Welded]
									   : FVector::ZeroVector;
			Mesh.SetVertex(Raw, FVector3d(Position.X, Position.Y, Position.Z));
		}

		// Both build paths append exactly one overlay element per raw vertex on
		// a fresh mesh, so element id == raw vertex id; anything else means the
		// overlay is not ours to rewrite and the last normals are kept.
		UE::Geometry::FDynamicMeshNormalOverlay* Normals =
			(WeldedNormals.Num() > 0 && Mesh.HasAttributes())
				? Mesh.Attributes()->PrimaryNormals()
				: nullptr;
		if (Normals != nullptr && Normals->ElementCount() == NumRenderVerts)
		{
			for (int32 Raw = 0; Raw < NumRenderVerts; ++Raw)
			{
				const int32 Welded = RawToWelded[Raw];
				if (Welded < 0 || Welded >= FlexVertNum)
				{
					continue;
				}
				const FVector Nn = WeldedNormals[Welded].GetSafeNormal();
				if (Nn.IsNearlyZero())
				{
					// Every incident triangle degenerate (or cancelled) this
					// frame: keep the previous normal rather than writing zero.
					continue;
				}
				Normals->SetElement(Raw, FVector3f(static_cast<float>(Nn.X),
											 static_cast<float>(Nn.Y), static_cast<float>(Nn.Z)));
			}
			bWroteNormals = true;
		}
	},
		EDynamicMeshComponentRenderUpdateMode::NoUpdate);

	// Positions plus the normals recomputed from them: the topology, the UVs and
	// the vertex colours are all unchanged. bNormals=true makes the proxy's
	// FastUpdateVertices re-read the PrimaryNormals overlay written above and
	// re-upload it -- it never recomputes normals itself.
	DynamicMesh->FastNotifyPositionsUpdated(/*bNormals=*/bWroteNormals, /*bColors=*/false, /*bUVs=*/false);
}

void UMjFlexcomp::UpdateFromBodyTransforms(const mjModel& Model, const double* Bxpos,
	const double* Bxquat, int32 NBody, const FVector& WorldOffset)
{
	if (Bxpos == nullptr || Bxquat == nullptr || NBody <= 0)
	{
		return;
	}
	// Resolve + build the surface on first sight, exactly like the producer tick.
	// A renderer-created mirror element (SetMirrorFlex) has no child static mesh
	// for EnsureFlex to build from: EnsureFlex resolves its ids and reports "no
	// surface", and the surface is built below from the model's own topology at
	// this frame's reconstructed positions instead (the UMjSkincomp pattern).
	const bool bHaveSurface = EnsureFlex(Model);
	if (!bHaveSurface && (MirrorFlexId == INDEX_NONE || bSurfaceAttempted))
	{
		return;
	}
	if (FlexId < 0 || FlexId >= Model.nflex || FlexVertNum <= 0
		|| (bHaveSurface && (DynamicMesh == nullptr || NumRenderVerts == 0)))
	{
		return;
	}

	auto SafeBody = [NBody](int b) { return b >= 0 && b < NBody; };

	TArray<FVector> WorldPositions;
	WorldPositions.SetNum(FlexVertNum);

	double V[3];

	// mj_flex, engine_core_smooth.c:558-608, replayed with no mjData: the flex
	// vertices are a pure function of the static model and the per-body transforms
	// already on the wire (bxpos/bxquat). xmat . v is computed as rotate(v, quat).
	if (Model.flex_interp[FlexId] == 0)
	{
		// Vertex mode: each flex vertex tracks one body.
		const bool bCentered = Model.flex_centered != nullptr && Model.flex_centered[FlexId] != 0;
		for (int32 i = 0; i < FlexVertNum; ++i)
		{
			const int32 Gi = FlexVertAdr + i;
			const int B = Model.flex_vertbodyid[Gi];
			if (!SafeBody(B))
			{
				WorldPositions[i] = FVector::ZeroVector;
				continue;
			}
			if (bCentered)
			{
				// Centered (the flexcomp common case): copy the body position.
				mju_copy3(V, Bxpos + 3 * B);
			}
			else
			{
				// Non-centered: map the local vertex into the world body frame.
				mju_rotVecQuat(V, Model.flex_vert + 3 * Gi, Bxquat + 4 * B);
				mju_addTo3(V, Bxpos + 3 * B);
			}
			WorldPositions[i] = URLabAxisConv::MjPositionToUe(V);
		}
	}
	else
	{
		// Trilinear (order 1 = 8-node, order 2 = 27-node cage): build the node
		// world positions, then interpolate every render vertex from them.
		const int Order = Model.flex_interp[FlexId];
		const int NStart = Model.flex_nodeadr[FlexId];
		const int NNum = Model.flex_nodenum[FlexId];
		const int NPoint = (Order + 1) * (Order + 1) * (Order + 1);
		if (Order < 1 || Order > 2 || NNum != NPoint || NNum > mjMAXFLEXNODES)
		{
			return; // malformed cage -- leave the surface at its last pose
		}
		const bool bCentered = Model.flex_centered != nullptr && Model.flex_centered[FlexId] != 0;

		double NodeXPos[3 * mjMAXFLEXNODES];
		for (int j = 0; j < NNum; ++j)
		{
			const int Ni = NStart + j;
			const int B = Model.flex_nodebodyid[Ni];
			if (!SafeBody(B))
			{
				mju_zero3(NodeXPos + 3 * j);
				continue;
			}
			if (bCentered)
			{
				mju_copy3(NodeXPos + 3 * j, Bxpos + 3 * B);
			}
			else
			{
				mju_rotVecQuat(NodeXPos + 3 * j, Model.flex_node + 3 * Ni, Bxquat + 4 * B);
				mju_addTo3(NodeXPos + 3 * j, Bxpos + 3 * B);
			}
		}

		for (int32 i = 0; i < FlexVertNum; ++i)
		{
			const int32 Gi = FlexVertAdr + i;
			mju_zero3(V);
			FlexInterpolate3D(V, Model.flex_vert0 + 3 * Gi, NodeXPos, Order);
			WorldPositions[i] = URLabAxisConv::MjPositionToUe(V);
		}
	}

	// A renderer-created element renders in the mirror's scene frame: apply the
	// same world offset ApplyBodyTransforms adds to every rigid geom (authored
	// elements are passed zero, keeping their behaviour unchanged).
	if (!WorldOffset.IsNearlyZero())
	{
		for (FVector& P : WorldPositions)
		{
			P += WorldOffset;
		}
	}

	if (DynamicMesh == nullptr)
	{
		// First streamed frame on a renderer-created element: build the surface
		// from the model topology at these positions, so the static shading
		// normals are computed on real geometry rather than a placeholder pose.
		bSurfaceAttempted = true;
		BuildModelSurface(Model, WorldPositions);
		if (DynamicMesh == nullptr)
		{
			return;
		}
		LastExternalDriveFrame = GFrameCounter;
		return;
	}

	LastExternalDriveFrame = GFrameCounter;
	ApplyFlexWorldPositions(WorldPositions);
}

void UMjFlexcomp::BuildModelSurface(const mjModel& Model, const TArray<FVector>& WorldPositions)
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr || FlexId < 0 || FlexId >= Model.nflex || FlexVertNum <= 0
		|| WorldPositions.Num() < FlexVertNum)
	{
		return;
	}

	// The drawable surface, in vertex ids local to this flex (flex_elem and
	// flex_shell index the flex's own vertex block -- mjv_updateActiveFlex,
	// engine_vis_visualize.c:3252,3341-3359): a 2D flex's elements are its
	// triangles and draw both windings, exactly like MuJoCo's flexskin mode; a
	// 3D flex draws its outward-oriented shell fragments. A 1D flex (lines) has
	// no surface to mesh. Normals accumulate over one winding only -- the
	// mirrored 2D faces would cancel their own normals otherwise. That winding is
	// stored SWAPPED (v0,v2,v1): MjPositionToUe's Y-flip reverses winding sense, so
	// raw MuJoCo element order yields inward normals (surface lit from inside --
	// shadows land on the wrong side); the swap makes them outward. Feeds both the
	// build-time and the per-frame (ApplyFlexWorldPositions) normal accumulation.
	const int32 Dim = Model.flex_dim[FlexId];
	TArray<UE::Geometry::FIndex3i> Tris;
	NormalTris.Reset();
	if (Dim == 2)
	{
		const int32 NElem = Model.flex_elemnum[FlexId];
		Tris.Reserve(2 * NElem);
		NormalTris.Reserve(NElem);
		for (int32 e = 0; e < NElem; ++e)
		{
			const int* E = Model.flex_elem + Model.flex_elemdataadr[FlexId] + e * 3;
			Tris.Add(UE::Geometry::FIndex3i(E[0], E[1], E[2]));
			Tris.Add(UE::Geometry::FIndex3i(E[0], E[2], E[1]));
			NormalTris.Add(UE::Geometry::FIndex3i(E[0], E[2], E[1]));
		}
	}
	else if (Dim == 3)
	{
		const int32 NShell = Model.flex_shellnum[FlexId];
		Tris.Reserve(NShell);
		NormalTris.Reserve(NShell);
		for (int32 s = 0; s < NShell; ++s)
		{
			const int* S = Model.flex_shell + Model.flex_shelldataadr[FlexId] + s * 3;
			Tris.Add(UE::Geometry::FIndex3i(S[0], S[1], S[2]));
			NormalTris.Add(UE::Geometry::FIndex3i(S[0], S[2], S[1]));
		}
	}
	if (Tris.Num() == 0)
	{
		return;
	}

	// The model's own topology IS the welded topology: one render vertex per
	// MuJoCo flex vertex, so the weld map is the identity.
	NumRenderVerts = FlexVertNum;
	RawToWelded.SetNum(NumRenderVerts);
	for (int32 i = 0; i < NumRenderVerts; ++i)
	{
		RawToWelded[i] = i;
	}

	// Unique per element, as on the authored path.
	const FName MeshName = MakeUniqueObjectName(
		Owner, UDynamicMeshComponent::StaticClass(), *(GetName() + TEXT("_FlexMesh")));

	DynamicMesh = NewObject<UDynamicMeshComponent>(Owner, MeshName);
	DynamicMesh->SetupAttachment(this);
	DynamicMesh->RegisterComponent();
	DynamicMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	// Positions move every frame and nothing recomputes tangents by hand, so
	// the component derives them itself through MikkTSpace.
	DynamicMesh->SetTangentsType(EDynamicMeshComponentTangentsMode::AutoCalculated);

	// This frame's positions in the mesh's local space -- the same convention
	// ApplyFlexWorldPositions writes with on every later frame.
	const USceneComponent* MeshParent = DynamicMesh->GetAttachParent();
	const FTransform ParentTransform =
		MeshParent != nullptr ? MeshParent->GetComponentTransform() : FTransform::Identity;
	TArray<FVector> Local;
	Local.SetNum(FlexVertNum);
	for (int32 v = 0; v < FlexVertNum; ++v)
	{
		Local[v] = ParentTransform.InverseTransformPosition(WorldPositions[v]);
	}

	// Area-weighted vertex normals at this pose (the UMjSkincomp build pattern).
	// NormalTris is kept on the component: ApplyFlexWorldPositions re-runs this
	// exact accumulation at every deformed pose, so shading follows the surface.
	TArray<FVector> VertNormals;
	VertNormals.Init(FVector::ZeroVector, FlexVertNum);
	for (const UE::Geometry::FIndex3i& T : NormalTris)
	{
		if (T.A < 0 || T.A >= FlexVertNum || T.B < 0 || T.B >= FlexVertNum
			|| T.C < 0 || T.C >= FlexVertNum)
		{
			continue;
		}
		const FVector N = FVector::CrossProduct(Local[T.B] - Local[T.A], Local[T.C] - Local[T.A]);
		VertNormals[T.A] += N;
		VertNormals[T.B] += N;
		VertNormals[T.C] += N;
	}

	const int32 TexAdr = Model.flex_texcoordadr[FlexId];
	const bool bHasUVs = TexAdr >= 0 && Model.flex_texcoord != nullptr;

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

		// One overlay element per vertex, so triangles index identically below.
		for (int32 v = 0; v < FlexVertNum; ++v)
		{
			Mesh.AppendVertex(FVector3d(Local[v].X, Local[v].Y, Local[v].Z));

			const FVector Nn = VertNormals[v].GetSafeNormal();
			Normals->AppendElement(FVector3f(
				static_cast<float>(Nn.X), static_cast<float>(Nn.Y), static_cast<float>(Nn.Z)));

			if (UVs != nullptr)
			{
				const float* T = Model.flex_texcoord + 2 * (TexAdr + v);
				UVs->AppendElement(FVector2f(T[0], T[1]));
			}
		}

		// Degenerate triangles are dropped: FDynamicMesh3 rejects them and would
		// leave the overlays pointing at a triangle that does not exist.
		for (const UE::Geometry::FIndex3i& T : Tris)
		{
			if (T.A == T.B || T.B == T.C || T.A == T.C
				|| T.A < 0 || T.A >= FlexVertNum || T.B < 0 || T.B >= FlexVertNum
				|| T.C < 0 || T.C >= FlexVertNum)
			{
				continue;
			}
			const int32 TriangleId = Mesh.AppendTriangle(T.A, T.B, T.C);
			if (TriangleId >= 0)
			{
				Normals->SetTriangle(TriangleId, T);
				if (UVs != nullptr)
				{
					UVs->SetTriangle(TriangleId, T);
				}
			}
		}
	},
		EDynamicMeshComponentRenderUpdateMode::FullUpdate);
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
	NormalTris.Reset();
	NumRenderVerts = 0;
	FlexId = INDEX_NONE;
	FlexVertAdr = 0;
	FlexVertNum = 0;
	ResolvedFlexName.Reset();
	bSurfaceAttempted = false;
}
