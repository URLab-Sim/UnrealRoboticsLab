// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjSkincomp.h"

#include "CoreGlobals.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "GameFramework/Actor.h"
#include "UDynamicMesh.h"

#include "MuJoCo/Utils/URLabAxisConv.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

UMjSkincomp::UMjSkincomp()
{
	// Purely renderer-driven (AMjRenderer::UpdateMirrorSkin); it holds no local
	// engine and needs no self-tick, unlike the authored UMjFlexcomp.
	PrimaryComponentTick.bCanEverTick = false;
}

void UMjSkincomp::OnComponentDestroyed(bool bDestroyingHierarchy)
{
	ReleaseProceduralMesh();
	Super::OnComponentDestroyed(bDestroyingHierarchy);
}

bool UMjSkincomp::EnsureSkin(const mjModel& Model)
{
	if (SkinId < 0 || SkinId >= Model.nskin)
	{
		return false;
	}

	// Cheap to re-read; these are this skin's slices of the shared skin pools.
	VertAdr = Model.skin_vertadr[SkinId];
	VertNum = Model.skin_vertnum[SkinId];
	TexAdr = Model.skin_texcoordadr[SkinId]; // -1 when the skin has no texcoords
	FaceAdr = Model.skin_faceadr[SkinId];
	FaceNum = Model.skin_facenum[SkinId];
	BoneAdr = Model.skin_boneadr[SkinId];
	BoneNum = Model.skin_bonenum[SkinId];

	if (DynamicMesh == nullptr)
	{
		if (bSurfaceAttempted)
		{
			return false;
		}
		bSurfaceAttempted = true;
		CreateProceduralMesh(Model);
	}
	return DynamicMesh != nullptr;
}

void UMjSkincomp::CreateProceduralMesh(const mjModel& Model)
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr || VertNum <= 0 || FaceNum <= 0)
	{
		return;
	}

	// Unique per element -- a model can carry several skins on one actor.
	const FName MeshName = MakeUniqueObjectName(
		Owner, UDynamicMeshComponent::StaticClass(), *(GetName() + TEXT("_SkinMesh")));

	DynamicMesh = NewObject<UDynamicMeshComponent>(Owner, MeshName);
	DynamicMesh->SetupAttachment(this);
	DynamicMesh->RegisterComponent();
	DynamicMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	// Positions move every frame and nothing hand-computes tangents, so the
	// component derives them itself through MikkTSpace (as UMjFlexcomp does).
	DynamicMesh->SetTangentsType(EDynamicMeshComponentTangentsMode::AutoCalculated);

	const bool bHasUV = TexAdr >= 0;

	// Rest-pose vertices in UE space. skin_face keeps MuJoCo's order (0,1,2):
	// MjPositionToUe negates Y (a reflection that flips winding), so that order
	// is already front-facing in Unreal -- the same rule the mesh baker follows
	// (MjRendererAssetBaker.cpp:79-85). Outward normal = cross(P1-P0, P2-P0).
	TArray<FVector> RestUe;
	RestUe.SetNum(VertNum);
	for (int32 v = 0; v < VertNum; ++v)
	{
		RestUe[v] = URLabAxisConv::MjPositionToUe(Model.skin_vert + 3 * (VertAdr + v));
	}

	// Area-weighted rest normals. These are the static shading normals; the
	// per-frame LBS writeback moves only positions (like flex), and the inflate
	// offset uses per-frame normals recomputed in the MuJoCo frame in the update.
	TArray<FVector> VertNormals;
	VertNormals.Init(FVector::ZeroVector, VertNum);
	for (int32 f = 0; f < FaceNum; ++f)
	{
		const int32* Face = Model.skin_face + 3 * (FaceAdr + f);
		const int32 A = Face[0], B = Face[1], C = Face[2];
		if (A < 0 || A >= VertNum || B < 0 || B >= VertNum || C < 0 || C >= VertNum)
		{
			continue;
		}
		const FVector N = FVector::CrossProduct(RestUe[B] - RestUe[A], RestUe[C] - RestUe[A]);
		VertNormals[A] += N;
		VertNormals[B] += N;
		VertNormals[C] += N;
	}

	DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh) {
		Mesh.Clear();
		Mesh.EnableAttributes();
		Mesh.Attributes()->SetNumNormalLayers(1);
		if (bHasUV)
		{
			Mesh.Attributes()->SetNumUVLayers(1);
		}

		UE::Geometry::FDynamicMeshNormalOverlay* Normals = Mesh.Attributes()->PrimaryNormals();
		UE::Geometry::FDynamicMeshUVOverlay* UVs = bHasUV ? Mesh.Attributes()->PrimaryUV() : nullptr;

		for (int32 v = 0; v < VertNum; ++v)
		{
			Mesh.AppendVertex(FVector3d(RestUe[v].X, RestUe[v].Y, RestUe[v].Z));

			const FVector Nn = VertNormals[v].GetSafeNormal();
			Normals->AppendElement(FVector3f(
				static_cast<float>(Nn.X), static_cast<float>(Nn.Y), static_cast<float>(Nn.Z)));

			if (UVs != nullptr)
			{
				const float* T = Model.skin_texcoord + 2 * (TexAdr + v);
				UVs->AppendElement(FVector2f(T[0], T[1]));
			}
		}

		// Local (0-based) face indices, so the overlay elements added above index
		// identically. Degenerate triangles are dropped (FDynamicMesh3 rejects
		// them and would leave the overlays pointing at a missing triangle).
		for (int32 f = 0; f < FaceNum; ++f)
		{
			const int32* Face = Model.skin_face + 3 * (FaceAdr + f);
			const int32 A = Face[0], B = Face[1], C = Face[2];
			if (A == B || B == C || A == C)
			{
				continue;
			}
			if (A < 0 || A >= VertNum || B < 0 || B >= VertNum || C < 0 || C >= VertNum)
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

void UMjSkincomp::UpdateFromBodyTransforms(const mjModel& Model, const double* Bxpos,
	const double* Bxquat, int32 NBody, const FVector& SceneOrigin)
{
	if (Bxpos == nullptr || Bxquat == nullptr || NBody <= 0)
	{
		return;
	}
	// Resolve + build the surface on first sight, exactly like the producer tick.
	if (!EnsureSkin(Model))
	{
		return;
	}
	if (DynamicMesh == nullptr || VertNum <= 0)
	{
		return;
	}

	auto SafeBody = [NBody](int b) { return b >= 0 && b < NBody; };

	// Accumulated skinned positions in the MuJoCo world frame plus the total blend
	// weight per vertex. This is mjv_updateActiveSkin (engine_vis_visualize.c:
	// 3258-3316) replayed with no mjData: the deformed surface is a pure function
	// of the static bind pose/weights and the per-body transforms already on the
	// wire (bxpos/bxquat). CPU LBS iterates every bone-vertex entry, so a vertex
	// with >4 influences is handled with no 4-bone cap; WSum renormalizes it.
	TArray<double> Pos;
	Pos.SetNumZeroed(VertNum * 3);
	TArray<double> WSum;
	WSum.SetNumZeroed(VertNum);

	for (int32 j = BoneAdr; j < BoneAdr + BoneNum; ++j)
	{
		const int32 BodyId = Model.skin_bonebodyid[j];
		if (!SafeBody(BodyId))
		{
			continue;
		}

		const double BindPos[3] = {Model.skin_bonebindpos[3 * j], Model.skin_bonebindpos[3 * j + 1],
			Model.skin_bonebindpos[3 * j + 2]};
		const double BindQuat[4] = {Model.skin_bonebindquat[4 * j], Model.skin_bonebindquat[4 * j + 1],
			Model.skin_bonebindquat[4 * j + 2], Model.skin_bonebindquat[4 * j + 3]};

		// rotation = xquat[body] * conj(bindquat);  translate = xpos[body] - rotate*bindpos.
		double QuatNeg[4], Quat[4], Rotate[9];
		mju_negQuat(QuatNeg, BindQuat);
		mju_mulQuat(Quat, Bxquat + 4 * BodyId, QuatNeg);
		mju_quat2Mat(Rotate, Quat);

		double Translate[3];
		mju_mulMatVec3(Translate, Rotate, BindPos);
		mju_sub3(Translate, Bxpos + 3 * BodyId, Translate);

		for (int32 k = Model.skin_bonevertadr[j];
			 k < Model.skin_bonevertadr[j] + Model.skin_bonevertnum[j]; ++k)
		{
			const int32 Vid = Model.skin_bonevertid[k];
			const double Weight = Model.skin_bonevertweight[k];
			if (Vid < 0 || Vid >= VertNum)
			{
				continue;
			}
			const double P[3] = {Model.skin_vert[3 * (VertAdr + Vid)],
				Model.skin_vert[3 * (VertAdr + Vid) + 1], Model.skin_vert[3 * (VertAdr + Vid) + 2]};
			double P1[3];
			mju_mulMatVec3(P1, Rotate, P);
			mju_addTo3(P1, Translate);

			Pos[3 * Vid + 0] += Weight * P1[0];
			Pos[3 * Vid + 1] += Weight * P1[1];
			Pos[3 * Vid + 2] += Weight * P1[2];
			WSum[Vid] += Weight;
		}
	}

	// Renormalize by the accumulated weight: an identity when the weights already
	// sum to 1 (the compiled-model norm), and the >4-influence safety the plan
	// asks for. A vertex reached by no bone (sum 0) stays at the origin, as in
	// upstream's un-normalized accumulate.
	for (int32 v = 0; v < VertNum; ++v)
	{
		const double S = WSum[v];
		if (S > 1e-9)
		{
			Pos[3 * v + 0] /= S;
			Pos[3 * v + 1] /= S;
			Pos[3 * v + 2] /= S;
		}
	}

	// Inflate along the per-frame area-weighted vertex normal, computed in the
	// MuJoCo frame so it matches upstream exactly (engine_vis_visualize.c:
	// 3319-3366): normal from face cross products, normalized, scaled by inflate.
	const double Inflate = (Model.skin_inflate != nullptr) ? static_cast<double>(Model.skin_inflate[SkinId]) : 0.0;
	if (Inflate != 0.0 && FaceNum > 0)
	{
		TArray<double> Nrm;
		Nrm.SetNumZeroed(VertNum * 3);
		for (int32 f = FaceAdr; f < FaceAdr + FaceNum; ++f)
		{
			const int32* Face = Model.skin_face + 3 * f;
			const int32 A = Face[0], B = Face[1], C = Face[2];
			if (A < 0 || A >= VertNum || B < 0 || B >= VertNum || C < 0 || C >= VertNum)
			{
				continue;
			}
			double V01[3], V02[3], N[3];
			for (int32 r = 0; r < 3; ++r)
			{
				V01[r] = Pos[3 * B + r] - Pos[3 * A + r];
				V02[r] = Pos[3 * C + r] - Pos[3 * A + r];
			}
			mju_cross(N, V01, V02);
			for (int32 r = 0; r < 3; ++r)
			{
				Nrm[3 * A + r] += N[r];
				Nrm[3 * B + r] += N[r];
				Nrm[3 * C + r] += N[r];
			}
		}
		for (int32 v = 0; v < VertNum; ++v)
		{
			double* Nv = Nrm.GetData() + 3 * v;
			const double Len = mju_normalize3(Nv);
			if (Len > mjMINVAL)
			{
				Pos[3 * v + 0] += Inflate * Nv[0];
				Pos[3 * v + 1] += Inflate * Nv[1];
				Pos[3 * v + 2] += Inflate * Nv[2];
			}
		}
	}

	// MuJoCo world -> UE world (+ the renderer's scene offset, like every geom),
	// then into the mesh's local space. Same positions-only writeback as flex:
	// topology, UVs and colours are unchanged and the tangent mode recomputes
	// what depends on them.
	const USceneComponent* MeshParent = DynamicMesh->GetAttachParent();
	const FTransform ParentTransform =
		MeshParent != nullptr ? MeshParent->GetComponentTransform() : FTransform::Identity;

	DynamicMesh->EditMesh([&](UE::Geometry::FDynamicMesh3& Mesh) {
		for (int32 v = 0; v < VertNum; ++v)
		{
			const FVector World = URLabAxisConv::MjPositionToUe(&Pos[3 * v]) + SceneOrigin;
			const FVector Local = ParentTransform.InverseTransformPosition(World);
			Mesh.SetVertex(v, FVector3d(Local.X, Local.Y, Local.Z));
		}
	},
		EDynamicMeshComponentRenderUpdateMode::NoUpdate);

	DynamicMesh->FastNotifyPositionsUpdated(/*bNormals=*/false, /*bColors=*/false, /*bUVs=*/false);
}

void UMjSkincomp::ReleaseProceduralMesh()
{
	if (DynamicMesh != nullptr)
	{
		DynamicMesh->DestroyComponent();
		DynamicMesh = nullptr;
	}
	VertNum = 0;
	FaceNum = 0;
	SkinId = INDEX_NONE;
	bSurfaceAttempted = false;
}
