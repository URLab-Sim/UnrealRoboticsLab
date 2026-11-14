// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "MuJoCo/Convert/MjQuickConvertComponent.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCoSpec/MJUnitConverter.h"
#include "mujoco/mujoco.h"
#include "CoACD/CoacdInterface.h"
#include "Utils/CoordinateConversion.h"
#include "Utils/URLabLogging.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"


namespace
{
/** One collision hull represented directly by editable MJCF mesh fields. */
struct FMjConvertedHull
{
	FString Name;
	TArray<float> Vertex;
	TArray<int32> Face;
};

/**
 * True when this mesh component is part of a MuJoCo spec rather than the
 * actor's own content.
 *
 * A geom previews itself with static mesh components of its own, so the second
 * pass over an actor would otherwise convert the first pass's previews and grow
 * the body a geom per compile. The test for it is structural rather than by
 * name: anything under an element is the spec's, and nothing the spec
 * owns is source geometry.
 */
bool IsSpecPreview(const USceneComponent& Component)
{
	for (const USceneComponent* Node = &Component; Node != nullptr; Node = Node->GetAttachParent())
	{

	}
	return false;
}

template <typename VertexType, typename IndexType>
bool CopyCollisionHull(const Chaos::TArrayCollectionArray<VertexType>& Vertices,
	const TArray<Chaos::TVector<IndexType, 3>>& Indices, const FString& Name,
	FMjConvertedHull& Out, FString& OutError)
{
	if (Vertices.Num() < 4 || Indices.Num() < 4)
	{
		OutError = FString::Printf(TEXT("quick-convert collision hull '%s' has insufficient vertices or faces"), *Name);
		return false;
	}
	Out.Name = Name;
	Out.Vertex.Reserve(Vertices.Num() * 3);
	for (const VertexType& Vertex : Vertices)
	{
		double Native[3];
		URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(
			FVector(Vertex.X, Vertex.Y, Vertex.Z), Native);
		for (const double Value : Native) Out.Vertex.Add(static_cast<float>(Value));
	}
	Out.Face.Reserve(Indices.Num() * 3);
	for (const auto& Triangle : Indices)
	{
		// Match the reflected RH winding emitted by the former OBJ path.
		Out.Face.Add(static_cast<int32>(Triangle.Z));
		Out.Face.Add(static_cast<int32>(Triangle.Y));
		Out.Face.Add(static_cast<int32>(Triangle.X));
	}
	return true;
}

/** Copy the exact Chaos collision input used by the former OBJ exporter. */
bool ExportHulls(UStaticMeshComponent& Smc, bool bComplex, float CoacdThreshold,
	TArray<FMjConvertedHull>& Out, FString& OutError)
{
	Out.Reset();
	OutError.Reset();
	UStaticMesh* Mesh = Smc.GetStaticMesh();
	if (Mesh == nullptr)
	{
		OutError = FString::Printf(TEXT("quick-convert component '%s' has no static mesh"), *Smc.GetName());
		return false;
	}
	UBodySetup* BodySetup = Mesh->GetBodySetup();
	if (BodySetup == nullptr) return true;
	if (BodySetup->TriMeshGeometries.Num() == 0) BodySetup->CreatePhysicsMeshes();
	if (BodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjQuickConvert] '%s' has no triangle collision to convert."), *Mesh->GetName());
		return true;
	}

	const auto& TriGeom = BodySetup->TriMeshGeometries[0];
	const auto& Vertices = TriGeom.GetReference()->Particles().X();
	const auto& Elements = TriGeom.GetReference()->Elements();
	FString BaseName = Mesh->GetName();
	for (TCHAR& Ch : BaseName)
	{
		if (!FChar::IsAlnum(Ch) && Ch != TEXT('_')) Ch = TEXT('_');
	}
	if (!bComplex)
	{
		const bool bLargeIndices = Elements.RequiresLargeIndices();
		const FString HullName = FString::Printf(TEXT("%s_SimpleHull"), *BaseName);
		FMjConvertedHull& Hull = Out.AddDefaulted_GetRef();
		if (bLargeIndices)
		{
			if (!CopyCollisionHull(Vertices, Elements.GetLargeIndexBuffer(), HullName, Hull, OutError)) return false;
		}
		else if (!CopyCollisionHull(Vertices, Elements.GetSmallIndexBuffer(), HullName, Hull, OutError)) return false;
		return true;
	}

#if !WITH_EDITOR
	(void)CoacdThreshold;
	OutError = TEXT("quick-convert complex convex decomposition requires an editor build");
	return false;
#else
	CoACD_Mesh Input{};
	TArray<double> InputVertices;
	TArray<int> InputTriangles;
	Input.vertices_count = Vertices.Num();
	Input.triangles_count = Elements.RequiresLargeIndices()
		? Elements.GetLargeIndexBuffer().Num() : Elements.GetSmallIndexBuffer().Num();
	if (Input.vertices_count > MAX_int32 / 3 || Input.triangles_count > MAX_int32 / 3)
	{
		OutError = FString::Printf(TEXT("quick-convert collision source '%s' exceeds supported inline mesh size"), *BaseName);
		return false;
	}
	InputVertices.Reserve(static_cast<int32>(Input.vertices_count * 3));
	for (const auto& Vertex : Vertices)
	{
		InputVertices.Add(Vertex.X);
		InputVertices.Add(Vertex.Y);
		InputVertices.Add(Vertex.Z);
	}
	InputTriangles.Reserve(static_cast<int32>(Input.triangles_count * 3));
	auto AppendInputTriangles = [&InputTriangles](const auto& Source)
	{
		for (const auto& Triangle : Source)
		{
			InputTriangles.Add(static_cast<int>(Triangle.X));
			InputTriangles.Add(static_cast<int>(Triangle.Y));
			InputTriangles.Add(static_cast<int>(Triangle.Z));
		}
	};
	if (Elements.RequiresLargeIndices()) AppendInputTriangles(Elements.GetLargeIndexBuffer());
	else AppendInputTriangles(Elements.GetSmallIndexBuffer());
	Input.vertices_ptr = InputVertices.GetData();
	Input.triangles_ptr = InputTriangles.GetData();
	CoACD_MeshArray Result = CoACD_run(Input, CoacdThreshold, -1, preprocess_auto, 30, 1000, 20, 150,
		3, false, true, false, 100, false, 0.01, apx_ch, 0, false);
	if (Result.meshes_count <= 0 || Result.meshes_ptr == nullptr)
	{
		CoACD_freeMeshArray(Result);
		OutError = FString::Printf(TEXT("quick-convert produced no collision hulls for '%s'"), *BaseName);
		return false;
	}
	if (Result.meshes_count > MAX_int32)
	{
		CoACD_freeMeshArray(Result);
		OutError = TEXT("quick-convert CoACD result has too many inline collision hulls");
		return false;
	}
	for (uint64 Index = 0; Index < Result.meshes_count; ++Index)
	{
		const CoACD_Mesh& NativeHull = Result.meshes_ptr[Index];
		if (NativeHull.vertices_count > MAX_int32 / 3 || NativeHull.triangles_count > MAX_int32 / 3)
		{
			CoACD_freeMeshArray(Result);
			OutError = TEXT("quick-convert CoACD result exceeds supported inline mesh size");
			return false;
		}
		FMjConvertedHull& Hull = Out.AddDefaulted_GetRef();
		Hull.Name = FString::Printf(TEXT("%s_ComplexHull_%llu"), *BaseName, Index);
		Hull.Vertex.Reserve(static_cast<int32>(NativeHull.vertices_count * 3));
		for (uint64 IndexValue = 0; IndexValue < NativeHull.vertices_count; ++IndexValue)
		{
			const FVector Vertex(NativeHull.vertices_ptr[3 * IndexValue], NativeHull.vertices_ptr[3 * IndexValue + 1],
				NativeHull.vertices_ptr[3 * IndexValue + 2]);
			double Native[3];
			URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(Vertex, Native);
			for (const double Value : Native) Hull.Vertex.Add(static_cast<float>(Value));
		}
		Hull.Face.Reserve(static_cast<int32>(NativeHull.triangles_count * 3));
		for (uint64 FaceIndex = 0; FaceIndex < NativeHull.triangles_count; ++FaceIndex)
		{
			Hull.Face.Add(NativeHull.triangles_ptr[3 * FaceIndex + 2]);
			Hull.Face.Add(NativeHull.triangles_ptr[3 * FaceIndex + 1]);
			Hull.Face.Add(NativeHull.triangles_ptr[3 * FaceIndex]);
		}
	}
	CoACD_freeMeshArray(Result);
	return true;
#endif
}
} // namespace

UMjQuickConvertComponent::UMjQuickConvertComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

FString UMjQuickConvertComponent::GetBodyName()
{
	return m_BodyName;
}

int32 UMjQuickConvertComponent::GetMjBodyId() const
{
	return -1;
}

void UMjQuickConvertComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UMjQuickConvertComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
	Super::OnComponentDestroyed(bDestroyingHierarchy);
}




FString UMjQuickConvertComponent::GetScenePrefix() const
{
	const AActor* Owner = GetOwner();
	return Owner != nullptr ? Owner->GetName() + TEXT("_") : GetName() + TEXT("_");
}

FTransform UMjQuickConvertComponent::GetScenePlacement() const
{
	const AActor* Owner = GetOwner();
	return Owner != nullptr ? Owner->GetActorTransform() : FTransform::Identity;
}

void UMjQuickConvertComponent::OnSceneBound()
{
	m_geomName2ID.Reset();
}

bool UMjQuickConvertComponent::ContributeToSpec(mjSpec* SceneSpec, const FString& Prefix, FString& OutError)
{
	OutError.Reset();
	m_BodyName.Reset();

	AActor* Owner = GetOwner();
	if (Owner == nullptr || SceneSpec == nullptr)
	{
		OutError = FString::Printf(TEXT("quick-convert component '%s' has no owning actor or scene spec"), *GetName());
		return false;
	}

	if (!AuthorConvertedBody(*Owner, SceneSpec, Prefix, GetConvertSettings(), OutError))
	{
		return OutError.IsEmpty();
	}
	m_BodyName = Prefix + TEXT("body");
	UE_LOG(LogURLab, Log, TEXT("[MjQuickConvert] '%s': authored body '%s'."), *Owner->GetName(), *m_BodyName);
	return true;
}

UMjQuickConvertComponent::FConvertSettings UMjQuickConvertComponent::GetConvertSettings() const
{
	FConvertSettings S;
	S.Static = Static;
	S.ComplexMeshRequired = ComplexMeshRequired;
	S.bDrivenByUnreal = bDrivenByUnreal;
	S.CoACDThreshold = CoACDThreshold;
	S.Friction = friction;
	S.Solref = solref;
	S.Solimp = solimp;
	return S;
}

bool UMjQuickConvertComponent::AuthorConvertedBody(
	AActor& SourceActor,
	mjSpec* SceneSpec,
	const FString& Prefix,
	const FConvertSettings& Settings,
	FString& OutError)
{
	OutError.Reset();
	if (!SceneSpec)
	{
		OutError = TEXT("SceneSpec is null");
		return false;
	}
	TArray<UStaticMeshComponent*> Meshes;
	SourceActor.GetComponents(Meshes);
	Meshes.RemoveAll([](const UStaticMeshComponent* Smc) {
		return Smc == nullptr || Smc->GetStaticMesh() == nullptr || IsSpecPreview(*Smc);
	});
	Meshes.Sort([](const UStaticMeshComponent& A, const UStaticMeshComponent& B) {
		return A.GetName() < B.GetName();
	});
	if (Meshes.Num() == 0)
	{
		return false;
	}

	mjsBody* WorldBody = mjs_findBody(SceneSpec, "world");
	if (!WorldBody)
	{
		OutError = TEXT("SceneSpec has no world body");
		return false;
	}

	mjsBody* Body = mjs_addBody(WorldBody, nullptr);
	if (!Body)
	{
		OutError = TEXT("mjs_addBody failed");
		return false;
	}
	const FString BodyName = Prefix + TEXT("body");
	mjs_setName(Body->element, TCHAR_TO_UTF8(*BodyName));

	const FTransform OwnerTransform = SourceActor.GetActorTransform();
	FMJUnitConverter::UETransformToMJ(OwnerTransform, Body->pos, Body->quat);

	if (Settings.bDrivenByUnreal)
	{
		Body->mocap = 1;
	}
	else if (!Settings.Static)
	{
		mjsJoint* FreeJnt = mjs_addJoint(Body, nullptr);
		if (FreeJnt)
		{
			FreeJnt->type = mjJNT_FREE;
			const FString JntName = Prefix + TEXT("free");
			mjs_setName(FreeJnt->element, TCHAR_TO_UTF8(*JntName));
		}
	}

	int32 MeshIndex = 0;
	for (UStaticMeshComponent* Smc : Meshes)
	{
		const FVector RelativeLocation =
			OwnerTransform.GetRotation().UnrotateVector(Smc->GetComponentLocation() - OwnerTransform.GetLocation());
		const FQuat RelativeRotation = OwnerTransform.GetRotation().Inverse() * Smc->GetComponentQuat();
		const FVector MeshScale = Smc->GetComponentScale();
		const FTransform RelXform(RelativeRotation, RelativeLocation, FVector::OneVector);

		TArray<FMjConvertedHull> CollisionHulls;
		if (!ExportHulls(*Smc, Settings.ComplexMeshRequired, Settings.CoACDThreshold, CollisionHulls, OutError))
		{
			return false;
		}
		if (CollisionHulls.IsEmpty())
		{
			OutError = FString::Printf(TEXT("quick-convert mesh '%s' has no collision hull; a massless visual-only body is not valid"),
				*Smc->GetName());
			return false;
		}
		for (int32 HullIndex = 0; HullIndex < CollisionHulls.Num(); ++HullIndex)
		{
			const FMjConvertedHull& Hull = CollisionHulls[HullIndex];
			const FString ScopedMeshName = Prefix + Hull.Name;
			mjsMesh* Mesh = mjs_addMesh(SceneSpec, nullptr);
			if (Mesh)
			{
				mjs_setName(Mesh->element, TCHAR_TO_UTF8(*ScopedMeshName));
				Mesh->scale[0] = MeshScale.X;
				Mesh->scale[1] = MeshScale.Y;
				Mesh->scale[2] = MeshScale.Z;
				TArray<float> Verts;
				Verts.Reserve(Hull.Vertex.Num());
				for (double V : Hull.Vertex) { Verts.Add(static_cast<float>(V)); }
				mjs_setFloat(Mesh->uservert, Verts.GetData(), Verts.Num());
				mjs_setInt(Mesh->userface, Hull.Face.GetData(), Hull.Face.Num());
			}
			mjsGeom* Collision = mjs_addGeom(Body, nullptr);
			if (Collision)
			{
				const FString GeomName = Prefix + FString::Printf(TEXT("Geom_%d_collision_%d"), MeshIndex, HullIndex);
				mjs_setName(Collision->element, TCHAR_TO_UTF8(*GeomName));
				Collision->type = mjGEOM_MESH;
				mjs_setString(Collision->meshname, TCHAR_TO_UTF8(*ScopedMeshName));
				FMJUnitConverter::UETransformToMJ(RelXform, Collision->pos, Collision->quat);
				Collision->group = 3;
				Collision->friction[0] = Settings.Friction.X;
				Collision->friction[1] = Settings.Friction.Y;
				Collision->friction[2] = Settings.Friction.Z;
				Collision->solref[0] = Settings.Solref.X;
				Collision->solref[1] = Settings.Solref.Y;
				Collision->solimp[0] = Settings.Solimp.X;
				Collision->solimp[1] = Settings.Solimp.Y;
				Collision->solimp[2] = Settings.Solimp.Z;
			}
		}
		++MeshIndex;
	}
	return true;
}



void UMjQuickConvertComponent::SetSourceMeshesHiddenInGame(bool bHidden)
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return;
	}
	TArray<UStaticMeshComponent*> Meshes;
	Owner->GetComponents(Meshes);
	for (UStaticMeshComponent* Smc : Meshes)
	{
		// The actor's own content, not the spec's previews: those are never drawn
		// by the play view, and the source meshes are what would double-draw.
		if (Smc != nullptr && !IsSpecPreview(*Smc))
		{
			Smc->SetHiddenInGame(bHidden);
		}
	}
}
