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
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjFreeJoint.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMesh.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "CoACD/CoacdInterface.h"
#include "Utils/CoordinateConversion.h"
#include "Utils/URLabLogging.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjNodeFactories.h"
#endif

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
		if (Node->IsA<UMjNodeComponent>())
		{
			return true;
		}
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
	const FString BaseName = MjSanitizeAssetName(Mesh->GetName());
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
	return m_CreatedBody ? m_CreatedBody->GetBoundId().Get(-1) : -1;
}

UMjNodeComponent* UMjQuickConvertComponent::GetBodyElement() const
{
	return m_CreatedBody;
}

void UMjQuickConvertComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UMjQuickConvertComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
	if (Spec != nullptr && !bDestroyingHierarchy)
	{
		MjDestroySpecChildren(*Spec);
		Spec->DestroyComponent();
	}
	Spec = nullptr;
	m_CreatedBody = nullptr;
	m_GeomElements.Reset();
	Super::OnComponentDestroyed(bDestroyingHierarchy);
}



UMjModel* UMjQuickConvertComponent::EnsureSpecRoot()
{
	if (Spec != nullptr)
	{
		return Spec;
	}
	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return nullptr;
	}
	Spec = NewObject<UMjModel>(Owner, TEXT("MjQuickConvertSpec"), RF_Transactional);
	Spec->SetupAttachment(Owner->GetRootComponent());
	Spec->RegisterComponent();
	Owner->AddInstanceComponent(Spec);
	return Spec;
}

FSpecRef UMjQuickConvertComponent::GetSceneSpec() const
{
	// No body means the actor had nothing convertible on it, and an empty
	// participant in the scene reads as a converted actor that simply fell
	// through the floor.
	AActor* Owner = GetOwner();
	if (Owner == nullptr || m_CreatedBody == nullptr)
	{
		return FSpecRef();
	}
	return FSpecRef::OverActor(*Owner);
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
	for (const TObjectPtr<UMjNodeComponent>& Geom : m_GeomElements)
	{
		if (Geom != nullptr && Geom->MjName.IsSet())
		{
			m_geomName2ID.Add(GetScenePrefix() + *Geom->MjName, Geom->GetBoundId().Get(-1));
		}
	}
}

bool UMjQuickConvertComponent::AuthorSceneSpec(FString& OutError)
{
	OutError.Reset();
	m_CreatedBody = nullptr;
	m_GeomElements.Reset();
	m_BodyName.Reset();

#if URLAB_MJ_GEN
	AActor* Owner = GetOwner();
	UMjModel* Root = EnsureSpecRoot();
	if (Owner == nullptr || Root == nullptr)
	{
		OutError = FString::Printf(TEXT("quick-convert component '%s' has no owning actor or spec root"), *GetName());
		return false;
	}
	MjDestroySpecChildren(*Root);

	urlab::spec::FMjInstanceScope Scope(*Owner);
	UMjBodyBase* Body = AuthorConvertedBody(*Owner, *Root, GetConvertSettings(), m_GeomElements, OutError);
	if (Body == nullptr)
	{
		return OutError.IsEmpty();
	}
	m_CreatedBody = Body;
	m_BodyName = GetScenePrefix() + TEXT("body");

	UE_LOG(LogURLab, Log, TEXT("[MjQuickConvert] '%s': authored body '%s' with %d geom(s)."), *Owner->GetName(),
		*m_BodyName, m_GeomElements.Num());
#endif
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

#if URLAB_MJ_GEN
UMjBodyBase* UMjQuickConvertComponent::AuthorConvertedBody(
	AActor& SourceActor,
	UMjModel& Root,
	const FConvertSettings& Settings,
	TArray<TObjectPtr<UMjNodeComponent>>& OutGeoms,
	FString& OutError)
{
	OutError.Reset();
	TArray<UStaticMeshComponent*> Meshes;
	SourceActor.GetComponents(Meshes);
	Meshes.RemoveAll([](const UStaticMeshComponent* Smc) {
		return Smc == nullptr || Smc->GetStaticMesh() == nullptr || IsSpecPreview(*Smc);
	});
	// Registration order is not a spec order. Sorting on the component's own
	// name is, and it is stable across sessions, which is what geom naming and
	// the compiled id map both rest on.
	Meshes.Sort([](const UStaticMeshComponent& A, const UStaticMeshComponent& B) {
		return A.GetName() < B.GetName();
	});
	if (Meshes.Num() == 0)
	{
		return nullptr;
	}

	UMjAsset& Assets = urlab::spec::FInstanceNodeFactory::Create<UMjAsset>(Root);
	UMjBodyBase& WorldBody = urlab::spec::FInstanceNodeFactory::Create<UMjBodyBase>(Root);
	UMjBodyBase& Body = urlab::spec::FInstanceNodeFactory::Create<UMjBodyBase>(WorldBody);
	Body.MjName = TEXT("body");

	if (Settings.bDrivenByUnreal)
	{
		Body.Mocap = true;
	}
	else if (!Settings.Static)
	{
		urlab::spec::FInstanceNodeFactory::Create<UMjFreeJoint>(Body).MjName = TEXT("free");
	}

	const FTransform OwnerTransform = SourceActor.GetActorTransform();
	int32 MeshIndex = 0;
	for (UStaticMeshComponent* Smc : Meshes)
	{
		// The participant frame carries the actor's placement, so a geom's pose
		// is its mesh component's offset within the actor -- expressed in the
		// actor's own unscaled frame, because an MJCF frame has no scale and the
		// scale rides on the mesh asset instead.
		const FVector RelativeLocation =
			OwnerTransform.GetRotation().UnrotateVector(Smc->GetComponentLocation() - OwnerTransform.GetLocation());
		const FQuat RelativeRotation = OwnerTransform.GetRotation().Inverse() * Smc->GetComponentQuat();
		const FVector MeshScale = Smc->GetComponentScale();

		auto AddMesh = [&](const FString& MeshName, const FMjConvertedHull* Hull,
			UStaticMesh* VisualAsset) -> UMjMeshBase& {
			UMjMeshBase& Mesh = urlab::spec::FInstanceNodeFactory::Create<UMjMeshBase>(Assets);
			Mesh.MjName = MeshName;
			Mesh.Scale = FMjVec3(MeshScale.X, MeshScale.Y, MeshScale.Z);
			if (Hull != nullptr)
			{
				Mesh.Vertex = Hull->Vertex;
				Mesh.Face = Hull->Face;
			}
			if (VisualAsset != nullptr)
			{
				if (UMjMesh* Visual = Cast<UMjMesh>(&Mesh))
				{
					Visual->MeshAsset = VisualAsset;
					Visual->FileAsset = FSoftObjectPath(VisualAsset);
				}
			}
			return Mesh;
		};

		auto AddGeom = [&](const FString& GeomName, const FString& MeshName) -> UMjGeomBase& {
			UMjGeomBase& Geom = urlab::spec::FInstanceNodeFactory::Create<UMjGeomBase>(Body);
			Geom.MjName = GeomName;
			Geom.Type = EMjGeomType::mesh;
			Geom.Mesh = MeshName;
			Geom.Pos = FMjPosition3::FromUnreal(RelativeLocation);
			Geom.Quat = FMjQuatRot::FromUnreal(RelativeRotation);
			OutGeoms.Add(&Geom);
			return Geom;
		};

		auto ApplyContact = [&](UMjGeomBase& Geom) {
			Geom.Friction = TArray<double>{Settings.Friction.X, Settings.Friction.Y, Settings.Friction.Z};
			Geom.Solref = TArray<double>{Settings.Solref.X, Settings.Solref.Y};
			Geom.Solimp = TArray<double>{Settings.Solimp.X, Settings.Solimp.Y, Settings.Solimp.Z};
		};

		// Render the editable source mesh through the same compiled model that
		// supplies physics. This is a resource input for native MuJoCo, not a
		// renderer override; arbitrary Unreal material graphs have no native MJCF
		// equivalent and must be authored as MuJoCo materials separately.
		const FString VisualMeshName = FString::Printf(TEXT("VisualMesh_%d_%s"), MeshIndex,
			*MjSanitizeAssetName(Smc->GetStaticMesh()->GetName()));
		AddMesh(VisualMeshName, nullptr, Smc->GetStaticMesh());
		UMjGeomBase& Visual = AddGeom(FString::Printf(TEXT("Geom_%d_visual"), MeshIndex), VisualMeshName);
		Visual.Contype = 0;
		Visual.Conaffinity = 0;
		Visual.Group = 2;
		Visual.Mass = 0.0;
		if (Smc->GetNumMaterials() > 0)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[MjQuickConvert] Unreal material graphs on '%s' are not converted to MJCF; assign native MuJoCo materials for the compiled render."),
				*Smc->GetName());
		}

		TArray<FMjConvertedHull> CollisionHulls;
		if (!ExportHulls(*Smc, Settings.ComplexMeshRequired, Settings.CoACDThreshold, CollisionHulls, OutError))
		{
			return nullptr;
		}
		if (CollisionHulls.IsEmpty())
		{
			OutError = FString::Printf(TEXT("quick-convert mesh '%s' has no collision hull; a massless visual-only body is not valid"),
				*Smc->GetName());
			return nullptr;
		}
		for (int32 HullIndex = 0; HullIndex < CollisionHulls.Num(); ++HullIndex)
		{
			const FMjConvertedHull& Hull = CollisionHulls[HullIndex];
			AddMesh(Hull.Name, &Hull, nullptr);
			UMjGeomBase& Collision = AddGeom(FString::Printf(TEXT("Geom_%d_collision_%d"), MeshIndex, HullIndex), Hull.Name);
			Collision.Group = 3;
			ApplyContact(Collision);
		}
		++MeshIndex;
	}

	return &Body;
}
#endif



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
