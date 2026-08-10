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
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjFreeJoint.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMesh.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/IO.h"
#include "Utils/MeshUtils.h"
#include "Utils/URLabLogging.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjNodeFactories.h"
#endif

namespace
{
/** One hull exported out of a static mesh, and where its OBJ landed. */
struct FMjConvertedHull
{
	FString Name;
	FString ObjPath;
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

/** Where a converted actor's OBJs live. Per actor, so two actors converting the
 *  same source mesh at different decomposition settings do not evict each other's
 *  cache on every compile. */
FString HullDirectory(const FString& OwnerName)
{
	return FPaths::ConvertRelativePathToFull(
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("ConvertedMeshes"), OwnerName));
}

/**
 * Write a static mesh component's collision geometry out as OBJ, and say what
 * was written.
 *
 * The bytes go to disk rather than into the spec because that is where every
 * other asset's bytes are: the compile's asset pass resolves each `<mesh>`
 * element's `file` and mounts what it reads, so a generated hull and an imported
 * one take the same route into the VFS and survive a recompile the same way. The
 * content hash beside the OBJ is what keeps a recompile from re-running a convex
 * decomposition that nothing invalidated.
 */
TArray<FMjConvertedHull> ExportHulls(UStaticMeshComponent& Smc, const FString& OwnerName, bool bComplex,
	float CoacdThreshold)
{
	TArray<FMjConvertedHull> Out;

	UStaticMesh* Mesh = Smc.GetStaticMesh();
	if (Mesh == nullptr)
	{
		return Out;
	}
	UBodySetup* BodySetup = Mesh->GetBodySetup();
	if (BodySetup == nullptr)
	{
		return Out;
	}
	// A body setup that has never been asked for its physics meshes has no
	// triangle geometry to export, which is the usual state of a mesh nothing
	// has simulated yet.
	if (BodySetup->TriMeshGeometries.Num() == 0)
	{
		BodySetup->CreatePhysicsMeshes();
	}
	if (BodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjQuickConvert] '%s' has no triangle collision to convert."),
			*Mesh->GetName());
		return Out;
	}

	const FVector Scale = Smc.GetComponentScale();
	FString AssetName = Mesh->GetName();
	if (!Scale.Equals(FVector::OneVector, 0.001f))
	{
		AssetName = FString::Printf(TEXT("%s_s%.3f_%.3f_%.3f"), *AssetName, Scale.X, Scale.Y, Scale.Z);
	}

	const FString Directory = HullDirectory(OwnerName);
	const FString ObjPath = FPaths::Combine(Directory,
		FString::Printf(TEXT("%s_%s.obj"), bComplex ? TEXT("Complex") : TEXT("Simple"), *AssetName));

	const auto& TriGeom = BodySetup->TriMeshGeometries[0];
	const auto& Vertices = TriGeom.GetReference()->Particles().X();
	const bool bLargeIndices = TriGeom.GetReference()->Elements().RequiresLargeIndices();

	FString Hash;
	if (bLargeIndices)
	{
		Hash = IO::ComputeMeshHash(Vertices, TriGeom.GetReference()->Elements().GetLargeIndexBuffer());
	}
	else
	{
		Hash = IO::ComputeMeshHash(Vertices, TriGeom.GetReference()->Elements().GetSmallIndexBuffer());
	}
	Hash += bComplex ? TEXT("_complex") : TEXT("_simple");
	if (bComplex)
	{
		Hash += FString::Printf(TEXT("_t%.4f"), CoacdThreshold);
	}

	int32 Count = IO::NumFilesExist(ObjPath, bComplex);
	if (Count > 0 && IO::LoadMeshHash(ObjPath) != Hash)
	{
		IO::DeleteMeshCache(ObjPath, bComplex);
		Count = 0;
	}

	if (Count == 0)
	{
		IFileManager::Get().MakeDirectory(*Directory, true);
		Count = bLargeIndices
			? MeshUtils::SaveMesh(ObjPath, Vertices, TriGeom.GetReference()->Elements().GetLargeIndexBuffer(),
				  bComplex, CoacdThreshold)
			: MeshUtils::SaveMesh(ObjPath, Vertices, TriGeom.GetReference()->Elements().GetSmallIndexBuffer(),
				  bComplex, CoacdThreshold);
		if (Count == 0)
		{
			UE_LOG(LogURLab, Error, TEXT("[MjQuickConvert] could not export '%s' to '%s'."), *AssetName, *ObjPath);
			return Out;
		}
		IO::SaveMeshHash(ObjPath, Hash);
	}

	const FString BaseName = FPaths::GetBaseFilename(ObjPath);
	if (!bComplex)
	{
		Out.Add(FMjConvertedHull{AssetName, ObjPath});
		return Out;
	}
	for (int32 Index = 0; Index < Count; ++Index)
	{
		Out.Add(FMjConvertedHull{FString::Printf(TEXT("%s_%d"), *AssetName, Index),
			FPaths::Combine(Directory, FString::Printf(TEXT("%s_sub_%d.obj"), *BaseName, Index))});
	}
	return Out;
}
}  // namespace

UMjQuickConvertComponent::UMjQuickConvertComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
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

void UMjQuickConvertComponent::DrawDebugCollision()
{
	if (!m_debug_meshes || !m_CreatedBody)
		return;

	const TOptional<int32>& BodyId = m_CreatedBody->GetBoundId();
	if (!BodyId.IsSet())
		return;

	// The engine is resolved per call rather than cached: this component never
	// holds a model or a data pointer, so there is no way for it to outlive a
	// recompile holding one.
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
	const mjModel* const Model = Engine != nullptr ? Engine->GetModel() : nullptr;
	if (Model == nullptr)
		return;

	float Multiplier = 100.0f;
	UWorld* World = GetWorld();
	if (!World)
		return;

	const int32 FirstGeom = Model->body_geomadr[*BodyId];
	const int32 GeomCount = Model->body_geomnum[*BodyId];

	// The hull is model data, fixed by the compile; only the pose comes from
	// simulation state, and that is read from the engine's published snapshot in
	// one visit. The drawing runs after the visitor returns rather than under
	// the lock it holds, because a hull is thousands of debug lines.
	struct FHullPose
	{
		int32 MeshId = INDEX_NONE;
		FVector Position = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
	};

	TArray<FHullPose> Poses;
	Poses.Reserve(FMath::Max(GeomCount, 0));

	Engine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
		for (int32 GeomId = FirstGeom; GeomId < FirstGeom + GeomCount; ++GeomId)
		{
			const int32 MeshId = Model->geom_dataid[GeomId];
			if (MeshId < 0 || Model->mesh_graphadr[MeshId] == -1)
			{
				continue;
			}
			if (!Snap.GeomXPos.IsValidIndex(GeomId * 3 + 2)
				|| !Snap.GeomXMat.IsValidIndex(GeomId * 9 + 8))
			{
				continue;
			}

			const mjtNum* pos = &Snap.GeomXPos[GeomId * 3];
			mjtNum _quat[4];
			mju_mat2Quat(_quat, &Snap.GeomXMat[GeomId * 9]);

			FHullPose& Pose = Poses.AddDefaulted_GetRef();
			Pose.MeshId = MeshId;
			Pose.Position = FVector(pos[0], -pos[1], pos[2]) * Multiplier;
			Pose.Rotation = URLabAxisConv::MjQuatToUe(_quat);
		}
	});

	for (const FHullPose& Pose : Poses)
	{
		const int32 meshId = Pose.MeshId;
		const FVector Position = Pose.Position;
		const FQuat quat = Pose.Rotation;

		int graphStart = Model->mesh_graphadr[meshId];
		int* graphData = Model->mesh_graph + graphStart;

		int numVert = graphData[0];
		int numFace = graphData[1];
		int* edgeLocalId = &graphData[2 + numVert * 2];
		int* faceGlobalId = &edgeLocalId[numVert + 3 * numFace];

		float* vertices = Model->mesh_vert + Model->mesh_vertadr[meshId] * 3;
		for (int j = 0; j < numFace; ++j)
		{
			int v1_global = faceGlobalId[3 * j];
			int v2_global = faceGlobalId[3 * j + 1];
			int v3_global = faceGlobalId[3 * j + 2];

			// Negate Y for MJ -> UE handedness conversion
			FVector vertex1(vertices[3 * v1_global], -vertices[3 * v1_global + 1], vertices[3 * v1_global + 2]);
			FVector vertex2(vertices[3 * v2_global], -vertices[3 * v2_global + 1], vertices[3 * v2_global + 2]);
			FVector vertex3(vertices[3 * v3_global], -vertices[3 * v3_global + 1], vertices[3 * v3_global + 2]);

			vertex1 *= Multiplier;
			vertex2 *= Multiplier;
			vertex3 *= Multiplier;

			vertex1 = quat.RotateVector(vertex1);
			vertex2 = quat.RotateVector(vertex2);
			vertex3 = quat.RotateVector(vertex3);

			vertex1 += Position;
			vertex2 += Position;
			vertex3 += Position;

			DrawDebugLine(World, vertex1, vertex2, FColor::Magenta, false, -1, 0, 0.15f);
			DrawDebugLine(World, vertex2, vertex3, FColor::Magenta, false, -1, 0, 0.15f);
			DrawDebugLine(World, vertex3, vertex1, FColor::Magenta, false, -1, 0, 0.15f);
		}
	}
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

void UMjQuickConvertComponent::AuthorSceneSpec()
{
	m_CreatedBody = nullptr;
	m_GeomElements.Reset();
	m_BodyName.Reset();

#if URLAB_MJ_GEN
	AActor* Owner = GetOwner();
	UMjModel* Root = EnsureSpecRoot();
	if (Owner == nullptr || Root == nullptr)
	{
		return;
	}
	MjDestroySpecChildren(*Root);

	TArray<UStaticMeshComponent*> Meshes;
	Owner->GetComponents(Meshes);
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
		return;
	}

	urlab::spec::FMjInstanceScope Scope(*Owner);

	UMjAsset& Assets = urlab::spec::FInstanceNodeFactory::Create<UMjAsset>(*Root);
	UMjBodyBase& WorldBody = urlab::spec::FInstanceNodeFactory::Create<UMjBodyBase>(*Root);
	UMjBodyBase& Body = urlab::spec::FInstanceNodeFactory::Create<UMjBodyBase>(WorldBody);
	Body.MjName = TEXT("body");
	m_BodyName = GetScenePrefix() + TEXT("body");

	if (bDrivenByUnreal)
	{
		Body.Mocap = true;
	}
	else if (!Static)
	{
		urlab::spec::FInstanceNodeFactory::Create<UMjFreeJoint>(Body).MjName = TEXT("free");
	}

	const FTransform OwnerTransform = Owner->GetActorTransform();
	TSet<FString> EmittedMeshes;
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

		auto AddMeshAsset = [&](const FMjConvertedHull& Hull) {
			// Two mesh components sharing one static mesh share one <mesh>: the
			// hull is keyed on the asset and its scale, so a second element of
			// the same name would be a duplicate MuJoCo rejects outright.
			if (!EmittedMeshes.Contains(Hull.Name))
			{
				EmittedMeshes.Add(Hull.Name);
				// The generated base: this element's geometry is the hull file the
				// conversion just wrote, not an imported asset, so there is
				// nothing for the subclass's reference to hold.
				UMjMeshBase& Mesh = urlab::spec::FInstanceNodeFactory::Create<UMjMeshBase>(Assets);
				Mesh.MjName = Hull.Name;
				// The absolute path: the asset pass resolves `file` against the
				// element's own source directory, and a generated hull has none.
				// What it mounts is the basename, prefixed with the
				// participant's, which is what keeps two converted actors from
				// sharing a hull.
				Mesh.File = Hull.ObjPath;
				Mesh.Scale = FMjVec3(MeshScale.X, MeshScale.Y, MeshScale.Z);
			}
			return Hull.Name;
		};

		auto AddGeom = [&](const FString& GeomName, const FString& MeshName) -> UMjGeomBase& {
			UMjGeomBase& Geom = urlab::spec::FInstanceNodeFactory::Create<UMjGeomBase>(Body);
			Geom.MjName = GeomName;
			Geom.Type = EMjGeomType::mesh;
			Geom.Mesh = MeshName;
			Geom.Pos = FMjPosition3::FromUnreal(RelativeLocation);
			Geom.Quat = FMjQuatRot::FromUnreal(RelativeRotation);
			m_GeomElements.Add(&Geom);
			return Geom;
		};

		auto ApplyContact = [&](UMjGeomBase& Geom) {
			Geom.Friction = TArray<double>{friction.X, friction.Y, friction.Z};
			Geom.Solref = TArray<double>{solref.X, solref.Y};
			Geom.Solimp = TArray<double>{solimp.X, solimp.Y, solimp.Z};
		};

		if (ComplexMeshRequired)
		{
			// The visual hull is the undecomposed mesh with contact switched
			// off: it exists so the viewer sees the shape the artist made
			// rather than the decomposition that collides.
			for (const FMjConvertedHull& Hull : ExportHulls(*Smc, Owner->GetName(), false, CoACDThreshold))
			{
				UMjGeomBase& Visual =
					AddGeom(FString::Printf(TEXT("Geom_%d_visual"), MeshIndex), AddMeshAsset(Hull));
				Visual.Contype = 0;
				Visual.Conaffinity = 0;
				Visual.Group = 2;
				break;
			}

			int32 HullIndex = 0;
			for (const FMjConvertedHull& Hull : ExportHulls(*Smc, Owner->GetName(), true, CoACDThreshold))
			{
				UMjGeomBase& Collision =
					AddGeom(FString::Printf(TEXT("Geom_%d_%d"), MeshIndex, HullIndex), AddMeshAsset(Hull));
				Collision.Group = 3;
				ApplyContact(Collision);
				++HullIndex;
			}
		}
		else
		{
			int32 HullIndex = 0;
			for (const FMjConvertedHull& Hull : ExportHulls(*Smc, Owner->GetName(), false, CoACDThreshold))
			{
				ApplyContact(AddGeom(FString::Printf(TEXT("Geom_%d_%d"), MeshIndex, HullIndex), AddMeshAsset(Hull)));
				++HullIndex;
			}
		}
		++MeshIndex;
	}

	m_CreatedBody = &Body;

	UE_LOG(LogURLab, Log, TEXT("[MjQuickConvert] '%s': authored body '%s' with %d geom(s)."), *Owner->GetName(),
		*m_BodyName, m_GeomElements.Num());
#endif
}

void UMjQuickConvertComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();

	if (bDrivenByUnreal && m_CreatedBody && Owner)
	{
		UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(this);
		const int32 BodyId = GetMjBodyId();
		if (Engine && BodyId >= 0)
		{
			double Pos[3];
			double Quat[4];
			URLabAxisConv::UePositionToMj(Owner->GetActorLocation(), Pos);
			URLabAxisConv::UeQuatToMj(Owner->GetActorQuat(), Quat);
			Engine->SubmitMocapPose(BodyId, Pos, Quat);
		}
	}

	if (m_debug_meshes)
	{
		DrawDebugCollision();
	}
}

void UMjQuickConvertComponent::ApplyRenderState(const FMjRenderSnapshot& Snap)
{
	AActor* Owner = GetOwner();

	if (!m_CreatedBody || !Owner || bDrivenByUnreal)
	{
		return;
	}

	const int32 Id = GetMjBodyId();
	if (Id < 0)
	{
		return;
	}

	const int32 PosIdx = Id * 3;
	const int32 QuatIdx = Id * 4;
	if (Snap.XPos.Num() <= PosIdx + 2 || Snap.XQuat.Num() <= QuatIdx + 3)
	{
		return;
	}

	const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XPos[PosIdx]);
	const FQuat Quat = URLabAxisConv::MjQuatToUe(&Snap.XQuat[QuatIdx]);

	// Mirror UMjBody::ApplyRenderState: never apply a zero/NaN snapshot row --
	// it writes a degenerate transform that NaN-floods the renderer.
	if (Pos.ContainsNaN() || Quat.ContainsNaN()
		|| Quat.SizeSquared() < KINDA_SMALL_NUMBER)
	{
		return;
	}

	Owner->SetActorRotation(Quat);
	Owner->SetActorLocation(Pos);
}
