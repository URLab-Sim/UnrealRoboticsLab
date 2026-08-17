// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjRendererOverlay.h"

#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Utils/MjColor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/MeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Utils/URLabLogging.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_EDITOR
namespace
{
// Which body a geom belongs to is a fact of the compiled model, not of the spec, so
// it is read out of geom_bodyid at the id the element bound to. Negative when there is
// no compiled model or the geom did not survive the compile — the set the overlays skip.
int32 AuthoringGeomBodyId(const mjModel* Model, const UMjGeom* Geom)
{
	if (!Model || !Geom || !Geom->GetBoundId().IsSet())
		return -1;

	const int32 GeomId = Geom->GetBoundId().GetValue();
	if (GeomId < 0 || GeomId >= Model->ngeom)
		return -1;

	return Model->geom_bodyid[GeomId];
}
} // namespace
#endif

void FMjRendererOverlay::Initialize(AMjRenderer& Scene)
{
	if (Scene.OverlayParentMaterial)
	{
		return;
	}

	UMaterialInterface* Parent = LoadObject<UMaterialInterface>(
		nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (!Parent)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjRenderer] failed to load /Engine/BasicShapes/BasicShapeMaterial — overlays disabled"));
		return;
	}

	TArray<FMaterialParameterInfo> VecInfos;
	TArray<FGuid> Guids;
	Parent->GetAllVectorParameterInfo(VecInfos, Guids);
	if (VecInfos.Num() == 0)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjRenderer] BasicShapeMaterial exposes no vector params — overlays disabled"));
		return;
	}

	// Prefer well-known colour-param names so we don't accidentally drive an emissive tint etc.
	static const FName PreferredNames[] = {
		TEXT("Color"), TEXT("BaseColor"), TEXT("Tint"), TEXT("TintColor"), TEXT("DiffuseColor")};
	FName Chosen = NAME_None;
	for (const FName& Pref : PreferredNames)
	{
		for (const FMaterialParameterInfo& Info : VecInfos)
		{
			if (Info.Name == Pref)
			{
				Chosen = Pref;
				break;
			}
		}
		if (!Chosen.IsNone())
			break;
	}
	if (Chosen.IsNone())
		Chosen = VecInfos[0].Name;

	Scene.OverlayParentMaterial = Parent;
	Scene.OverlayColorParam = Chosen;
}

void FMjRendererOverlay::Clear()
{
	for (auto& Pair : OriginalMaterials)
	{
		UMeshComponent* Mesh = Pair.Key.Get();
		if (!Mesh)
			continue;
		Mesh->SetMaterial(0, Pair.Value);

		if (const TMap<int32, TObjectPtr<UMaterialInterface>>* Extra = OriginalSlotMaterials.Find(Pair.Key))
		{
			for (const auto& SlotPair : *Extra)
			{
				Mesh->SetMaterial(SlotPair.Key, SlotPair.Value);
			}
		}
	}
	OriginalMaterials.Reset();
	OriginalSlotMaterials.Reset();
	ActiveMIDs.Reset();
}

void FMjRendererOverlay::Apply(AMjRenderer& Scene, EMjDebugShaderMode Mode, const TArray<int32>& BodyAwake,
	const TArray<int32>& BodyIslandSeed, bool bModulateBySleep,
	float SleepValueScale, float SleepSaturationScale)
{
	if (Mode == EMjDebugShaderMode::Off)
	{
		if (OriginalMaterials.Num() > 0)
			Clear();
		return;
	}

	if (!Scene.OverlayParentMaterial || Scene.OverlayColorParam.IsNone())
		return;

	AAMjManager* Manager = Scene.ResolveManager();

	auto ApplyToMesh = [&](UMeshComponent* Mesh, int32 BodyId, uint32 GroupHash) {
		if (!Mesh)
			return;

		const bool bAwake =
			(BodyAwake.IsValidIndex(BodyId) ? BodyAwake[BodyId] != 0 : true);
		const int32 Seed =
			(BodyIslandSeed.IsValidIndex(BodyId) ? BodyIslandSeed[BodyId] : -1);

		TWeakObjectPtr<UMeshComponent> WeakMesh(Mesh);
		const int32 NumSlots = FMath::Max(1, Mesh->GetNumMaterials());

		if (!OriginalMaterials.Contains(WeakMesh))
		{
			OriginalMaterials.Add(WeakMesh, Mesh->GetMaterial(0));
			for (int32 SlotIdx = 1; SlotIdx < NumSlots; ++SlotIdx)
			{
				OriginalSlotMaterials.FindOrAdd(WeakMesh).Add(SlotIdx, Mesh->GetMaterial(SlotIdx));
			}
		}

		const bool bColourAsAwake = bAwake || !bModulateBySleep;
		FLinearColor Color;
		switch (Mode)
		{
			case EMjDebugShaderMode::Island:
				Color = MjColor::IslandColor(Seed, bColourAsAwake,
					SleepValueScale, SleepSaturationScale);
				break;
			case EMjDebugShaderMode::InstanceSegmentation:
				Color = MjColor::InstanceSegmentationColor(GroupHash, BodyId, bColourAsAwake,
					SleepValueScale, SleepSaturationScale);
				break;
			case EMjDebugShaderMode::SemanticSegmentation:
				Color = MjColor::SemanticSegmentationColor(GroupHash, bColourAsAwake,
					SleepValueScale, SleepSaturationScale);
				break;
			default:
				return;
		}

		UMaterialInstanceDynamic* MID = nullptr;
		if (TObjectPtr<UMaterialInstanceDynamic>* Existing = ActiveMIDs.Find(WeakMesh))
		{
			MID = *Existing;
		}
		if (!MID)
		{
			MID = UMaterialInstanceDynamic::Create(Scene.OverlayParentMaterial, &Scene);
			ActiveMIDs.Add(WeakMesh, MID);
		}

		for (int32 SlotIdx = 0; SlotIdx < NumSlots; ++SlotIdx)
		{
			if (Mesh->GetMaterial(SlotIdx) != MID)
			{
				Mesh->SetMaterial(SlotIdx, MID);
			}
		}

		MID->SetVectorParameterValue(Scene.OverlayColorParam, Color);
	};

	// This renderer draws the geometry, so the per-body overlay swaps the material on
	// its own geom components (keyed by mj geom id), grouped by geom_bodyid and coloured
	// by the geom's originating participant.
	if (Scene.GetModelPtr())
	{
		const int32 NGeom = Scene.NumGeoms();
		for (int32 G = 0; G < NGeom; ++G)
		{
			UMeshComponent* Mesh = Cast<UMeshComponent>(Scene.GetGeomComponent(G));
			if (!Mesh)
				continue;
			const int32 BodyId = Scene.GetModelPtr()->geom_bodyid[G];
			if (BodyId < 0)
				continue;

			uint32 ArtHash = GetTypeHash(Scene.GetClass()->GetFName());
			if (UMjGeom* Origin = Scene.GetGeomOrigin(G))
			{
				if (AActor* GeomOwner = Origin->GetOwner())
					ArtHash = GetTypeHash(GeomOwner->GetClass()->GetFName());
			}

			ApplyToMesh(Mesh, BodyId, ArtHash);

			TArray<USceneComponent*> ChildComps;
			Mesh->GetChildrenComponents(true, ChildComps);
			for (USceneComponent* Child : ChildComps)
			{
				if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
					ApplyToMesh(SMC, BodyId, ArtHash);
			}
		}
	}

	if (!Manager)
		return;

#if WITH_EDITOR
	const mjModel* AuthoringModel = Manager->PhysicsEngine ? Manager->PhysicsEngine->m_model : nullptr;

	// The editor preview (and the test harness) keeps the authoring visualizer meshes,
	// plus any static mesh a caller hung under a geom themselves.
	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;

		// Semantic grouping hashes the Blueprint class so two instances share colour.
		const uint32 ArtHash = GetTypeHash(Art->GetClass()->GetFName());

		for (UMjGeom* Geom : Art->GetGeoms())
		{
			const int32 BodyId = AuthoringGeomBodyId(AuthoringModel, Geom);
			if (BodyId < 0)
				continue;

			ApplyToMesh(Geom->GetVisualizerMesh(), BodyId, ArtHash);

			TArray<USceneComponent*> ChildComps;
			Geom->GetChildrenComponents(true, ChildComps);
			for (USceneComponent* Child : ChildComps)
			{
				if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
				{
					ApplyToMesh(SMC, BodyId, ArtHash);
				}
			}
		}
	}
#endif

	for (UMjQuickConvertComponent* QC : Manager->GetAllQuickComponents())
	{
		if (!QC)
			continue;
		const int32 BodyId = QC->GetMjBodyId();
		if (BodyId < 0)
			continue;

		AActor* GeomOwner = QC->GetOwner();
		if (!GeomOwner)
			continue;

		TArray<UStaticMeshComponent*> MeshComps;
		GeomOwner->GetComponents<UStaticMeshComponent>(MeshComps);

		// Semantic grouping hashes the first static mesh so props sharing a mesh read as one "type".
		uint32 GroupHash = GetTypeHash(GeomOwner->GetClass()->GetFName());
		for (UStaticMeshComponent* SMC : MeshComps)
		{
			if (SMC && SMC->GetStaticMesh())
			{
				GroupHash = GetTypeHash(SMC->GetStaticMesh()->GetFName());
				break;
			}
		}

		for (UStaticMeshComponent* SMC : MeshComps)
		{
			ApplyToMesh(SMC, BodyId, GroupHash);
		}
	}
}
