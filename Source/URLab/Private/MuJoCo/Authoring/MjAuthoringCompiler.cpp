// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "MuJoCo/Authoring/MjAuthoringCompiler.h"

#include "EngineUtils.h"
#include "Components/ActorComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjBinding.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjSceneContributor.h"
#include "MuJoCo/Spec/MjSceneSpec.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "MuJoCo/Spec/MjSpecNodes.h"
#include "Utils/CoordinateConversion.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
void AddParticipant(FSceneAssembly& Scene, const FSpecRef& Spec, const FString& Prefix,
	const FTransform& Placement, TOptional<EMjConflict> Conflict = {})
{
	if (!Spec.IsValid()) return;
	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(Placement.GetLocation(), MjPos);
	URLabCoordinateConversion::UnrealQuaternionToRightHandedWxyz(Placement.GetRotation(), MjQuat);
	Scene.Add(Spec, Prefix, FVector(MjPos[0], MjPos[1], MjPos[2]),
		FQuat(MjQuat[1], MjQuat[2], MjQuat[3], MjQuat[0]), Conflict);
}

TArray<UObject*> GatherContributors(UWorld& World)
{
	TArray<UObject*> Out;
	for (TActorIterator<AActor> It(&World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor) continue;
		if (Actor->Implements<UMjSceneContributor>()) Out.Add(Actor);
		for (UActorComponent* Component : Actor->GetComponents())
		{
			if (Component && Component->Implements<UMjSceneContributor>()) Out.Add(Component);
		}
	}
	return Out;
}

void PublishAuthoringBinding(UWorld& World, const FMjBinding& Binding)
{
	// Bound ids are authoring feedback only. Runtime receives neither these
	// components nor this binding.
	for (TActorIterator<AActor> It(&World); It; ++It)
	{
		TArray<UMjNodeComponent*> Nodes;
		It->GetComponents(Nodes);
		for (UMjNodeComponent* Node : Nodes)
		{
			if (Node) Node->Unbind();
		}
	}
	for (const FMjBinding::FEntry& Entry : Binding.GetEntries())
	{
		if (Entry.Node && Entry.Id >= 0)
		{
			const_cast<UMjNodeComponent*>(Entry.Node)->BindTo(Entry.Id);
		}
	}
}

FMjResolvedRenderCameraOverrides ResolveCameraOverrides(const FMjBinding& Binding)
{
	FMjResolvedRenderCameraOverrides Out;
	for (const FMjBinding::FEntry& Entry : Binding.GetEntries())
	{
		UMjCamera* Camera = Entry.ObjType == mjOBJ_CAMERA
			? Cast<UMjCamera>(const_cast<UMjNodeComponent*>(Entry.Node)) : nullptr;
		if (!Camera || Entry.Id < 0) continue;
		FMjResolvedRenderCameraOverride& Override = Out.FindOrAdd(Entry.Id);
		Override.Capture = Camera->ExportCaptureSettings();
		Override.bEnableZmq = Camera->bEnableZmqBroadcast;
		Override.bEnableShm = Camera->bEnableShmBroadcast;
		Override.ZmqEndpoint = Camera->ZmqEndpoint;
		Override.CustomNearClipCm = Camera->CustomNearClipCm;
		Override.CustomFarClipCm = Camera->CustomFarClipCm;
	}
	return Out;
}
}

FMjAuthoredSimulationPackage FMjAuthoringCompiler::Compile(AAMjManager& SceneRoot)
{
	FMjAuthoredSimulationPackage Out;
	check(IsInGameThread());

#if !URLAB_MJ_GEN
	Out.Error = TEXT("built without the generated MuJoCo profile");
	return Out;
#else
	UWorld* World = SceneRoot.GetWorld();
	if (!World)
	{
		Out.Error = TEXT("authored scene root has no world");
		return Out;
	}
	for (TActorIterator<AAMjManager> It(World); It; ++It)
	{
		if (*It && *It != &SceneRoot && !It->IsActorBeingDestroyed())
		{
			Out.Error = TEXT("the world contains more than one authored scene root");
			return Out;
		}
	}

	FSceneAssembly Assembly;
	Assembly.SetSceneRoot(SceneRoot.GetSceneSpec());
	for (TActorIterator<AMjArticulation> It(World); It; ++It)
	{
		AMjArticulation* Articulation = *It;
		if (Articulation)
		{
			AddParticipant(Assembly, Articulation->GetSpec(), Articulation->GetCompiledPrefix(),
				Articulation->GetActorTransform(), Articulation->AttachConflict);
		}
	}

	const TArray<UObject*> Contributors = GatherContributors(*World);
	for (UObject* Object : Contributors)
	{
		IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object);
		if (!Contributor) continue;
		FString ContributorError;
		if (!Contributor->AuthorSceneSpec(ContributorError))
		{
			Out.Error = ContributorError.IsEmpty()
				? FString::Printf(TEXT("scene contributor '%s' failed to author its spec"), *Object->GetName())
				: ContributorError;
			return Out;
		}
		AddParticipant(Assembly, Contributor->GetSceneSpec(), Contributor->GetScenePrefix(),
			Contributor->GetScenePlacement());
	}

	const FSpecRef RootSpec = SceneRoot.GetSceneSpec();
	auto PrepareResources = [&Out](const FSpecRef& Spec) -> bool {
#if WITH_EDITOR
		FString ReferenceError;
		if (!urlab::spec::MjSynchronizeReferenceBindings(Spec, ReferenceError))
		{
			Out.Error = ReferenceError;
			return false;
		}
		for (UMjNodeComponent* Node : urlab::spec::MjSpecNodesOf(Spec).Nodes)
		{
			if (!Node) continue;
			FString Error;
			if (!MjPrepareAssetResource(*Node, Error))
			{
				Out.Error = Error;
				return false;
			}
		}
#endif
		return true;
	};
	if (!PrepareResources(RootSpec)) return Out;
	urlab::spec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(RootSpec);
	for (const FMjSceneParticipant& Participant : Assembly.GetParticipants())
	{
		if (!PrepareResources(Participant.Spec)) return Out;
		urlab::spec::FMjSceneSpecParticipant Placed;
		Placed.Spec = Participant.Spec;
		Placed.Prefix = Participant.Prefix;
		Placed.MjPos = Participant.MjPos;
		Placed.MjQuat = Participant.MjQuat;
		Placed.Conflict = Participant.Conflict;
		Builder.AddParticipant(Placed);
	}

	urlab::spec::FMjCompiledScene Compiled = Builder.Compile();
	for (const FMjSpecDiagnostic& Warning : Compiled.Warnings)
		Out.Warnings.Add(Warning.ToString());
	for (const FMjSpecDiagnostic& Info : Compiled.Infos)
		Out.Infos.Add(Info.ToString());
	if (!Compiled.IsValid())
	{
		TArray<FString> Reasons;
		for (const FMjSpecDiagnostic& Error : Compiled.Errors) Reasons.Add(Error.ToString());
		Out.Error = Reasons.Num() ? FString::Join(Reasons, TEXT("; "))
			: TEXT("the authored scene did not compile");
		return Out;
	}

	const FMjBinding Binding = MjBindingOf(Compiled);
	PublishAuthoringBinding(*World, Binding);
	for (UObject* Object : Contributors)
	{
		if (IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object))
			Contributor->OnSceneBound();
	}

	Out.Presentation.CameraOverrides = ResolveCameraOverrides(Binding);
	// Participant placement is already authored in world coordinates, matching
	// the legacy compiled-view path. Applying the manager transform again would
	// double-transform articulations and world-sampled contributors.
	Out.Presentation.SceneOptions.SceneTransform = FTransform::Identity;
	Out.Presentation.SceneOptions.bPersistentEditorActors = false;
	Out.Presentation.SceneOptions.bInitiallyVisible = true;
	Out.Presentation.CameraMaxHeight = -1;
	Out.Presentation.bIncludeUserCamera = false;

	const FString Label = FString::Printf(TEXT("authored:%s"), *World->GetPathName());
	if (!FMjOwnedModelCandidate::CopyCompiled(Compiled.Model, Out.Candidate, Out.Error, Label))
	{
		return Out;
	}
	return Out;
#endif
}
