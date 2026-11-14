// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
#include "MuJoCo/Authoring/MjSceneContributor.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjSceneOptions.h"
#include "MuJoCoSpec/MJAssetLibrary.h"
#include "MuJoCoSpec/MJModelComponent.h"
#include "MuJoCoSpec/MJSceneComponents.h"
#include "MuJoCoSpec/MJSpecHandle.h"
#include "MuJoCoSpec/MJUnitConverter.h"
#include "MuJoCoSpec/MJActorComponents.h"
#include "EngineUtils.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
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

void RefreshActorComponentIds(AActor* Actor, const mjModel* Model, const FString& Prefix)
{
	if (!Actor || !Model) return;

	auto ResolveId = [Model, &Prefix](mjtObj Type, const FString& Name) -> int32
	{
		const FString FullName = Prefix.IsEmpty() ? Name : Prefix + Name;
		return mj_name2id(Model, Type, TCHAR_TO_UTF8(*FullName));
	};

	TArray<UMJBodyComponent*> Bodies;
	Actor->GetComponents<UMJBodyComponent>(Bodies);
	for (UMJBodyComponent* B : Bodies)
	{
		if (B) B->BodyId = ResolveId(mjOBJ_BODY, B->GetMJName());
	}

	TArray<UMJGeomComponent*> Geoms;
	Actor->GetComponents<UMJGeomComponent>(Geoms);
	for (UMJGeomComponent* G : Geoms)
	{
		if (G) G->GeomId = ResolveId(mjOBJ_GEOM, G->GetMJName());
	}

	TArray<UMJJointComponent*> Joints;
	Actor->GetComponents<UMJJointComponent>(Joints);
	for (UMJJointComponent* J : Joints)
	{
		if (J) J->JointId = ResolveId(mjOBJ_JOINT, J->GetMJName());
	}

	TArray<UMJSiteComponent*> Sites;
	Actor->GetComponents<UMJSiteComponent>(Sites);
	for (UMJSiteComponent* S : Sites)
	{
		if (S) S->SiteId = ResolveId(mjOBJ_SITE, S->GetMJName());
	}
}
} // namespace

FMjAuthoredSimulationPackage FMjAuthoringCompiler::Compile(AAMjManager& SceneRoot)
{
	FMjAuthoredSimulationPackage Out;
	check(IsInGameThread());

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

	FMJSpecHandle SceneSpec(mj_makeSpec());
	if (!SceneSpec)
	{
		Out.Error = TEXT("mj_makeSpec failed");
		return Out;
	}
	MjApplySceneOptionDefaults(SceneSpec.Get());

	FMJVFSHandle Vfs;
	Vfs.Init();

	if (SceneRoot.SceneModel && SceneRoot.SceneModel->AssetLibrary)
	{
		SceneRoot.SceneModel->AssetLibrary->PopulateVFS(Vfs.Get());
	}

	mjsBody* WorldBody = mjs_findBody(SceneSpec.Get(), "world");
	if (!WorldBody)
	{
		Out.Error = TEXT("scene spec has no world body");
		return Out;
	}

	struct FAttachedParticipant
	{
		AActor* Actor = nullptr;
		FString Prefix;
	};
	TArray<FAttachedParticipant> AttachedParticipants;

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor || Actor == &SceneRoot || Actor->IsActorBeingDestroyed())
		{
			continue;
		}

		UMJModelComponent* ModelComp = Actor->FindComponentByClass<UMJModelComponent>();
		if (!ModelComp)
		{
			continue;
		}

		if (ModelComp->AssetLibrary)
		{
			ModelComp->AssetLibrary->PopulateVFS(Vfs.Get());
		}

		FString BuildError;
		if (!ModelComp->BuildSpecFromActor(BuildError))
		{
			Out.Error = BuildError;
			return Out;
		}

		mjSpec* ChildSpec = ModelComp->GetSpec();
		if (!ChildSpec)
		{
			continue;
		}

		FString Prefix;
		if (AMjArticulation* Art = Cast<AMjArticulation>(Actor))
		{
			Prefix = Art->GetCompiledPrefix();
		}

		mjsFrame* Frame = mjs_addFrame(WorldBody, nullptr);
		if (Frame)
		{
			FMJUnitConverter::UETransformToMJ(Actor->GetActorTransform(), Frame->pos, Frame->quat);
			FTCHARToUTF8 PrefixUtf8(*Prefix);
			if (!mjs_attach(Frame->element, ChildSpec->element, PrefixUtf8.Get(), ""))
			{
				const char* Err = mjs_getError(SceneSpec.Get());
				Out.Error = UTF8_TO_TCHAR(Err && Err[0] ? Err : "mjs_attach failed");
				return Out;
			}
		}

		AttachedParticipants.Add({Actor, Prefix});
	}

	const TArray<UObject*> Contributors = GatherContributors(*World);
	for (UObject* Object : Contributors)
	{
		IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object);
		if (!Contributor) continue;
		FString ContributorError;
		if (!Contributor->ContributeToSpec(SceneSpec.Get(), Contributor->GetScenePrefix(), ContributorError))
		{
			Out.Error = ContributorError.IsEmpty()
				? FString::Printf(TEXT("scene contributor '%s' failed to author its spec"), *Object->GetName())
				: ContributorError;
			return Out;
		}
	}

	FMJModelHandle CompiledModel(mj_compile(SceneSpec.Get(), Vfs.Get()));
	if (!CompiledModel)
	{
		const char* Err = mjs_getError(SceneSpec.Get());
		Out.Error = UTF8_TO_TCHAR(Err && Err[0] ? Err : "mj_compile failed");
		return Out;
	}

	for (const FAttachedParticipant& Participant : AttachedParticipants)
	{
		RefreshActorComponentIds(Participant.Actor, CompiledModel.Get(), Participant.Prefix);
	}
	for (UObject* Object : Contributors)
	{
		if (IMjSceneContributor* Contributor = Cast<IMjSceneContributor>(Object))
			Contributor->OnSceneBound();
	}

	Out.Presentation.SceneOptions.SceneTransform = FTransform::Identity;
	Out.Presentation.SceneOptions.bPersistentEditorActors = false;
	Out.Presentation.SceneOptions.bInitiallyVisible = true;
	Out.Presentation.CameraMaxHeight = -1;
	Out.Presentation.bIncludeUserCamera = false;

	const FString Label = FString::Printf(TEXT("authored:%s"), *World->GetPathName());
	if (!FMjOwnedModelCandidate::CopyCompiled(CompiledModel.Get(), Out.Candidate, Out.Error, Label))
	{
		return Out;
	}
	return Out;
}
