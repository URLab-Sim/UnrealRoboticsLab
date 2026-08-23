// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjRenderer.h"

#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "EngineUtils.h"
#include "DrawDebugHelpers.h"

#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Utils/MjColor.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Fast/MjRendererAssetBaker.h"
#include "MuJoCo/Fast/MjRendererBus.h"
#include "MuJoCo/Fast/MjLauncherFlags.h"
#include "MuJoCo/Entity/MjAppearance.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Entity/MjBakedAssetResolver.h"
#include "MuJoCo/Entity/MjImportedAssetResolver.h"
#include "MuJoCo/Entity/MjModelSource.h"
#include "MuJoCo/Fast/MjSkyImporter.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Capture/MjCameraSubsystem.h"
#include "MuJoCo/Capture/MjCameraTypes.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjCameraRegistry.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "Transport/NetworkManager.h"
#include "Transport/RpcClientTransport.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraActor.h"
#include "GameFramework/PlayerController.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Utils/URLabLogging.h"
#include "Utils/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Base64.h"
#include "Misc/SecureHash.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// Re-index tag channel: a body/geom/instance/camera carries a "<prefix><id>" name
// tag so a saved level can map its components back to MuJoCo ids. One maker + one
// parser keep the writer and reader in lockstep -- no hardcoded prefix lengths.
constexpr const TCHAR* kTagBody = TEXT("MjbBody=");
constexpr const TCHAR* kTagGeom = TEXT("MjbGeom=");
constexpr const TCHAR* kTagIsm = TEXT("MjbIsm=");
constexpr const TCHAR* kTagCam = TEXT("MjbCam=");

FName MjbIdTag(const TCHAR* Prefix, int32 Id)
{
	return FName(*FString::Printf(TEXT("%s%d"), Prefix, Id));
}

int32 MjbParseIdTag(const FString& Tag, const TCHAR* Prefix)
{
	return Tag.StartsWith(Prefix) ? FCString::Atoi(*Tag.Mid(FCString::Strlen(Prefix))) : -1;
}

// Convert a MuJoCo 3x3 orientation (row-major geom_xmat/cam_xmat) to a UE quat,
// via a wxyz quaternion. Shared by every apply/build path that reads mjData mats.
FQuat MjMat3ToUeQuat(const double* Mat3)
{
	double Quat[4];
	mju_mat2Quat(Quat, Mat3);
	return URLabAxisConv::MjQuatToUe(Quat);
}

// Fast-path render components never contribute to distance-field lighting or AO.
// The shared engine primitive meshes (the Plane especially) carry a mesh distance
// field, and a flat/non-uniformly-scaled primitive yields a degenerate DF matrix
// that spams "InverseFast: NIL/non-invertible matrix" ensures every frame in -game.
// Dropping the component from the DF scene removes the cost and the noise.
void DisableDistanceFields(UPrimitiveComponent* Comp)
{
	if (Comp)
	{
		Comp->SetAffectDistanceFieldLighting(false);
	}
}

#if WITH_EDITOR
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
#endif

} // namespace

AMjRenderer::AMjRenderer()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AMjRenderer::BeginPlay()
{
	Super::BeginPlay();

	// The compiled play view is built and driven by the manager off the shared engine
	// model; it has no MJB of its own to load and nothing to stand up here.
	if (bExternallyDriven)
	{
		return;
	}

	// Pick the single pose driver for this session before anything binds: the forced
	// eval regime owns the render-control REP, the mirror regime owns the transform
	// bus, and the two are mutually exclusive so only one source writes the pose.
	// Phase 1.1: -URLabDrive=push is the new spelling of -URLabFastForcedOnly (source-of-truth §14);
	// both select the forced-render (push) regime that owns the render-control REP.
	bForcedRenderOnly = FParse::Param(FCommandLine::Get(), TEXT("URLabFastForcedOnly"))
		|| URLabLauncherFlags::DriveIsPush();

	// -URLabFastCameras: build + stream the camera components. Honour it here so a
	// renderer PLACED IN THE MAP (not spawned via SpawnRenderer, which sets this from
	// its bCameras arg) also enables cameras -- otherwise a client that hot-swaps a
	// model into the map's placeholder renderer gets no cameras back.
	// Phase 1.1: -URLabCaps=cameras is the new spelling of -URLabFastCameras, and (unlike the legacy
	// flag) is negatable (-cameras) so a headless server can turn the capability OFF from the CLI
	// (source-of-truth §5/§14). An explicit cap wins over the legacy flag / the SpawnRenderer arg.
	if (FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras")))
	{
		bEnableCameraStreaming = true;
	}
	{
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		if (Caps.bCameras.IsSet())
		{
			bEnableCameraStreaming = Caps.bCameras.GetValue();
		}
	}

	// -URLabFastCamMaxHeight=N overrides the per-camera height cap (0 = honour the
	// model's resolution exactly), so an eval can request higher-res frames than the
	// 480 default without editing the level.
	// Phase 1.1: -URLabScene=cammax=N is the new spelling of -URLabFastCamMaxHeight (source-of-truth §14).
	int32 CamMaxHeightArg = -1;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabFastCamMaxHeight="), CamMaxHeightArg))
	{
		URLabLauncherFlags::SceneCamMaxHeight(CamMaxHeightArg);
	}
	if (CamMaxHeightArg >= 0)
	{
		CameraMaxHeight = CamMaxHeightArg;
	}

	// The raw mjModel/mjData pointers were shallow-copied from the editor actor on
	// the PIE duplication and are stale -- clear them WITHOUT freeing (the editor
	// actor still owns its own). The transient index maps do not duplicate either.
	Model = nullptr;
	Data = nullptr;
	BodyActors.Reset();
	GeomComps.Reset();
	CameraComps.Reset();

	const bool bHaveModel = !MjbFilePath.IsEmpty() || MjbBytes.Num() > 0;
	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] BeginPlay (game=%d): file='%s' bytes=%d bus='%s' cameras=%d"),
		GetWorld() && GetWorld()->IsGameWorld(), *MjbFilePath, MjbBytes.Num(),
		*BusEndpoint, bEnableCameraStreaming);

	if (!bHaveModel)
	{
		UE_LOG(LogURLab, Error,
			TEXT("[MjRenderer] BeginPlay: no MJB path or bytes -- nothing to stream (discovery/duplication issue?)"));
		return;
	}

	// Reuse the editor-world preview that duplicated into PIE: reload only the
	// (small) model for the index space and re-wire GeomComps to the already-built
	// components, instead of destroying every actor and rebuilding all the meshes
	// (the expensive part). Falls back to a full build when there is no preview to
	// reuse (e.g. spawned fresh in -game).
	int32 Geoms = ReindexFromLevel();
	if (Geoms > 0)
	{
		// Dynamic material instances made in the editor world do not survive the
		// PIE duplication, and the texture cache is transient -- re-apply materials
		// onto the reused components. Cheap next to a mesh rebuild.
		for (int32 G = 0; G < GeomComps.Num(); ++G)
		{
			if (GeomComps[G])
			{
				AssetBaker->ApplyGeomMaterial(GeomComps[G], G);
			}
		}
		// Same for the instanced statics on the world body (their MID is re-applied
		// via the rep-geom stored in the MjbIsm tag).
		if (BodyActors.IsValidIndex(0) && BodyActors[0])
		{
			TArray<UInstancedStaticMeshComponent*> Isms;
			BodyActors[0]->GetComponents<UInstancedStaticMeshComponent>(Isms);
			for (UInstancedStaticMeshComponent* Ism : Isms)
			{
				for (const FName& Tag : Ism->ComponentTags)
				{
					const int32 RepG = MjbParseIdTag(Tag.ToString(), kTagIsm);
					if (Model && RepG >= 0 && RepG < static_cast<int32>(Model->ngeom))
					{
						AssetBaker->ApplyGeomMaterial(Ism, RepG);
					}
				}
			}
		}
		UE_LOG(LogURLab, Log, TEXT("[MjRenderer] BeginPlay reused preview (%d geoms) -- no mesh rebuild"), Geoms);
	}
	else
	{
		TArray<AActor*> Inherited;
		GetAttachedActors(Inherited, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
		for (AActor* A : Inherited)
		{
			if (A)
			{
				A->Destroy();
			}
		}
		BodyActors.Reset();
		GeomComps.Reset();
		CameraComps.Reset();
		Geoms = LoadAndBuild();
		if (Geoms < 0)
		{
			UE_LOG(LogURLab, Error, TEXT("[MjRenderer] BeginPlay build FAILED (no usable MJB)"));
			return;
		}
	}

	// The camera components exist already -- built dormant in the preview (reused via
	// reindex) or by the fallback LoadAndBuild. Turn capture + streaming on for this
	// play session, so the render server goes live without rebuilding them.
	StartCameraStreaming();

	// Resolve the primary Drive axis BEFORE the fork, which switches on it directly (Phase 1.4).
	// Precedence: -URLabDrive names it explicitly; otherwise SpawnRenderer already set Drive=Sim for
	// the stepped/Direct path (kept by the Drive==Sim arm below); otherwise it is derived from the
	// remaining boot signals. Derivation order:
	//   Drive==Sim (set by SpawnRenderer) -> Sim    (owns physics; the Direct path)
	//   bForcedRenderOnly                 -> Push   (fastpath_render drives the pose; no bus)
	//   -URLabFastServe                   -> Await  (served placeholder, no real model yet)
	//   otherwise (bus / gRPC join / VR / bare mirror) -> Stream (transform-mirror consumer)
	// FastServe is checked before the bus signal because its placeholder renderer has an empty
	// BusEndpoint and would otherwise fall through the fork to no branch; -URLabFastGrpcJoin and
	// -URLabVrViewer are both gRPC-driven mirrors and resolve to Stream.
	// Phase 1.1: when -URLabDrive is given it names the axis directly (source-of-truth §14); its
	// value matches what the legacy-signal derivation below produces, since the launcher/BeginPlay
	// already OR'd -URLabDrive into bForcedRenderOnly / BusEndpoint. Fall back to the legacy
	// derivation when -URLabDrive is absent.
	FString DriveStreamEp;
	const URLabLauncherFlags::EDriveKind DriveKind = URLabLauncherFlags::ParseDrive(DriveStreamEp);
	if (DriveKind == URLabLauncherFlags::EDriveKind::Sim)
	{
		Drive = EMjDrive::Sim;
	}
	else if (DriveKind == URLabLauncherFlags::EDriveKind::Push)
	{
		Drive = EMjDrive::Push;
	}
	else if (DriveKind == URLabLauncherFlags::EDriveKind::Await)
	{
		Drive = EMjDrive::Await;
	}
	else if (DriveKind == URLabLauncherFlags::EDriveKind::Stream)
	{
		Drive = EMjDrive::Stream;
	}
	else if (Drive == EMjDrive::Sim)
	{
		// SpawnRenderer(bStepped=true) already set Drive=Sim before BeginPlay; keep it
		// (this is the old RunMode==Stepped -> Sim derivation, now that Drive is the axis).
		Drive = EMjDrive::Sim;
	}
	else if (bForcedRenderOnly)
	{
		Drive = EMjDrive::Push;
	}
	else if (FParse::Param(FCommandLine::Get(), TEXT("URLabFastServe")))
	{
		Drive = EMjDrive::Await;
	}
	else
	{
		Drive = EMjDrive::Stream;
	}

	// Direct: step this scene's own model through the shared engine and render the
	// stepped state. Puppet (default): mirror an owner's transform stream.
	if (Drive == EMjDrive::Sim)
	{
		Direct.Begin(*this);
	}
	else if (bForcedRenderOnly)
	{
		// Eval regime: the forced request drives the pose, so no bus is connected --
		// but stand up the manager so its bridge serves the fastpath_render op (which
		// replaced the renderer's own REP socket).
		EnsureManager();
	}
	else if (!BusEndpoint.IsEmpty())
	{
		// A subscribe-only mirror (-URLabFastGrpcJoin=<ep>) is the LEAN fast path: a
		// pure client that only consumes the owner's transform stream, so it needs
		// no manager/bridge (hence no interactive MjSimulate widget). A serving bus
		// mirror still stands up the manager so a Driver can push fastpath_load.
		// NOTE: it's a VALUE flag (-URLabFastGrpcJoin=host:port), so detect it with
		// FParse::Value -- FParse::Param only matches a bare switch and misses it.
		// Phase 1.1: -URLabDrive=stream:grpc://<ep> is the new spelling of -URLabFastGrpcJoin (a lean
		// subscribe-only mirror, no manager); recognise both so the new flag stays lean too.
		FString GrpcJoinEp;
		const bool bSubscribeOnly =
			FParse::Value(FCommandLine::Get(), TEXT("URLabFastGrpcJoin="), GrpcJoinEp)
			|| URLabLauncherFlags::DriveStreamGrpcEndpoint(GrpcJoinEp);
		if (!bSubscribeOnly)
		{
			EnsureManager();
		}
		StartBus();
	}
	else if (FParse::Param(FCommandLine::Get(), TEXT("URLabVrViewer")) || URLabLauncherFlags::CapsWantVr())
	{
		// A VR viewer with no bus is a gRPC-driven mirror that KEEPS rendering its
		// main view (the free-fly drone). Stand up the manager (bridge + RPC) so an
		// owner can drive it via fastpath_load/render over gRPC -- without the
		// forced-only regime above, which disables the main view the drone renders.
		EnsureManager();
	}
}

void AMjRenderer::EndPlay(const EEndPlayReason::Type Reason)
{
	Teardown();
	Super::EndPlay(Reason);
}

void AMjRenderer::BeginDestroy()
{
	// The editor can destroy/GC this actor without EndPlay; stop the worker
	// thread before its members are torn down to avoid a use-after-free.
	StopBus();
	// Editor-world preview actors never receive EndPlay (so Teardown never runs),
	// which is the only other place Model/Data are freed -- free them here so the
	// preview does not leak its mjModel/mjData. On a play-session actor Teardown
	// already ran and nulled them, so this is a no-op there; the PIE-duplicated
	// copy nulled its shallow-copied pointers in BeginPlay, so it is safe too.
	if (Data)
	{
		mj_deleteData(Data);
		Data = nullptr;
	}
	if (Model)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
	Super::BeginDestroy();
}

void AMjRenderer::Launch()
{
	LoadAndBuild();
	if (!BusEndpoint.IsEmpty() && !bForcedRenderOnly)
	{
		StartBus();
	}
}

int32 AMjRenderer::BuildStaticPreview()
{
	// Editor-world preview: build geometry at the rest pose and stop. No bus, no
	// tick -- the scene must never animate outside a play session. When the user
	// presses Play, the PIE duplicate of this actor connects the bus and streams.
	return LoadAndBuild();
}

namespace
{
// A cache key for the compiled model's baked fallback assets. The model carries no
// wire bytes to hash, so this keys on its pool sizes -- stable within a session,
// which is all the transient fallback cache needs.
FString CompiledModelContentHash(const mjModel_* M)
{
	const FString Key = FString::Printf(TEXT("compiled_%d_%d_%d_%d_%d"),
		(int)M->nbody, (int)M->ngeom, (int)M->nmesh, (int)M->nq, (int)M->nv);
	uint8 Digest[20];
	FSHA1::HashBuffer((const uint8*)TCHAR_TO_ANSI(*Key), Key.Len(), Digest);
	return BytesToHex(Digest, 20).Left(16);
}

// Whether geom G answers to a domain-randomization request for `Want`. A raw MJB
// carries unprefixed names, so it matches exactly. A compiled model prefixes every
// participant name, so a client's local name matches on the prefix-stripped tail --
// the same rule UMjAppearanceStore::ResolveMjId uses to find the id.
bool CompiledGeomNameMatches(const mjModel_* M, int32 G, const FString& Want, bool bAllowTail)
{
	const char* Nm = mj_id2name(M, mjOBJ_GEOM, G);
	if (!Nm || !*Nm)
	{
		return false;
	}
	const FString Full = ANSI_TO_TCHAR(Nm);
	return Full == Want || (bAllowTail && Full.EndsWith(TEXT("_") + Want));
}
} // namespace

int32 AMjRenderer::BuildFromCompiledModel(mjModel_* InModel, const TArray<AMjArticulation*>& Participants,
	const TArray<UMjQuickConvertComponent*>& QuickProps, bool bBuildCameras)
{
	Teardown();
	if (!InModel)
	{
		return -1;
	}
	Model = InModel;
	Data = nullptr;
	bOwnsModel = false;
	bExternallyDriven = true;
	bAllowInstancedStatics = false;

	// Map every compiled geom id back to the authoring element that bound to it, so
	// the resolver reads each geom's mesh/material off its own prefix-free spec and
	// the overlays/segmentation pools can find each geom's originating participant.
	GeomOrigins.Reset();
	for (AMjArticulation* Art : Participants)
	{
		if (!Art)
		{
			continue;
		}
		for (UMjGeom* Geom : Art->GetGeoms())
		{
			if (Geom && Geom->GetBoundId().IsSet())
			{
				GeomOrigins.Add(Geom->GetBoundId().GetValue(), Geom);
			}
		}
	}
	// Right-click-converted props are not articulations, but their visual geoms
	// are UMjGeom just the same: index them so a prop draws from its own
	// StaticMesh. The collision hulls index harmlessly -- a hidden group never
	// reaches the resolver.
	for (UMjQuickConvertComponent* Quick : QuickProps)
	{
		if (!Quick)
		{
			continue;
		}
		for (const TObjectPtr<UMjNodeComponent>& Node : Quick->GetGeomElements())
		{
			if (UMjGeom* Geom = Cast<UMjGeom>(Node))
			{
				if (Geom->GetBoundId().IsSet())
				{
					GeomOrigins.Add(Geom->GetBoundId().GetValue(), Geom);
				}
			}
		}
	}

	if (!AssetBaker)
	{
		AssetBaker = NewObject<UMjRendererAssetBaker>(this);
	}
	AssetBaker->Init(Model, CompiledModelContentHash(Model), false);

	GeomResolver = MakeUnique<FMjImportedAssetResolver>(Model, AssetBaker, GeomOrigins);

	BuildBodies();
	BuildGeoms();
	// Body-fixed model cameras (wrist/head) re-home onto this view; the manager places
	// them each frame via ApplyCameraPosesFromMat. Built dormant and named through the
	// camera registry so the RPC surface addresses them by the same canonical identity
	// the articulation path produced.
	if (bBuildCameras)
	{
		BuildCompiledViewCameras();
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] compiled play view: nbody=%d ngeom=%d (%d geom comps, %d authored origins)"),
		(int)Model->nbody, (int)Model->ngeom, NumBuiltGeoms(), GeomOrigins.Num());
	return static_cast<int32>(Model->ngeom);
}

UPrimitiveComponent* AMjRenderer::GetGeomComponent(int32 GeomId) const
{
	return GeomComps.IsValidIndex(GeomId) ? GeomComps[GeomId].Get() : nullptr;
}

UMjGeom* AMjRenderer::GetGeomOrigin(int32 GeomId) const
{
	if (const TWeakObjectPtr<UMjGeom>* Found = GeomOrigins.Find(GeomId))
	{
		return Found->Get();
	}
	return nullptr;
}

void AMjRenderer::SetGeomsVisible(bool bVisible)
{
	for (const TObjectPtr<UPrimitiveComponent>& Comp : GeomComps)
	{
		if (Comp)
		{
			Comp->SetVisibility(bVisible, /*bPropagateToChildren=*/true);
		}
	}
}

AAMjManager* AMjRenderer::ResolveManager() const
{
	if (AAMjManager* Mgr = AAMjManager::GetManager())
	{
		return Mgr;
	}
	// The singleton is only set at BeginPlay, which a test/editor world does not
	// dispatch; fall back to a level scan so the authoring walks still find it.
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AAMjManager> It(World); It; ++It)
		{
			if (AAMjManager* Mgr = *It)
			{
				return Mgr;
			}
		}
	}
	return nullptr;
}

void AMjRenderer::InitializeOverlayMaterial()
{
	Overlay.Initialize(*this);
}

void AMjRenderer::ClearMaterialOverlay()
{
	Overlay.Clear();
}

void AMjRenderer::ApplyMaterialOverlay(EMjDebugShaderMode Mode, const TArray<int32>& BodyAwake,
	const TArray<int32>& BodyIslandSeed, bool bModulateBySleep,
	float SleepValueScale, float SleepSaturationScale)
{
	Overlay.Apply(*this, Mode, BodyAwake, BodyIslandSeed, bModulateBySleep,
		SleepValueScale, SleepSaturationScale);
}

// ---------------------------------------------------------------------------
// Per-camera segmentation pool
// ---------------------------------------------------------------------------

TArray<TObjectPtr<UStaticMeshComponent>>* AMjRenderer::GetSegPoolArray(EMjCameraMode Mode)
{
	switch (Mode)
	{
		case EMjCameraMode::InstanceSegmentation:
			return &InstanceSegSiblings;
		case EMjCameraMode::SemanticSegmentation:
			return &SemanticSegSiblings;
		default:
			return nullptr;
	}
}

TSet<TWeakObjectPtr<UMjCamera>>* AMjRenderer::GetSegSubscribers(EMjCameraMode Mode)
{
	switch (Mode)
	{
		case EMjCameraMode::InstanceSegmentation:
			return &InstanceSegSubscribers;
		case EMjCameraMode::SemanticSegmentation:
			return &SemanticSegSubscribers;
		default:
			return nullptr;
	}
}

UStaticMeshComponent* AMjRenderer::SpawnSegSibling(
	UStaticMeshComponent* Original, int32 BodyId, uint32 GroupHash, EMjCameraMode Mode)
{
	if (!Original || !Original->GetStaticMesh())
		return nullptr;
	if (!OverlayParentMaterial || OverlayColorParam.IsNone())
		return nullptr;

	AActor* GeomOwner = Original->GetOwner();
	if (!GeomOwner)
		return nullptr;

	UStaticMeshComponent* Sibling = NewObject<UStaticMeshComponent>(GeomOwner);
	Sibling->SetStaticMesh(Original->GetStaticMesh());

	// Attach to the same parent as the original so it inherits body transforms
	// for free — no per-tick sync needed.
	if (USceneComponent* Parent = Original->GetAttachParent())
	{
		Sibling->SetupAttachment(Parent);
	}
	Sibling->SetRelativeTransform(Original->GetRelativeTransform());

	// Isolation: siblings must not contribute indirect lighting, shadows, or
	// reflections to other views. bVisibleInSceneCaptureOnly hides the primitive
	// from the main viewport; the rest prevents secondary lighting/reflection
	// passes from picking it up (source of the "faint tinge" in viewport
	// otherwise). Leave bRenderInMainPass at default true — the seg capture's
	// own rendering uses the main pass.
	Sibling->bVisibleInSceneCaptureOnly = true;
	Sibling->SetCastShadow(false);
	Sibling->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Sibling->SetGenerateOverlapEvents(false);
	Sibling->bAffectDynamicIndirectLighting = false;
	Sibling->bAffectDistanceFieldLighting = false;
	Sibling->bVisibleInReflectionCaptures = false;
	Sibling->bVisibleInRealTimeSkyCaptures = false;
	Sibling->bVisibleInRayTracing = false;
	Sibling->bReceivesDecals = false;

	// Unlit tint material — parented on the same material the viewport overlay uses.
	// Seg cameras set CaptureSource = SCS_BaseColor, which bypasses lighting so the
	// tint value lands in the RT unmodified.
	UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(OverlayParentMaterial, Sibling);
	const FLinearColor Tint = (Mode == EMjCameraMode::SemanticSegmentation)
								? MjColor::SemanticSegmentationColor(GroupHash, /*bAwake=*/true, /*SleepValueScale=*/1.0f, /*SleepSatScale=*/1.0f)
								: MjColor::InstanceSegmentationColor(GroupHash, BodyId, /*bAwake=*/true, /*SleepValueScale=*/1.0f, /*SleepSatScale=*/1.0f);
	MID->SetVectorParameterValue(OverlayColorParam, Tint);

	const int32 NumSlots = FMath::Max(1, Original->GetNumMaterials());
	for (int32 Slot = 0; Slot < NumSlots; ++Slot)
	{
		Sibling->SetMaterial(Slot, MID);
	}

	Sibling->ComponentTags.Add(Mode == EMjCameraMode::InstanceSegmentation
								   ? FName(TEXT("URLab_Seg_Instance"))
								   : FName(TEXT("URLab_Seg_Semantic")));

	Sibling->RegisterComponent();
	return Sibling;
}

void AMjRenderer::BuildSegPool(EMjCameraMode Mode)
{
	TArray<TObjectPtr<UStaticMeshComponent>>* Pool = GetSegPoolArray(Mode);
	if (!Pool)
		return;

	Pool->Reset();

	auto AddSibling = [&](UStaticMeshComponent* Original, int32 BodyId, uint32 GroupHash) {
		if (UStaticMeshComponent* Sib = SpawnSegSibling(Original, BodyId, GroupHash, Mode))
		{
			Pool->Add(Sib);
		}
	};

	// This renderer's own geom components are the segmentation subjects; the seg cameras
	// render siblings of those, keyed by geom_bodyid and the originating participant.
	if (Model)
	{
		const int32 NGeom = NumGeoms();
		for (int32 G = 0; G < NGeom; ++G)
		{
			UStaticMeshComponent* Mesh = Cast<UStaticMeshComponent>(GetGeomComponent(G));
			if (!Mesh)
				continue;
			const int32 BodyId = Model->geom_bodyid[G];
			if (BodyId < 0)
				continue;

			uint32 ArtHash = GetTypeHash(GetClass()->GetFName());
			if (UMjGeom* Origin = GetGeomOrigin(G))
			{
				if (AActor* GeomOwner = Origin->GetOwner())
					ArtHash = GetTypeHash(GeomOwner->GetClass()->GetFName());
			}

			AddSibling(Mesh, BodyId, ArtHash);

			TArray<USceneComponent*> ChildComps;
			Mesh->GetChildrenComponents(true, ChildComps);
			for (USceneComponent* Child : ChildComps)
			{
				if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
					AddSibling(SMC, BodyId, ArtHash);
			}
		}
	}

	AAMjManager* Manager = ResolveManager();
	if (!Manager)
	{
		UE_LOG(LogURLab, Log,
			TEXT("[MjRenderer] Built seg pool mode=%s size=%d"),
			*UEnum::GetValueAsString(Mode), Pool->Num());
		return;
	}

#if WITH_EDITOR
	const mjModel* AuthoringModel = Manager->PhysicsEngine ? Manager->PhysicsEngine->m_model : nullptr;

	// Authoring visual meshes (the editor preview and the test harness), plus any static
	// mesh a caller hung under a geom themselves.
	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;
		const uint32 ArtHash = GetTypeHash(Art->GetClass()->GetFName());

		for (UMjGeom* Geom : Art->GetGeoms())
		{
			const int32 BodyId = AuthoringGeomBodyId(AuthoringModel, Geom);
			if (BodyId < 0)
				continue;

			AddSibling(Geom->GetVisualizerMesh(), BodyId, ArtHash);

			TArray<USceneComponent*> ChildComps;
			Geom->GetChildrenComponents(true, ChildComps);
			for (USceneComponent* Child : ChildComps)
			{
				if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
				{
					AddSibling(SMC, BodyId, ArtHash);
				}
			}
		}
	}
#endif

	// Quick-Convert primitives — group hash keyed off the first static mesh.
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
			AddSibling(SMC, BodyId, GroupHash);
		}
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] Built seg pool mode=%s size=%d"),
		*UEnum::GetValueAsString(Mode), Pool->Num());
}

void AMjRenderer::DestroySegPool(EMjCameraMode Mode)
{
	TArray<TObjectPtr<UStaticMeshComponent>>* Pool = GetSegPoolArray(Mode);
	if (!Pool)
		return;

	for (const TObjectPtr<UStaticMeshComponent>& Sib : *Pool)
	{
		if (Sib)
			Sib->DestroyComponent();
	}
	Pool->Reset();
}

void AMjRenderer::AcquireSegPool(EMjCameraMode Mode, UMjCamera* Camera,
	TArray<UPrimitiveComponent*>& OutSiblings)
{
	OutSiblings.Reset();

	TArray<TObjectPtr<UStaticMeshComponent>>* Pool = GetSegPoolArray(Mode);
	TSet<TWeakObjectPtr<UMjCamera>>* Subs = GetSegSubscribers(Mode);
	if (!Pool || !Subs)
		return;

	const bool bFirstSubscriber = Subs->Num() == 0;
	Subs->Add(Camera);

	if (bFirstSubscriber)
	{
		BuildSegPool(Mode);
	}

	OutSiblings.Reserve(Pool->Num());
	for (const TObjectPtr<UStaticMeshComponent>& Sib : *Pool)
	{
		if (Sib)
			OutSiblings.Add(Sib);
	}
}

void AMjRenderer::ReleaseSegPool(EMjCameraMode Mode, UMjCamera* Camera)
{
	TSet<TWeakObjectPtr<UMjCamera>>* Subs = GetSegSubscribers(Mode);
	if (!Subs)
		return;

	Subs->Remove(Camera);
	// Also drop any stale weak pointers so refcount reflects reality.
	for (auto It = Subs->CreateIterator(); It; ++It)
	{
		if (!It->IsValid())
			It.RemoveCurrent();
	}

	if (Subs->Num() == 0)
	{
		DestroySegPool(Mode);
	}
}

void AMjRenderer::GetSegPoolSiblings(EMjCameraMode Mode,
	TArray<UPrimitiveComponent*>& OutSiblings) const
{
	OutSiblings.Reset();
	const TArray<TObjectPtr<UStaticMeshComponent>>* Pool = nullptr;
	switch (Mode)
	{
		case EMjCameraMode::InstanceSegmentation:
			Pool = &InstanceSegSiblings;
			break;
		case EMjCameraMode::SemanticSegmentation:
			Pool = &SemanticSegSiblings;
			break;
		default:
			return;
	}

	OutSiblings.Reserve(Pool->Num());
	for (const TObjectPtr<UStaticMeshComponent>& Sib : *Pool)
	{
		if (Sib)
			OutSiblings.Add(Sib);
	}
}

int32 AMjRenderer::ReindexFromLevel()
{
	// Need the model for sizing (nbody/ngeom) and the index space the stream uses.
	if (!LoadModelOnly())
	{
		return -1;
	}
	const int32 NBody = static_cast<int32>(Model->nbody);
	const int32 NGeom = static_cast<int32>(Model->ngeom);

	BodyActors.Reset();
	BodyActors.SetNum(NBody);
	GeomComps.Reset();
	GeomComps.SetNum(NGeom);
	CameraComps.Reset();
	CameraComps.SetNum(static_cast<int32>(Model->ncam));

	// Recursive: the tagged geom components live on per-geom child actors, which
	// are grandchildren of this scene (scene -> body actor -> geom actor).
	TArray<AActor*> AttachedActors;
	GetAttachedActors(AttachedActors, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	int32 Found = 0;
	for (AActor* A : AttachedActors)
	{
		if (!A)
		{
			continue;
		}
		for (const FName& Tag : A->Tags)
		{
			const int32 BodyId = MjbParseIdTag(Tag.ToString(), kTagBody);
			if (BodyActors.IsValidIndex(BodyId))
			{
				BodyActors[BodyId] = A;
			}
		}
		TArray<UPrimitiveComponent*> Comps;
		A->GetComponents<UPrimitiveComponent>(Comps);
		for (UPrimitiveComponent* C : Comps)
		{
			for (const FName& Tag : C->ComponentTags)
			{
				const int32 GeomId = MjbParseIdTag(Tag.ToString(), kTagGeom);
				if (GeomComps.IsValidIndex(GeomId))
				{
					GeomComps[GeomId] = C;
					++Found;
				}
			}
		}
		// Cameras are UMjCamera scene components (not primitives); re-wire them too.
		TArray<UMjCamera*> Cams;
		A->GetComponents<UMjCamera>(Cams);
		for (UMjCamera* Cam : Cams)
		{
			for (const FName& Tag : Cam->ComponentTags)
			{
				const int32 CamId = MjbParseIdTag(Tag.ToString(), kTagCam);
				if (CameraComps.IsValidIndex(CamId))
				{
					CameraComps[CamId] = Cam;
				}
			}
		}
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] re-indexed %d geoms across %d attached actors from the level"),
		Found, AttachedActors.Num());
	return Found > 0 ? Found : -1;
}

bool AMjRenderer::LoadModelOnly()
{
	if (Model)
	{
		return true; // already loaded
	}
	// Prefer an in-memory MJB (received over the wire) over a file path, so a
	// renderer never needs a shared file. Fall back to reading the file into a
	// buffer; either way we load from the buffer with mj_loadModelBuffer.
	TArray<uint8> LocalBytes;
	const TArray<uint8>* Bytes = nullptr;
	if (MjbBytes.Num() > 0)
	{
		Bytes = &MjbBytes;
	}
	else if (!MjbFilePath.IsEmpty() && FFileHelper::LoadFileToArray(LocalBytes, *MjbFilePath))
	{
		Bytes = &LocalBytes;
	}
	if (!Bytes || Bytes->Num() == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] no MJB to load (bytes empty, file '%s' unreadable)"),
			*MjbFilePath);
		return false;
	}

	Model = mj_loadModelBuffer(Bytes->GetData(), Bytes->Num());
	if (!Model)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] mj_loadModelBuffer failed (%d bytes; version-mismatched MJB?)"),
			Bytes->Num());
		return false;
	}
	Data = mj_makeData(Model);
	if (!Data)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] mj_makeData failed"));
		Teardown();
		return false;
	}
	// One-shot forward for the rest pose (not stepping; the stream overrides it).
	mj_forward(Model, Data);

	// Content id for the cached, persistent asset folder: a hash of the MJB bytes.
	// The compiled model IS the content, so an identical model hits the same cache.
	FString ContentHash;
	{
		uint8 Digest[20];
		FSHA1::HashBuffer(Bytes->GetData(), Bytes->Num(), Digest);
		ContentHash = BytesToHex(Digest, 20).Left(16);
	}

	// Prime the asset baker for this model (loads the shared master material).
	if (!AssetBaker)
	{
		AssetBaker = NewObject<UMjRendererAssetBaker>(this);
	}
	AssetBaker->Init(Model, ContentHash, bForceRebuildAssets);
	return true;
}

int32 AMjRenderer::LoadAndBuild()
{
	Teardown();
	if (!LoadModelOnly())
	{
		return -1;
	}

	BuildBodies();
	BuildGeoms();
	BuildCameras();
	BuildUserCaptureCamera();  // the returnable free/user camera (independent of ncam)
	ApplyFromData();

	// Import the model's environment (its <light> elements, headlight fill, and skybox)
	// so camera renders match MuJoCo's lighting instead of a hardcoded rig / black void.
	if (UWorld* W = GetWorld())
	{
		MjSkyImporter::ApplyMjEnvironment(*W, Model, Data, bBaseLevel, SceneOrigin);
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] built from %s: nbody=%d ngeom=%d (%d geom comps built, %d body actors)"),
		*MjbFilePath, (int)Model->nbody, (int)Model->ngeom, NumBuiltGeoms(), BodyActors.Num());
	return Model->ngeom;
}

int32 AMjRenderer::NumBuiltGeoms() const
{
	int32 N = 0;
	for (const TObjectPtr<UPrimitiveComponent>& C : GeomComps)
	{
		if (C)
		{
			++N;
		}
	}
	return N;
}

USceneComponent* AMjRenderer::GetBodyRootComponent(int32 BodyId) const
{
	if (!BodyActors.IsValidIndex(BodyId) || !BodyActors[BodyId])
	{
		return nullptr;
	}
	return BodyActors[BodyId]->GetRootComponent();
}

void AMjRenderer::BuildBodies()
{
	const int32 NBody = static_cast<int32>(Model->nbody);
	BodyActors.SetNum(NBody);
	// In the editor world the body actors are persistent (saveable, re-indexable
	// -- see ReindexFromLevel); in a play world they are transient scratch actors.
	const bool bEditorPreview = GetWorld() && !GetWorld()->IsGameWorld();
	for (int32 B = 0; B < NBody; ++B)
	{
		// One lightweight actor per body so the renderer culls per body. World
		// body (0) holds static geoms (floor); include it.
		FActorSpawnParameters Params;
		Params.Owner = this;
		if (!bEditorPreview)
		{
			Params.ObjectFlags |= RF_Transient;
		}
		AActor* Body = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!Body)
		{
			continue;
		}
		USceneComponent* Root = NewObject<USceneComponent>(Body, *FString::Printf(TEXT("MjbBody_%d"), B));
		Body->SetRootComponent(Root);
		Root->RegisterComponent();
		Body->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
		// Stable re-index key: the MuJoCo body id, plus the body name for a
		// readable outliner label. Lets a saved scene rebuild its body->actor map.
		Body->Tags.Add(MjbIdTag(kTagBody, B));
		const char* Name = mj_id2name(Model, mjOBJ_BODY, B);
		if (bEditorPreview && Name && *Name)
		{
			Body->Tags.Add(FName(*FString::Printf(TEXT("MjbBodyName=%s"), ANSI_TO_TCHAR(Name))));
#if WITH_EDITOR
			Body->SetActorLabel(FString::Printf(TEXT("Mjb_%s"), ANSI_TO_TCHAR(Name)));
#endif
		}
		BodyActors[B] = Body;
	}
}

void AMjRenderer::BuildGeoms()
{
	const int32 NGeom = static_cast<int32>(Model->ngeom);
	GeomComps.SetNum(NGeom);

	// Repeated static world-body geoms become instanced components; the rest are
	// built individually below. GeomComps stays null for instanced geoms -- they are
	// static, so the transform stream simply never touches them.
	TSet<int32> Instanced;
#if WITH_EDITOR
	if (bAllowInstancedStatics)
	{
		BuildInstancedStatics(Instanced);
	}
#endif

	for (int32 G = 0; G < NGeom; ++G)
	{
		if (Instanced.Contains(G))
		{
			continue;
		}
		GeomComps[G] = BuildGeom(G);
		if (GeomComps[G])
		{
			// Stable re-index key for the geom -> component map.
			GeomComps[G]->ComponentTags.Add(MjbIdTag(kTagGeom, G));
		}
	}
}

#if WITH_EDITOR
void AMjRenderer::BuildInstancedStatics(TSet<int32>& OutHandled)
{
	if (!Model || !Data)
	{
		return;
	}
	// Only the static world body (id 0): its geoms never move, so instance transforms
	// are set once here from the rest pose.
	const int32 WorldBody = 0;
	if (!BodyActors.IsValidIndex(WorldBody) || !BodyActors[WorldBody])
	{
		return;
	}
	AActor* Host = BodyActors[WorldBody].Get();

	// Group visible world-body mesh geoms by (mesh id, material id) -- a group only
	// shares one instanced component if it shares both the mesh and the material.
	TMap<TPair<int32, int32>, TArray<int32>> Groups;
	for (int32 G = 0; G < static_cast<int32>(Model->ngeom); ++G)
	{
		if (Model->geom_bodyid[G] != WorldBody || Model->geom_type[G] != mjGEOM_MESH)
		{
			continue;
		}
		if (!IsGeomVisible(G))
		{
			continue;
		}
		const int32 MatId = Model->geom_matid[G];
		if (GeomRgba(G)[3] <= 0.0f)
		{
			continue;
		}
		const int32 MeshId = Model->geom_dataid[G];
		if (MeshId < 0)
		{
			continue;
		}
		Groups.FindOrAdd(TPair<int32, int32>(MeshId, MatId)).Add(G);
	}

	int32 NumGroups = 0;
	for (const TPair<TPair<int32, int32>, TArray<int32>>& KV : Groups)
	{
		const TArray<int32>& GeomIds = KV.Value;
		if (GeomIds.Num() < 2)
		{
			continue; // instancing only pays for a repeated mesh
		}
		UStaticMesh* Mesh = AssetBaker->GetOrBuildStaticMesh(KV.Key.Key);
		if (!Mesh)
		{
			continue;
		}
		UInstancedStaticMeshComponent* Ism = NewObject<UInstancedStaticMeshComponent>(Host);
		Ism->SetStaticMesh(Mesh);
		Ism->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DisableDistanceFields(Ism);
		Ism->RegisterComponent();
		Ism->AttachToComponent(Host->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		AssetBaker->ApplyGeomMaterial(Ism, GeomIds[0]); // the group shares one material
		// Rep-geom in the tag so a reused (PIE-duplicated) scene can re-apply the MID.
		Ism->ComponentTags.Add(MjbIdTag(kTagIsm, GeomIds[0]));
		for (int32 G : GeomIds)
		{
			const FVector Loc = URLabAxisConv::MjPositionToUe(Data->geom_xpos + 3 * G) + SceneOrigin;
			const FQuat Rot = MjMat3ToUeQuat(Data->geom_xmat + 9 * G);
			Ism->AddInstance(FTransform(Rot, Loc), /*bWorldSpace=*/true);
			OutHandled.Add(G);
		}
		++NumGroups;
	}
	if (NumGroups > 0)
	{
		UE_LOG(LogURLab, Log, TEXT("[MjRenderer] instanced %d static geoms into %d ISM group(s)"),
			OutHandled.Num(), NumGroups);
	}
}
#endif // WITH_EDITOR

void AMjRenderer::BuildCameras()
{
	if (!bEnableCameraStreaming || !Model || Model->ncam == 0)
	{
		return;
	}
	// Builds the camera components in BOTH the editor preview and a play session, so
	// the scene has one representation. The components are built DORMANT here (no
	// render target, no ZMQ bind); StartCameraStreaming() turns capture + streaming
	// on for a play session only, which is what keeps the editor preview cheap and
	// avoids the PIE duplicate double-binding the ports.
	const int32 NCam = static_cast<int32>(Model->ncam);
	CameraComps.SetNum(NCam);
	for (int32 C = 0; C < NCam; ++C)
	{
		// Host the camera under its MuJoCo body actor when it has one, else the
		// scene actor. Its world pose is driven explicitly each frame regardless.
		const int32 BodyId = Model->cam_bodyid[C];
		AActor* Host = (BodyActors.IsValidIndex(BodyId) && BodyActors[BodyId]) ? BodyActors[BodyId].Get() : this;

		UMjCamera* Cam = NewObject<UMjCamera>(Host);
		if (!Cam)
		{
			continue;
		}
		Cam->CaptureMode = EMjCameraMode::Real;
		// Resolution: MuJoCo leaves an unspecified camera at 1x1, so treat <=1 as
		// "use a sane default". Then optionally cap the height (each camera is a
		// full scene capture) keeping aspect.
		int32 W = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C]) : 0;
		int32 H = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C + 1]) : 0;
		if (W <= 1 || H <= 1)
		{
			W = 640;
			H = 480;
		}
		if (CameraMaxHeight > 0 && H > CameraMaxHeight)
		{
			W = FMath::Max(1, FMath::RoundToInt(W * (static_cast<double>(CameraMaxHeight) / H)));
			H = CameraMaxHeight;
		}
		TArray<int32> Res;
		Res.Add(W);
		Res.Add(H);
		Cam->SetResolution(Res);
		const float ZNearCm = (Model->vis.map.znear > 0.0f && Model->stat.extent > 0.0f)
			? (Model->vis.map.znear * Model->stat.extent * 100.0f)
			: 2.0f;
		const float ZFarCm = (Model->vis.map.zfar > 0.0f && Model->stat.extent > 0.0f)
			? (Model->vis.map.zfar * Model->stat.extent * 100.0f)
			: 10000.0f;
		Cam->SetClippingPlanes(ZNearCm, ZFarCm);

		if (Model->cam_sensorsize && Model->cam_sensorsize[2 * C + 1] > 0.0f && Model->cam_intrinsic)
		{
			const float SensorW = Model->cam_sensorsize[2 * C];
			const float SensorH = Model->cam_sensorsize[2 * C + 1];
			const float* Intrinsic = Model->cam_intrinsic + 4 * C;

			TArray<float> SensorSizeArr;
			SensorSizeArr.Add(SensorW);
			SensorSizeArr.Add(SensorH);
			Cam->SetSensorsize(SensorSizeArr);

			const float FxPx = Intrinsic[0] * (static_cast<float>(W) / SensorW);
			const float FyPx = Intrinsic[1] * (static_cast<float>(H) / SensorH);
			const float CxOffsetPx = Intrinsic[2] * (static_cast<float>(W) / SensorW);
			const float CyOffsetPx = Intrinsic[3] * (static_cast<float>(H) / SensorH);

			TArray<float> FocalPxArr;
			FocalPxArr.Add(FxPx);
			FocalPxArr.Add(FyPx);
			Cam->SetFocalpixel(FocalPxArr);

			TArray<float> PrincipalPxArr;
			PrincipalPxArr.Add(CxOffsetPx);
			PrincipalPxArr.Add(CyOffsetPx);
			Cam->SetPrincipalpixel(PrincipalPxArr);
		}
		else if (Model->cam_fovy[C] > 0.0)
		{
			Cam->SetFovy(Model->cam_fovy[C]);
		}
		Cam->SetupProjectionMatrix();

		const char* CamName = (Model && Model->names) ? mj_id2name(Model, mjOBJ_CAMERA, C) : nullptr;
		if (CamName && *CamName)
		{
			Cam->MjName = FString(UTF8_TO_TCHAR(CamName));
			Cam->SetCanonicalIdentity(FName(CamName));
		}

		// Transport: one ZMQ PUB per camera (port base + id); no manager here, so
		// the authored endpoint is used directly. Optional SHM ring alongside.
		Cam->bEnableZmqBroadcast = true;
		Cam->ZmqEndpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), CameraStreamBasePort + C);
		Cam->bEnableShmBroadcast = bEnableCameraShm;
		Cam->SetStreamPortIndex(C);

		Cam->SetupAttachment(Host->GetRootComponent());
		Cam->ComponentTags.Add(MjbIdTag(kTagCam, C));
		Cam->RegisterComponent();
		CameraComps[C] = Cam;
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] built %d camera component(s) (dormant)"), NCam);
}

const TCHAR* AMjRenderer::UserCameraName()
{
	return TEXT("user");
}

void AMjRenderer::BuildUserCaptureCamera()
{
	// A single capturing free/user camera, independent of the model's cameras.
	// Hosted on the scene actor (its world pose is driven explicitly each render),
	// and kept OUT of CameraComps so it never participates in the per-model cxpos
	// indexing. Built dormant -- StartCameraStreaming turns capture + streaming on,
	// exactly like the model cameras. Exists even when the model has no cameras of
	// its own, so an operator's viewpoint is always renderable.
	if (!bEnableCameraStreaming || !Model)
	{
		return;
	}
	UMjCamera* Cam = NewObject<UMjCamera>(this);
	if (!Cam)
	{
		return;
	}
	Cam->CaptureMode = EMjCameraMode::Real;

	const int32 W = UserCamWidth > 1 ? UserCamWidth : 1280;
	const int32 H = UserCamHeight > 1 ? UserCamHeight : 720;
	TArray<int32> Res;
	Res.Add(W);
	Res.Add(H);
	Cam->SetResolution(Res);
	Cam->SetFovy(45.0);  // MuJoCo's free camera has no model fovy; a sane default

	Cam->MjName = FString(UserCameraName());
	// Pin the canonical identity so the wire topic and the fastpath_render reply
	// name are exactly "user", regardless of the (transient) host actor name.
	Cam->SetCanonicalIdentity(FName(UserCameraName()));

	// Stream on a port PAST the model cameras so nothing collides with them.
	const int32 PortIndex = static_cast<int32>(Model->ncam);
	Cam->bEnableZmqBroadcast = true;
	Cam->ZmqEndpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), CameraStreamBasePort + PortIndex);
	Cam->bEnableShmBroadcast = bEnableCameraShm;
	Cam->SetStreamPortIndex(PortIndex);

	Cam->SetupAttachment(GetRootComponent());
	Cam->ComponentTags.Add(MjbIdTag(kTagCam, PortIndex));
	Cam->RegisterComponent();

	UserCaptureCam = Cam;
	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] built capturing user camera '%s' %dx%d (port %d, dormant)"),
		UserCameraName(), W, H, CameraStreamBasePort + PortIndex);
}

void AMjRenderer::PoseUserCaptureCam(const double* Pos, const double* Fwd, const double* Up)
{
	UMjCamera* Cam = UserCaptureCam.Get();
	if (!Cam || !Pos || !Fwd || !Up)
	{
		return;
	}
	// Build the MuJoCo camera frame (looks along -z_cam, +y_cam up) from the view
	// (fwd, up) as a cam_xmat, then reuse the SAME conversion the model cameras use
	// (MjMat3ToUeQuat). Going through the MuJoCo frame is what makes this correct:
	// the right-handed -> left-handed change is a handedness flip, not a per-axis Y
	// negation, so composing MjDirectionToUe on fwd/up and MakeFromXZ mirrors the
	// orientation. cam_xmat columns are the camera's local axes in world coords:
	// x = right, y = up, z = -forward.
	double f[3] = {Fwd[0], Fwd[1], Fwd[2]};
	double u[3] = {Up[0], Up[1], Up[2]};
	if (mju_normalize3(f) < 1e-9)
	{
		return; // degenerate view direction -- keep the last good pose
	}
	mju_normalize3(u);
	double z[3] = {-f[0], -f[1], -f[2]};
	double x[3];
	mju_cross(x, u, z);
	if (mju_normalize3(x) < 1e-9)
	{
		return; // up parallel to view -- keep the last good pose
	}
	double y[3];
	mju_cross(y, z, x);
	// Row-major 3x3, columns [x y z] (MuJoCo xmat layout).
	const double Mat[9] = {
		x[0], y[0], z[0],
		x[1], y[1], z[1],
		x[2], y[2], z[2],
	};
	const FVector LocUe = URLabAxisConv::MjPositionToUe(Pos) + SceneOrigin;
	const FQuat Rot = MjMat3ToUeQuat(Mat);
	Cam->SetWorldLocationAndRotation(LocUe, Rot);
}

void AMjRenderer::BuildCompiledViewCameras()
{
	if (!Model || Model->ncam == 0)
	{
		return;
	}

	// Canonical identity per camera, computed off the compiled model + the engine's entity
	// partition. The registry yields the same "<art>/<part>" string ResolveCameraCanonical
	// produced for the articulation, so a re-homed camera keeps the exact topic/stem the RPC
	// surface keys on -- the art segment derives from the entity's ActorId (via PublicName),
	// not the compiled-prefix stem.
	FMjCameraRegistry Registry;
	UMjNetworkManager* NetworkManager = nullptr;
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			Registry.Build(Model, Manager->PhysicsEngine->GetEntityPartition());
		}
		NetworkManager = Manager->NetworkManager;
	}

	const int32 NCam = static_cast<int32>(Model->ncam);
	CameraComps.SetNum(NCam);
	for (int32 C = 0; C < NCam; ++C)
	{
		// Host the camera under its MuJoCo body actor when it has one, else the scene
		// actor. Its world pose is driven explicitly each render tick regardless.
		const int32 BodyId = Model->cam_bodyid[C];
		AActor* Host = (BodyActors.IsValidIndex(BodyId) && BodyActors[BodyId]) ? BodyActors[BodyId].Get() : this;

		UMjCamera* Cam = NewObject<UMjCamera>(Host);
		if (!Cam)
		{
			continue;
		}
		Cam->CaptureMode = EMjCameraMode::Real;

		// Resolution: MuJoCo leaves an unspecified camera at 1x1, so treat <=1 as a sane
		// default. Then optionally cap the height (each camera is a full scene capture),
		// keeping aspect.
		int32 W = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C]) : 0;
		int32 H = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C + 1]) : 0;
		if (W <= 1 || H <= 1)
		{
			W = 640;
			H = 480;
		}
		if (CameraMaxHeight > 0 && H > CameraMaxHeight)
		{
			W = FMath::Max(1, FMath::RoundToInt(W * (static_cast<double>(CameraMaxHeight) / H)));
			H = CameraMaxHeight;
		}
		TArray<int32> Res;
		Res.Add(W);
		Res.Add(H);
		Cam->SetResolution(Res);

		const float ZNearCm = (Model->vis.map.znear > 0.0f && Model->stat.extent > 0.0f)
			? (Model->vis.map.znear * Model->stat.extent * 100.0f)
			: 2.0f;
		const float ZFarCm = (Model->vis.map.zfar > 0.0f && Model->stat.extent > 0.0f)
			? (Model->vis.map.zfar * Model->stat.extent * 100.0f)
			: 10000.0f;
		Cam->SetClippingPlanes(ZNearCm, ZFarCm);

		if (Model->cam_sensorsize && Model->cam_sensorsize[2 * C + 1] > 0.0f && Model->cam_intrinsic)
		{
			const float SensorW = Model->cam_sensorsize[2 * C];
			const float SensorH = Model->cam_sensorsize[2 * C + 1];
			const float* Intrinsic = Model->cam_intrinsic + 4 * C;

			TArray<float> SensorSizeArr;
			SensorSizeArr.Add(SensorW);
			SensorSizeArr.Add(SensorH);
			Cam->SetSensorsize(SensorSizeArr);

			const float FxPx = Intrinsic[0] * (static_cast<float>(W) / SensorW);
			const float FyPx = Intrinsic[1] * (static_cast<float>(H) / SensorH);
			const float CxOffsetPx = Intrinsic[2] * (static_cast<float>(W) / SensorW);
			const float CyOffsetPx = Intrinsic[3] * (static_cast<float>(H) / SensorH);

			TArray<float> FocalPxArr;
			FocalPxArr.Add(FxPx);
			FocalPxArr.Add(FyPx);
			Cam->SetFocalpixel(FocalPxArr);

			TArray<float> PrincipalPxArr;
			PrincipalPxArr.Add(CxOffsetPx);
			PrincipalPxArr.Add(CyOffsetPx);
			Cam->SetPrincipalpixel(PrincipalPxArr);
		}
		else if (Model->cam_fovy[C] > 0.0)
		{
			Cam->SetFovy(Model->cam_fovy[C]);
		}
		Cam->SetupProjectionMatrix();

		// Name for logs, then pin the canonical identity from the registry so the wire
		// topic is stable regardless of the (transient, possibly demoted) host actor name.
		const char* CamName = mj_id2name(Model, mjOBJ_CAMERA, C);
		if (CamName && *CamName)
		{
			Cam->MjName = FString(UTF8_TO_TCHAR(CamName));
		}
		if (Registry.Cameras.IsValidIndex(C))
		{
			Cam->SetCanonicalIdentity(Registry.Cameras[C].CanonicalName);
		}

		// Dormant: no ZMQ bind, no port, no capture. The RPC path (set_camera_streaming /
		// include_cameras) turns capture + streaming on per camera via SetStreamingEnabled.
		Cam->SetupAttachment(Host->GetRootComponent());
		Cam->ComponentTags.Add(MjbIdTag(kTagCam, C));
		Cam->RegisterComponent();

		// A component added after the world's BeginPlay may never receive its own, so the
		// registrations UMjCamera::BeginPlay would have done are made explicitly here.
		if (UWorld* World = GetWorld())
		{
			if (UMjCameraSubsystem* Subsystem = World->GetSubsystem<UMjCameraSubsystem>())
			{
				Subsystem->RegisterCamera(Cam);
			}
		}
		if (NetworkManager)
		{
			NetworkManager->RegisterCamera(Cam);
		}

		CameraComps[C] = Cam;
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] compiled view: re-homed %d body-fixed camera(s) (dormant)"), NCam);
}

void AMjRenderer::StartCameraStreaming()
{
	// Turn dormant cameras into a live render server: set up the render target, bind
	// the per-camera ZMQ port, and capture every frame. Network config is re-applied
	// here so it is correct whether the cameras were just built or reused from the
	// duplicated preview. Play session only.
	for (int32 C = 0; C < CameraComps.Num(); ++C)
	{
		UMjCamera* Cam = CameraComps[C];
		if (!Cam)
		{
			continue;
		}
		// This manager-less render server is the camera's applied-state source, so
		// capture-stamping and delay reveal read the owner-broadcast frame_id/sim_time
		// (IMjSimClock) instead of the absent AAMjManager singleton.
		Cam->SetSimClock(this);
		// Forced-render-only mode (-URLabFastForcedOnly): the camera is set up (render
		// target + REP-servable) but does NOT auto-capture; only the forced-render
		// request drives it, so the async stream never competes with the request
		// render for the single render thread.
		Cam->bManualCaptureOnly = bForcedRenderOnly;
		Cam->bEnableZmqBroadcast = true;
		Cam->ZmqEndpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), CameraStreamBasePort + C);
		Cam->bEnableShmBroadcast = bEnableCameraShm;
		Cam->SetStreamPortIndex(C);
		if (!bForcedRenderOnly)
		{
			// Render every frame: with no AAMjManager the state-change capture gate
			// never advances, so it would render once then stall without this.
			Cam->SetCaptureRate(/*bOnStateChange=*/false, /*MaxFps=*/30.0f);
		}
		Cam->SetStreamingEnabled(true);
	}
	// The capturing user camera streams on the same regime as the model cameras:
	// manual-capture-only under -URLabFastForcedOnly (driven solely by the forced
	// request, so it never contends with the request render), else free-running for
	// the viewer/stream path.
	if (UMjCamera* UCam = UserCaptureCam.Get())
	{
		UCam->SetSimClock(this);
		UCam->bManualCaptureOnly = bForcedRenderOnly;
		if (!bForcedRenderOnly)
		{
			UCam->SetCaptureRate(/*bOnStateChange=*/false, /*MaxFps=*/30.0f);
		}
		UCam->SetStreamingEnabled(true);
	}
	if (CameraComps.Num() > 0)
	{
		UE_LOG(LogURLab, Log, TEXT("[MjRenderer] camera server: %d camera(s) streaming from port %d"),
			CameraComps.Num(), CameraStreamBasePort);
		// Forced-render eval regime: the bridge dispatcher's fastpath_render op drives
		// exact-state, blocking, fresh captures over the shared transport. This regime
		// is the sole pose driver; the mirror regime drives from the bus instead, so
		// the two never write the pose together.
		if (bForcedRenderOnly)
		{
			// This eval render server only produces the model's SceneCapture frames,
			// never a player view. Skip the primary-view render each tick so it does
			// not contend with the captures on the GPU queue (paired with
			// -RenderOffScreen, which drops the swapchain present). The SceneCaptures
			// are independent renders and keep working. Cuts each tick's cost, which
			// is what bounds the tick-driven forced-render latency.
			if (UWorld* W = GetWorld())
			{
				if (UGameViewportClient* VP = W->GetGameViewport())
				{
					VP->bDisableWorldRendering = true;
				}
			}
			// The tick-driven forced render completes in ~1-2 ticks, so its latency is
			// bounded by the tick interval. Frame-rate smoothing otherwise silently
			// clamps this headless server toward ~30 fps (a ~33 ms tick), which would
			// dominate the latency; disable it so ticks run as fast as the captures
			// allow (paired with -RenderOffScreen so there is no present pacing either).
			if (GEngine)
			{
				GEngine->bSmoothFrameRate = false;
				GEngine->bUseFixedFrameRate = false;
			}
		}
	}
}

void AMjRenderer::ApplyForcedRenderState(const TSharedPtr<FJsonObject>& Req)
{
	const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
	auto ReadArr = [&](const TCHAR* Key, TArray<double>& Out) {
		if (Req->TryGetArrayField(Key, A) && A)
		{
			Out.Reserve(A->Num());
			for (const TSharedPtr<FJsonValue>& V : *A)
			{
				Out.Add(V.IsValid() ? V->AsNumber() : 0.0);
			}
		}
	};

	const int32 NGeom = Model ? static_cast<int32>(Model->ngeom) : 0;
	TArray<double> GPos, GQuat;
	ReadArr(TEXT("geom_pos"), GPos);
	ReadArr(TEXT("geom_quat"), GQuat);
	if (NGeom > 0 && GPos.Num() == 3 * NGeom && GQuat.Num() == 4 * NGeom && Model->geom_pos && Model->geom_quat)
	{
		FMemory::Memcpy(Model->geom_pos, GPos.GetData(), sizeof(double) * 3 * NGeom);
		FMemory::Memcpy(Model->geom_quat, GQuat.GetData(), sizeof(double) * 4 * NGeom);
	}

	TArray<double> Gp, Gq;
	ReadArr(TEXT("gxpos"), Gp);
	ReadArr(TEXT("gxquat"), Gq);
	if (NGeom > 0 && Gp.Num() == 3 * NGeom && Gq.Num() == 4 * NGeom)
	{
		ApplyGeomTransforms(Gp.GetData(), Gq.GetData());
	}
	else
	{
		const int32 NBody = Model ? static_cast<int32>(Model->nbody) : 0;
		TArray<double> Bp, Bq;
		ReadArr(TEXT("bxpos"), Bp);
		ReadArr(TEXT("bxquat"), Bq);
		if (NBody > 0 && Bp.Num() == 3 * NBody && Bq.Num() == 4 * NBody)
		{
			ApplyBodyTransforms(Bp.GetData(), Bq.GetData());
		}
	}

	if (CameraComps.Num() > 0)
	{
		const int32 NCam = CameraComps.Num();
		TArray<double> Cp, Cq;
		ReadArr(TEXT("cxpos"), Cp);
		ReadArr(TEXT("cxquat"), Cq);
		if (Cp.Num() >= NCam * 3 && Cq.Num() >= NCam * 4)
		{
			ApplyCameraPoses(Cp.GetData(), Cq.GetData());
		}
	}

	// Optional free/user camera pose (ucpos/ucfwd/ucup): drive the returnable "user"
	// camera so a forced render can include the operator's viewpoint. Absent on
	// requests that don't move it -- it then holds its last pose.
	{
		TArray<double> Ucp, Ucf, Ucu;
		ReadArr(TEXT("ucpos"), Ucp);
		ReadArr(TEXT("ucfwd"), Ucf);
		ReadArr(TEXT("ucup"), Ucu);
		if (Ucp.Num() == 3 && Ucf.Num() == 3 && Ucu.Num() == 3)
		{
			PoseUserCaptureCam(Ucp.GetData(), Ucf.GetData(), Ucu.GetData());
		}
	}

	// The applied post-step state this render shows, for capture-stamping.
	double SimTime = 0.0;
	double Fid = 0.0;
	Req->TryGetNumberField(TEXT("sim_time"), SimTime);
	if (!Req->TryGetNumberField(TEXT("frame_id"), Fid))
	{
		Req->TryGetNumberField(TEXT("f"), Fid);
	}
	const double PrevSimTime = GetAppliedSimTime();
	if (SimTime == 0.0 || SimTime < PrevSimTime)
	{
		for (UMjCamera* C : CameraComps)
		{
			if (C)
			{
				C->ClearHistory();
			}
		}
		if (UMjCamera* UCam = UserCaptureCam.Get())
		{
			UCam->ClearHistory();
		}
	}
	SetAppliedState(static_cast<uint64>(Fid), SimTime);
}

void AMjRenderer::RenderForcedRequest(const TSharedPtr<FJsonObject>& Req,
	TArray<UMjCamera*>& OutCams, uint64& OutTargetId)
{
	OutCams.Reset();
	OutTargetId = 0;

	// Apply the exact requested state to the scene actors + clock (bumps
	// AppliedFrameId/AppliedSimTime first), so the fresh frames stamp against it.
	ApplyForcedRenderState(Req);

	// The requested camera list: an explicit "cameras" name array, else every camera.
	// The capturing user camera lives OUTSIDE CameraComps (so it never collides with
	// the per-model cxpos indexing), so it is matched / appended explicitly.
	UMjCamera* UCam = UserCaptureCam.Get();
	const TArray<TSharedPtr<FJsonValue>>* CamArr = nullptr;
	if (Req->TryGetArrayField(TEXT("cameras"), CamArr) && CamArr && CamArr->Num() > 0)
	{
		for (const TSharedPtr<FJsonValue>& V : *CamArr)
		{
			const FString Name = V.IsValid() ? V->AsString() : FString();
			bool bMatched = false;
			for (UMjCamera* C : CameraComps)
			{
				if (C && C->GetCanonicalName() == Name)
				{
					OutCams.Add(C);
					bMatched = true;
					break;
				}
			}
			if (!bMatched && UCam && UCam->GetCanonicalName() == Name)
			{
				OutCams.Add(UCam);
			}
		}
	}
	else
	{
		for (UMjCamera* C : CameraComps)
		{
			if (C)
			{
				OutCams.Add(C);
			}
		}
		if (UCam)  // "all cameras" includes the user camera
		{
			OutCams.Add(UCam);
		}
	}

	// The post-step id + applied sim time the fresh frames show.
	{
		double T = 0.0;
		if (Req->TryGetNumberField(TEXT("frame_id"), T))
		{
			OutTargetId = static_cast<uint64>(T);
		}
	}
	const double SimTime = GetAppliedSimTime();

	// SPEAR synchronous single-pass: capture each camera's scene (the capture renders
	// queue ahead on the render thread), then one batched readback + one flush fills
	// every camera's frame with the freshly-rendered pixels before we reply.
	for (UMjCamera* C : OutCams)
	{
		if (C && C->CaptureComponent && C->CaptureComponent->TextureTarget)
		{
			C->CaptureComponent->CaptureScene();
		}
	}
	MjSpearForcedCapture(OutCams, OutTargetId, SimTime);
}

void AMjRenderer::ApplyCameraPoses(const double* Cxpos, const double* Cxquat)
{
	for (int32 C = 0; C < CameraComps.Num(); ++C)
	{
		UMjCamera* Cam = CameraComps[C];
		if (!Cam)
		{
			continue;
		}
		FVector Loc;
		FQuat Rot;
		if (Cxpos && Cxquat)
		{
			// Streamed camera world transforms (wxyz), same convention as geoms.
			Loc = URLabAxisConv::MjPositionToUe(Cxpos + 3 * C) + SceneOrigin;
			Rot = URLabAxisConv::MjQuatToUe(Cxquat + 4 * C);
		}
		else if (Data)
		{
			// Rest pose from this process's mjData (geom_xmat-style 3x3).
			Loc = URLabAxisConv::MjPositionToUe(Data->cam_xpos + 3 * C) + SceneOrigin;
			Rot = MjMat3ToUeQuat(Data->cam_xmat + 9 * C);
		}
		else
		{
			continue;
		}
		Cam->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjRenderer::ApplyCameraPosesFromMat(const double* CamXPos, const double* CamXMat)
{
	if (!CamXPos || !CamXMat)
	{
		return;
	}
	const int32 NCam = CameraComps.Num();
	if (NCam == 0)
	{
		return;
	}
	// The snapshot carries cam_xmat as a MuJoCo 3x3; convert to the wxyz quats
	// ApplyCameraPoses expects (mju_mat2Quat), then let ApplyCameraPoses apply the
	// axis convention (URLabAxisConv::MjQuatToUe) -- the exact conversion Direct mode uses.
	TArray<double> Cxpos;
	TArray<double> Cxquat;
	Cxpos.SetNumUninitialized(NCam * 3);
	Cxquat.SetNumUninitialized(NCam * 4);
	for (int32 C = 0; C < NCam; ++C)
	{
		Cxpos[3 * C + 0] = CamXPos[3 * C + 0];
		Cxpos[3 * C + 1] = CamXPos[3 * C + 1];
		Cxpos[3 * C + 2] = CamXPos[3 * C + 2];
		mju_mat2Quat(Cxquat.GetData() + 4 * C, CamXMat + 9 * C);
	}
	ApplyCameraPoses(Cxpos.GetData(), Cxquat.GetData());
}

void AMjRenderer::ApplyUserCamera(const double* Pos, const double* Fwd, const double* Up)
{
	UWorld* World = GetWorld();
	APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
	if (!PC)
	{
		return;
	}
	// Lock onto a plain view camera once and keep driving THAT one. The player's
	// default view target can change across a live scene swap, so re-reading it
	// every frame silently drops the copycat; the cached actor (not attached to this
	// scene) survives the geometry rebuild. Re-acquire only if it was destroyed.
	ACameraActor* Cam = UserCam.Get();
	if (!Cam)
	{
		Cam = Cast<ACameraActor>(PC->GetViewTarget());
		if (!Cam)
		{
			return;
		}
		UserCam = Cam;
	}
	if (PC->GetViewTarget() != Cam)
	{
		PC->SetViewTargetWithBlend(Cam);
	}

	const FVector FwdUe = URLabAxisConv::MjDirectionToUe(Fwd).GetSafeNormal();
	const FVector UpUe = URLabAxisConv::MjDirectionToUe(Up).GetSafeNormal();
	if (FwdUe.IsNearlyZero())
	{
		return; // a degenerate frame would spin the view; keep the last good pose
	}
	// MuJoCo camera looks along its view direction with +Up; a UE camera looks down
	// +X with +Z up. MakeFromXZ builds that basis directly from the two vectors.
	const FVector LocUe = URLabAxisConv::MjPositionToUe(Pos) + SceneOrigin;
	const FQuat Rot = FRotationMatrix::MakeFromXZ(FwdUe, UpUe).ToQuat();
	Cam->SetActorLocationAndRotation(LocUe, Rot);
}

bool AMjRenderer::IsGeomVisible(int32 G) const
{
	const int32 Group = Model->geom_group[G];
	return Group >= 0 && Group <= 30 && (VisibleGroupMask & (1 << Group)) != 0;
}

const float* AMjRenderer::GeomRgba(int32 G) const
{
	const int32 MatId = Model->geom_matid[G];
	return (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * G);
}

UPrimitiveComponent* AMjRenderer::BuildGeom(int32 G)
{
	const int32 BodyId = Model->geom_bodyid[G];

	// Geom-group visibility: hide collision/other groups the mask excludes
	// (default shows 0-2). Matches MuJoCo's group-toggled visualization.
	if (!IsGeomVisible(G))
	{
		return nullptr;
	}

	// A fully transparent geom is MJCF's "do not draw" (collision/inertial
	// proxies routinely carry rgba="0 0 0 0"). Honour it.
	if (GeomRgba(G)[3] <= 0.0f)
	{
		return nullptr;
	}
	if (BodyId < 0 || BodyId >= BodyActors.Num() || !BodyActors[BodyId])
	{
		return nullptr;
	}

	// One actor per geom, attached under its body actor: every geom becomes a
	// named, individually selectable entry in the outliner (bare components on the
	// body actor are not surfaced there), and the per-geom transform stream drives
	// its mesh component directly.
	const bool bEditorPreview = GetWorld() && !GetWorld()->IsGameWorld();
	FActorSpawnParameters GeomParams;
	GeomParams.Owner = this;
	if (!bEditorPreview)
	{
		GeomParams.ObjectFlags |= RF_Transient;
	}
	AActor* GeomActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, GeomParams);
	if (!GeomActor)
	{
		return nullptr;
	}
	USceneComponent* GeomRoot = NewObject<USceneComponent>(GeomActor, TEXT("GeomRoot"));
	GeomActor->SetRootComponent(GeomRoot);
	GeomRoot->RegisterComponent();
	GeomActor->AttachToActor(BodyActors[BodyId], FAttachmentTransformRules::KeepRelativeTransform);
	GeomActor->Tags.Add(MjbIdTag(kTagGeom, G));
	{
		// Label by the geom's MJCF name, else its mesh name + id, else the id.
		const char* GeomName = mj_id2name(Model, mjOBJ_GEOM, G);
		FString Label;
		if (GeomName && *GeomName)
		{
			Label = ANSI_TO_TCHAR(GeomName);
		}
		else if (Model->geom_dataid[G] >= 0)
		{
			const char* MeshName = mj_id2name(Model, mjOBJ_MESH, Model->geom_dataid[G]);
			Label = FString::Printf(TEXT("%s_g%d"), MeshName ? ANSI_TO_TCHAR(MeshName) : TEXT("mesh"), G);
		}
		else
		{
			Label = FString::Printf(TEXT("geom_%d"), G);
		}
		GeomActor->Tags.Add(FName(*FString::Printf(TEXT("MjbGeomName=%s"), *Label)));
#if WITH_EDITOR
		if (bEditorPreview)
		{
			GeomActor->SetActorLabel(Label);
		}
#endif
	}
	// The geom actor is the parent for this geom's mesh component(s). The compiled
	// play view supplies an imported-asset resolver; the wire path builds straight
	// from the model through the baked resolver. Both share one component contract.
	if (GeomResolver.IsValid())
	{
		return GeomResolver->MakeGeomComponent(G, GeomActor);
	}
	FMjBakedAssetResolver Resolver(Model, AssetBaker);
	return Resolver.MakeGeomComponent(G, GeomActor);
}

void AMjRenderer::SendPerturbation(int32 Select, bool bActive,
	const double LocalPosMj[3], const double RefSelPosMj[3])
{
	if (OwnerControlEndpoint.IsEmpty() || Select < 0)
	{
		return;
	}

	// Forward the drag INTENT (not a force): the owner runs the real
	// mjv_applyPerturbForce -- a mass-scaled, critically-damped spring. `select` is
	// the body, `localpos` the grab point in that body's local MuJoCo frame, and
	// `refselpos` the drag target in the MuJoCo world frame (all metres).
	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetStringField(TEXT("op"), TEXT("fastpath_perturb"));
	Obj->SetNumberField(TEXT("select"), Select);
	Obj->SetBoolField(TEXT("active"), bActive);
	Obj->SetNumberField(TEXT("body"), Select);   // back-compat alias
	TArray<TSharedPtr<FJsonValue>> LP, RP;
	for (int32 i = 0; i < 3; ++i)
	{
		LP.Add(MakeShared<FJsonValueNumber>(LocalPosMj[i]));
		RP.Add(MakeShared<FJsonValueNumber>(RefSelPosMj[i]));
	}
	Obj->SetArrayField(TEXT("localpos"), LP);
	Obj->SetArrayField(TEXT("refselpos"), RP);
	TArray<uint8> Buf;
	FURLabMsgpackUtil::PackJsonObject(Obj, Buf);

	// Short-lived REQ; fire the request and read the ack so the REP stays in sync.
	if (UURLabRpcClientTransport* Client = UURLabRpcClientTransport::Create(this, OwnerControlEndpoint))
	{
		TArray<uint8> Ack;
		Client->Request(Buf, Ack, 500); // best-effort ack
		Client->TransportShutdown();
	}
}

int32 AMjRenderer::PickBodyIdAlongRay(const FVector& Origin, const FVector& Dir, float& OutDepthCm) const
{
	OutDepthCm = 0.0f;
	if (!Model)
	{
		return -1;
	}

	int32 BestBody = -1;
	float BestDepth = TNumericLimits<float>::Max();

	// The render geoms carry no query collision, so intersect the cursor ray with each
	// built geom's world bounding sphere and keep the nearest. The sphere over-grabs a
	// little versus the mesh, which is the forgiving behaviour a drag-pick wants.
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		const UPrimitiveComponent* Comp = GeomComps[G].Get();
		if (!Comp || !Comp->IsVisible() || G >= static_cast<int32>(Model->ngeom))
		{
			continue;
		}
		const int32 BodyId = Model->geom_bodyid[G];
		if (BodyId <= 0) // body 0 is the static world; never draggable
		{
			continue;
		}

		const FBoxSphereBounds Bounds = Comp->Bounds;
		const FVector L = Bounds.Origin - Origin;
		const float Tca = static_cast<float>(FVector::DotProduct(L, Dir));
		if (Tca < 0.0f)
		{
			continue; // sphere is behind the cursor
		}
		const float R = static_cast<float>(Bounds.SphereRadius);
		const float D2 = static_cast<float>(L.SizeSquared()) - Tca * Tca;
		if (D2 > R * R)
		{
			continue; // ray misses the sphere
		}
		const float Thc = FMath::Sqrt(R * R - D2);
		const float Entry = Tca - Thc;
		const float Depth = Entry > 0.0f ? Entry : Tca; // inside the sphere: grab at center depth
		if (Depth < BestDepth)
		{
			BestDepth = Depth;
			BestBody = BodyId;
		}
	}

	if (BestBody > 0)
	{
		OutDepthCm = FMath::Max(BestDepth, 1.0f);
	}
	return BestBody;
}

void AMjRenderer::ProcessMirrorPerturbationInput()
{
	// A mirror (stream/push/await consumer) never owns physics; forwarding requires an owner to apply
	// the wrench. A Sim producer never reaches here -- Tick returns earlier.
	if (Drive == EMjDrive::Sim || OwnerControlEndpoint.IsEmpty() || bExternallyDriven)
	{
		return;
	}

	UWorld* W = GetWorld();
	APlayerController* PC = W ? W->GetFirstPlayerController() : nullptr;
	if (!PC)
	{
		return;
	}

	// Ctrl+LMB is the drag gesture, chosen so a plain click still reaches the HUD
	// buttons and the camera controls unmodified.
	const bool bCtrl = PC->IsInputKeyDown(EKeys::LeftControl) || PC->IsInputKeyDown(EKeys::RightControl);
	const bool bDragHeld = bCtrl && PC->IsInputKeyDown(EKeys::LeftMouseButton);

	if (!bDragHeld)
	{
		// Release edge: stop the pull and clear the owner's latched wrench so the
		// body is not left drifting under the last force.
		if (bMirrorDragActive)
		{
			if (MirrorDragBodyId >= 0)
			{
				const double Zero[3] = {0.0, 0.0, 0.0};
				SendPerturbation(MirrorDragBodyId, /*bActive=*/false, Zero, Zero);
			}
			bMirrorDragActive = false;
			MirrorDragBodyId = -1;
		}
		return;
	}

	FVector CursorOrigin, CursorDir;
	if (!PC->DeprojectMousePositionToWorld(CursorOrigin, CursorDir))
	{
		return;
	}
	CursorDir = CursorDir.GetSafeNormal();

	// Press edge: grab the nearest model body under the cursor ray, and capture the
	// grab point in that body's LOCAL MuJoCo frame (so the owner's spring anchors on
	// the clicked point and tracks the body as it moves/rotates).
	if (!bMirrorDragActive)
	{
		float DepthCm = 0.0f;
		const int32 BodyId = PickBodyIdAlongRay(CursorOrigin, CursorDir, DepthCm);
		if (BodyId <= 0)
		{
			return; // nothing draggable under the cursor
		}
		bMirrorDragActive = true;
		MirrorDragBodyId = BodyId;
		MirrorDragDepthCm = DepthCm;

		// localpos = R_body^-1 * (grab_world_mj - body_pos_mj), from the cached bus
		// pose. Falls back to the body origin if no transform frame has arrived yet.
		MirrorGrabLocalMj[0] = MirrorGrabLocalMj[1] = MirrorGrabLocalMj[2] = 0.0;
		const FVector GrabWorldUE = CursorOrigin + CursorDir * DepthCm;
		if (LastBxpos.Num() >= 3 * (BodyId + 1) && LastBxquat.Num() >= 4 * (BodyId + 1))
		{
			double GrabWorldMj[3];
			URLabAxisConv::UePositionToMj(GrabWorldUE - SceneOrigin, GrabWorldMj);
			double Rel[3] = {
				GrabWorldMj[0] - LastBxpos[3 * BodyId + 0],
				GrabWorldMj[1] - LastBxpos[3 * BodyId + 1],
				GrabWorldMj[2] - LastBxpos[3 * BodyId + 2]};
			double NegQuat[4];
			mju_negQuat(NegQuat, &LastBxquat[4 * BodyId]);
			mju_rotVecQuat(MirrorGrabLocalMj, Rel, NegQuat);
		}
	}

	// Drag: the target point on the cursor ray at the grab depth, in MuJoCo world.
	// The owner turns (select, localpos, refselpos) into a real mjv_applyPerturbForce
	// -- mass-scaled + critically damped, so it settles on the target instead of
	// flying off. All magnitude/damping now lives on the owner; the mirror only aims.
	const FVector TargetWorldUE = CursorOrigin + CursorDir * MirrorDragDepthCm;
	double RefSelPosMj[3];
	URLabAxisConv::UePositionToMj(TargetWorldUE - SceneOrigin, RefSelPosMj);

	// Render the perturbation gizmo: a yellow arrow from the live grab point on the
	// body to the drag target, the way MuJoCo's own Ctrl-drag arrow is. The origin
	// is the grab point recomputed from the CURRENT streamed body pose
	// (selpos = body_xpos + R_body * localpos), so it tracks the body as it moves --
	// the body actor roots aren't animated (the geom components are), so their
	// location would be stale. (ENABLE_DRAW_DEBUG is on in Development.)
	if (UWorld* GW = GetWorld())
	{
		FVector GrabWorldUE = TargetWorldUE;   // fallback if no pose frame yet
		const int32 B = MirrorDragBodyId;
		if (LastBxpos.Num() >= 3 * (B + 1) && LastBxquat.Num() >= 4 * (B + 1))
		{
			double Rot[3];
			mju_rotVecQuat(Rot, MirrorGrabLocalMj, &LastBxquat[4 * B]);
			const double SelPosMj[3] = {
				LastBxpos[3 * B + 0] + Rot[0],
				LastBxpos[3 * B + 1] + Rot[1],
				LastBxpos[3 * B + 2] + Rot[2]};
			GrabWorldUE = URLabAxisConv::MjPositionToUe(SelPosMj) + SceneOrigin;
		}
		DrawDebugDirectionalArrow(GW, GrabWorldUE, TargetWorldUE, 24.0f, FColor::Yellow,
			/*bPersistent=*/false, /*Life=*/-1.0f, SDPG_Foreground, /*Thickness=*/1.5f);
		DrawDebugSphere(GW, GrabWorldUE, 4.0f, 10, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, 0.8f);
	}

	SendPerturbation(MirrorDragBodyId, /*bActive=*/true, MirrorGrabLocalMj, RefSelPosMj);
}

int32 AMjRenderer::NumGeomsNamed(FName GeomName) const
{
	if (!Model)
	{
		return 0;
	}
	const FString Want = GeomName.ToString();
	int32 Count = 0;
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		if (!GeomComps[G])
		{
			continue;
		}
		if (CompiledGeomNameMatches(Model, G, Want, bExternallyDriven))
		{
			++Count;
		}
	}
	return Count;
}

int32 AMjRenderer::ApplyAppearanceOverride(FName GeomName, const FMjGeomAppearance* Override,
	TFunctionRef<UTexture*(FName)> ResolveTexture)
{
	if (!Model)
	{
		return 0;
	}
	const FString Want = GeomName.ToString();
	int32 Applied = 0;
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		if (!CompiledGeomNameMatches(Model, G, Want, bExternallyDriven))
		{
			continue;
		}

		if (Override != nullptr)
		{
			if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(Comp->GetMaterial(0)))
			{
				MjAppearance::Apply(Mid, *Override, ResolveTexture);
				++Applied;
			}
		}
		else if (AssetBaker)
		{
			// Restore the baked appearance by rebuilding this geom's MID from the model.
			AssetBaker->ApplyGeomMaterial(Comp, G);
			++Applied;
		}
	}
	return Applied;
}

void AMjRenderer::ApplyGeomTransforms(const double* Xpos, const double* Xquat)
{
	if (!Xpos || !Xquat)
	{
		return;
	}
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		double WorldPos[3] = {Xpos[3 * G + 0], Xpos[3 * G + 1], Xpos[3 * G + 2]};
		double WorldQuat[4] = {Xquat[4 * G + 0], Xquat[4 * G + 1], Xquat[4 * G + 2], Xquat[4 * G + 3]};
		CorrectMeshFrameWorld(G, WorldPos, WorldQuat);
		const FVector Loc = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FQuat Rot = URLabAxisConv::MjQuatToUe(WorldQuat);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjRenderer::CorrectMeshFrameWorld(int32 GeomId, double* WorldPos, double* WorldQuat) const
{
	if (!GeomResolver.IsValid())
	{
		return;
	}
	const FMjMeshFrameInverse* Inv = GeomResolver->FindMeshFrameInverse(GeomId);
	if (!Inv)
	{
		return;
	}
	// World = geom-frame  ∘  inverse-mesh-recentre: rotate the inverse offset into
	// the geom frame and add it, then fold in the inverse rotation.
	double Rotated[3];
	mju_rotVecQuat(Rotated, Inv->Pos, WorldQuat);
	WorldPos[0] += Rotated[0];
	WorldPos[1] += Rotated[1];
	WorldPos[2] += Rotated[2];
	double Composed[4];
	mju_mulQuat(Composed, WorldQuat, Inv->Quat);
	WorldQuat[0] = Composed[0];
	WorldQuat[1] = Composed[1];
	WorldQuat[2] = Composed[2];
	WorldQuat[3] = Composed[3];
}

void AMjRenderer::ApplyBodyTransforms(const double* Bxpos, const double* Bxquat)
{
	if (!Bxpos || !Bxquat || !Model)
	{
		return;
	}
	const int32 NBody = static_cast<int32>(Model->nbody);

	// Cache the raw MuJoCo body poses so the Mirror perturb path can map a cursor
	// grab into the MuJoCo frame (it forwards drag intent, not a computed force).
	LastBxpos.SetNumUninitialized(3 * NBody);
	LastBxquat.SetNumUninitialized(4 * NBody);
	FMemory::Memcpy(LastBxpos.GetData(), Bxpos, sizeof(double) * 3 * NBody);
	FMemory::Memcpy(LastBxquat.GetData(), Bxquat, sizeof(double) * 4 * NBody);

	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		const int32 B = Model->geom_bodyid[G];
		if (B < 0 || B >= NBody)
		{
			continue;
		}
		// geom world = body world  ∘  geom-in-body offset (from the model). The
		// offset is constant, so the wire only carries the body transforms.
		double Rotated[3];
		double WorldPos[3];
		double WorldQuat[4];
		mju_rotVecQuat(Rotated, Model->geom_pos + 3 * G, Bxquat + 4 * B);
		WorldPos[0] = Bxpos[3 * B + 0] + Rotated[0];
		WorldPos[1] = Bxpos[3 * B + 1] + Rotated[1];
		WorldPos[2] = Bxpos[3 * B + 2] + Rotated[2];
		mju_mulQuat(WorldQuat, Bxquat + 4 * B, Model->geom_quat + 4 * G);
		CorrectMeshFrameWorld(G, WorldPos, WorldQuat);
		const FVector Loc = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FQuat Rot = URLabAxisConv::MjQuatToUe(WorldQuat);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjRenderer::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// Streaming is a play-session behaviour only. In the editor world this actor
	// is a static, persistent preview and must never animate from the network or
	// the dev sweep -- authoring the level shouldn't mutate scene actors. (The
	// editor preview also never connects the bus, so this is belt-and-suspenders.)
	const UWorld* W = GetWorld();
	if (!W || !W->IsGameWorld())
	{
		return;
	}

	// The compiled play view holds no pose source of its own; the manager applies the
	// engine's render snapshot to it each frame.
	if (bExternallyDriven)
	{
		return;
	}

	// Direct mode renders the engine's stepped state, not a streamed frame.
	if (Drive == EMjDrive::Sim)
	{
		Direct.ApplyFromSnapshot(*this);
		return;
	}

	// Mirror role: this scene runs no physics, so a viewer's drag is forwarded to
	// the owner (which applies it via xfrc_applied). Guarded on an owner endpoint so
	// an owner-less placed scene captures no input.
	if (!OwnerControlEndpoint.IsEmpty())
	{
		ProcessMirrorPerturbationInput();
	}

	// A streamed frame takes priority: pull the newest raw payload from the bus,
	// then decode + apply here on the game thread (UE components and all
	// UObject/TArray work must stay on the game thread).
	{
		TArray<uint8> Local;
		if (TransportBus)
		{
			TransportBus->TakeLatestFrame(Local);
		}
		if (Local.Num() > 0)
		{
			TSharedPtr<FJsonObject> Obj;
			if (FURLabMsgpackUtil::UnpackToJsonObject(Local.GetData(), Local.Num(), Obj) && Obj.IsValid())
			{
				const int32 NGeom = Model ? static_cast<int32>(Model->ngeom) : 0;
				const int32 NBody = Model ? static_cast<int32>(Model->nbody) : 0;

				// Applied post-step state the owner resolved for this pose payload, so
				// this render server's cameras stamp readbacks and drive delay reveal
				// against it (IMjSimClock) exactly as on the manager paths. Absent from
				// a legacy owner -> stays 0, and the sim-clock delay is simply inert
				// (wall-clock delay + forced render still work).
				double PayloadSimTime = 0.0;
				double PayloadFrameId = 0.0;
				Obj->TryGetNumberField(TEXT("sim_time"), PayloadSimTime);
				// The owner's monotonic pose counter ("f"); doubles as the applied
				// frame id (accept "frame_id" too if a future owner sends it).
				if (!Obj->TryGetNumberField(TEXT("frame_id"), PayloadFrameId))
				{
					Obj->TryGetNumberField(TEXT("f"), PayloadFrameId);
				}
				SetAppliedState(static_cast<uint64>(PayloadFrameId), PayloadSimTime);

				const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
				auto ReadArr = [&](const TCHAR* Key, TArray<double>& Out) -> bool {
					if (Obj->TryGetArrayField(Key, A) && A)
					{
						Out.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Out.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
						return true;
					}
					return false;
				};

				// Prefer the per-body stream (fewer transforms, covers mocap); fall
				// back to a per-geom stream from a legacy owner.
				TArray<double> Bp;
				TArray<double> Bq;
				ReadArr(TEXT("bxpos"), Bp);
				ReadArr(TEXT("bxquat"), Bq);
				if (NBody > 0 && Bp.Num() == 3 * NBody && Bq.Num() == 4 * NBody)
				{
					ApplyBodyTransforms(Bp.GetData(), Bq.GetData());
				}
				else
				{
					TArray<double> Xp;
					TArray<double> Xq;
					ReadArr(TEXT("xpos"), Xp);
					ReadArr(TEXT("xquat"), Xq);
					if (NGeom > 0 && Xp.Num() == 3 * NGeom && Xq.Num() == 4 * NGeom)
					{
						ApplyGeomTransforms(Xp.GetData(), Xq.GetData());
					}
				}

				// Optional camera world transforms, so streamed cameras track
				// moving bodies. Absent from owners that don't send them; cameras
				// then stay at their rest pose.
				if (CameraComps.Num() > 0)
				{
					const int32 NCam = CameraComps.Num();
					TArray<double> Cp, Cq;
					if (Obj->TryGetArrayField(TEXT("cxpos"), A) && A)
					{
						Cp.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Cp.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
					}
					if (Obj->TryGetArrayField(TEXT("cxquat"), A) && A)
					{
						Cq.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Cq.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
					}
					if (Cp.Num() == 3 * NCam && Cq.Num() == 4 * NCam)
					{
						ApplyCameraPoses(Cp.GetData(), Cq.GetData());
					}
				}

				// Optional free/user camera ("copycat"): the Driver mirrors its
				// MuJoCo-viewer camera as eye position + forward + up, and this
				// Renderer points its game viewport at the same view.
				{
					TArray<double> Ucp, Ucf, Ucu;
					if (ReadArr(TEXT("ucpos"), Ucp) && ReadArr(TEXT("ucfwd"), Ucf) &&
						ReadArr(TEXT("ucup"), Ucu) &&
						Ucp.Num() == 3 && Ucf.Num() == 3 && Ucu.Num() == 3)
					{
						// On-screen copycat (non-headless viewer) ...
						ApplyUserCamera(Ucp.GetData(), Ucf.GetData(), Ucu.GetData());
						// ... and the returnable capturing user camera (headless
						// viewer/stream), so a subscriber gets the operator's viewpoint.
						PoseUserCaptureCam(Ucp.GetData(), Ucf.GetData(), Ucu.GetData());
					}
				}
			}
			return;
		}
	}
}

void AMjRenderer::ApplyFromData()
{
	if (!Model || !Data)
	{
		return;
	}
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		// mjData stores geom orientation as a 3x3 (geom_xmat); convert to a wxyz quat.
		double WorldPos[3] = {Data->geom_xpos[3 * G + 0], Data->geom_xpos[3 * G + 1], Data->geom_xpos[3 * G + 2]};
		double WorldQuat[4];
		mju_mat2Quat(WorldQuat, Data->geom_xmat + 9 * G);
		CorrectMeshFrameWorld(G, WorldPos, WorldQuat);
		const FVector Loc = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FQuat Rot = URLabAxisConv::MjQuatToUe(WorldQuat);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
	ApplyCameraPoses(nullptr, nullptr); // rest pose from mjData
}

AAMjManager* AMjRenderer::EnsureManager()
{
	if (AAMjManager* Cached = Direct.Manager.Get())
	{
		return Cached;
	}
	// Find a manager already in the level (placed, or spawned by something else)
	// via a level scan rather than the Instance singleton, so a placed manager
	// that has not begun play yet is still found and we do not double-spawn.
	AAMjManager* Mgr = nullptr;
	{
		TArray<AActor*> Found;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAMjManager::StaticClass(), Found);
		if (Found.Num() > 0)
		{
			Mgr = Cast<AAMjManager>(Found[0]);
		}
	}
	if (!Mgr)
	{
		// The fast path never wants the interactive MjSimulate widget. The manager
		// auto-creates it during its compile (which runs on BeginPlay), so spawn
		// DEFERRED and clear bAutoCreateSimulateWidget before BeginPlay. A fast-path
		// render server needs the manager only for its bridge/stepping context.
		Mgr = GetWorld()->SpawnActorDeferred<AAMjManager>(
			AAMjManager::StaticClass(), FTransform::Identity, /*Owner=*/nullptr,
			/*Instigator=*/nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (Mgr)
		{
			Mgr->bAutoCreateSimulateWidget = false;
			Mgr->FinishSpawning(FTransform::Identity);
			UE_LOG(LogURLab, Log, TEXT("[MjRenderer] spawned a manager (bridge/RPC; no simulate widget)"));
		}
	}
	Direct.Manager = Mgr;

	// Phase 1.1: close the CLI capability gap (source-of-truth §5/§14). bStreamCameras and
	// bAcceptInput were editor/Blueprint-only (AMjManager.h:272,281) — unreachable from the command
	// line. -URLabCaps=cameras/input (negatable: -input / -cameras) now sets them here. HasCapability
	// reads these live per request (AMjManager.cpp:1217-1224), so applying them after the manager has
	// spawned is sufficient. Unset caps leave the manager's existing defaults untouched.
	if (Mgr)
	{
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		if (Caps.bCameras.IsSet())
		{
			Mgr->bStreamCameras = Caps.bCameras.GetValue();
		}
		if (Caps.bInput.IsSet())
		{
			Mgr->bAcceptInput = Caps.bInput.GetValue();
		}
	}
	return Mgr;
}

void AMjRenderer::InstallIntoEngine()
{
	// Timer target for the deferred Direct install; the logic lives in FMjRendererStepMode.
	Direct.InstallIntoEngine(*this);
}

void AMjRenderer::ReloadFromBytes(const TArray<uint8>& NewMjb)
{
	if (NewMjb.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjRenderer] ReloadFromBytes: empty MJB, ignoring"));
		return;
	}

	// Retire the current model, shadow articulation and geometry, but KEEP the
	// manager + engine so the swap reuses the same physics + RPC context.
	AAMjManager* Mgr = Direct.Manager.Get();
	Direct.RetireForReload();

	// Destroy the current geometry tree (body actors + their per-geom child actors;
	// Destroy does not cascade to attached actors, so gather the whole tree first).
	TArray<AActor*> Attached;
	GetAttachedActors(Attached, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	for (AActor* A : Attached)
	{
		if (A)
		{
			A->Destroy();
		}
	}
	BodyActors.Reset();
	GeomComps.Reset();
	CameraComps.Reset();
	if (AssetBaker)
	{
		AssetBaker->Reset();
	}
	if (Data)
	{
		mj_deleteData(Data);
		Data = nullptr;
	}
	if (Model)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}

	// Rebuild from the new bytes. In Direct mode reinstall into the (retained)
	// engine; in Puppet mode reconnect the transform bus, which LoadAndBuild's
	// Teardown tore down (mismatched in-flight frames are skipped by the nbody guard).
	MjbBytes = NewMjb;
	MjbFilePath.Empty(); // bytes take precedence on the next build
	const int32 Geoms = LoadAndBuild();
	if (Geoms < 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] ReloadFromBytes: build failed for the new MJB"));
		return;
	}
	StartCameraStreaming();
	// LoadAndBuild -> Teardown -> StopBus dropped the subscription; bring it back so
	// the puppet keeps mirroring the owner after the swap. Not in the eval regime,
	// where the forced-render REP is the one pose driver and the bus stays down.
	if (!BusEndpoint.IsEmpty() && !bForcedRenderOnly)
	{
		StartBus();
	}
	if (Drive == EMjDrive::Sim && Mgr)
	{
		// The manager has long since begun play, so install immediately (the timer
		// poll in Direct.Begin is only for the first-frame race at level start).
		Direct.Manager = Mgr;
		InstallIntoEngine();
	}

	// The swap built fresh MIDs, so any visual-DR overrides the client set on the
	// retired scene are gone; re-drive them onto the new components by name.
	if (Mgr)
	{
		Mgr->GetAppearanceStore()->ReapplyAll();
	}
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] ReloadFromBytes: swapped model -- %d geoms built"), Geoms);
}

// Bring the Renderer up at high quality with the noisy, temporally-accumulated
// post effects turned down: full scalability groups (Lumen GI / reflections / shadows
// at Epic so the final-gather grain converges), film grain and motion blur off. This
// is what makes a movable-light scene read clean instead of grainy. Overridable at
// runtime from the console (set URLAB_NO_RENDER_QUALITY=1 to skip).
static void ApplyRendererQuality()
{
	// Phase 1.1: -URLabScene=quality=off is the new spelling of -URLabFastNoQuality (source-of-truth §14).
	if (!GEngine || FParse::Param(FCommandLine::Get(), TEXT("URLabFastNoQuality"))
		|| URLabLauncherFlags::SceneNoQuality())
	{
		return;
	}
	static const TCHAR* const Cmds[] = {
		TEXT("sg.ViewDistanceQuality 4"), TEXT("sg.AntiAliasingQuality 4"),
		TEXT("sg.ShadowQuality 4"), TEXT("sg.GlobalIlluminationQuality 4"),
		TEXT("sg.ReflectionQuality 4"), TEXT("sg.PostProcessQuality 4"),
		TEXT("sg.TextureQuality 4"), TEXT("sg.EffectsQuality 4"),
		TEXT("sg.FoliageQuality 4"), TEXT("sg.ShadingQuality 4"),
		TEXT("r.FilmGrain 0"), TEXT("r.MotionBlurQuality 0"),
		TEXT("r.DefaultFeature.MotionBlur 0"),
	};
	for (const TCHAR* Cmd : Cmds)
	{
		GEngine->Exec(nullptr, Cmd);
	}
}

AMjRenderer* AMjRenderer::SpawnRenderer(UWorld* World, const TArray<uint8>& MjbBytes,
	const FString& MjbFilePath, const FString& BusEndpoint, const FVector& Origin,
	bool bStepped, bool bBaseLevel, bool bCameras)
{
	if (!World)
	{
		return nullptr;
	}
	ApplyRendererQuality();
	// Deferred spawn so the fields are set BEFORE BeginPlay runs; BeginPlay then
	// owns the whole build (geometry + camera streaming + Direct/bus connect).
	AMjRenderer* Scene = World->SpawnActorDeferred<AMjRenderer>(AMjRenderer::StaticClass(), FTransform::Identity);
	if (!Scene)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] SpawnRenderer: failed to spawn AMjRenderer"));
		return nullptr;
	}
	Scene->Drive = bStepped ? EMjDrive::Sim : EMjDrive::Stream;
	Scene->MjbBytes = MjbBytes;
	Scene->MjbFilePath = MjbFilePath;
	Scene->BusEndpoint = BusEndpoint;
	Scene->bEnableCameraStreaming = bCameras;
	Scene->bBaseLevel = bBaseLevel;
	Scene->SceneOrigin = Origin;
	UGameplayStatics::FinishSpawningActor(Scene, FTransform::Identity);

	// Lighting is imported from the model in LoadAndBuild (MjSkyImporter): the scene's
	// own <light> elements + headlight + skybox, so nothing is hardcoded here. A
	// curated base level (bBaseLevel) keeps its own rig and is left untouched there.

	// Framing camera at the scene origin (the copycat retargets it once a Driver
	// streams its free camera). Skip it when a VR/drone viewer owns the view
	// (-URLabVrViewer / Phase 1.1 -URLabCaps=vr): the possessed drone pawn IS the viewport, so a
	// static framing camera would steal it and the free-fly controls would move an off-screen pawn.
	if (!FParse::Param(FCommandLine::Get(), TEXT("URLabVrViewer")) && !URLabLauncherFlags::CapsWantVr())
	{
		if (APlayerController* PC = World->GetFirstPlayerController())
		{
			const FTransform View(FRotator(-18.0, 0.0, 0.0), FVector(-450.0, 0.0, 190.0) + Origin);
			if (ACameraActor* Cam = World->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), View))
			{
				PC->SetViewTargetWithBlend(Cam);
			}
		}
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] SpawnRenderer: mode=%s bus=%s baseLevel=%d cameras=%d bytes=%d origin=(%s)"),
		bStepped ? TEXT("direct") : TEXT("puppet"),
		BusEndpoint.IsEmpty() ? TEXT("(none)") : *BusEndpoint, bBaseLevel ? 1 : 0,
		bCameras ? 1 : 0, MjbBytes.Num(), *Origin.ToString());
	return Scene;
}

void AMjRenderer::StartBus()
{
	if (BusEndpoint.IsEmpty())
	{
		return;
	}
	if (!TransportBus)
	{
		TransportBus = NewObject<UMjRendererBus>(this);
	}
	TransportBus->Start(BusEndpoint);
}

void AMjRenderer::StopBus()
{
	if (TransportBus)
	{
		TransportBus->Stop();
	}
}

bool AMjRenderer::HasReceivedFrame() const
{
	return TransportBus ? TransportBus->HasEverReceived() : false;
}

void AMjRenderer::Teardown()
{
	StopBus();
	// Direct mode aliased our raw model+data into the shared engine. Stop-join the
	// physics worker and unalias (and clear the deferred-install timer) BEFORE the
	// deletes below, so the worker is never mid-step against memory we are about to
	// free.
	Direct.Teardown(*this);
	// Destroy body actors AND their per-geom child actors (Destroy does not cascade
	// to attached actors, so gather the whole attached tree first).
	TArray<AActor*> Attached;
	GetAttachedActors(Attached, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	for (AActor* A : Attached)
	{
		if (A)
		{
			A->Destroy();
		}
	}
	// The overlay tint MIDs and seg siblings hang off the geom components we are
	// about to destroy; drop the caches so a rebuild starts clean.
	ClearMaterialOverlay();
	DestroySegPool(EMjCameraMode::InstanceSegmentation);
	DestroySegPool(EMjCameraMode::SemanticSegmentation);
	InstanceSegSubscribers.Reset();
	SemanticSegSubscribers.Reset();
	// The capturing user camera is a component on the scene actor (so it is NOT
	// destroyed by the attached-actor sweep above); tear it down explicitly so a
	// model reload does not leak its render target or re-bind its stream port.
	if (UMjCamera* UCam = UserCaptureCam.Get())
	{
		UCam->SetStreamingEnabled(false);
		UCam->DestroyComponent();
	}
	UserCaptureCam = nullptr;
	BodyActors.Reset();
	GeomComps.Reset();
	GeomOrigins.Reset();
	GeomResolver.Reset();
	if (AssetBaker)
	{
		AssetBaker->Reset();
	}
	// A borrowed model belongs to the shared engine; clear the pointers without
	// freeing them.
	if (!bOwnsModel)
	{
		Data = nullptr;
		Model = nullptr;
		return;
	}
	if (Data)
	{
		mj_deleteData(Data);
		Data = nullptr;
	}
	if (Model)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
}
