// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Entity/MjLightStore.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

#include "Utils/CoordinateConversion.h"

const TCHAR* MjLightKindName(EMjLightKind Kind)
{
	switch (Kind)
	{
	case EMjLightKind::Directional: return TEXT("directional");
	case EMjLightKind::Point:       return TEXT("point");
	case EMjLightKind::Spot:        return TEXT("spot");
	case EMjLightKind::Sky:         return TEXT("sky");
	default:                        return TEXT("unknown");
	}
}

UMjLightHandle::UMjLightHandle()
{
	PrimaryComponentTick.bCanEverTick = false;
	bAutoActivate = false;
}

namespace
{
/** Classify a level light that carries no handle, so a hand-placed sun still
 *  reports the kind an override has to respect. */
EMjLightKind KindOfComponent(const ULightComponentBase* Comp)
{
	if (Comp == nullptr)
	{
		return EMjLightKind::Unknown;
	}
	if (Comp->IsA<USkyLightComponent>())     { return EMjLightKind::Sky; }
	if (Comp->IsA<UDirectionalLightComponent>()) { return EMjLightKind::Directional; }
	if (Comp->IsA<USpotLightComponent>())    { return EMjLightKind::Spot; }
	if (Comp->IsA<UPointLightComponent>())   { return EMjLightKind::Point; }
	return EMjLightKind::Unknown;
}

/** What this light's intensity number means, so a client setting an absolute
 *  intensity is not guessing between lux and candelas. */
const TCHAR* IntensityUnitsOf(const ULightComponentBase* Comp)
{
	if (Comp == nullptr)
	{
		return TEXT("unitless");
	}
	if (Comp->IsA<USkyLightComponent>())
	{
		return TEXT("unitless");
	}
	if (Comp->IsA<UDirectionalLightComponent>())
	{
		return TEXT("lux");
	}
	if (const ULocalLightComponent* Local = Cast<ULocalLightComponent>(Comp))
	{
		switch (Local->IntensityUnits)
		{
		case ELightUnits::Candelas: return TEXT("candelas");
		case ELightUnits::Lumens:   return TEXT("lumens");
		case ELightUnits::EV:       return TEXT("ev");
		default:                    return TEXT("unitless");
		}
	}
	return TEXT("unitless");
}
} // namespace

void UMjLightStore::InitForWorld(UWorld* InWorld)
{
	World = InWorld;
}

UWorld* UMjLightStore::ResolveWorld() const
{
	if (UWorld* W = World.Get())
	{
		return W;
	}
	return GetWorld();
}

void UMjLightStore::CollectTargets(FName LightName, TArray<FTarget>& Out) const
{
	UWorld* W = ResolveWorld();
	if (W == nullptr)
	{
		return;
	}
	const bool bAll = LightName.IsNone();

	for (TActorIterator<AActor> It(W); It; ++It)
	{
		AActor* Actor = *It;
		if (Actor == nullptr)
		{
			continue;
		}

		// An imported light's name is the MJCF one on its handle; anything else in
		// the level answers to its actor name, so a hand-built render-server level
		// is addressable too.
		UMjLightHandle* Handle = Actor->FindComponentByClass<UMjLightHandle>();
		const FName Addressed = (Handle != nullptr && !Handle->MjName.IsNone())
			? Handle->MjName : Actor->GetFName();
		if (!bAll && Addressed != LightName)
		{
			continue;
		}

		TArray<ULightComponentBase*> Comps;
		Actor->GetComponents<ULightComponentBase>(Comps);
		for (ULightComponentBase* Comp : Comps)
		{
			if (Comp == nullptr)
			{
				continue;
			}
			FTarget Target;
			Target.Owner = Actor;
			Target.Component = Comp;
			Target.Handle = Handle;
			Target.bSky = Comp->IsA<USkyLightComponent>();
			Target.Kind = (Handle != nullptr && Handle->Kind != EMjLightKind::Unknown)
				? Handle->Kind : KindOfComponent(Comp);
			Out.Add(Target);
		}
	}
}

TArray<UMjLightStore::FLightInfo> UMjLightStore::ListLights() const
{
	TArray<FTarget> Targets;
	CollectTargets(NAME_None, Targets);

	TArray<FLightInfo> Out;
	Out.Reserve(Targets.Num());
	for (const FTarget& T : Targets)
	{
		FLightInfo Info;
		Info.Name = (T.Handle != nullptr && !T.Handle->MjName.IsNone())
			? T.Handle->MjName
			: (T.Owner != nullptr ? T.Owner->GetFName() : NAME_None);
		// Prefer the exact MJCF spelling; fall back to the FName only for a light
		// that has no handle at all (a level light the model never declared).
		Info.NameExact = (T.Handle != nullptr && !T.Handle->MjNameExact.IsEmpty())
			? T.Handle->MjNameExact
			: Info.Name.ToString();
		Info.Kind = T.Kind;
		Info.MjId = T.Handle != nullptr ? T.Handle->MjId : INDEX_NONE;
		Info.Intensity = T.Component->Intensity;
		Info.IntensityUnits = IntensityUnitsOf(T.Component);
		Info.Colour = T.Component->GetLightColor();
		Info.bEnabled = T.Component->IsVisible();
		Info.bCastShadow = T.Component->CastShadows != 0;
		Info.bImported = T.Handle != nullptr;
		Info.bOverridden = Overrides.Contains(Info.Name);
		Out.Add(Info);
	}
	return Out;
}

UMjLightStore::FResolution UMjLightStore::ResolveLight(FName LightName) const
{
	TArray<FTarget> Targets;
	CollectTargets(LightName, Targets);

	FResolution Out;
	Out.Components = Targets.Num();
	for (const FTarget& T : Targets)
	{
		if (Out.Kind == EMjLightKind::Unknown)
		{
			Out.Kind = T.Kind;
		}
		if (Out.MjId == INDEX_NONE && T.Handle != nullptr)
		{
			Out.MjId = T.Handle->MjId;
		}
	}
	return Out;
}

void UMjLightStore::ApplyToTarget(const FTarget& Target, const FMjLightOverride* Override)
{
	ULightComponentBase* Comp = Target.Component;
	if (Comp == nullptr)
	{
		return;
	}
	USkyLightComponent* Sky = Target.bSky ? Cast<USkyLightComponent>(Comp) : nullptr;
	ULightComponent* Light = Sky != nullptr ? nullptr : Cast<ULightComponent>(Comp);

	if (Override == nullptr)
	{
		// Restore: nothing to put back if this component was never touched.
		const FBaseline* Base = Baselines.Find(Comp);
		if (Base == nullptr)
		{
			return;
		}
		if (Sky != nullptr)
		{
			Sky->SetIntensity(Base->Intensity);
			Sky->SetLightColor(Base->Colour);
		}
		else if (Light != nullptr)
		{
			Light->SetIntensity(Base->Intensity);
			Light->SetLightColor(Base->Colour);
		}
		Comp->SetCastShadows(Base->bCastShadow);
		Comp->SetVisibility(Base->bVisible);
		if (Target.Owner != nullptr && Base->bPoseOverridden)
		{
			Target.Owner->SetActorTransform(Base->OwnerTransform);
		}
		Baselines.Remove(Comp);
		if (Sky != nullptr)
		{
			Sky->RecaptureSky();
		}
		return;
	}

	// Snapshot once, before the FIRST override touches this component. A second
	// override must not re-snapshot: it would capture the already-overridden
	// values, and clearing would then restore the override instead of the
	// authored look.
	FBaseline* Existing = Baselines.Find(Comp);
	if (Existing == nullptr)
	{
		FBaseline Fresh;
		Fresh.Intensity = Comp->Intensity;
		Fresh.Colour = Comp->GetLightColor();
		Fresh.bVisible = Comp->IsVisible();
		Fresh.bCastShadow = Comp->CastShadows != 0;
		Fresh.OwnerTransform = Target.Owner != nullptr
			? Target.Owner->GetActorTransform() : FTransform::Identity;
		Existing = &Baselines.Add(Comp, Fresh);
	}
	FBaseline& Base = *Existing;

	if (Override->Intensity.IsSet() || Override->IntensityScale.IsSet())
	{
		const float Absolute = Override->Intensity.IsSet()
			? Override->Intensity.GetValue() : Base.Intensity;
		const float Scaled = Absolute * (Override->IntensityScale.IsSet()
			? Override->IntensityScale.GetValue() : 1.0f);
		if (Sky != nullptr)
		{
			Sky->SetIntensity(FMath::Max(Scaled, 0.0f));
		}
		else if (Light != nullptr)
		{
			Light->SetIntensity(Scaled);
		}
	}
	if (Override->Colour.IsSet())
	{
		if (Sky != nullptr)
		{
			Sky->SetLightColor(Override->Colour.GetValue());
		}
		else if (Light != nullptr)
		{
			Light->SetLightColor(Override->Colour.GetValue());
		}
	}
	if (Override->bCastShadow.IsSet())
	{
		Comp->SetCastShadows(Override->bCastShadow.GetValue());
	}
	if (Override->bEnabled.IsSet())
	{
		Comp->SetVisibility(Override->bEnabled.GetValue());
	}

	// Pose. A sky light has no meaningful pose and a directional light has no
	// meaningful position, so those are silently skipped rather than refused --
	// the same override can then be swept across a mixed set of lights.
	if (Target.Owner != nullptr && Sky == nullptr)
	{
		FVector Location = Target.Owner->GetActorLocation();
		FRotator Rotation = Target.Owner->GetActorRotation();
		bool bMoved = false;

		if (Override->MjPosition.IsSet() && Target.Kind != EMjLightKind::Directional)
		{
			const FVector& P = Override->MjPosition.GetValue();
			const double Mj[3] = {P.X, P.Y, P.Z};
			const FVector Origin = Target.Handle != nullptr
				? Target.Handle->SceneOrigin : FVector::ZeroVector;
			Location = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Mj) + Origin;
			bMoved = true;
		}
		if (Override->MjDirection.IsSet())
		{
			const FVector& D = Override->MjDirection.GetValue();
			const double Mj[3] = {D.X, D.Y, D.Z};
			const FVector Dir = URLabCoordinateConversion::RightHandedDirectionToUnreal(Mj).GetSafeNormal();
			if (!Dir.IsNearlyZero())
			{
				Rotation = Dir.Rotation();
				bMoved = true;
			}
		}
		if (bMoved)
		{
			// The importer had to force Movable to make a spawn rotation stick; an
			// override moving a level light needs the same, or a Static light silently
			// ignores the move.
			if (USceneComponent* Root = Target.Owner->GetRootComponent())
			{
				Root->SetMobility(EComponentMobility::Movable);
			}
			Comp->SetMobility(EComponentMobility::Movable);
			Target.Owner->SetActorLocationAndRotation(Location, Rotation);
			Base.bPoseOverridden = true;
		}
	}

	if (Sky != nullptr)
	{
		Sky->MarkRenderStateDirty();
		Sky->RecaptureSky();
	}
}

int32 UMjLightStore::ApplyToWorld(FName LightName, const FMjLightOverride* Override)
{
	TArray<FTarget> Targets;
	CollectTargets(LightName, Targets);
	for (const FTarget& T : Targets)
	{
		ApplyToTarget(T, Override);
	}
	return Targets.Num();
}

int32 UMjLightStore::SetOverride(FName LightName, const FMjLightOverride& Override)
{
	FMjLightOverride& Stored = Overrides.FindOrAdd(LightName);
	// Merge rather than replace: a client that set a colour and then sets an
	// intensity means both, matching how each field is independently optional.
	if (Override.bEnabled.IsSet())        { Stored.bEnabled = Override.bEnabled; }
	if (Override.Intensity.IsSet())       { Stored.Intensity = Override.Intensity; }
	if (Override.IntensityScale.IsSet())  { Stored.IntensityScale = Override.IntensityScale; }
	if (Override.Colour.IsSet())          { Stored.Colour = Override.Colour; }
	if (Override.MjDirection.IsSet())     { Stored.MjDirection = Override.MjDirection; }
	if (Override.MjPosition.IsSet())      { Stored.MjPosition = Override.MjPosition; }
	if (Override.bCastShadow.IsSet())     { Stored.bCastShadow = Override.bCastShadow; }
	return ApplyToWorld(LightName, &Stored);
}

int32 UMjLightStore::ClearOverride(FName LightName)
{
	if (Overrides.Remove(LightName) == 0)
	{
		return -1;
	}
	return ApplyToWorld(LightName, nullptr);
}

int32 UMjLightStore::ClearAll()
{
	TArray<FName> Names;
	Overrides.GetKeys(Names);
	Overrides.Reset();
	int32 Restored = 0;
	for (const FName& Name : Names)
	{
		Restored += ApplyToWorld(Name, nullptr);
	}
	return Restored;
}

void UMjLightStore::ReapplyAll()
{
	// A rebuild re-spawned the imported lights, so the old components (and their
	// baselines) are gone; drop lapsed entries so the fresh components snapshot
	// their own authored values rather than inheriting a dead light's.
	for (auto It = Baselines.CreateIterator(); It; ++It)
	{
		if (!It->Key.IsValid())
		{
			It.RemoveCurrent();
		}
	}
	for (TPair<FName, FMjLightOverride>& Pair : Overrides)
	{
		ApplyToWorld(Pair.Key, &Pair.Value);
	}
}
