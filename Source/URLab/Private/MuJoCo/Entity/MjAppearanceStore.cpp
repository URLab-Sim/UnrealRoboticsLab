// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Entity/MjAppearanceStore.h"

#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "TextureResource.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Modules/ModuleManager.h"

#include "Utils/AssetCache.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Elements/MjGeom.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
/** The authored name a `UMjGeom` carries: its MJCF name, else the component name. */
FString GeomComponentName(const UMjGeom& Geom)
{
	if (Geom.MjName.IsSet() && !Geom.MjName.GetValue().IsEmpty())
	{
		return Geom.MjName.GetValue();
	}
	return Geom.GetName();
}
} // namespace

void UMjAppearanceStore::Init(AAMjManager* InManager)
{
	Manager = InManager;
}

void UMjAppearanceStore::InitForWorld(UWorld* InWorld)
{
	BoundWorld = InWorld;
}

UWorld* UMjAppearanceStore::ResolveWorld() const
{
	if (AAMjManager* M = Manager.Get())
	{
		if (UWorld* W = M->GetWorld())
		{
			return W;
		}
	}
	if (UWorld* W = BoundWorld.Get())
	{
		return W;
	}
	return GetWorld();
}





UMjAppearanceStore::FResolution UMjAppearanceStore::ResolveGeom(FName GeomName, FName Entity) const
{
	FResolution Out;

	UWorld* World = ResolveWorld();
	if (World == nullptr)
	{
		return Out;
	}

	const FString Want = GeomName.ToString();
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (Actor == nullptr)
		{
			continue;
		}

		if (Entity != NAME_None && Actor->GetFName() != Entity)
		{
			continue;
		}
		TArray<UMjGeom*> Geoms;
		Actor->GetComponents<UMjGeom>(Geoms);
		for (const UMjGeom* Geom : Geoms)
		{
			if (Geom != nullptr && GeomComponentName(*Geom) == Want)
			{
				++Out.AuthoringComponents;
			}
		}
	}
	return Out;
}

int32 UMjAppearanceStore::ApplyToWorld(FName GeomName, FName Entity, const FMjGeomAppearance* Override)
{
	UWorld* World = ResolveWorld();
	if (World == nullptr)
	{
		return 0;
	}

	auto Resolver = [this](FName Key) { return ResolveTexture(Key); };
	const FString Want = GeomName.ToString();
	int32 Applied = 0;

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (Actor == nullptr)
		{
			continue;
		}

		// Authoring geom previews are scoped by their owning entity actor.
		if (Entity != NAME_None && Actor->GetFName() != Entity)
		{
			continue;
		}
		TArray<UMjGeom*> Geoms;
		Actor->GetComponents<UMjGeom>(Geoms);
		for (UMjGeom* Geom : Geoms)
		{
			if (Geom != nullptr && GeomComponentName(*Geom) == Want)
			{
				Applied += Geom->ApplyAppearanceOverride(Override, Resolver);
			}
		}
	}
	return Applied;
}

int32 UMjAppearanceStore::SetOverride(FName GeomName, const FMjGeomAppearance& Appearance, FName Entity)
{
	FStoredOverride& Stored = Overrides.FindOrAdd(GeomName);
	Stored.Appearance = Appearance;
	Stored.Entity = Entity;
	return ApplyToWorld(GeomName, Entity, &Stored.Appearance);
}

int32 UMjAppearanceStore::ClearOverride(FName GeomName)
{
	FStoredOverride Removed;
	if (!Overrides.RemoveAndCopyValue(GeomName, Removed))
	{
		return -1;
	}
	return ApplyToWorld(GeomName, Removed.Entity, nullptr);
}

int32 UMjAppearanceStore::ClearAll()
{
	TArray<TPair<FName, FStoredOverride>> Removed;
	for (const TPair<FName, FStoredOverride>& Pair : Overrides)
	{
		Removed.Emplace(Pair.Key, Pair.Value);
	}
	Overrides.Reset();
	int32 Restored = 0;
	for (const TPair<FName, FStoredOverride>& Pair : Removed)
	{
		Restored += ApplyToWorld(Pair.Key, Pair.Value.Entity, nullptr);
	}
	return Restored;
}

void UMjAppearanceStore::ReapplyAll()
{
	for (TPair<FName, FStoredOverride>& Pair : Overrides)
	{
		ApplyToWorld(Pair.Key, Pair.Value.Entity, &Pair.Value.Appearance);
	}
}

UTexture* UMjAppearanceStore::ResolveTexture(FName Key)
{
	if (Key.IsNone())
	{
		return nullptr;
	}
	if (const TObjectPtr<UTexture>* Cached = TextureCache.Find(Key))
	{
		return Cached->Get();
	}

	const FString KeyStr = Key.ToString();
	UTexture* Result = nullptr;

	// A UE content asset addressed by its object path (a cooked-in library texture).
	if (KeyStr.Contains(TEXT("/")))
	{
		Result = LoadObject<UTexture>(nullptr, *KeyStr);
	}

	// An already-uploaded blob in the content-addressed cache, keyed by its SHA-256:
	// the same store the model-upload path fills, decoded here into a transient
	// texture rather than a second upload channel.
	if (Result == nullptr && FURLabAssetCache::IsValidSha256Hex(KeyStr))
	{
		FString BlobPath;
		if (FURLabAssetCache::Get().GetPath(KeyStr, BlobPath))
		{
			TArray<uint8> Bytes;
			if (FFileHelper::LoadFileToArray(Bytes, *BlobPath))
			{
				IImageWrapperModule& Module =
					FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
				const EImageFormat Format = Module.DetectImageFormat(Bytes.GetData(), Bytes.Num());
				TSharedPtr<IImageWrapper> Wrapper =
					Format != EImageFormat::Invalid ? Module.CreateImageWrapper(Format) : nullptr;
				TArray<uint8> Raw;
				if (Wrapper.IsValid() && Wrapper->SetCompressed(Bytes.GetData(), Bytes.Num())
					&& Wrapper->GetRaw(ERGBFormat::BGRA, 8, Raw))
				{
					UTexture2D* Tex = UTexture2D::CreateTransient(
						Wrapper->GetWidth(), Wrapper->GetHeight(), PF_B8G8R8A8);
					if (Tex != nullptr)
					{
						Tex->SRGB = true;
						FTexturePlatformData* Data = Tex->GetPlatformData();
						void* Dst = Data->Mips[0].BulkData.Lock(LOCK_READ_WRITE);
						FMemory::Memcpy(Dst, Raw.GetData(), Raw.Num());
						Data->Mips[0].BulkData.Unlock();
						Tex->UpdateResource();
						Result = Tex;
					}
				}
			}
		}
	}

	if (Result != nullptr)
	{
		TextureCache.Add(Key, Result);
	}
	return Result;
}
