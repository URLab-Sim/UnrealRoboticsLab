// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Spec/MjAssetResolve.h"

#include "Dom/JsonObject.h"

namespace
{
/** Read a scalar field into an override slot when the request carries it. */
void ReadScalar(const TSharedPtr<FJsonObject>& Req, const TCHAR* Field, TOptional<float>& Out)
{
	double Value = 0.0;
	if (Req->TryGetNumberField(Field, Value))
	{
		Out = static_cast<float>(Value);
	}
}

/** Parse the appearance fields of a set_geom_appearance request into an override.
 *  Only the fields the request names are set, so an unmentioned field is left for
 *  the base pass -- matching the sparse semantics of FMjGeomAppearance. */
FMjGeomAppearance ParseAppearance(const TSharedPtr<FJsonObject>& Req)
{
	FMjGeomAppearance Out;

	const TArray<TSharedPtr<FJsonValue>>* Color = nullptr;
	if (Req->TryGetArrayField(TEXT("color"), Color) && Color->Num() >= 3)
	{
		Out.BaseColor = FLinearColor(
			static_cast<float>((*Color)[0]->AsNumber()),
			static_cast<float>((*Color)[1]->AsNumber()),
			static_cast<float>((*Color)[2]->AsNumber()),
			Color->Num() >= 4 ? static_cast<float>((*Color)[3]->AsNumber()) : 1.0f);
	}

	ReadScalar(Req, TEXT("metallic"), Out.Metallic);
	ReadScalar(Req, TEXT("roughness"), Out.Roughness);
	ReadScalar(Req, TEXT("specular"), Out.Specular);
	ReadScalar(Req, TEXT("reflectance"), Out.Reflectance);
	ReadScalar(Req, TEXT("emission"), Out.Emission);

	const TArray<TSharedPtr<FJsonValue>>* Repeat = nullptr;
	if (Req->TryGetArrayField(TEXT("tex_repeat"), Repeat) && Repeat->Num() >= 2)
	{
		Out.TexRepeat = FVector2D((*Repeat)[0]->AsNumber(), (*Repeat)[1]->AsNumber());
	}

	// `textures` maps an MJCF role token (rgb, normal, orm, ...) to a content key:
	// a UE texture asset path, or an uploaded blob's SHA-256. The store resolves the
	// key to a UTexture at apply time.
	const TSharedPtr<FJsonObject>* Textures = nullptr;
	if (Req->TryGetObjectField(TEXT("textures"), Textures) && Textures->IsValid())
	{
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : (*Textures)->Values)
		{
			const EMjMaterialRole Role = MjMaterialRoleFromName(Pair.Key);
			FString KeyStr;
			if (Role != EMjMaterialRole::Count && Pair.Value.IsValid() && Pair.Value->TryGetString(KeyStr))
			{
				Out.TextureBindings.Add(Role, FName(*KeyStr));
			}
		}
	}

	return Out;
}
} // namespace

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleResolveGeom(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
	{
		return MakeError(URLabError::NoActiveManager, TEXT("resolve_geom requires an active manager"));
	}

	FString Geom;
	Req->TryGetStringField(TEXT("geom"), Geom);
	FString Entity;
	Req->TryGetStringField(TEXT("entity"), Entity);

	const UMjAppearanceStore::FResolution Res = Mgr->GetAppearanceStore()->ResolveGeom(
		FName(*Geom), Entity.IsEmpty() ? NAME_None : FName(*Entity));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("resolve_geom_ok"));
	Reply->SetStringField(TEXT("geom"), Geom);
	Reply->SetBoolField(TEXT("found"), Res.IsFound());
	// The name is the stable handle the client re-addresses with; the compiled id is
	// carried alongside for callers that index the mjModel directly.
	Reply->SetStringField(TEXT("handle"), Geom);
	Reply->SetNumberField(TEXT("mj_id"), Res.MjId);
	Reply->SetNumberField(TEXT("authoring"), Res.AuthoringComponents);
	Reply->SetNumberField(TEXT("fastpath"), Res.FastpathComponents);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetGeomAppearance(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
	{
		return MakeError(URLabError::NoActiveManager, TEXT("set_geom_appearance requires an active manager"));
	}

	FString Geom;
	Req->TryGetStringField(TEXT("geom"), Geom);
	FString Entity;
	Req->TryGetStringField(TEXT("entity"), Entity);
	const FName GeomName(*Geom);
	const FName EntityName = Entity.IsEmpty() ? NAME_None : FName(*Entity);

	UMjAppearanceStore* Store = Mgr->GetAppearanceStore();

	bool bClear = false;
	Req->TryGetBoolField(TEXT("clear"), bClear);

	int32 Applied = 0;
	bool bFound = false;
	if (bClear)
	{
		Applied = Store->ClearOverride(GeomName);
		// -1 marks "no override was held"; the geom itself may still exist, so a
		// clear of an unoverridden geom is not an error -- report zero restored.
		bFound = Applied >= 0;
		Applied = FMath::Max(Applied, 0);
	}
	else
	{
		const FMjGeomAppearance Appearance = ParseAppearance(Req);
		Applied = Store->SetOverride(GeomName, Appearance, EntityName);
		bFound = Applied > 0;
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_geom_appearance_ok"));
	Reply->SetStringField(TEXT("geom"), Geom);
	Reply->SetNumberField(TEXT("applied"), Applied);
	Reply->SetBoolField(TEXT("found"), bFound);
	Reply->SetBoolField(TEXT("cleared"), bClear);
	return Reply;
}
