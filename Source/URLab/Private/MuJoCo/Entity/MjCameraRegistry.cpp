// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjCameraRegistry.h"

#include "mujoco/mujoco.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "State/MjCanonicalName.h"

namespace
{
	FString CameraName(const mjModel* Model, int32 CameraId)
	{
		const char* N = mj_id2name(Model, mjOBJ_CAMERA, CameraId);
		return N ? FString(UTF8_TO_TCHAR(N)) : FString();
	}

	// The entity a camera belongs to = the one whose compiled prefix (Name + "_") the camera name
	// starts with. Elements are named "<prefix><element>", so the longest matching prefix wins when
	// one entity name is itself a prefix of another.
	const FMjEntity* OwningEntity(const FString& Name, const TArray<FMjEntity>& Entities)
	{
		const FMjEntity* Best = nullptr;
		int32 BestPrefixLen = -1;
		for (const FMjEntity& E : Entities)
		{
			const FString Prefix = E.Name.ToString() + TEXT("_");
			if (Prefix.Len() > 1 && Name.StartsWith(Prefix) && Prefix.Len() > BestPrefixLen)
			{
				Best = &E;
				BestPrefixLen = Prefix.Len();
			}
		}
		return Best;
	}
}

void FMjCameraRegistry::Build(const mjModel* Model, const TArray<FMjEntity>& Entities)
{
	Cameras.Reset();
	if (Model == nullptr)
	{
		return;
	}

	for (int32 i = 0; i < Model->ncam; ++i)
	{
		const FString Name = CameraName(Model, i);

		FMjCameraInfo Info;
		Info.CameraId = i;

		if (const FMjEntity* Owner = OwningEntity(Name, Entities))
		{
			// Same canonical form as ResolveCameraCanonical: "<art>/<part>". The art is the
			// entity's PublicName -- the ActorId-derived segment FMjCanonicalName::ArtSegment
			// produced for the articulation (NOT the compiled-prefix stem, which diverges when
			// the ActorId differs from the UE object name). The part is the camera name with
			// the compiled "<entity>_" prefix (the object-name stem) stripped, then sanitized.
			// Both match the actor-derived path so the topic/stem strings are identical.
			const FName Art = Owner->PublicName;
			const FString Prefix = Owner->Name.ToString() + TEXT("_");
			const FString Local = Name.RightChop(Prefix.Len());
			const FName Part = FName(*FMjCanonicalName::Sanitize(Local));

			Info.EntityName = Owner->Name;
			Info.CanonicalName = FName(*FMjCanonicalName::Full(Art, Part));
		}
		else
		{
			// A world camera owns no entity: no art segment, so the sanitized raw camera name is
			// its stable public identity.
			Info.EntityName = NAME_None;
			Info.CanonicalName = FName(*FMjCanonicalName::Sanitize(Name));
		}

		Cameras.Add(Info);
	}
}

const FMjCameraInfo* FMjCameraRegistry::Find(FName Canonical) const
{
	return Cameras.FindByPredicate([Canonical](const FMjCameraInfo& C) { return C.CanonicalName == Canonical; });
}
