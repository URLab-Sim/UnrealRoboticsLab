// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityMembers.h"

#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjEntity.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
	const FMjEntity* FindEntity(const UMjPhysicsEngine* Engine, FName EntityName)
	{
		if (Engine == nullptr)
		{
			return nullptr;
		}
		for (const FMjEntity& Candidate : Engine->GetEntityPartition())
		{
			if (Candidate.Name == EntityName)
			{
				return &Candidate;
			}
		}
		return nullptr;
	}

	int32 ObjTypeOf(EMjEntityMember Family)
	{
		switch (Family)
		{
			case EMjEntityMember::Joint:    return mjOBJ_JOINT;
			case EMjEntityMember::Actuator: return mjOBJ_ACTUATOR;
			case EMjEntityMember::Geom:     return mjOBJ_GEOM;
		}
		return mjOBJ_UNKNOWN;
	}

	// The compiled ids this family owns within the entity. Joints and actuators are stored slices;
	// an entity's geoms are the geoms attached to the bodies it owns.
	TArray<int32> FamilyIds(const mjModel* Model, const FMjEntity& E, EMjEntityMember Family)
	{
		switch (Family)
		{
			case EMjEntityMember::Joint:    return E.JointIds;
			case EMjEntityMember::Actuator: return E.ActuatorIds;
			case EMjEntityMember::Geom:
			{
				TArray<int32> Geoms;
				const TSet<int32> Bodies(E.BodyIds);
				for (int G = 0; G < Model->ngeom; ++G)
				{
					if (Bodies.Contains(Model->geom_bodyid[G]))
					{
						Geoms.Add(G);
					}
				}
				return Geoms;
			}
		}
		return TArray<int32>();
	}

	FString CompiledName(const mjModel* Model, int32 ObjType, int32 Id)
	{
		const char* N = mj_id2name(Model, ObjType, Id);
		return N ? FString(UTF8_TO_TCHAR(N)) : FString();
	}

	// The short name a caller means the element by: the compiled name with the entity's prefix taken
	// off. The prefix is the entity name plus '_', matching how the scene assembly attaches a
	// participant. An empty entity name (raw single-root) strips nothing.
	FString ShortName(FName EntityName, const FString& Compiled)
	{
		if (EntityName.IsNone())
		{
			return Compiled;
		}
		const FString Prefix = EntityName.ToString() + TEXT("_");
		return Compiled.StartsWith(Prefix) ? Compiled.RightChop(Prefix.Len()) : Compiled;
	}
}

TArray<FName> MjEntityMembers::Names(const UMjPhysicsEngine* Engine, FName EntityName,
	EMjEntityMember Family)
{
	TArray<FName> Out;
	const mjModel* Model = Engine ? Engine->GetModel() : nullptr;
	const FMjEntity* E = FindEntity(Engine, EntityName);
	if (Model == nullptr || E == nullptr)
	{
		return Out;
	}
	const int32 ObjType = ObjTypeOf(Family);
	for (int32 Id : FamilyIds(Model, *E, Family))
	{
		const FString Compiled = CompiledName(Model, ObjType, Id);
		if (!Compiled.IsEmpty())
		{
			Out.Add(FName(*ShortName(EntityName, Compiled)));
		}
	}
	return Out;
}

int32 MjEntityMembers::ResolveId(const UMjPhysicsEngine* Engine, FName EntityName,
	EMjEntityMember Family, FName Member)
{
	const mjModel* Model = Engine ? Engine->GetModel() : nullptr;
	const FMjEntity* E = FindEntity(Engine, EntityName);
	if (Model == nullptr || E == nullptr || Member.IsNone())
	{
		return -1;
	}
	const int32 ObjType = ObjTypeOf(Family);
	const FString Wanted = Member.ToString();
	for (int32 Id : FamilyIds(Model, *E, Family))
	{
		const FString Compiled = CompiledName(Model, ObjType, Id);
		if (Compiled == Wanted || ShortName(EntityName, Compiled) == Wanted)
		{
			return Id;
		}
	}
	return -1;
}
