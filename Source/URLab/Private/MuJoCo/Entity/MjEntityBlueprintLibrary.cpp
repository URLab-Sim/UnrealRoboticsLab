// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityBlueprintLibrary.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityApi.h"

namespace
{
	// The entity face a picker points at. A picker usually references its AMjEntity directly; when it
	// references another actor (an articulation used as world context) the entity of the same name is
	// fetched from the manager instead.
	const AMjEntity* FaceForPicker(const TSoftObjectPtr<AActor>& Ref)
	{
		AActor* Actor = Ref.Get();
		if (Actor == nullptr)
		{
			Actor = Ref.LoadSynchronous();
		}
		if (const AMjEntity* Direct = Cast<AMjEntity>(Actor))
		{
			return Direct;
		}
		if (Actor != nullptr)
		{
			if (AAMjManager* Manager = AAMjManager::GetManager())
			{
				return Manager->GetEntity(FName(*Actor->GetName()));
			}
		}
		return nullptr;
	}

	FMjJointHandle Mirror(const FMjJoint& H)
	{
		FMjJointHandle Out;
		Out.Id = H.Id;
		Out.Entity = H.Entity;
		return Out;
	}

	FMjActuatorHandle Mirror(const FMjActuator& H)
	{
		FMjActuatorHandle Out;
		Out.Id = H.Id;
		Out.Entity = H.Entity;
		return Out;
	}

	FMjGeomHandle Mirror(const FMjGeom& H)
	{
		FMjGeomHandle Out;
		Out.Id = H.Id;
		Out.Entity = H.Entity;
		return Out;
	}
}

FMjJointHandle UMjEntityBlueprintLibrary::Joint(const AMjEntity* Entity, FName Name)
{
	return Entity ? Mirror(Entity->Joint(Name)) : FMjJointHandle();
}

FMjActuatorHandle UMjEntityBlueprintLibrary::Actuator(const AMjEntity* Entity, FName Name)
{
	return Entity ? Mirror(Entity->Actuator(Name)) : FMjActuatorHandle();
}

FMjGeomHandle UMjEntityBlueprintLibrary::Geom(const AMjEntity* Entity, FName Name)
{
	return Entity ? Mirror(Entity->Geom(Name)) : FMjGeomHandle();
}

FMjJointHandle UMjEntityBlueprintLibrary::ResolveJoint(const FMjJointPicker& Picker)
{
	const AMjEntity* Face = FaceForPicker(Picker.Entity);
	return Face ? Mirror(Face->Joint(Picker.Name)) : FMjJointHandle();
}

FMjActuatorHandle UMjEntityBlueprintLibrary::ResolveActuator(const FMjActuatorPicker& Picker)
{
	const AMjEntity* Face = FaceForPicker(Picker.Entity);
	return Face ? Mirror(Face->Actuator(Picker.Name)) : FMjActuatorHandle();
}

FMjGeomHandle UMjEntityBlueprintLibrary::ResolveGeom(const FMjGeomPicker& Picker)
{
	const AMjEntity* Face = FaceForPicker(Picker.Entity);
	return Face ? Mirror(Face->Geom(Picker.Name)) : FMjGeomHandle();
}

float UMjEntityBlueprintLibrary::JointPos(const FMjJointHandle& Joint)
{
	FMjJoint H;
	H.Id = Joint.Id;
	H.Entity = Joint.Entity;
	return H.Pos();
}

float UMjEntityBlueprintLibrary::JointVel(const FMjJointHandle& Joint)
{
	FMjJoint H;
	H.Id = Joint.Id;
	H.Entity = Joint.Entity;
	return H.Vel();
}

void UMjEntityBlueprintLibrary::SetCtrl(const FMjActuatorHandle& Actuator, double Value)
{
	FMjActuator H;
	H.Id = Actuator.Id;
	H.Entity = Actuator.Entity;
	H.SetCtrl(Value);
}
