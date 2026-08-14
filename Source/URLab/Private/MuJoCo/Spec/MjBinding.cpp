// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjBinding.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSceneSpec.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{
/**
 * How many objects of `ObjType` the model holds.
 *
 * MuJoCo publishes the counts as named size fields rather than as a table, so
 * this is the table.
 */
int32 ObjectCountOf(const mjModel* Model, int32 ObjType)
{
	switch (ObjType)
	{
		case mjOBJ_BODY:
			return Model->nbody;
		case mjOBJ_JOINT:
			return Model->njnt;
		case mjOBJ_GEOM:
			return Model->ngeom;
		case mjOBJ_SITE:
			return Model->nsite;
		case mjOBJ_CAMERA:
			return Model->ncam;
		case mjOBJ_LIGHT:
			return Model->nlight;
		case mjOBJ_FLEX:
			return Model->nflex;
		case mjOBJ_MESH:
			return Model->nmesh;
		case mjOBJ_SKIN:
			return Model->nskin;
		case mjOBJ_HFIELD:
			return Model->nhfield;
		case mjOBJ_TEXTURE:
			return Model->ntex;
		case mjOBJ_MATERIAL:
			return Model->nmat;
		case mjOBJ_PAIR:
			return Model->npair;
		case mjOBJ_EXCLUDE:
			return Model->nexclude;
		case mjOBJ_EQUALITY:
			return Model->neq;
		case mjOBJ_TENDON:
			return Model->ntendon;
		case mjOBJ_ACTUATOR:
			return Model->nu;
		case mjOBJ_SENSOR:
			return Model->nsensor;
		case mjOBJ_NUMERIC:
			return Model->nnumeric;
		case mjOBJ_TEXT:
			return Model->ntext;
		case mjOBJ_TUPLE:
			return Model->ntuple;
		case mjOBJ_KEY:
			return Model->nkey;
		case mjOBJ_PLUGIN:
			return Model->nplugin;
		default:
			return 0;
	}
}

/** An id of the expected family, or nothing. Guards every address lookup below. */
TOptional<int32> IdOfType(const FMjBinding& Binding, const UMjNodeComponent& Node, int32 ObjType)
{
	if (Binding.ObjTypeOf(Node) != ObjType)
	{
		return TOptional<int32>();
	}
	return Binding.Id(Node);
}
} // namespace

void FMjBinding::Add(FEntry Entry)
{
	const int32 Index = Entries.Num();
	ByNode.Add(Entry.Node, Index);
	Entries.Add(MoveTemp(Entry));
}

const FMjBinding::FEntry* FMjBinding::Find(const UMjNodeComponent& Node) const
{
	const int32* Index = ByNode.Find(&Node);
	return Index != nullptr ? &Entries[*Index] : nullptr;
}

TOptional<int32> FMjBinding::Id(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	if (Entry == nullptr || Entry->Id < 0)
	{
		return TOptional<int32>();
	}
	return Entry->Id;
}

int32 FMjBinding::ObjTypeOf(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	return Entry != nullptr ? Entry->ObjType : mjOBJ_UNKNOWN;
}

FString FMjBinding::NameOf(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	return Entry != nullptr ? Entry->Name : FString();
}

TOptional<int32> FMjBinding::QposAdr(const UMjNodeComponent& Joint) const
{
	const TOptional<int32> JointId = IdOfType(*this, Joint, mjOBJ_JOINT);
	if (!JointId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->jnt_qposadr[*JointId];
}

TOptional<int32> FMjBinding::DofAdr(const UMjNodeComponent& Joint) const
{
	const TOptional<int32> JointId = IdOfType(*this, Joint, mjOBJ_JOINT);
	if (!JointId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->jnt_dofadr[*JointId];
}

TOptional<int32> FMjBinding::SensorAdr(const UMjNodeComponent& Sensor) const
{
	const TOptional<int32> SensorId = IdOfType(*this, Sensor, mjOBJ_SENSOR);
	if (!SensorId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->sensor_adr[*SensorId];
}

TOptional<int32> FMjBinding::ActAdr(const UMjNodeComponent& Actuator) const
{
	const TOptional<int32> ActuatorId = IdOfType(*this, Actuator, mjOBJ_ACTUATOR);
	if (!ActuatorId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	const int32 Address = Model->actuator_actadr[*ActuatorId];
	if (Address < 0)
	{
		return TOptional<int32>();
	}
	return Address;
}

TArray<int32> FMjBinding::Find(int32 ObjType, const FString& Glob) const
{
	TArray<int32> Out;
	if (Model == nullptr)
	{
		return Out;
	}
	const int32 Count = ObjectCountOf(Model, ObjType);
	for (int32 Id = 0; Id < Count; ++Id)
	{
		const char* Name = mj_id2name(Model, ObjType, Id);
		if (Name != nullptr && FString(UTF8_TO_TCHAR(Name)).MatchesWildcard(Glob))
		{
			Out.Add(Id);
		}
	}
	return Out;
}

#if URLAB_MJ_GEN

FMjBinding MjBindingOf(const urlab::spec::FMjCompiledScene& Scene)
{
	FMjBinding Out;
	if (Scene.Model == nullptr)
	{
		return Out;
	}

	// Sorted rather than taken in map order, because the entries are read as a
	// sequence downstream -- an articulation's actuator ids come off this walk --
	// and a hash order is not a sequence anybody chose. Family then id is the
	// order the compiler laid the model out in.
	TArray<FMjBinding::FEntry> Entries;
	Entries.Reserve(Scene.BoundIds.Num());
	for (const TPair<TObjectPtr<const UMjNodeComponent>, urlab::spec::FMjBoundElement>& Bound : Scene.BoundIds)
	{
		UMjNodeComponent* const Node = const_cast<UMjNodeComponent*>(Bound.Key.Get());
		if (Node == nullptr || Bound.Value.Id < 0
			|| Bound.Value.Id >= ObjectCountOf(Scene.Model, Bound.Value.ObjType))
		{
			continue;
		}
		Node->EnsureSerial();

		FMjBinding::FEntry Entry;
		Entry.Node = Node;
		Entry.ObjType = Bound.Value.ObjType;
		Entry.Id = Bound.Value.Id;
		const char* const Name = mj_id2name(Scene.Model, Bound.Value.ObjType, Bound.Value.Id);
		Entry.Name = Name != nullptr ? FString(UTF8_TO_TCHAR(Name)) : FString();
		Entry.Serial = Node->Serial;
		Entries.Add(MoveTemp(Entry));
	}
	Entries.Sort([](const FMjBinding::FEntry& A, const FMjBinding::FEntry& B) {
		return A.ObjType != B.ObjType ? A.ObjType < B.ObjType : A.Id < B.Id;
	});

	for (FMjBinding::FEntry& Entry : Entries)
	{
		Out.Add(MoveTemp(Entry));
	}
	Out.SetModel(Scene.Model);
	return Out;
}

#endif // URLAB_MJ_GEN
