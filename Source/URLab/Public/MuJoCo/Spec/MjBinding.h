// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Which compiled-model id each spec element received, and the addresses that
// follow from it.
//
// A compile records the correspondence while it is known; this is the shape it
// is read back through. Elements the compiler removed -- discardvisual and
// fusestatic both remove some -- simply report nothing and stay as authoring
// data, because an id into a table that does not hold the element is worse than
// no id at all.

#include "CoreMinimal.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

class UMjNodeComponent;

namespace urlab::spec
{
struct FMjCompiledScene;
}

/**
 * Which compiled-model id each spec element received.
 *
 * A snapshot of one compile. Value edits never invalidate it; structural edits
 * -- adding, removing or reordering elements -- always do, and the remedy is a
 * fresh compile rather than a repair.
 */
struct URLAB_API FMjBinding
{
	/** The compiled id of `Node`, or unset when it did not survive the compile. */
	TOptional<int32> Id(const UMjNodeComponent& Node) const;

	/** The `mjtObj` family `Node` compiled into, or `mjOBJ_UNKNOWN`. */
	int32 ObjTypeOf(const UMjNodeComponent& Node) const;

	/** The name `Node` compiled under, authored or reserved. Empty when unbound. */
	FString NameOf(const UMjNodeComponent& Node) const;

	// --- Address sugar ------------------------------------------------------ //
	//
	// The addresses a caller reading mjData needs, resolved once here rather than
	// at every read site.

	/** `jnt_qposadr` of a joint element. */
	TOptional<int32> QposAdr(const UMjNodeComponent& Joint) const;

	/** `jnt_dofadr` of a joint element. */
	TOptional<int32> DofAdr(const UMjNodeComponent& Joint) const;

	/** `sensor_adr` of a sensor element. */
	TOptional<int32> SensorAdr(const UMjNodeComponent& Sensor) const;

	/** `actuator_actadr` of an actuator element; unset when it has no activation. */
	TOptional<int32> ActAdr(const UMjNodeComponent& Actuator) const;

	/**
	 * Every id of `ObjType` whose name matches `Glob` (`*` and `?`).
	 *
	 * The only way to reach elements MuJoCo generated rather than the spec:
	 * composite and flexcomp expansions exist in the model and not in the tree.
	 */
	TArray<int32> Find(int32 ObjType, const FString& Glob) const;

	/** Every element that took part in the compile, whether or not it bound. */
	struct FEntry
	{
		const UMjNodeComponent* Node = nullptr;
		FString Name;
		int32 ObjType = 0;
		int32 Id = -1;

		/**
		 * The node's creation serial: the identity that survives a recompile.
		 *
		 * `Node` would answer the same question for a spec edited in place,
		 * but a serial is minted once and never reissued, so it cannot be
		 * confused by an address a deleted element's allocation gave back. It is
		 * what state migration keys on.
		 */
		uint64 Serial = 0;
	};
	const TArray<FEntry>& GetEntries() const { return Entries; }

	void Add(FEntry Entry);
	void SetModel(const mjModel* InModel) { Model = InModel; }

private:
	const FEntry* Find(const UMjNodeComponent& Node) const;

	TArray<FEntry> Entries;
	TMap<const UMjNodeComponent*, int32> ByNode;
	const mjModel* Model = nullptr;
};

/**
 * The engine-facing binding of a scene the spec path compiled.
 *
 * The ids are the ones the compile recorded against the elements themselves, so
 * nothing here searches the model: names are read back out of it with
 * `mj_id2name` rather than used to find anything. Elements of a family the model
 * holds no objects of -- a `<default>` class, which exists in the spec and not
 * in the model -- are left out, because an id into a table that does not exist
 * is worse than no id at all.
 */
URLAB_API FMjBinding MjBindingOf(const urlab::spec::FMjCompiledScene& Scene);
