// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Compiling a spec tree to an mjModel, and recovering the ids it was given.
//
// The route is MJCF text: the writer emits the spec, the assets it names go
// into an in-memory VFS beside it, and MuJoCo's own mj_loadXML does the rest.
// Nothing here reimplements a compiler, and nothing here has an opinion about
// what MJCF means -- which is the whole reason the differential gate can be
// byte-exact against a stock load of the same file.
//
// Binding is by name, because ids cannot be predicted: discardvisual and
// fusestatic compact them, so the element at index 3 before the compile is not
// the element at index 3 after it. Elements the compiler removed simply report
// nothing and stay as authoring data. An element the spec left unnamed is
// given a reserved name in the emitted text ONLY -- the tree is handed back
// exactly as it was -- so that every element is reachable without authoring
// names nobody asked for.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjSceneAssembly.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

class UMjNodeComponent;

/** How a compile is performed. */
struct URLAB_API FMjCompileOptions
{
	/**
	 * Give unnamed bindable elements a reserved name in the emitted MJCF.
	 *
	 * Off means unnamed elements compile as they were authored and are simply
	 * not bindable, which is what a caller wanting a pristine name table asks
	 * for. The name is derived from the element's serial, so it is stable across
	 * edits that do not change identity.
	 */
	bool bAutoName = true;

	/** The prefix reserved names carry. Shared with ProtoSpec's own default. */
	FString AutoNamePrefix = TEXT("_ps:");
};

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
 * One compile: the model, the ids, and the text it all came from.
 *
 * Owns the model. Move-only, because two owners of one mjModel is a double free
 * waiting for a bad day.
 */
struct URLAB_API FMjCompiled
{
	FMjCompiled() = default;
	~FMjCompiled();
	FMjCompiled(FMjCompiled&& Other);
	FMjCompiled& operator=(FMjCompiled&& Other);
	FMjCompiled(const FMjCompiled&) = delete;
	FMjCompiled& operator=(const FMjCompiled&) = delete;

	mjModel* Model = nullptr;
	FMjBinding Binding;

	/** The MJCF handed to MuJoCo. The root spec when a scene was compiled. */
	FString Xml;

	/** ParticipantXml the root references, keyed by the VFS name it references them by. */
	TMap<FString, FString> ParticipantXml;

	TArray<FMjSpecDiagnostic> Errors;

	bool IsOk() const { return Model != nullptr && Errors.Num() == 0; }

	/** Hand the model to the caller and stop owning it. */
	mjModel* Release();
};

/** Compile one spec on its own: no attach, no scene, no prefix. */
URLAB_API FMjCompiled MjCompileSpec(const FSpecRef& Spec, const FMjCompileOptions& Options = {});

/**
 * Compile a scene: the root spec's sections plus one `<attach>` per
 * participant, exactly as `FSceneAssembly` projects it.
 */
URLAB_API FMjCompiled MjCompileScene(const FSceneAssembly& Scene, const FMjCompileOptions& Options = {});
