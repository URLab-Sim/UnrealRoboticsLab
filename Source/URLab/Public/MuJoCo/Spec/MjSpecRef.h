// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// A non-owning handle on one MuJoCo spec.
//
// There is no spec object. A spec is a component tree plus the graph it
// lives in, and this is the pair. It owns nothing, copies freely, and is the
// argument every operation that spans a whole spec takes: write to MJCF,
// name lookups behind the details panel's dropdowns, and the per-articulation
// entry into a scene assembly.
//
// The graph is not inferred per call. Which of the two tree adapters applies is
// a property of where the tree lives, decided once when the handle is made.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

class AActor;
class UActorComponent;
class UBlueprint;
class UMjNodeComponent;

/** Which Unreal object graph holds a spec's components. */
UENUM()
enum class EMjSpecGraph : uint8
{
	/** A spawned actor's attachment hierarchy. */
	Instance,
	/** A Blueprint's construction-script template graph. */
	Scs,
};

/** Whether a diagnostic stopped the work or only described it. */
UENUM()
enum class EMjDiagnosticSeverity : uint8
{
	/** Nothing usable came of it. */
	Error,
	/**
	 * The work carried on, and the user is being told something about it.
	 *
	 * A build reports both into one array, so a caller cannot decide "did this
	 * work" by whether the array is empty; it asks the entries.
	 */
	Warning,
};

/** One reader or writer diagnostic, flattened out of ProtoSpec's own. */
struct URLAB_API FMjSpecDiagnostic
{
	FString Message;
	FString File;
	int32 Line = 0;
	bool bUnsupportedElement = false;

	/**
	 * Error unless something says otherwise.
	 *
	 * Every construction site that predates this field records a failure, so the
	 * default keeps their meaning without touching them. The scene builder's own
	 * `Warnings` and `Infos` arrays classify by which array an entry is in and do
	 * not read this; unifying the two conventions is a change to that builder.
	 */
	EMjDiagnosticSeverity Severity = EMjDiagnosticSeverity::Error;

	bool IsError() const { return Severity == EMjDiagnosticSeverity::Error; }

	FString ToString() const;
};

/** True when any of `Diagnostics` is a failure rather than a remark. */
inline bool MjAnyError(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		if (Diagnostic.IsError())
		{
			return true;
		}
	}
	return false;
}

struct URLAB_API FSpecRef
{
	FSpecRef() = default;

	/** The spec rooted at `Actor`'s MuJoCo root component. */
	static FSpecRef OverActor(AActor& Actor);

	/** The spec held as `Blueprint`'s construction-script templates. */
	static FSpecRef OverBlueprint(UBlueprint& Blueprint);

	/**
	 * The spec `Component` belongs to, whichever graph that is.
	 *
	 * This is what the generated `GetOptions` dropdowns call: a details panel row
	 * knows only the component it is editing and has to find the rest of the
	 * spec from there.
	 */
	static FSpecRef OverOwner(const UActorComponent* Component);

	bool IsValid() const { return Root != nullptr; }
	EMjSpecGraph GetGraph() const { return Graph; }
	UMjNodeComponent* GetRoot() const { return Root; }
	UBlueprint* GetBlueprint() const { return Blueprint; }
	AActor* GetActor() const { return Actor; }

	/**
	 * Serialize the spec to canonical MJCF.
	 *
	 * Emits exactly the authored fields, so a fixpoint is reached by the second
	 * write. Empty on failure, with the reasons in `OutErrors`.
	 */
	FString WriteMjcf(TArray<FMjSpecDiagnostic>* OutErrors = nullptr) const;

	/**
	 * Serialize one element of the spec and its subtree, tag included.
	 *
	 * The tag is the element's own, so the result is a fragment rather than a
	 * document and a caller assembles whatever surrounds it. Same all-or-nothing
	 * contract as the whole-spec write: empty on failure, reasons in `OutErrors`,
	 * never a partial that would parse.
	 */
	FString WriteMjcfElement(const UMjNodeComponent& Node, TArray<FMjSpecDiagnostic>* OutErrors = nullptr) const;

	/** Every named element of the spec whose class is `Type`, in spec order. */
	TArray<FString> NamesOfType(const UClass* Type) const;

private:
	UMjNodeComponent* Root = nullptr;
	UBlueprint* Blueprint = nullptr;
	AActor* Actor = nullptr;
	EMjSpecGraph Graph = EMjSpecGraph::Instance;
};

/** The outcome of parsing MJCF into a component tree. */
struct URLAB_API FMjSpecParseResult
{
	/** The spec root, or null on failure. */
	UMjNodeComponent* Root = nullptr;
	TArray<FMjSpecDiagnostic> Errors;
	TArray<FMjSpecDiagnostic> Warnings;

	bool IsOk() const { return Root != nullptr && Errors.Num() == 0; }

	/**
	 * True when the spec is well formed but uses element families ProtoSpec
	 * does not model. A corpus harness skips such files rather than failing.
	 */
	bool IsUnsupportedOnly() const;
};

/** Parse-time options; `<include>` resolution is a security boundary. */
struct URLAB_API FMjDocParseOptions
{
	/**
	 * Allow an `<include>` whose resolved path escapes the root model's directory
	 * tree. Off by default: the reader opens untrusted MJCF inside a GUI host, and
	 * an unbounded include is exfiltration-shaped.
	 */
	bool bAllowExternalIncludes = false;
};

#if WITH_EDITOR
/**
 * Parse MJCF text into `Blueprint`'s construction-script templates.
 *
 * On success the Blueprint's SCS tree IS the spec. There is no second
 * artifact, no intermediate representation, and nothing to keep in step.
 */
URLAB_API FMjSpecParseResult MjParseIntoBlueprint(UBlueprint& Blueprint, const FString& Xml,
	const FString& Filename, const FMjDocParseOptions& Options = {});
#endif

/** Parse MJCF text into live components on `Actor`. Game thread. */
URLAB_API FMjSpecParseResult MjParseIntoActor(AActor& Actor, const FString& Xml, const FString& Filename,
	const FMjDocParseOptions& Options = {});
