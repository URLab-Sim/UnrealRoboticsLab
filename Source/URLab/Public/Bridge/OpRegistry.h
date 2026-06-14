// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/**
 * Cross-module registry for RPC handlers. Each handler is registered
 * with an FOpDecl — a small metadata block — so the dispatcher can
 * categorise ops without a hardcoded list, and clients can introspect
 * the schema at runtime via the `meta` op.
 *
 * URLabEditor's StartupModule registers editor-only handlers; the
 * dispatcher's static block registers manager-required ones.
 * `FURLabRpcDispatcher::Dispatch` looks up an FOpDecl by name and routes.
 *
 * Cooked builds: editor-only handlers don't register, every editor-only
 * op replies `not_in_editor` because the registry has no entry.
 */
namespace URLabOpRegistry
{
/** Op category. Drives dispatcher routing decisions:
 *   - EditorOnly: requires WITH_EDITOR; replies `not_in_editor` otherwise.
 *   - ManagerRequired: needs an active AAMjManager (PIE running).
 *   - NoManager: always available (hello, set_mode, etc). */
enum class EOpCategory : uint8
{
	EditorOnly,
	ManagerRequired,
	NoManager
};

using FHandler = TFunction<TSharedPtr<FJsonObject>(const TSharedPtr<FJsonObject>&)>;

/** One registered op. `Body` is the handler; `Category` is consulted
 *  by the dispatcher; `Namespace` drives Python-side method routing
 *  (`URLabClient.<namespace>.<name>`). `RequiredFields` is checked
 *  declaratively in `FURLabRpcDispatcher::Dispatch` before the
 *  handler runs — missing fields fail with `missing_field`.
 *
 *  `ReplyFields` carries one entry per field the *_ok reply emits.
 *  Encoding: "name:type" with an optional "?" suffix for fields
 *  that may be absent. Types are wire-shape names: `string`, `int`,
 *  `float`, `bool`, `array`, `object`. The Python stub generator
 *  uses this to emit return-type signatures. */
struct FOpDecl
{
	FString Name;
	EOpCategory Category = EOpCategory::NoManager;
	FString Namespace; // empty = top-level on URLabClient
	FHandler Body;
	TArray<FString> RequiredFields;
	TArray<FString> ReplyFields;
};

/** Declarative form: register an op with metadata. */
URLAB_API void RegisterOp(FOpDecl Decl);

/** Register a handler with no metadata. Equivalent to RegisterOp
 *  with `Category=EditorOnly`. Prefer RegisterOp for new code. */
URLAB_API void RegisterHandler(const FString& OpName, FHandler Fn);

/** Remove a registered op (any category). */
URLAB_API void UnregisterHandler(const FString& OpName);

/** Returns the registered handler body or an empty TFunction. */
URLAB_API FHandler GetHandler(const FString& OpName);

/** Snapshot a registered op by name. Returns a value copy under a
 *  reader lock so callers can safely invoke the handler even if the
 *  registry is mutated concurrently. */
URLAB_API TOptional<FOpDecl> FindOp(const FString& OpName);

/** Snapshot of every registered op. Used by the `meta` op to ship
 *  the schema to the bridge for runtime method synthesis. */
URLAB_API TArray<FOpDecl> GetAllOps();

/** True if the registered op (if any) is categorised as
 *  EditorOnly. Drives the dispatcher's `not_in_editor` branch. */
URLAB_API bool IsEditorOnlyOp(const FString& OpName);
} // namespace URLabOpRegistry
