// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "MuJoCo/Core/MjCapabilities.h"

/**
 * Cross-module registry for RPC handlers. Each handler is registered
 * with an FOpDecl — a small metadata block — so the dispatcher can
 * categorise typed editor handlers without a hardcoded list.
 *
 * URLabEditor's StartupModule registers editor-only handlers; the
 * typed dispatcher looks up editor handlers by name and routes.
 *
 * Cooked builds: editor-only handlers don't register, every editor-only
 * op replies `not_in_editor` because the registry has no entry.
 */
class FURLabRpcDispatcher;
namespace URLabOpRegistry
{
/** What an op needs before it can run. Each name states the requirement the
 *  dispatcher actually checks -- a category that named something else (the old
 *  `ManagerRequired` had become shorthand for "is there a bridge") refused ops
 *  in exactly the modes a render farm runs in.
 *   - EditorOnly: requires WITH_EDITOR; replies `not_in_editor` otherwise.
 *   - SimRequired: needs a live simulation -- an AAMjManager with its physics
 *     engine, entities and cameras. Replies `no_simulation`.
 *   - WorldRequired: needs a live world to act in (a renderer's scene, the
 *     replay actor, the appearance and light stores). No physics involved, so a
 *     render server with no manager serves these. Replies `no_world`.
 *   - BridgeOnly: needs nothing but the bridge process -- the handshake, the
 *     farm lease, blob staging. Always available. */
enum class EOpCategory : uint8
{
	EditorOnly,
	SimRequired,
	WorldRequired,
	BridgeOnly
};

using FHandler = TFunction<TSharedPtr<FJsonObject>(
	FURLabRpcDispatcher&, const TSharedPtr<FJsonObject>&)>;

/** One registered op. `Body` is the handler; `Category` is consulted
 *  by the typed editor adapter; `Namespace` drives Python-side method routing
 *  (`URLabClient.<namespace>.<name>`). `RequiredFields` is checked
 *  declaratively before the
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
	EOpCategory Category = EOpCategory::BridgeOnly;
	FString Namespace; // empty = top-level on URLabClient
	FHandler Body;
	TArray<FString> RequiredFields;
	TArray<FString> ReplyFields;

	/** The permission a peer must hold before this op runs, unset when it needs none.
	 *  Read from the generated table rather than typed here, and enforced once in
	 *  the typed dispatcher: a check written inside a handler is one
	 *  no caller can read and one the next handler forgets. */
	TOptional<EMjCapability> Permission;

	/** True when the two lists above came from wire/urlab_wire.yaml via
	 *  UrlabWire.gen.h rather than from literals at the registration site.
	 *  A hand-typed list is a second copy of the schema that nothing compares,
	 *  and it rots silently; this flag is what makes the remaining hand-typed
	 *  ops countable instead of invisible. */
	bool bFromSchema = false;
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

/** The permission `OpName` declares in wire/urlab_wire.yaml, or unset for the declared
 *  default of none. Answers for EVERY op, registered or not, because the pre-session ops
 *  (the render family) are gated before the registry is consulted at all. */
URLAB_API TOptional<EMjCapability> PermissionForOp(const FString& OpName);
} // namespace URLabOpRegistry
