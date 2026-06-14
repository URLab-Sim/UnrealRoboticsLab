// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

class UURLabBridgeServer;

/**
 * Cross-module hook so URLab (runtime) can ask URLabEditor (editor-only)
 * for an existing bridge server without taking a hard dependency on the
 * editor module. URLabEditor's StartupModule installs the resolver;
 * URLab's AAMjManager calls ResolveEditorServer() at BeginPlay.
 *
 * Cooked builds: the resolver is never installed, ResolveEditorServer
 * returns nullptr, AAMjManager falls through to creating its own server.
 */
namespace URLabBridgeProvider
{
using FResolverFn = TFunction<UURLabBridgeServer*()>;

/** Install a resolver. Replaces any previous one. Pass nullptr to clear. */
URLAB_API void RegisterResolver(FResolverFn Fn);

/** Returns the editor server if a resolver is installed and finds one,
 *  else nullptr. */
URLAB_API UURLabBridgeServer* ResolveEditorServer();
} // namespace URLabBridgeProvider
