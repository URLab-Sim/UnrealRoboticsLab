// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServerProvider.h"

namespace
{
URLabBridgeProvider::FResolverFn GResolver;
}

namespace URLabBridgeProvider
{
void RegisterResolver(FResolverFn Fn)
{
	GResolver = MoveTemp(Fn);
}

UURLabBridgeServer* ResolveEditorServer()
{
	return GResolver ? GResolver() : nullptr;
}
} // namespace URLabBridgeProvider
