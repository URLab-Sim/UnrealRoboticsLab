// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/OpRegistry.h"

#include "Misc/ScopeRWLock.h"

namespace
{
// Meyers singleton — constructed on first use regardless of TU init
// order. Replacing this with a non-Meyers static loses early
// registrations.
TMap<FString, URLabOpRegistry::FOpDecl>& Map()
{
	static TMap<FString, URLabOpRegistry::FOpDecl> M;
	return M;
}

// Guards Map() against concurrent access. Mutations (Register /
// Unregister) happen on the game thread; FindOp / GetAllOps are
// called from RPC worker threads via the dispatcher.
FRWLock& MapLock()
{
	static FRWLock L;
	return L;
}
} // namespace

namespace URLabOpRegistry
{
void RegisterOp(FOpDecl Decl)
{
	const FString Name = Decl.Name;
	FRWScopeLock Lock(MapLock(), SLT_Write);
	Map().Add(Name, MoveTemp(Decl));
}

void RegisterHandler(const FString& OpName, FHandler Fn)
{
	FOpDecl Decl;
	Decl.Name = OpName;
	Decl.Category = EOpCategory::EditorOnly;
	Decl.Body = MoveTemp(Fn);
	FRWScopeLock Lock(MapLock(), SLT_Write);
	Map().Add(OpName, MoveTemp(Decl));
}

void UnregisterHandler(const FString& OpName)
{
	FRWScopeLock Lock(MapLock(), SLT_Write);
	Map().Remove(OpName);
}

FHandler GetHandler(const FString& OpName)
{
	FRWScopeLock Lock(MapLock(), SLT_ReadOnly);
	if (const FOpDecl* D = Map().Find(OpName))
		return D->Body;
	return FHandler();
}

TOptional<FOpDecl> FindOp(const FString& OpName)
{
	FRWScopeLock Lock(MapLock(), SLT_ReadOnly);
	if (const FOpDecl* D = Map().Find(OpName))
	{
		// Copy under the read lock so callers hold a value, not a
		// pointer into a map that may rehash on the next register.
		return *D;
	}
	return TOptional<FOpDecl>();
}

TArray<FOpDecl> GetAllOps()
{
	FRWScopeLock Lock(MapLock(), SLT_ReadOnly);
	TArray<FOpDecl> Out;
	Out.Reserve(Map().Num());
	for (const TPair<FString, FOpDecl>& Kv : Map())
	{
		Out.Add(Kv.Value);
	}
	return Out;
}

TOptional<EMjCapability> PermissionForOp(const FString& OpName)
{
	for (int32 i = 0; i < URLabWire::OpPermission::Count; ++i)
	{
		if (OpName.Equals(URLabWire::OpPermission::All[i].Op))
		{
			EMjCapability Cap = EMjCapability::Rpc;
			if (MjCapabilityFromToken(URLabWire::OpPermission::All[i].Permission, Cap))
			{
				return Cap;
			}
		}
	}
	return TOptional<EMjCapability>();
}

bool IsEditorOnlyOp(const FString& OpName)
{
	FRWScopeLock Lock(MapLock(), SLT_ReadOnly);
	const FOpDecl* D = Map().Find(OpName);
	return D && D->Category == EOpCategory::EditorOnly;
}
} // namespace URLabOpRegistry
