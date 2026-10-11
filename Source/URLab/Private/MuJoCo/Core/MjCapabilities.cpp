// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Core/MjCapabilities.h"

bool MjCapabilityFromToken(const FString& Token, EMjCapability& OutCap)
{
	for (int32 i = 0; i < URLabWire::Capability::Count; ++i)
	{
		const TCHAR* Cli = URLabWire::Capability::All[i].Cli;
		if (Cli != nullptr && Token.Equals(Cli, ESearchCase::IgnoreCase))
		{
			OutCap = static_cast<EMjCapability>(i);
			return true;
		}
	}
	return false;
}

void FMjCapabilitySet::CollectWireTokens(URLabWire::Capability::EKind Kind,
	TArray<FString>& Out) const
{
	for (int32 i = 0; i < URLabWire::Capability::Count; ++i)
	{
		const URLabWire::Capability::FEntry& E = URLabWire::Capability::All[i];
		if (E.Kind == Kind && E.Wire != nullptr && Has(static_cast<EMjCapability>(i)))
		{
			Out.Add(E.Wire);
		}
	}
}
