// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/RenderBusTopics.h"
#include "Wire/UrlabWire.gen.h"

namespace URLabRenderBusTopic
{
// Both spellings come from wire/urlab_wire.yaml. Python builds its topics from
// the same emission, because two spellings of a topic is two buses -- and a
// topic name is exactly the kind of string that gets retyped rather than shared.
const TCHAR* Lean = URLabWire::RenderBusTopic::Lean;

namespace
{
const TCHAR* TieredRoot = URLabWire::RenderBusTopic::TieredRoot;
constexpr int32 TieredRootLen = URLabWire::RenderBusTopic::TieredRootLen;
} // namespace

FString ForCaps(bool bContacts, bool bOverlay, int32 MaxContacts)
{
	if (!bContacts && !bOverlay)
	{
		return Lean;
	}
	// The cap is only meaningful alongside contacts, and it is pinned to 0 when
	// contacts are off so two subscribers wanting the same thing land on the same
	// topic and the owner builds that frame once.
	const int32 Cap = bContacts ? FMath::Max(0, MaxContacts) : 0;
	return FString::Printf(TEXT("%sc%do%dm%d/"), TieredRoot,
		bContacts ? 1 : 0, bOverlay ? 1 : 0, Cap);
}

bool IsTiered(const FString& Topic)
{
	return Topic.StartsWith(TieredRoot, ESearchCase::CaseSensitive);
}

bool ParseCaps(const FString& Topic, bool& bOutContacts, bool& bOutOverlay,
	int32& OutMaxContacts)
{
	if (!IsTiered(Topic))
	{
		return false;
	}
	// "c<0|1>o<0|1>m<digits>/" -- fixed positions up to the cap, which is the only
	// variable-length part.
	const FString Body = Topic.Mid(TieredRootLen);
	if (Body.Len() < 7 || Body[0] != TCHAR('c') || Body[2] != TCHAR('o')
		|| Body[4] != TCHAR('m') || Body[Body.Len() - 1] != TCHAR('/'))
	{
		return false;
	}
	if ((Body[1] != TCHAR('0') && Body[1] != TCHAR('1'))
		|| (Body[3] != TCHAR('0') && Body[3] != TCHAR('1')))
	{
		return false;
	}
	// Digits only: FString::IsNumeric would accept "-1" and ".5", and a signed or
	// fractional cap is not a topic this publisher ever wrote.
	const FString Cap = Body.Mid(5, Body.Len() - 6);
	if (Cap.IsEmpty())
	{
		return false;
	}
	for (const TCHAR C : Cap)
	{
		if (C < TCHAR('0') || C > TCHAR('9'))
		{
			return false;
		}
	}
	bOutContacts = (Body[1] == TCHAR('1'));
	bOutOverlay = (Body[3] == TCHAR('1'));
	OutMaxContacts = FCString::Atoi(*Cap);
	return true;
}
} // namespace URLabRenderBusTopic
