// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "State/MjCanonicalName.h"
#include "MuJoCo/Core/MjArticulation.h"

FString FMjCanonicalName::Sanitize(const FString& Segment)
{
	if (Segment.IsEmpty())
		return Segment;

	FString Out;
	Out.Reserve(Segment.Len() + 1);
	for (TCHAR C : Segment)
	{
		const bool bLegal = (C >= TEXT('A') && C <= TEXT('Z')) || (C >= TEXT('a') && C <= TEXT('z'))
						 || (C >= TEXT('0') && C <= TEXT('9')) || C == TEXT('_');
		Out.AppendChar(bLegal ? C : TEXT('_'));
	}
	if (Out[0] >= TEXT('0') && Out[0] <= TEXT('9'))
		Out = FString(TEXT("_")) + Out;
	return Out;
}

FName FMjCanonicalName::ArtSegment(const AMjArticulation* Art)
{
	if (!Art)
		return FName();
	// The art's public identity for topics, tf frames, and control-ownership keys.
	// Prefer the stable, user-supplied ActorId ("franka") over the UE object name,
	// which is auto-generated and regenerated per spawn (panda_C_UAID_...); the
	// latter makes ROS topic/frame names unstable across runs. GetName() is the
	// fallback for arts spawned without an ActorId (e.g. placed in-editor).
	const FString Public = Art->ActorId.IsEmpty() ? Art->GetName() : Art->ActorId;
	return FName(*Sanitize(Public));
}

FName FMjCanonicalName::PartSegment(const AMjArticulation* Art, const FString& MjName)
{
	// Child mj names are compiled with the UE object-name prefix (not ActorId), so
	// prefix stripping stays keyed on GetName() even though ArtSegment is ActorId.
	FString Local = MjName;
	if (Art)
	{
		const FString Prefix = Art->GetName() + TEXT("_");
		if (Local.StartsWith(Prefix))
			Local = Local.Mid(Prefix.Len());
	}
	return FName(*Sanitize(Local));
}

FString FMjCanonicalName::Full(FName Art, FName Part)
{
	return FString::Printf(TEXT("%s/%s"), *Art.ToString(), *Part.ToString());
}
