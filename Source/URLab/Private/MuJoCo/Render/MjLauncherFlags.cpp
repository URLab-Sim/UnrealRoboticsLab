// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Render/MjLauncherFlags.h"

#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Utils/URLabLogging.h"
#include "Wire/UrlabWire.gen.h"

namespace URLabLauncherFlags
{
namespace
{
// Read the WHOLE value of a csv flag. FParse's separator-stop is disabled so a comma-delimited
// value ("a=1,b,c=2") is not truncated at the first comma. FlagName includes the trailing '='.
bool ReadFullFlagValue(const TCHAR* FlagName, FString& Out)
{
	return FParse::Value(FCommandLine::Get(), FlagName, Out, /*bShouldStopOnSeparator=*/false);
}

// Split a csv flag value into its comma tokens (whitespace-trimmed, empties dropped).
void SplitCsv(const FString& Value, TArray<FString>& OutTokens)
{
	Value.ParseIntoArray(OutTokens, TEXT(","), /*InCullEmpty=*/true);
	for (FString& Tok : OutTokens)
	{
		Tok.TrimStartAndEndInline();
	}
}
} // namespace

bool GetCsvValue(const TCHAR* FlagName, const TCHAR* Key, FString& OutValue)
{
	FString Value;
	if (!ReadFullFlagValue(FlagName, Value))
	{
		return false;
	}
	TArray<FString> Tokens;
	SplitCsv(Value, Tokens);
	const FString Prefix = FString(Key) + TEXT("=");
	for (const FString& Tok : Tokens)
	{
		if (Tok.StartsWith(Prefix, ESearchCase::IgnoreCase))
		{
			OutValue = Tok.Mid(Prefix.Len());
			return true;
		}
	}
	return false;
}

bool HasCsvToken(const TCHAR* FlagName, const TCHAR* Key)
{
	FString Value;
	if (!ReadFullFlagValue(FlagName, Value))
	{
		return false;
	}
	TArray<FString> Tokens;
	SplitCsv(Value, Tokens);
	for (const FString& Tok : Tokens)
	{
		if (Tok.Equals(Key, ESearchCase::IgnoreCase))
		{
			return true;
		}
	}
	return false;
}

// ---- -URLabWorkflow -----------------------------------------------------------------------------

EWorkflowKind ParseWorkflow(FString& OutValue)
{
	OutValue.Empty();
	FString Legacy;
	if (FParse::Value(FCommandLine::Get(), TEXT("URLabDrive="), Legacy, false))
	{
		OutValue=Legacy;
		return EWorkflowKind::Invalid;
	}
	FString Value;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabWorkflow="), Value, false))
	{
		return EWorkflowKind::None;
	}
	Value.TrimStartAndEndInline();
	OutValue=Value;
	if(Value.Equals(TEXT("simulation"),ESearchCase::IgnoreCase))return EWorkflowKind::Simulation;
	if(Value.Equals(TEXT("render"),ESearchCase::IgnoreCase))return EWorkflowKind::Render;
	if(Value.Equals(TEXT("viewer"),ESearchCase::IgnoreCase))return EWorkflowKind::Viewer;
	return EWorkflowKind::Invalid;
}

bool SimulationSourceManifest(FString& OutManifestPath)
{
	OutManifestPath.Reset();
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabSimulationSource="), OutManifestPath,
			/*bShouldStopOnSeparator=*/true))
	{
		return false;
	}
	OutManifestPath.TrimStartAndEndInline();
	return true;
}

// ---- -URLabClock ------------------------------------------------------------------------------

EClockKind ParseClock(FString& OutValue)
{
	OutValue.Reset();
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabClock="), OutValue,
			/*bShouldStopOnSeparator=*/true))
	{
		return EClockKind::Absent;
	}
	OutValue.TrimStartAndEndInline();
	if (OutValue.Equals(TEXT("stepped"), ESearchCase::IgnoreCase))
	{
		return EClockKind::Stepped;
	}
	if (OutValue.Equals(TEXT("freerun"), ESearchCase::IgnoreCase))
	{
		return EClockKind::FreeRun;
	}
	return EClockKind::Invalid;
}

// ---- -URLabCaps --------------------------------------------------------------------------------

FMjCapabilitySet ParseCaps()
{
	FMjCapabilitySet Caps;
	FString Value;
	if (!ReadFullFlagValue(TEXT("URLabCaps="), Value))
	{
		return Caps;
	}
	TArray<FString> Tokens;
	SplitCsv(Value, Tokens);
	for (const FString& Tok : Tokens)
	{
		// A leading '-' negates the capability (e.g. "-accept_input"); anything else enables it.
		bool bEnable = true;
		FString Name = Tok;
		if (Name.StartsWith(TEXT("-")))
		{
			bEnable = false;
			Name = Name.Mid(1);
		}
		Name.TrimStartAndEndInline();

		EMjCapability Cap = EMjCapability::Rpc;
		if (MjCapabilityFromToken(Name, Cap))
		{
			Caps.Set(Cap, bEnable);
		}
		else
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[MjLauncherFlags] -URLabCaps: no capability is called '%s'; ignoring it"),
				*Name);
		}
	}
	return Caps;
}

bool RetiredSourceFindValue(FString& OutValue)
{
	OutValue.Reset();
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabSourceFind="), OutValue,
		/*bShouldStopOnSeparator=*/false))
	{
		return false;
	}
	OutValue.TrimStartAndEndInline();
	return true;
}

// ---- -URLabScene -------------------------------------------------------------------------------

bool SceneOrigin(FVector& OutOrigin)
{
	FString Val;
	if (!GetCsvValue(TEXT("URLabScene="), TEXT("origin"), Val))
	{
		return false;
	}
	TArray<FString> Parts;
	Val.ParseIntoArray(Parts, TEXT(";"), /*InCullEmpty=*/true);
	if (Parts.Num() != 3)
	{
		return false;
	}
	OutOrigin = FVector(
		FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2]));
	return true;
}

bool SceneLevel(FString& OutLevel)
{
	return GetCsvValue(TEXT("URLabScene="), TEXT("level"), OutLevel);
}

bool SceneBaseLevel()
{
	return HasCsvToken(TEXT("URLabScene="), TEXT("base"));
}

bool SceneNoQuality()
{
	FString Val;
	return GetCsvValue(TEXT("URLabScene="), TEXT("quality"), Val) && Val.Equals(TEXT("off"), ESearchCase::IgnoreCase);
}

bool SceneCamMaxHeight(int32& OutPx)
{
	FString Val;
	if (!GetCsvValue(TEXT("URLabScene="), TEXT("cammax"), Val) || !Val.IsNumeric())
	{
		return false;
	}
	OutPx = FCString::Atoi(*Val);
	return true;
}

bool SceneCamNearClipCm(float& OutCm)
{
	FString Val;
	if (!GetCsvValue(TEXT("URLabScene="), TEXT("camnear"), Val))
	{
		return false;
	}
	OutCm = FCString::Atof(*Val);
	return true;
}

bool SceneNoAutoExposure()
{
	return HasCsvToken(TEXT("URLabScene="), TEXT("noexposure"));
}

bool SceneOverlayMask(int32& OutMask)
{
	FString Val;
	if (!GetCsvValue(TEXT("URLabScene="), TEXT("overlay"), Val))
	{
		return false;
	}
	Val.TrimStartAndEndInline();
	// Accept a decimal or 0x-prefixed hex bitmask; anything unparsable => not set.
	if (Val.StartsWith(TEXT("0x"), ESearchCase::IgnoreCase))
	{
		OutMask = static_cast<int32>(FCString::Strtoi(*Val, nullptr, 16));
		return true;
	}
	if (!Val.IsNumeric())
	{
		return false;
	}
	OutMask = FCString::Atoi(*Val);
	return true;
}

bool SceneOverlayMaxContacts(int32& OutMax)
{
	FString Val;
	if (!GetCsvValue(TEXT("URLabScene="), TEXT("maxcontacts"), Val) || !Val.IsNumeric())
	{
		return false;
	}
	OutMax = FCString::Atoi(*Val);
	return true;
}

// ---- -URLabNet ---------------------------------------------------------------------------------

bool NetValue(const TCHAR* Key, FString& OutValue)
{
	return GetCsvValue(TEXT("URLabNet="), Key, OutValue);
}

bool DmEnvPort(int32& OutPort)
{
	int32 Port = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("URLabDmEnvPort="), Port) && Port > 0)
	{
		OutPort = Port;
		return true;
	}
	return false;
}

void EnsureDmEnvPortToken(int32 Port)
{
	FString Existing;
	if (FParse::Value(FCommandLine::Get(), TEXT("URLabDmEnvPort="), Existing))
	{
		return;   // already carried; appending again would leave two tokens to disagree
	}
	FCommandLine::Append(*FString::Printf(TEXT(" -URLabDmEnvPort=%d"), Port));
}

// ---- test support ------------------------------------------------------------------------------

FScopedCommandLine::FScopedCommandLine(const TCHAR* Line)
{
	Saved = FCommandLine::Get();
	FCommandLine::Set(Line);
}

FScopedCommandLine::~FScopedCommandLine()
{
	FCommandLine::Set(*Saved);
}
} // namespace URLabLauncherFlags
