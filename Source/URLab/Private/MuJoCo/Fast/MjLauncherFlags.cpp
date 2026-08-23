// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjLauncherFlags.h"

#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

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

// ---- -URLabDrive -------------------------------------------------------------------------------

EDriveKind ParseDrive(FString& OutStreamEndpoint)
{
	OutStreamEndpoint.Empty();
	FString Value;
	// Drive endpoints can carry a scheme with no spaces; keep the whole token.
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabDrive="), Value, /*bShouldStopOnSeparator=*/false))
	{
		return EDriveKind::None;
	}
	Value.TrimStartAndEndInline();
	if (Value.Equals(TEXT("sim"), ESearchCase::IgnoreCase))
	{
		return EDriveKind::Sim;
	}
	if (Value.Equals(TEXT("push"), ESearchCase::IgnoreCase))
	{
		return EDriveKind::Push;
	}
	if (Value.Equals(TEXT("await"), ESearchCase::IgnoreCase))
	{
		return EDriveKind::Await;
	}
	if (Value.StartsWith(TEXT("stream:"), ESearchCase::IgnoreCase))
	{
		OutStreamEndpoint = Value.Mid(FCString::Strlen(TEXT("stream:")));
		return EDriveKind::Stream;
	}
	return EDriveKind::None;
}

bool DriveIsSim()
{
	FString Ep;
	return ParseDrive(Ep) == EDriveKind::Sim;
}

bool DriveIsPush()
{
	FString Ep;
	return ParseDrive(Ep) == EDriveKind::Push;
}

bool DriveIsAwait()
{
	FString Ep;
	return ParseDrive(Ep) == EDriveKind::Await;
}

bool DriveStreamEndpoint(FString& OutEndpoint)
{
	FString Ep;
	if (ParseDrive(Ep) == EDriveKind::Stream && !Ep.IsEmpty())
	{
		OutEndpoint = Ep;
		return true;
	}
	return false;
}

bool DriveStreamTcpEndpoint(FString& OutEndpoint)
{
	FString Ep;
	if (DriveStreamEndpoint(Ep) && Ep.StartsWith(TEXT("tcp://"), ESearchCase::IgnoreCase))
	{
		OutEndpoint = Ep;
		return true;
	}
	return false;
}

bool DriveStreamGrpcEndpoint(FString& OutEndpoint)
{
	FString Ep;
	if (DriveStreamEndpoint(Ep) && Ep.StartsWith(TEXT("grpc://"), ESearchCase::IgnoreCase))
	{
		OutEndpoint = Ep;
		return true;
	}
	return false;
}

// ---- -URLabCaps --------------------------------------------------------------------------------

FCaps ParseCaps()
{
	FCaps Caps;
	FString Value;
	if (!ReadFullFlagValue(TEXT("URLabCaps="), Value))
	{
		return Caps;
	}
	TArray<FString> Tokens;
	SplitCsv(Value, Tokens);
	for (const FString& Tok : Tokens)
	{
		// A leading '-' negates the capability (e.g. "-input"); anything else enables it.
		bool bEnable = true;
		FString Name = Tok;
		if (Name.StartsWith(TEXT("-")))
		{
			bEnable = false;
			Name = Name.Mid(1);
		}
		Name.TrimStartAndEndInline();

		if (Name.Equals(TEXT("serve"), ESearchCase::IgnoreCase))
		{
			Caps.bServe = bEnable;
		}
		else if (Name.Equals(TEXT("publish"), ESearchCase::IgnoreCase))
		{
			Caps.bPublish = bEnable;
		}
		else if (Name.Equals(TEXT("cameras"), ESearchCase::IgnoreCase))
		{
			Caps.bCameras = bEnable;
		}
		else if (Name.Equals(TEXT("input"), ESearchCase::IgnoreCase))
		{
			Caps.bInput = bEnable;
		}
		else if (Name.Equals(TEXT("vr"), ESearchCase::IgnoreCase))
		{
			Caps.bVr = bEnable;
		}
	}
	return Caps;
}

bool CapsWantVr()
{
	const FCaps Caps = ParseCaps();
	return Caps.bVr.IsSet() && Caps.bVr.GetValue();
}

// ---- -URLabSourceFind --------------------------------------------------------------------------

bool SourceFindDiscover(FString& OutScene)
{
	OutScene.Empty();
	FString Value;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabSourceFind="), Value, /*bShouldStopOnSeparator=*/false))
	{
		return false;
	}
	Value.TrimStartAndEndInline();
	if (Value.Equals(TEXT("discover"), ESearchCase::IgnoreCase))
	{
		return true;
	}
	if (Value.StartsWith(TEXT("discover:"), ESearchCase::IgnoreCase))
	{
		OutScene = Value.Mid(FCString::Strlen(TEXT("discover:")));
		return true;
	}
	return false;
}

bool SourceFindBrowse()
{
	FString Value;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabSourceFind="), Value, /*bShouldStopOnSeparator=*/false))
	{
		return false;
	}
	Value.TrimStartAndEndInline();
	return Value.Equals(TEXT("browse"), ESearchCase::IgnoreCase);
}

// ---- -URLabModel -------------------------------------------------------------------------------

bool ParseModel(FString& OutPath, FString& OutFormat)
{
	FString Path;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabModel="), Path, /*bShouldStopOnSeparator=*/false)
		|| Path.IsEmpty())
	{
		return false;
	}
	Path.TrimStartAndEndInline();
	const FString Ext = FPaths::GetExtension(Path).ToLower();
	if (Ext == TEXT("xml"))
	{
		OutFormat = TEXT("xml");
	}
	else if (Ext == TEXT("mjz"))
	{
		OutFormat = TEXT("mjz");
	}
	else
	{
		// Default (and .mjb): a compiled MuJoCo binary, version-locked to this libmujoco.
		OutFormat = TEXT("mjb");
	}
	OutPath = Path;
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
} // namespace URLabLauncherFlags
