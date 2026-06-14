// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServerConfigUtils.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"

namespace URLabBridgeServerConfigUtils
{
const TCHAR* SectionName = TEXT("BridgeServer");

static const TCHAR* PluginName = TEXT("UnrealRoboticsLab");
static const TCHAR* IniBaseName = TEXT("LocalUnrealRoboticsLab.ini");

FString GetIniPath()
{
	TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(PluginName);
	const FString BaseDir = Plugin.IsValid()
							  ? Plugin->GetBaseDir()
							  : FPaths::ConvertRelativePathToFull(FPaths::ProjectPluginsDir() / PluginName);
	return FPaths::ConvertRelativePathToFull(BaseDir / TEXT("Config") / IniBaseName);
}

void LoadFromIni(FURLabBridgeServerConfig& Out)
{
	const FString Path = GetIniPath();
	if (!FPaths::FileExists(Path))
		return;

	// Use FConfigFile directly so this works against arbitrary paths
	// (including the test scratch paths). GConfig->Get* requires the
	// file to already be in its cache, which can't be assumed here.
	FConfigFile File;
	File.Read(Path);

	bool TmpBool = false;
	int32 TmpInt = 0;

	if (File.GetBool(SectionName, TEXT("AutoStart"), TmpBool))
		Out.bAutoStart = TmpBool;
	if (File.GetInt(SectionName, TEXT("StepPort"), TmpInt))
		Out.StepPort = TmpInt;
	if (File.GetInt(SectionName, TEXT("StatePort"), TmpInt))
		Out.StatePort = TmpInt;
	if (File.GetBool(SectionName, TEXT("StopOnPIEEnd"), TmpBool))
		Out.bStopOnPIEEnd = TmpBool;
}

void SaveToIni(const FURLabBridgeServerConfig& In)
{
	const FString Path = GetIniPath();
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);

	FConfigFile File;
	File.Read(Path); // preserve other sections / unknown keys

	File.SetString(SectionName, TEXT("AutoStart"), In.bAutoStart ? TEXT("True") : TEXT("False"));
	File.SetInt64(SectionName, TEXT("StepPort"), In.StepPort);
	File.SetInt64(SectionName, TEXT("StatePort"), In.StatePort);
	File.SetString(SectionName, TEXT("StopOnPIEEnd"), In.bStopOnPIEEnd ? TEXT("True") : TEXT("False"));

	File.Dirty = true;
	File.Write(Path);
}
} // namespace URLabBridgeServerConfigUtils
