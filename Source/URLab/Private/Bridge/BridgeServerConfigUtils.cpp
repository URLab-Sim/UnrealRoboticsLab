// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServerConfigUtils.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"

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

	FString TmpStr;
	if (File.GetString(SectionName, TEXT("InstanceId"), TmpStr))
		Out.InstanceId = TmpStr;
	if (File.GetInt(SectionName, TEXT("InstanceIndex"), TmpInt))
		Out.InstanceIndex = TmpInt;
	if (File.GetInt(SectionName, TEXT("PortBase"), TmpInt))
		Out.PortBase = TmpInt;
	if (File.GetInt(SectionName, TEXT("PortStride"), TmpInt))
		Out.PortStride = TmpInt;
	if (File.GetInt(SectionName, TEXT("CamBasePort"), TmpInt))
		Out.CamBasePort = TmpInt;
	if (File.GetString(SectionName, TEXT("BindAddress"), TmpStr))
		Out.BindAddress = TmpStr;

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
	File.SetString(SectionName, TEXT("InstanceId"), *In.InstanceId);
	File.SetInt64(SectionName, TEXT("InstanceIndex"), In.InstanceIndex);
	File.SetInt64(SectionName, TEXT("PortBase"), In.PortBase);
	File.SetInt64(SectionName, TEXT("PortStride"), In.PortStride);
	File.SetInt64(SectionName, TEXT("CamBasePort"), In.CamBasePort);
	File.SetString(SectionName, TEXT("BindAddress"), *In.BindAddress);
	File.SetString(SectionName, TEXT("StopOnPIEEnd"), In.bStopOnPIEEnd ? TEXT("True") : TEXT("False"));

	File.Dirty = true;
	File.Write(Path);
}

void DerivePorts(FURLabBridgeServerConfig& Cfg,
	bool bStepExplicit, bool bStateExplicit, bool bCamExplicit)
{
	if (Cfg.InstanceIndex >= 0)
	{
		const int32 Slot = Cfg.PortBase + Cfg.InstanceIndex * Cfg.PortStride;
		if (!bStepExplicit)
			Cfg.StepPort = Slot + 0;
		if (!bStateExplicit)
			Cfg.StatePort = Slot + 1;
		if (!bCamExplicit)
			Cfg.CamBasePort = Slot + 2;
	}

	if (Cfg.CamBasePort == 0)
		Cfg.CamBasePort = Cfg.StepPort + 2;

	if (Cfg.InstanceId.IsEmpty() && Cfg.InstanceIndex >= 0)
		Cfg.InstanceId = FString::Printf(TEXT("instance_%d"), Cfg.InstanceIndex);
}

FString BuildCameraEndpoint(const FURLabBridgeServerConfig& Cfg, int32 CameraIndex)
{
	const int32 Index = FMath::Max(0, CameraIndex);
	return FString::Printf(TEXT("tcp://%s:%d"), *Cfg.BindAddress, Cfg.CamBasePort + Index);
}

void ApplyEnvAndCommandLineOverrides(FURLabBridgeServerConfig& Cfg)
{
	// Command line beats environment; a value from either replaces the INI
	// one. Returns true when a value was found so callers can flag explicit
	// port overrides (which then survive derivation).
	auto ResolveString = [](const TCHAR* CmdKey, const TCHAR* EnvKey, FString& Out) -> bool {
		if (FParse::Value(FCommandLine::Get(), CmdKey, Out))
			return true;
		const FString Env = FPlatformMisc::GetEnvironmentVariable(EnvKey);
		if (!Env.IsEmpty())
		{
			Out = Env;
			return true;
		}
		return false;
	};
	auto ResolveInt = [&ResolveString](const TCHAR* CmdKey, const TCHAR* EnvKey, int32& Out) -> bool {
		FString Str;
		if (!ResolveString(CmdKey, EnvKey, Str) || !Str.IsNumeric())
			return false;
		Out = FCString::Atoi(*Str);
		return true;
	};

	FString StrVal;
	int32 IntVal = 0;

	if (ResolveString(TEXT("URLabInstanceId="), TEXT("URLAB_INSTANCE_ID"), StrVal))
		Cfg.InstanceId = StrVal;
	if (ResolveInt(TEXT("URLabInstanceIndex="), TEXT("URLAB_INSTANCE_INDEX"), IntVal))
		Cfg.InstanceIndex = IntVal;
	if (ResolveInt(TEXT("URLabPortBase="), TEXT("URLAB_PORT_BASE"), IntVal))
		Cfg.PortBase = IntVal;
	if (ResolveInt(TEXT("URLabPortStride="), TEXT("URLAB_PORT_STRIDE"), IntVal))
		Cfg.PortStride = IntVal;

	const bool bStepExplicit = ResolveInt(TEXT("URLabStepPort="), TEXT("URLAB_STEP_PORT"), IntVal);
	if (bStepExplicit)
		Cfg.StepPort = IntVal;
	const bool bStateExplicit = ResolveInt(TEXT("URLabStatePort="), TEXT("URLAB_STATE_PORT"), IntVal);
	if (bStateExplicit)
		Cfg.StatePort = IntVal;
	const bool bCamExplicit = ResolveInt(TEXT("URLabCamBasePort="), TEXT("URLAB_CAM_BASE_PORT"), IntVal);
	if (bCamExplicit)
		Cfg.CamBasePort = IntVal;

	if (ResolveString(TEXT("URLabBindAddress="), TEXT("URLAB_BIND_ADDRESS"), StrVal))
		Cfg.BindAddress = StrVal;

	DerivePorts(Cfg, bStepExplicit, bStateExplicit, bCamExplicit);
}
} // namespace URLabBridgeServerConfigUtils
