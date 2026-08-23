// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServerConfigUtils.h"

#include "MuJoCo/Fast/MjLauncherFlags.h"
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

	if (File.GetString(SectionName, TEXT("StateSourceEndpoint"), TmpStr))
		Out.StateSourceEndpoint = TmpStr;
	if (File.GetBool(SectionName, TEXT("BroadcastViewers"), TmpBool))
		Out.bBroadcastViewers = TmpBool;
	if (File.GetInt(SectionName, TEXT("ViewerPort"), TmpInt))
		Out.ViewerPort = TmpInt;
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

	bool bStepExplicit = ResolveInt(TEXT("URLabStepPort="), TEXT("URLAB_STEP_PORT"), IntVal);
	if (bStepExplicit)
		Cfg.StepPort = IntVal;
	bool bStateExplicit = ResolveInt(TEXT("URLabStatePort="), TEXT("URLAB_STATE_PORT"), IntVal);
	if (bStateExplicit)
		Cfg.StatePort = IntVal;
	bool bCamExplicit = ResolveInt(TEXT("URLabCamBasePort="), TEXT("URLAB_CAM_BASE_PORT"), IntVal);
	if (bCamExplicit)
		Cfg.CamBasePort = IntVal;

	if (ResolveString(TEXT("URLabBindAddress="), TEXT("URLAB_BIND_ADDRESS"), StrVal))
		Cfg.BindAddress = StrVal;

	// Owner viewer bus. -URLabBroadcastViewers=1 makes an owner re-broadcast.
	// (-URLabStateSource is no longer parsed here: the launcher maps it to
	// -URLabDrive=stream:tcp://<ep>, spawning a transform-mirror AMjRenderer.)
	if (ResolveInt(TEXT("URLabViewerPort="), TEXT("URLAB_VIEWER_PORT"), IntVal))
		Cfg.ViewerPort = IntVal;
	if (ResolveInt(TEXT("URLabBroadcastViewers="), TEXT("URLAB_BROADCAST_VIEWERS"), IntVal))
		Cfg.bBroadcastViewers = (IntVal != 0);

	// Phase 1.1: -URLabCaps=publish is the new spelling of -URLabBroadcastViewers=1 (source-of-truth
	// §14). Both write bBroadcastViewers; the cap (negatable: -publish) wins when present.
	{
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		if (Caps.bPublish.IsSet())
			Cfg.bBroadcastViewers = Caps.bPublish.GetValue();
	}

	// Phase 1.1: -URLabNet=id=,index=,portbase=,stride=,step=,state=,bus=,cam=,grpc=,bind=
	// consolidates the legacy per-service net flags (source-of-truth §14). Each sub-key writes the
	// SAME config field its legacy flag did, and a present net key wins over the legacy read above.
	{
		using URLabLauncherFlags::GetCsvValue;
		FString NetVal;
		if (GetCsvValue(TEXT("URLabNet="), TEXT("id"), NetVal))
			Cfg.InstanceId = NetVal;
		if (GetCsvValue(TEXT("URLabNet="), TEXT("index"), NetVal) && NetVal.IsNumeric())
			Cfg.InstanceIndex = FCString::Atoi(*NetVal);
		if (GetCsvValue(TEXT("URLabNet="), TEXT("portbase"), NetVal) && NetVal.IsNumeric())
			Cfg.PortBase = FCString::Atoi(*NetVal);
		if (GetCsvValue(TEXT("URLabNet="), TEXT("stride"), NetVal) && NetVal.IsNumeric())
			Cfg.PortStride = FCString::Atoi(*NetVal);
		if (GetCsvValue(TEXT("URLabNet="), TEXT("step"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.StepPort = FCString::Atoi(*NetVal);
			bStepExplicit = true;
		}
		if (GetCsvValue(TEXT("URLabNet="), TEXT("state"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.StatePort = FCString::Atoi(*NetVal);
			bStateExplicit = true;
		}
		// bus= replaces ViewerPort (source-of-truth §14).
		if (GetCsvValue(TEXT("URLabNet="), TEXT("bus"), NetVal) && NetVal.IsNumeric())
			Cfg.ViewerPort = FCString::Atoi(*NetVal);
		if (GetCsvValue(TEXT("URLabNet="), TEXT("cam"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.CamBasePort = FCString::Atoi(*NetVal);
			bCamExplicit = true;
		}
		if (GetCsvValue(TEXT("URLabNet="), TEXT("bind"), NetVal))
			Cfg.BindAddress = NetVal;

		// grpc= replaces -URLabDmEnvPort (source-of-truth §14). That port has NO config field: the
		// gRPC transport reads it straight off the command line (URLabDmEnvPort=,
		// DmEnvRpcTransport.cpp:272), and that transport file is owned by another agent this wave, so
		// it cannot be re-plumbed here. To make -URLabNet=grpc= write the same thing the legacy flag
		// wrote, feed the transport's own reader: append the legacy token to the command line when
		// -URLabDmEnvPort is absent (config parse runs before EnsureExternalTransportsBound binds the
		// gRPC server: AMjManager.cpp:221 then :260). Guarded on absence so it stays idempotent
		// across the multiple ApplyEnvAndCommandLineOverrides calls and the legacy flag still wins.
		FString GrpcVal, ExistingDmEnv;
		if (GetCsvValue(TEXT("URLabNet="), TEXT("grpc"), GrpcVal) && GrpcVal.IsNumeric()
			&& !FParse::Value(FCommandLine::Get(), TEXT("URLabDmEnvPort="), ExistingDmEnv))
		{
			FCommandLine::Append(*FString::Printf(TEXT(" -URLabDmEnvPort=%s"), *GrpcVal));
		}
	}

	DerivePorts(Cfg, bStepExplicit, bStateExplicit, bCamExplicit);
}
} // namespace URLabBridgeServerConfigUtils
