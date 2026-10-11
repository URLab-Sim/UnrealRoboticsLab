// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServerConfigUtils.h"

#include "MuJoCo/Render/MjLauncherFlags.h"
#include "MuJoCo/Render/MjRendererBoot.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
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

	if (File.GetBool(SectionName, TEXT("BroadcastOnRenderBus"), TmpBool))
		Out.bBroadcastOnRenderBus = TmpBool;
	if (File.GetInt(SectionName, TEXT("RenderBusPort"), TmpInt))
		Out.RenderBusPort = TmpInt;
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
	// The CLI surface is the consolidated -URLabNet= keys read below, which win over
	// these. Env vars and INI equivalents STAY, so a value from the
	// environment still replaces the INI one. Returns true when a value was found so
	// callers can flag explicit port overrides (which then survive derivation).
	auto ResolveEnvString = [](const TCHAR* EnvKey, FString& Out) -> bool {
		const FString Env = FPlatformMisc::GetEnvironmentVariable(EnvKey);
		if (!Env.IsEmpty())
		{
			Out = Env;
			return true;
		}
		return false;
	};
	auto ResolveEnvInt = [&ResolveEnvString](const TCHAR* EnvKey, int32& Out) -> bool {
		FString Str;
		if (!ResolveEnvString(EnvKey, Str) || !Str.IsNumeric())
			return false;
		Out = FCString::Atoi(*Str);
		return true;
	};

	FString StrVal;
	int32 IntVal = 0;

	if (ResolveEnvString(TEXT("URLAB_INSTANCE_ID"), StrVal))
		Cfg.InstanceId = StrVal;
	if (ResolveEnvInt(TEXT("URLAB_INSTANCE_INDEX"), IntVal))
		Cfg.InstanceIndex = IntVal;
	if (ResolveEnvInt(TEXT("URLAB_PORT_BASE"), IntVal))
		Cfg.PortBase = IntVal;
	if (ResolveEnvInt(TEXT("URLAB_PORT_STRIDE"), IntVal))
		Cfg.PortStride = IntVal;

	bool bStepExplicit = ResolveEnvInt(TEXT("URLAB_STEP_PORT"), IntVal);
	if (bStepExplicit)
		Cfg.StepPort = IntVal;
	bool bStateExplicit = ResolveEnvInt(TEXT("URLAB_STATE_PORT"), IntVal);
	if (bStateExplicit)
		Cfg.StatePort = IntVal;
	bool bCamExplicit = ResolveEnvInt(TEXT("URLAB_CAM_BASE_PORT"), IntVal);
	if (bCamExplicit)
		Cfg.CamBasePort = IntVal;

	if (ResolveEnvString(TEXT("URLAB_BIND_ADDRESS"), StrVal))
		Cfg.BindAddress = StrVal;

	// Owner render bus. URLAB_BROADCAST_ON_RENDER_BUS makes an owner re-broadcast.
	if (ResolveEnvInt(TEXT("URLAB_RENDER_BUS_PORT"), IntVal))
		Cfg.RenderBusPort = IntVal;
	if (ResolveEnvInt(TEXT("URLAB_BROADCAST_ON_RENDER_BUS"), IntVal))
		Cfg.bBroadcastOnRenderBus = (IntVal != 0);

	// The two halves of what used to be one `publish` token, now independent: an output
	// (broadcast the frame) and a service (be listed in the registry). A farm node that
	// wants to be findable and quiet asks for exactly that. Either capability, named
	// explicitly, wins over the environment variable above.
	{
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		if (Caps.IsNamed(EMjCapability::PoseStream))
			Cfg.bBroadcastOnRenderBus = Caps.Has(EMjCapability::PoseStream);
		// Discovery is on unless the operator says otherwise: an instance nobody can find
		// is the unusual case, and the registry entry is what every browser, pool and farm
		// reads. `-URLabCaps=-discoverable` is how a private node opts out.
		if (Caps.IsNamed(EMjCapability::Discoverable))
			Cfg.bDiscoverable = Caps.Has(EMjCapability::Discoverable);
	}

	// -URLabNet=id=,index=,portbase=,stride=,step=,state=,bus=,cam=,grpc=,bind= is the
	// consolidated net-flags surface. Each sub-key writes the same
	// config field the per-service env vars above do, and a present net key wins over them.
	{
		using URLabLauncherFlags::NetValue;
		FString NetVal;

		// Instance identity and address come off the boot resolver, which reads them the
		// same way the renderer boot does; only the port arithmetic below is this file's.
		const FMjBootNet BootNet = FMjRendererBoot::ResolveNet();
		if (!BootNet.InstanceId.IsEmpty())
			Cfg.InstanceId = BootNet.InstanceId;
		if (BootNet.InstanceIndex >= 0)
			Cfg.InstanceIndex = BootNet.InstanceIndex;
		if (!BootNet.BindAddress.IsEmpty())
			Cfg.BindAddress = BootNet.BindAddress;

		if (NetValue(TEXT("portbase"), NetVal) && NetVal.IsNumeric())
			Cfg.PortBase = FCString::Atoi(*NetVal);
		if (NetValue(TEXT("stride"), NetVal) && NetVal.IsNumeric())
			Cfg.PortStride = FCString::Atoi(*NetVal);
		if (NetValue(TEXT("step"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.StepPort = FCString::Atoi(*NetVal);
			bStepExplicit = true;
		}
		if (NetValue(TEXT("state"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.StatePort = FCString::Atoi(*NetVal);
			bStateExplicit = true;
		}
		// bus= replaces RenderBusPort.
		if (NetValue(TEXT("bus"), NetVal) && NetVal.IsNumeric())
			Cfg.RenderBusPort = FCString::Atoi(*NetVal);
		if (NetValue(TEXT("cam"), NetVal) && NetVal.IsNumeric())
		{
			Cfg.CamBasePort = FCString::Atoi(*NetVal);
			bCamExplicit = true;
		}
		// The gRPC listen port. Cfg.DmEnvPort is the one value: the transport takes
		// its bind port from the bridge's config, and the registry writer advertises
		// the same field, so the port bound and the port advertised cannot differ.
		// The -URLabDmEnvPort= token survives only as an INPUT an orchestrator may
		// pass on a child's command line; the append keeps that token consistent for
		// a child spawned from this process.
		if (BootNet.GrpcPort > 0)
		{
			Cfg.DmEnvPort = BootNet.GrpcPort;
			URLabLauncherFlags::EnsureDmEnvPortToken(BootNet.GrpcPort);
		}
		else
		{
			// No -URLabNet=grpc= key: still honour a direct token (set by a prior
			// ApplyEnvAndCommandLineOverrides call, or by the RenderPool orchestrator) so
			// Cfg.DmEnvPort matches whatever the transport actually binds instead of silently
			// keeping the struct default.
			int32 DirectDmEnvPort = 0;
			if (URLabLauncherFlags::DmEnvPort(DirectDmEnvPort))
			{
				Cfg.DmEnvPort = DirectDmEnvPort;
			}
		}
	}

	DerivePorts(Cfg, bStepExplicit, bStateExplicit, bCamExplicit);
}
} // namespace URLabBridgeServerConfigUtils
