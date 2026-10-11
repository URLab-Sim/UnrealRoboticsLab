// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjCapabilities.h"
#include "MuJoCo/Render/MjDriverDiscovery.h"
#include "MuJoCo/Runtime/MjSimulationClock.h"

/**
 * @file MjRendererBoot.h
 * @brief The one resolved description of a renderer boot, and the only reader of the
 *        command line.
 *
 * There used to be three boot shapes -- the cooked binary, the editor binary under
 * `-game`, and editor PIE -- and only the first and last are supported targets. They
 * disagreed about which drives were handled, which model formats were accepted, how
 * discovery worked, what the capability defaults were, who owned the lighting and who
 * owned the bridge, because each re-read the command line for itself and read it
 * slightly differently. `FMjRendererBoot::FromCommandLine()` is now the single reading;
 * `FMjRendererBootPlan` is its result, and both launchers consume it.
 *
 * The invariant this exists to protect is greppable: no `FCommandLine` / `FParse::Value`
 * / `FParse::Param` outside this resolver and the flag parsers it delegates to
 * (`MjLauncherFlags.cpp`).
 */

/**
 * @enum EMjDrive
 * @brief Internal routing value derived from the explicit workflow selector.
 *
 * The boot parser accepts -URLabWorkflow=simulation|render|viewer. The simulation
 * action routes away before a renderer is created; other values choose a producer:
 *
 * - Sim:       selects the independent world simulation service; it is not a renderer source.
 * - Subscribe: no physics; applies a producer's ZMQ or SHM broadcast pose stream.
 * - Request:   no physics; the render request IS the frame, and the reply is its pixels.
 *
 * There is deliberately no value for "no model yet". That is a lifecycle state -- the
 * renderer has no model -- and modelling it as a pose source is what let an instance sit
 * in it forever producing nothing, and what made the camera regime depend on a promotion
 * that could fail to happen.
 */
enum class EMjDrive : uint8
{
	Sim,
	Subscribe,
	Request
};

/** Complete peer-permission defaults for the workflow being activated. */
enum class EMjPermissionProfile : uint8
{
	Simulation,
	Render
};

/** Internal routing token for diagnostics and capability resolution. */
URLAB_API const TCHAR* MjDriveToken(EMjDrive Drive);

/**
 * @enum EMjBootAction
 * @brief What the resolved command line asks a launcher to do.
 *
 * One vocabulary for both launchers. The editor used to have its own (build a preview /
 * simulate / build / discover / nothing / refuse) which is why PIE and the package could disagree
 * about a command line neither of them rejected.
 */
enum class EMjBootAction : uint8
{
	/** No launch flags: a normal map boot, nothing to stand up. */
	None,
	/** Start the manager-free simulation service, optionally from SimulationSourceManifest. */
	Simulation,
	/** Start the manager-free viewer authority/source/bind service. */
	Viewer,
	/** The flags name a shape that cannot be built; FMjRendererBootPlan::Error says why. */
	Fail,
	/** Start the manager-free typed render request service. */
	Build,
};

/**
 * @struct FMjBootScene
 * @brief -URLabScene, resolved. Placement and tuning that is not a capability.
 */
struct FMjBootScene
{
	/** origin=X;Y;Z in UE centimetres. */
	FVector Origin = FVector::ZeroVector;
	/** level=<n>: the environment a joined renderer is dropped into ("" = bare plane). */
	FString LevelPath;
	/** base: the boot map is curated, so do not import the model's own lighting over it. */
	bool bBaseLevel = false;
	/** quality=off: skip the cinematic scalability push. */
	bool bNoQuality = false;
	/** cammax=N, or -1 when absent (keep the renderer's own cap).
	 *
	 *  The other two -URLabScene camera keys (noexposure, camnear) are NOT here.
	 *  URenderCameraComponent reads them from URLabLauncherFlags at the point of use, because a
	 *  camera also exists in an editor session that never produced a boot plan;
	 *  carrying plan copies as well meant one flag with two independent readings,
	 *  and the plan's copy was the one nothing read. */
	int32 CamMaxHeight = -1;
	/** overlay=<mask>: which mjVIS_* overlays a consumer draws; non-zero also asks the
	 *  owner for its debug tier, since those overlays only exist on the wire. */
	int32 OverlayMask = 0;
	/** maxcontacts=<n>: cap on the contacts the owner serialises; 0 = uncapped. */
	int32 OverlayMaxContacts = 0;
};

/**
 * @struct FMjBootNet
 * @brief The -URLabNet sub-keys that identify and address THIS instance.
 *
 * The port-arithmetic keys (portbase/stride/step/state/bus/cam/control) stay with the
 * bridge config, which owns the derivation; these four are boot identity and are read
 * here so the config no longer parses a command line of its own.
 */
struct FMjBootNet
{
	/** id=<s>: the instance id written into the registry entry. */
	FString InstanceId;
	/** index=<n>, or -1 when absent. */
	int32 InstanceIndex = -1;
	/** grpc=<port>: the gRPC listen port, or 0 when absent. */
	int32 GrpcPort = 0;
	/** bind=<addr>. */
	FString BindAddress;
};

/**
 * @struct FMjRendererBootPlan
 * @brief One resolved boot: drive + its endpoints, model, capabilities, scene, net.
 *
 * Produced once per boot so nothing downstream re-derives the selected workflow
 * from the flags a second time. PIE and the cooked binary use the same reader.
 */
struct URLAB_API FMjRendererBootPlan
{
	EMjBootAction Action = EMjBootAction::None;

	/** Internal producer selector. The workflow action is authoritative. */
	EMjDrive Drive = EMjDrive::Subscribe;

	/** Resolved viewer pose-stream endpoint, never the command endpoint. */
	FString BusEndpoint;

	/** The owner's REQ/REP control endpoint. It comes from discovery or a browser row,
	 *  never from `subscribe:`. Also where perturbations are sent back. */
	FString ControlEndpoint;

	/** -URLabSimulationSource=<manifest.json>. Only valid for Action==Simulation.
	 *  Empty means the simulation service starts model-free. */
	FString SimulationSourceManifest;

	/** Local world clock selected by -URLabClock. Stepped is the inert default. */
	EMjSimulationClockMode ClockMode = EMjSimulationClockMode::Stepped;
	bool bClockModeExplicit = false;

	/** A compiled MJB on disk. Never a command-line path -- every scene arrives over the
	 *  wire now -- but the editor join caches the bytes it fetched to a file and names it
	 *  here, so the PIE copy can rebuild without carrying a byte array through
	 *  duplication. */
	FString ModelPath;
	/** Compiled model bytes. Non-empty wins over ModelPath. */
	TArray<uint8> ModelBytes;

	FMjCapabilitySet Caps;
	FMjBootScene Scene;
	FMjBootNet Net;

	/** Build geometry at the rest pose into the EDITOR world and stop -- no bus, no
	 *  streaming, no play session. The only thing the editor still does differently, and
	 *  it is a flag on the shared plan rather than a parallel launcher. */
	bool bPreviewOnly = false;

	/** Why Action is Fail. Empty otherwise. */
	FString Error;

	/** Whether a model has arrived yet. A plan without one still builds: it stands the
	 *  renderer up on a placeholder so the first client load has somewhere to land. */
	bool HasModel() const { return !ModelPath.IsEmpty() || ModelBytes.Num() > 0; }
	bool HasNativeSimulationSource() const { return !SimulationSourceManifest.IsEmpty(); }
	/** Whether an authored scene may offer a compiled source to the simulation subsystem.
	 *  If a native manifest is also present, the subsystem rejects the explicit conflict. */
	bool AllowsAuthoredSimulationSource() const
	{
		return Action == EMjBootAction::None || Action == EMjBootAction::Simulation;
	}
};

/**
 * @struct FMjRendererBoot
 * @brief Constructors for a boot plan. FromCommandLine is the ONLY reader of FCommandLine.
 */
struct URLAB_API FMjRendererBoot
{
	/** Resolve the whole launch surface in one pass. Pure: no world, no disk, no network,
	 *  so a test can pin the reading of every flag combination without booting an editor. */
	static FMjRendererBootPlan FromCommandLine();

	/** Just the -URLabNet identity keys, for the bridge config (which runs before, and
	 *  independently of, any renderer). Same reader, narrower result. */
	static FMjBootNet ResolveNet();

	/**
	 * An RPC-driven load: render_load arrived in a world with no renderer, so the wire
	 * bytes ARE the boot. Positional because its one caller still spells the boot as
	 * arguments; both routes end in this one plan type.
	 */
	static FMjRendererBootPlan ForRpcLoad(const TArray<uint8>& Mjb, const FString& MjbFilePath,
		const FString& BusEndpoint, const FVector& Origin, bool bBaseLevel,
		bool bCameras);

	/**
	 * The bridge-presence rule, in one place:
	 *
	 *     a bridge exists iff  resolved rpc  ||  discoverable  ||  pose_stream.
	 *
	 * The bridge is the serve-and-be-found surface, so a pure consumer with none of them
	 * is a lean subscriber that needs no wire at all. This used to be the MANAGER-presence
	 * rule, which made every serving render server carry a physics engine idling on an
	 * empty world. Simulation now reaches the world subsystem before renderer boot.
	 *
	 * The subtlety is that several drive shapes serve without ever naming the capability:
	 * simulation defaults rpc on but permits an explicit `-rpc` local-only boot; a
	 * request renderer serves the render requests that drive it; a non-lean subscriber
	 * lets its producer push a model load into it, and a free-fly viewer's bridge is how a
	 * producer drives it. Those shapes IMPLY `rpc`. An operator who names the capability
	 * either way overrides the implication, which is why the plan records whether each
	 * capability was named at all rather than only its value.
	 *
	 * bHasBus says whether this instance actually has a stream configured, which a
	 * renderer authored into a level may have without the plan mentioning it.
	 */
	static bool WantsBridge(const FMjRendererBootPlan& Plan, EMjDrive Drive, bool bHasBus);

	/**
	 * The PERMISSIONS this boot grants its peers, every one of them resolved.
	 *
	 * One place, because a permission that is decided in two is a permission that can
	 * disagree with itself -- which is exactly what happened when one CLI token set a
	 * camera bool on the manager and a different camera bool on the renderer.
	 *
	 * Simulation grants camera RPC, input and model load by default. Render workflows
	 * grant camera RPC and model load, while input remains owner-controlled. A named
	 * permission flag overrides only its own default. Camera production capabilities do
	 * not grant camera RPC authority.
	 */
	static FMjCapabilitySet ResolvePermissionPolicy(
		const FMjRendererBootPlan& Plan, EMjPermissionProfile Profile);
};
