// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Misc/Optional.h"

/**
 * @file MjLauncherFlags.h
 * @brief Parsers for the Phase 1.1 five-flag launcher surface (source-of-truth §14).
 *
 * ADDITIVE. These parse the new `-URLab{Drive,SourceFind,Model,Caps,Scene,Net}` flags and hand the
 * caller exactly the same values the legacy `-URLabFast*` / `-URLabVrViewer` / `-URLabStateSource`
 * (etc.) flags produced, so each new flag can be OR'd into the existing legacy read at the site that
 * already writes the underlying field. NOTHING here maps one flag onto another (no alias table / no
 * compat shim): both the old flag and the new flag independently produce the same field value; the
 * legacy readers stay in place this wave (they are deleted in Phase 8/W18).
 *
 * The grammar (source-of-truth §14):
 *   -URLabDrive=sim | stream:<endpoint> | push | await
 *   -URLabSourceFind=discover[:scene] | browse
 *   -URLabModel=<path.{mjb,xml,mjz}>                    (format from extension)
 *   -URLabCaps=serve,publish,cameras,input,vr          (csv; absent=lean; negatable: -input)
 *   -URLabScene=level=<n>,origin=X;Y;Z,base,quality=off,cammax=N,camnear=<cm>,noexposure
 *   -URLabNet=id=,index=,portbase=,stride=,step=,state=,bus=,cam=,grpc=,bind=
 */
namespace URLabLauncherFlags
{
// ---- generic csv sub-key readers (shared by Caps / Scene / Net) --------------------------------
//
// The csv flags carry commas as their own separators, so the whole "-URLabScene=a=1,b,c=2" value is
// read with FParse's separator-stop DISABLED (otherwise it would truncate at the first comma), then
// split here. FlagName includes the trailing '=' (e.g. TEXT("URLabScene=")).

/** value for a `key=value` token in a csv flag; false if the flag or key is absent. */
URLAB_API bool GetCsvValue(const TCHAR* FlagName, const TCHAR* Key, FString& OutValue);
/** true if a bare token (e.g. `base`) is present in a csv flag. */
URLAB_API bool HasCsvToken(const TCHAR* FlagName, const TCHAR* Key);

// ---- -URLabDrive -------------------------------------------------------------------------------

enum class EDriveKind : uint8
{
	None,	// -URLabDrive absent / unrecognised
	Sim,
	Stream,
	Push,
	Await
};

/** Parse -URLabDrive. Returns the kind (None when absent). For Stream, OutStreamEndpoint receives
 *  the `<endpoint>` after "stream:" verbatim (scheme preserved). */
URLAB_API EDriveKind ParseDrive(FString& OutStreamEndpoint);

/** -URLabDrive=sim (the -URLabFastDirect equivalent). */
URLAB_API bool DriveIsSim();
/** -URLabDrive=push (the -URLabFastForcedOnly equivalent). */
URLAB_API bool DriveIsPush();
/** -URLabDrive=await (the -URLabFastServe equivalent). */
URLAB_API bool DriveIsAwait();
/** -URLabDrive=stream:<ep>; fills OutEndpoint with the endpoint (scheme preserved). */
URLAB_API bool DriveStreamEndpoint(FString& OutEndpoint);
/** -URLabDrive=stream:tcp://<ep> (the -URLabFastBus equivalent); OutEndpoint keeps the tcp:// scheme. */
URLAB_API bool DriveStreamTcpEndpoint(FString& OutEndpoint);
/** -URLabDrive=stream:grpc://<ep> (the -URLabFastGrpcJoin equivalent); OutEndpoint keeps grpc://. */
URLAB_API bool DriveStreamGrpcEndpoint(FString& OutEndpoint);

// ---- -URLabCaps --------------------------------------------------------------------------------

/** The five composable capabilities (source-of-truth §5). Each optional is set only when the token
 *  (positive `cap`) or its negation (`-cap`) appears in -URLabCaps; unset = leave the default. */
struct FCaps
{
	TOptional<bool> bServe;
	TOptional<bool> bPublish;
	TOptional<bool> bCameras;
	TOptional<bool> bInput;
	TOptional<bool> bVr;
};

/** Parse -URLabCaps=serve,publish,cameras,input,vr (negatable: -input). */
URLAB_API FCaps ParseCaps();

/** true when -URLabCaps requests the vr add-on (the -URLabVrViewer equivalent). */
URLAB_API bool CapsWantVr();

// ---- -URLabSourceFind --------------------------------------------------------------------------

/** -URLabSourceFind=discover[:scene] (the -URLabFastDiscover / -URLabFastAutoJoin equivalent).
 *  OutScene is the optional scene filter (empty for a bare `discover`). */
URLAB_API bool SourceFindDiscover(FString& OutScene);
/** -URLabSourceFind=browse (the -URLabFastBrowser equivalent). */
URLAB_API bool SourceFindBrowse();

// ---- -URLabModel -------------------------------------------------------------------------------

/** -URLabModel=<path.{mjb,xml,mjz}> (the -URLabFastMjb/Xml/Mjz equivalent). Returns true when
 *  present; OutPath is the path, OutFormat is "mjb"/"xml"/"mjz" chosen by the file extension. */
URLAB_API bool ParseModel(FString& OutPath, FString& OutFormat);

// ---- -URLabScene -------------------------------------------------------------------------------

/** origin=X;Y;Z (UE cm) — the -URLabFastOrigin equivalent (note ';' separators, since ',' delimits
 *  scene keys). Returns true only for a well-formed triple. */
URLAB_API bool SceneOrigin(FVector& OutOrigin);
/** level=<n> — the -URLabFastLevel equivalent. */
URLAB_API bool SceneLevel(FString& OutLevel);
/** base — the -URLabFastBaseLevel equivalent. */
URLAB_API bool SceneBaseLevel();
/** quality=off — the -URLabFastNoQuality equivalent. */
URLAB_API bool SceneNoQuality();
/** cammax=N — the -URLabFastCamMaxHeight equivalent. */
URLAB_API bool SceneCamMaxHeight(int32& OutPx);
/** camnear=<cm> — the -URLabCamNearClip equivalent. */
URLAB_API bool SceneCamNearClipCm(float& OutCm);
/** noexposure — the -URLabDisableAutoExposure equivalent. */
URLAB_API bool SceneNoAutoExposure();
} // namespace URLabLauncherFlags
