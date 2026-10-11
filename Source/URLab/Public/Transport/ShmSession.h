// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * @struct FURLabShmSession
 * @brief The one shared-memory session directory for this instance.
 *
 * The latest `state.shm`, `render.shm`, and `cam_*.shm` buffers live in one
 * directory, and the hello handshake advertises that directory
 * as `shm_session_dir` so a client opens exactly what the server wrote.
 *
 * It used to be derived in three places that disagreed. The RPC transport built
 * `<label>_p<port>_<pid>`; the state transport built `<label>_<pid>` at
 * BeginPlay, when the dispatcher had no session id yet, so its label was always
 * "live"; the camera transport built `<label>_<pid>` at stream start, AFTER the
 * first hello had minted a GUID, so its label was that GUID. Three directories,
 * one of them advertised. A SHM camera client opened a path nothing ever wrote.
 *
 * So it is resolved ONCE, by the bridge, before any of the three exist, and
 * handed to them. The directory in the handshake is the directory in use
 * because there is only one.
 */
struct URLAB_API FURLabShmSession
{
	/** Resolved session id: the instance label made unique per process. */
	FString SessionId;

	/** Absolute directory holding every segment for this session. */
	FString Dir;

	bool IsValid() const { return !Dir.IsEmpty(); }

	/** Absolute path of `FileName` inside this session. */
	FString FilePath(const FString& FileName) const;

	/** Create the directory if it does not exist. Safe to call repeatedly. */
	void EnsureDirectoryExists() const;

	/**
	 * Resolve the session for an instance.
	 *
	 * `InstanceLabel` is the operator-facing name (the config's InstanceId),
	 * empty meaning "live". `InstancePort` is the instance's step port, folded
	 * in only to keep the directory traceable back to the instance; the process
	 * id is what actually makes it unique, so two editors acting as render
	 * servers never open each other's segments or steal each other's wakeups.
	 */
	static FURLabShmSession Resolve(const FString& InstanceLabel, int32 InstancePort);

	/** Directory a given resolved session id maps to. Separate from Resolve so
	 *  a reader holding only the id (a handshake fallback, a test) can find it
	 *  without re-deriving the id itself. */
	static FString DirForSessionId(const FString& InSessionId);

	/**
	 * File name (not path) of the per-camera ring for a canonical
	 * `<articulation>/<part>` camera name, relative to `Dir`.
	 *
	 * One definition, because the camera transport creates the file with it and
	 * the hello advertises the same string per camera. The two used to disagree:
	 * a client rebuilding the name from the canonical key kept the slash in it,
	 * so the open could never succeed and every SHM camera silently degraded.
	 */
	static FString MakeCameraSegmentFileName(const FString& CanonicalName);

	/** File name of the per-step state-snapshot ring inside `Dir`. */
	static const TCHAR* StateSegmentFileName;

	/** File name of the render-bus ring inside `Dir`. One definition, for the
	 *  same reason the camera name has one: the owner creates the file and a
	 *  subscriber opens it, and the two disagreeing is a segment nothing ever wrote. */
	static const TCHAR* RenderSegmentFileName;
};
