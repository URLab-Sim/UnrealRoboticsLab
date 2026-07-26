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

#pragma once

#include "CoreMinimal.h"

// Opaque handle from the UE-agnostic rcl seam; its definition lives only in
// UrlabRclCore.cpp. A Public header must not include the Private UrlabRclCore.h,
// so it forward-declares the handle and holds a pointer.
struct UrlabRclContext;

/**
 * @class FURLabRosContext
 * @brief Process-wide owner of the single rcl context + node the plugin
 *        publishes and subscribes through.
 *
 * ROS 2 has one context and, here, one node per process; every publisher and
 * subscription the plugin creates binds to them. This class owns their
 * lifetime and hands the underlying core handle to the transports that fill and
 * publish messages.
 *
 * Absent-ROS boot: `IsAvailable()` is the single gate every ROS caller checks.
 * When ROS is not linked (`URLAB_WITH_ROS2=0`) the whole class compiles out with
 * its callers; when it is linked but the runtime environment cannot bring up a
 * context (no DDS, misconfigured domain), `Initialize()` fails gracefully and
 * `IsAvailable()` stays false, so ROS work degrades to a no-op instead of
 * crashing the editor.
 *
 * Threading: `Get()`/`Initialize()`/`Shutdown()` are serialized by an internal
 * lock. The returned handle is created and destroyed here; per-handle rcl calls
 * (publish, spin) are serialized by their owning transport, per the core's
 * threading contract.
 */
class URLAB_API FURLabRosContext
{
public:
	/** The process-wide instance. */
	static FURLabRosContext& Get();

	/** Bring up the context + node if not already up. Idempotent: repeated calls
	 *  are no-ops once a context exists, and a failed first attempt is not
	 *  retried. Returns IsAvailable(). */
	bool Initialize();

	/** True when a usable rcl context + node is live. All ROS work is gated on
	 *  this. */
	bool IsAvailable() const;

	/** Fini the node then the context, in reverse creation order. Idempotent. */
	void Shutdown();

	/** The core context handle, or null when unavailable. Only translation units
	 *  that include UrlabRclCore.h use it. */
	UrlabRclContext* GetHandle() const { return Context; }

private:
	FURLabRosContext() = default;
	~FURLabRosContext();
	FURLabRosContext(const FURLabRosContext&) = delete;
	FURLabRosContext& operator=(const FURLabRosContext&) = delete;

	UrlabRclContext* Context = nullptr;
	bool bInitAttempted = false;
	mutable FCriticalSection Mutex;
};
