// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "BridgeSubsystem.generated.h"

class UURLabBridgeServer;

/**
 * @class UURLabBridgeSubsystem
 * @brief The bridge's owner outside the editor: one UURLabBridgeServer per world,
 *        stood up on first demand and torn down with the world.
 *
 * The bridge used to belong to AAMjManager, which made "a client can reach this
 * process" and "this process runs physics" the same question. They are not: a
 * render server serves geometry, appearance, lights and forced renders, and never
 * steps a simulation. It was that conflation that had every render server spawn a
 * manager whose engine idled on an empty world.
 *
 * So the bridge is a property of the world, and the manager is an optional
 * participant that registers itself when one exists (AAMjManager::BeginPlay).
 *
 * Editor builds keep their existing owner: URLabEditor's subsystem holds a bridge
 * whose lifetime spans PIE cycles, reached through URLabBridgeProvider. This
 * subsystem defers to it rather than binding a second listener on the same port.
 */
UCLASS()
class URLAB_API UURLabBridgeSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The bridge serving `World`, standing one up if this is the first caller.
	 *  Null only when there is no world (or no subsystem, in a world type that
	 *  does not carry them). */
	static UURLabBridgeServer* EnsureFor(UWorld* World);

	/** The bridge serving `World` if one is already up, without standing one up.
	 *  For code that wants to observe rather than to bind a port. */
	static UURLabBridgeServer* Find(UWorld* World);

	/** Test seam: stand the bridge up owned by this world and WITHOUT binding a
	 *  listener, so an automation test can drive the real standup path without
	 *  fighting the editor's bridge for the configured port. Exercises the cooked
	 *  branch, which is the one a render server takes. */
	static void SetUnboundForTest(bool bUnbound);

	virtual void Deinitialize() override;

	UURLabBridgeServer* GetServer() const { return Server; }

private:
	UURLabBridgeServer* EnsureServer();

	UPROPERTY(Transient)
	TObjectPtr<UURLabBridgeServer> Server;

	/** True when this subsystem created the server and must therefore stop it.
	 *  False when it is the editor subsystem's, which outlives this world. */
	bool bOwnsServer = false;
};
