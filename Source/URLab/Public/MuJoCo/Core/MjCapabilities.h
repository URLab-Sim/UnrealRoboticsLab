// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Wire/UrlabWire.gen.h"
#include "MjCapabilities.generated.h"

/**
 * @file MjCapabilities.h
 * @brief The one capability vocabulary, split by the direction each capability points.
 *
 * "Capability" used to cover four unrelated kinds of thing under one word, which is why
 * `cameras` was two booleans on two objects that could disagree, and why a free-fly pawn
 * was advertised as though a peer could ask for it. Asking which direction a capability
 * points separates them:
 *
 *   service     an ingress this process STANDS UP        -- enforced by the listener existing
 *   output      data this process EMITS                  -- enforced where it is produced
 *   permission  authority this process GRANTS its peers  -- enforced by the dispatcher
 *   local       a convenience for this process's own user -- not a contract at all
 *
 * The names, the -URLabCaps tokens and the advertised wire tokens all come from
 * wire/urlab_wire.yaml through URLabWire::Capability, so a rename can only happen there.
 */

/**
 * @enum EMjCapability
 * @brief One value per capability, in schema order.
 *
 * The values INDEX into URLabWire::Capability::All, which is what keeps the enum and the
 * schema from drifting: the static_assert below fails to build if the schema grows a
 * capability the enum does not have, and MjCapabilityName() reads the name out of the
 * generated table rather than repeating it here.
 */
UENUM(BlueprintType)
enum class EMjCapability : uint8
{
	// service
	Rpc,
	Discoverable,
	// output
	PoseStream,
	CameraFrames,
	// permission
	CameraRpc,
	AcceptInput,
	AcceptModelLoad,
	AcceptEditorCommands,
	// local
	FreeFlyPawn,

	Count UMETA(Hidden)
};

static_assert(static_cast<int32>(EMjCapability::Count) == URLabWire::Capability::Count,
	"EMjCapability and wire/urlab_wire.yaml disagree about how many capabilities exist");

/** The generated row for a capability: its name, its CLI token, its wire token, its kind. */
inline const URLabWire::Capability::FEntry& MjCapabilityEntry(EMjCapability Cap)
{
	return URLabWire::Capability::All[static_cast<int32>(Cap)];
}

/** The one spelling of a capability -- enum, CLI token and log line alike. */
inline const TCHAR* MjCapabilityName(EMjCapability Cap)
{
	return MjCapabilityEntry(Cap).Name;
}

/** The token a peer sees, or nullptr for a local capability, which is never advertised. */
inline const TCHAR* MjCapabilityWireToken(EMjCapability Cap)
{
	return MjCapabilityEntry(Cap).Wire;
}

inline URLabWire::Capability::EKind MjCapabilityKind(EMjCapability Cap)
{
	return MjCapabilityEntry(Cap).Kind;
}

/** Resolve a -URLabCaps token. False when nothing answers to it, which is a refusal
 *  rather than a silent skip: a token nobody parses is how a farm node boots lean. */
URLAB_API bool MjCapabilityFromToken(const FString& Token, EMjCapability& OutCap);

/**
 * @struct FMjCapabilitySet
 * @brief Which capabilities are on, and which were named at all.
 *
 * The two masks are not the same question. Several boot rules must tell "absent" from
 * "explicitly off": a renderer authored into a level keeps its own camera switch unless
 * the command line overrides it, and every implied capability applies only when the
 * operator did not name it either way.
 */
struct URLAB_API FMjCapabilitySet
{
	bool Has(EMjCapability Cap) const { return (Enabled & Bit(Cap)) != 0; }
	bool IsNamed(EMjCapability Cap) const { return (Named & Bit(Cap)) != 0; }

	/** Turn a capability on or off and record that it was named. */
	void Set(EMjCapability Cap, bool bOn)
	{
		Named |= Bit(Cap);
		if (bOn) { Enabled |= Bit(Cap); } else { Enabled &= ~Bit(Cap); }
	}

	/** Turn a capability on WITHOUT claiming the operator asked for it, so a later
	 *  reader can still tell an implication from an instruction. */
	void Imply(EMjCapability Cap)
	{
		if (!IsNamed(Cap)) { Enabled |= Bit(Cap); }
	}

	/** The value the operator named, or Fallback when they named nothing. */
	bool Resolve(EMjCapability Cap, bool bFallback) const
	{
		return IsNamed(Cap) ? Has(Cap) : bFallback;
	}

	/** Every enabled capability of one kind, as its advertised wire token. Local
	 *  capabilities have no token and can never appear here. */
	void CollectWireTokens(URLabWire::Capability::EKind Kind, TArray<FString>& Out) const;

	uint32 Enabled = 0;
	uint32 Named = 0;

private:
	static uint32 Bit(EMjCapability Cap) { return 1u << static_cast<uint32>(Cap); }
};
