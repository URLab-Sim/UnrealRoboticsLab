// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

// ============================================================================
// MjRenderBusTierTests.cpp
//
// The debug tier used to be negotiable over gRPC and nowhere else, which -- with
// the gRPC module built on Linux only -- quietly made overlays a Linux feature.
// It is now negotiated on the render bus itself, by the topic a subscriber
// subscribes to, so these pin the two things that makes true or false:
//
//  * the topic codec, because the topic IS the request. A publisher that cannot
//    read a subscriber's caps back out of the topic serves it the wrong tier, and a
//    topic that collides by PREFIX (which is how ZMQ matches) delivers one
//    subscriber's decor to another that never asked.
//  * the decline, because a backend with no subscription back-channel must say
//    so rather than accept a request it will never honour.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/RenderBusTopics.h"
#include "Transport/ShmClientSubscribeTransport.h"
#include "Transport/ZmqClientSubscribeTransport.h"

// ---------------------------------------------------------------------------
// URLab.RenderBus.TopicCarriesTheDebugRequest
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderBusTopicCarriesRequest,
	"URLab.RenderBus.TopicCarriesTheDebugRequest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenderBusTopicCarriesRequest::RunTest(const FString& Parameters)
{
	using namespace URLabRenderBusTopic;

	// A subscriber that asks for nothing lands on the lean topic, so the caller needs
	// no "did I want anything?" branch and a lean subscriber is indistinguishable from
	// one that predates the tier.
	TestEqual(TEXT("no caps -> the lean topic"), ForCaps(false, false, 0), FString(Lean));
	TestFalse(TEXT("the lean topic is not tiered"), IsTiered(Lean));

	// Round trip: what the subscriber asked for is what the publisher reads back.
	struct FCase { bool bContacts; bool bOverlay; int32 Max; };
	const FCase Cases[] = {
		{true, false, 0}, {false, true, 0}, {true, true, 0},
		{true, false, 10}, {true, true, 256},
	};
	for (const FCase& C : Cases)
	{
		const FString Topic = ForCaps(C.bContacts, C.bOverlay, C.Max);
		TestTrue(TEXT("a requested tier is a tiered topic"), IsTiered(Topic));
		bool bC = false, bO = false;
		int32 Max = -1;
		TestTrue(TEXT("the publisher can read the topic back"), ParseCaps(Topic, bC, bO, Max));
		TestEqual(TEXT("contacts round-trip"), bC, C.bContacts);
		TestEqual(TEXT("overlay round-trip"), bO, C.bOverlay);
		TestEqual(TEXT("cap round-trip"), Max, C.Max);
	}

	// The cap only means something alongside contacts, so it is pinned to 0 when
	// contacts are off: two overlay-only subscribers that happened to carry different
	// caps would otherwise land on two topics and the owner would build the
	// identical frame twice.
	TestEqual(TEXT("overlay-only subscribers collapse to one topic"),
		ForCaps(false, true, 64), ForCaps(false, true, 0));

	// -- Prefix safety. ZMQ matches subscriptions by prefix. -------------------
	const FString Tiered = ForCaps(true, true, 0);
	TestFalse(TEXT("a tiered topic must not extend the lean one"),
		Tiered.StartsWith(Lean, ESearchCase::CaseSensitive));
	TestFalse(TEXT("the lean topic must not extend a tiered one"),
		FString(Lean).StartsWith(Tiered, ESearchCase::CaseSensitive));

	// ...including between two caps where one number prefixes another: without the
	// terminator, a subscriber capped at 1 contact would also receive the frames built
	// for one capped at 10.
	const FString Cap1 = ForCaps(true, false, 1);
	const FString Cap10 = ForCaps(true, false, 10);
	TestFalse(TEXT("cap 1 must not prefix cap 10"),
		Cap10.StartsWith(Cap1, ESearchCase::CaseSensitive));

	// -- Malformed topics are refused, never guessed at ------------------------
	bool bC = false, bO = false;
	int32 Max = 0;
	TestFalse(TEXT("the lean topic carries no caps"), ParseCaps(Lean, bC, bO, Max));
	TestFalse(TEXT("a foreign topic carries no caps"), ParseCaps(TEXT("state"), bC, bO, Max));
	TestFalse(TEXT("a truncated tiered topic is refused"),
		ParseCaps(TEXT("debug/c1o1m"), bC, bO, Max));
	TestFalse(TEXT("a tiered topic with no terminator is refused"),
		ParseCaps(TEXT("debug/c1o1m0"), bC, bO, Max));
	TestFalse(TEXT("a non-numeric cap is refused"),
		ParseCaps(TEXT("debug/c1o1mx/"), bC, bO, Max));
	TestFalse(TEXT("a signed cap is refused"),
		ParseCaps(TEXT("debug/c1o1m-1/"), bC, bO, Max));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.RenderBus.BackendsDeclareWhetherTheyCanNegotiate
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderBusBackendsDeclareNegotiation,
	"URLab.RenderBus.BackendsDeclareWhetherTheyCanNegotiate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenderBusBackendsDeclareNegotiation::RunTest(const FString& Parameters)
{
	FMjRenderDebugCaps Caps;
	Caps.bContacts = true;
	Caps.bOverlay = true;
	TestTrue(TEXT("caps that ask for something report so"), Caps.WantsAny());
	TestFalse(TEXT("default caps ask for nothing"), FMjRenderDebugCaps().WantsAny());

	// ZMQ can ask, because its subscription reaches the owner.
	UURLabZmqClientSubscribeTransport* Zmq =
		NewObject<UURLabZmqClientSubscribeTransport>(GetTransientPackage());
	TestTrue(TEXT("the ZMQ backend carries the debug request"), Zmq->SetRenderDebugCaps(Caps));

	// Shared memory cannot: a ring is one writer and any number of anonymous
	// readers, so there is nobody to ask. It must decline rather than accept and
	// silently drop, which is what left a subscriber waiting for decor forever.
	UURLabShmClientSubscribeTransport* Shm =
		NewObject<UURLabShmClientSubscribeTransport>(GetTransientPackage());
	TestFalse(TEXT("the SHM backend declines the debug request"), Shm->SetRenderDebugCaps(Caps));

	// The base is the decline, so a backend added later without thinking about the
	// tier is refused rather than assumed capable.
	TestFalse(TEXT("the role's default is to decline"),
		Shm->UURLabClientSubscribeTransport::SetRenderDebugCaps(Caps));

	return true;
}
