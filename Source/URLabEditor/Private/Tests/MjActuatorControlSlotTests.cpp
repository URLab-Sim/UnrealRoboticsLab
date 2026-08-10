// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The staged-control slots, moved off the actuator element onto the articulation.
//
// Only one thing about them is a decision rather than a mechanism, and it is the
// one that has to survive the move byte for byte: control source 0 is the
// network and anything else is the UI. Everything downstream of the bridge, the
// ZMQ subscriber and both ROS transports depends on that mapping, and none of
// them says so out loud, so it is asserted here.
//
// The rest is range discipline. The ids indexing these slots are a compiled
// scene's, so they arrive from outside and can be stale by a whole recompile;
// every entry point has to treat an id it does not have as nothing to do rather
// than as an offset into someone else's memory.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "UObject/Package.h"

#include "MuJoCo/Core/MjArticulation.h"

namespace MjActuatorControlSlotTests
{
/**
 * An articulation with no world.
 *
 * The slots are plain memory on the actor and none of the entry points under
 * test reaches for a world, a component or a model, which is the point: this is
 * the half of actuator control that a compile does not have to exist for.
 */
AMjArticulation* MakeDetachedArticulation()
{
	return NewObject<AMjArticulation>(GetTransientPackage());
}
}  // namespace MjActuatorControlSlotTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlSourceSelectsSlot,
	"URLab.Elements.ControlSourceSelectsStagedSlot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlSourceSelectsSlot::RunTest(const FString& Parameters)
{
	AMjArticulation* Art = MjActuatorControlSlotTests::MakeDetachedArticulation();
	if (!TestNotNull(TEXT("detached articulation"), Art))
	{
		return false;
	}

	Art->ResetControlSlots(4, {0, 1, 2, 3});
	Art->StageNetworkControl(2, 0.75f);
	Art->StageInternalControl(2, -0.25f);

	// Both slots hold a value at once; the source decides which is read, and
	// staging one never disturbs the other.
	TestEqual(TEXT("source 0 reads the network slot"), Art->ResolveDesiredControl(2, 0), 0.75f);
	TestEqual(TEXT("source 1 reads the internal slot"), Art->ResolveDesiredControl(2, 1), -0.25f);

	Art->ControlSource = 0;
	TestEqual(TEXT("the articulation's own source is the default"), Art->ResolveDesiredControl(2), 0.75f);
	Art->ControlSource = 1;
	TestEqual(TEXT("and follows a change of source"), Art->ResolveDesiredControl(2), -0.25f);

	// Any source that is not the network is the UI: the mapping is a test
	// against 0, not an enumeration, and a value outside EControlSource must
	// not fall through to the network slot.
	TestEqual(TEXT("an unknown source is not the network"), Art->ResolveDesiredControl(2, 7), -0.25f);

	Art->ClearStagedControl(2);
	TestEqual(TEXT("clearing zeroes the network slot"), Art->ResolveDesiredControl(2, 0), 0.0f);
	TestEqual(TEXT("clearing zeroes the internal slot"), Art->ResolveDesiredControl(2, 1), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlSlotsAreSceneIndexed,
	"URLab.Elements.ControlSlotsAreSceneIndexed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlSlotsAreSceneIndexed::RunTest(const FString& Parameters)
{
	AMjArticulation* Art = MjActuatorControlSlotTests::MakeDetachedArticulation();
	if (!TestNotNull(TEXT("detached articulation"), Art))
	{
		return false;
	}

	// A scene compiles every participant into one model, so this articulation's
	// actuators sit at ids that are neither zero-based nor contiguous with each
	// other. The slots are sized to the scene and the ownership list is what
	// says which of them are this articulation's to write.
	Art->ResetControlSlots(8, {3, 5});

	TestEqual(TEXT("slots are sized to the scene"), Art->GetControlSlotCount(), 8);
	TestEqual(TEXT("ownership is the articulation's own"), Art->GetOwnedActuatorIds().Num(), 2);
	TestTrue(TEXT("ownership keeps the scene ids"), Art->GetOwnedActuatorIds().Contains(5));

	Art->StageNetworkControl(5, 1.5f);
	TestEqual(TEXT("a scene id addresses its own slot"), Art->ResolveDesiredControl(5, 0), 1.5f);
	TestEqual(TEXT("and no other"), Art->ResolveDesiredControl(3, 0), 0.0f);

	// An id outside the scene is not this articulation's business and is not
	// memory either. Staging it does nothing and reading it is zero.
	Art->StageNetworkControl(8, 9.0f);
	Art->StageNetworkControl(-1, 9.0f);
	TestEqual(TEXT("an id past the end reads zero"), Art->ResolveDesiredControl(8, 0), 0.0f);
	TestEqual(TEXT("a negative id reads zero"), Art->ResolveDesiredControl(-1, 0), 0.0f);

	// An ownership list carrying an id the scene does not have would put the
	// step loop past the end of d->ctrl, so it is filtered on the way in.
	Art->ResetControlSlots(2, {0, 2, -4});
	TestEqual(TEXT("ownership drops ids outside the scene"), Art->GetOwnedActuatorIds().Num(), 1);
	TestTrue(TEXT("and keeps the one inside it"), Art->GetOwnedActuatorIds().Contains(0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlSlotsResetOnRecompile,
	"URLab.Elements.ControlSlotsResetOnRecompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlSlotsResetOnRecompile::RunTest(const FString& Parameters)
{
	AMjArticulation* Art = MjActuatorControlSlotTests::MakeDetachedArticulation();
	if (!TestNotNull(TEXT("detached articulation"), Art))
	{
		return false;
	}

	// Before any compile there is nothing to stage onto, and asking is zero
	// rather than a crash: the RPC surface is reachable before the model is.
	TestEqual(TEXT("an uncompiled articulation has no slots"), Art->GetControlSlotCount(), 0);
	Art->StageNetworkControl(0, 1.0f);
	TestEqual(TEXT("staging before a compile is a no-op"), Art->ResolveDesiredControl(0, 0), 0.0f);

	Art->ResetControlSlots(3, {0, 1, 2});
	Art->StageNetworkControl(1, 4.0f);
	TestEqual(TEXT("staged"), Art->ResolveDesiredControl(1, 0), 4.0f);

	// A recompile invalidates every id that indexed the old slots, so the
	// values do not carry over -- a stale value surviving would be applied to
	// whichever actuator inherited the id.
	Art->ResetControlSlots(3, {0, 1, 2});
	TestEqual(TEXT("a recompile does not carry staged control over"), Art->ResolveDesiredControl(1, 0), 0.0f);

	Art->ClearControlSlots();
	TestEqual(TEXT("discarding the model discards the slots"), Art->GetControlSlotCount(), 0);
	TestEqual(TEXT("and the ownership list"), Art->GetOwnedActuatorIds().Num(), 0);
	TestEqual(TEXT("reading after the discard is zero"), Art->ResolveDesiredControl(1, 0), 0.0f);
	return true;
}

#endif  // WITH_EDITOR
