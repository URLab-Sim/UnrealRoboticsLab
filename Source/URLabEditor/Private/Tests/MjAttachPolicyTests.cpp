// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Attach conflicts, resolved by MuJoCo's own policy.
//
// A scene composes participants with `mjs_attach`, and each participant may
// carry model-wide blocks of its own. MuJoCo already owns what happens then: an
// `mjtConflict` on the scene's compiler block chooses between keeping the
// scene's value with a warning, merging the two, and refusing the attach. This
// file asserts that the policy reaches the spec URLab builds, that URLab's
// specs carry the authored flags the resolver reads, and that the resolution
// then follows the policy.
//
// It is also the drift instrument for two symbols MuJoCo exports and does not
// declare (see MjAuthored.h). Nothing in a compiler can check that declaration
// against upstream's, so the checks below stand in for one: they set a flag,
// attach, and assert the resolution changes -- and then clear the flag and
// assert it changes back. A signature that drifts stops producing those
// differences, and this fails rather than a user's session behaving oddly.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"

#include "MuJoCo/Spec/MjAuthored.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#include "Tests/MjParitySupport.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace MjAttachPolicyTests
{
/** The scene, authoring a timestep of its own well away from any default. */
const TCHAR* const SceneXml = TEXT(R"(<mujoco model="scene">
  <option timestep="0.01"/>
  <worldbody>
    <body name="anchor"><geom name="floor" type="sphere" size="0.1"/></body>
  </worldbody>
</mujoco>)");

/**
 * The participant, authoring MuJoCo's OWN default timestep on purpose.
 *
 * 0.002 is what an unauthored spec already holds, so this document is exactly
 * the case the resolver cannot see without the authored flags: with them it is
 * a conflict against the scene's 0.01, and without them it is indistinguishable
 * from a participant that never mentioned a timestep.
 */
const TCHAR* const ParticipantXml = TEXT(R"(<mujoco model="robot">
  <option timestep="0.002"/>
  <worldbody>
    <body name="link"><geom name="shell" type="sphere" size="0.1"/></body>
  </worldbody>
</mujoco>)");

/** Parse one document into a scratch Blueprint and build its spec. */
bool BuildFrom(FAutomationTestBase& Test, const FString& Label, const FString& Xml,
	urlab::spec::FMjBuiltSpec& Out)
{
	UBlueprint* const Blueprint =
		MjParitySupport::ParseFixture(Test, TEXT("AttachPolicy"), Label, Xml, Label + TEXT(".xml"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	TArray<FMjSpecDiagnostic> Diagnostics;
	Out = urlab::spec::BuildSpec(FSpecRef::OverBlueprint(*Blueprint), Diagnostics);
	if (Out.Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: the spec did not build: %s"), *Label,
			*MjParitySupport::DiagnosticsToString(Diagnostics)));
		return false;
	}
	return true;
}

/** What one attach did, read off the scene spec before anything else touches it. */
struct FAttachOutcome
{
	bool bRan = false;
	bool bAttached = false;
	int32 Warnings = 0;
	FString Error;
	double SceneTimestep = 0.0;
	int32 ParticipantFlagBefore = -1;
};

/**
 * Compose the two documents above under `Policy` and report what happened.
 *
 * `bParticipantAuthored` chooses whether the participant's flag is left as the
 * spec build wrote it or cleared first, which is how the same fixture answers
 * the question both ways: the values on both sides are identical either way, so
 * any difference in outcome is the flag's doing and nothing else's.
 */
FAttachOutcome Compose(FAutomationTestBase& Test, mjtConflict Policy, bool bParticipantAuthored)
{
	FAttachOutcome Out;

	// Declared participant-first so the scene spec is destroyed first, which is
	// the order the compiled scene keeps: attach moves elements into the scene,
	// and the participants are what the scene may still be pointing at.
	urlab::spec::FMjBuiltSpec Participant;
	urlab::spec::FMjBuiltSpec Scene;
	if (!BuildFrom(Test, TEXT("scene"), SceneXml, Scene)
		|| !BuildFrom(Test, TEXT("participant"), ParticipantXml, Participant))
	{
		return Out;
	}

	Scene.Spec->compiler.conflict = Policy;
	Out.ParticipantFlagBefore = mjs_isAuthored(Participant.Spec, &Participant.Spec->option.timestep);
	if (!bParticipantAuthored)
	{
		mjs_setAuthored(Participant.Spec, &Participant.Spec->option.timestep, 0);
	}

	mjsBody* const Anchor = mjs_findBody(Scene.Spec, "anchor");
	mjsFrame* const Frame = Anchor != nullptr ? mjs_addFrame(Anchor, nullptr) : nullptr;
	if (Frame == nullptr || Frame->element == nullptr || Participant.Spec->element == nullptr)
	{
		Test.AddError(TEXT("could not place the participant in the scene"));
		return Out;
	}

	Out.bRan = true;
	Out.bAttached = mjs_attach(Frame->element, Participant.Spec->element, "p0_", "") != nullptr;

	// Read before anything else touches the spec: a failed attach leaves it
	// unusable and its error string is the last thing on it worth reading.
	const char* const Error = mjs_getError(Scene.Spec);
	Out.Error = Error != nullptr ? FString(UTF8_TO_TCHAR(Error)) : FString();
	Out.Warnings = mjs_numWarnings(Scene.Spec);
	Out.SceneTimestep = Scene.Spec->option.timestep;
	return Out;
}
} // namespace MjAttachPolicyTests

// ============================================================================
// URLab.MuJoCo.AttachPolicy.PolicyReachesTheSpec
//   `<compiler conflict>` is an ordinary authored attribute, so it arrives the
//   ordinary way. Asserted because everything else here depends on it: a policy
//   that never reached the spec would leave every attach on the default.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAttachPolicyReachesTheSpec,
	"URLab.MuJoCo.AttachPolicy.PolicyReachesTheSpec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAttachPolicyReachesTheSpec::RunTest(const FString& Parameters)
{
	using namespace MjAttachPolicyTests;

	const TCHAR* const AuthoredXml = TEXT(R"(<mujoco model="policy">
  <compiler conflict="error"/>
  <worldbody><body name="b"><geom name="g" type="sphere" size="0.1"/></body></worldbody>
</mujoco>)");
	const TCHAR* const SilentXml = TEXT(R"(<mujoco model="policy_default">
  <worldbody><body name="b"><geom name="g" type="sphere" size="0.1"/></body></worldbody>
</mujoco>)");

	urlab::spec::FMjBuiltSpec Authored;
	if (BuildFrom(*this, TEXT("authored_policy"), AuthoredXml, Authored))
	{
		TestEqual(TEXT("an authored conflict policy reaches the compiler block"),
			static_cast<int32>(Authored.Spec->compiler.conflict), static_cast<int32>(mjCONFLICT_ERROR));
	}

	urlab::spec::FMjBuiltSpec Silent;
	if (BuildFrom(*this, TEXT("default_policy"), SilentXml, Silent))
	{
		TestEqual(TEXT("a document that says nothing keeps MuJoCo's default policy"),
			static_cast<int32>(Silent.Spec->compiler.conflict), static_cast<int32>(mjCONFLICT_WARNING));
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.MuJoCo.AttachPolicy.AuthoredFlagsAreWritten
//   The spec build flags what it wrote, in the two blocks the resolver reads
//   and in neither of the two it does not. A spec URLab built and a spec
//   MuJoCo's reader produced then look the same to the resolver.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAttachPolicyAuthoredFlagsAreWritten,
	"URLab.MuJoCo.AttachPolicy.AuthoredFlagsAreWritten",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAttachPolicyAuthoredFlagsAreWritten::RunTest(const FString& Parameters)
{
	using namespace MjAttachPolicyTests;

	// Every authored value here is MuJoCo's own default except the angle: the
	// flags are the only thing that can distinguish this document from silence.
	const TCHAR* const Xml = TEXT(R"(<mujoco model="flags">
  <compiler angle="degree"/>
  <option timestep="0.002"/>
  <visual><map znear="0.005"/></visual>
  <worldbody><body name="b"><geom name="g" type="sphere" size="0.1"/></body></worldbody>
</mujoco>)");

	urlab::spec::FMjBuiltSpec Built;
	if (!BuildFrom(*this, TEXT("flags"), Xml, Built))
	{
		return false;
	}
	mjSpec* const Spec = Built.Spec;

	TestEqual(TEXT("an authored <option> field is flagged, at the default value"),
		mjs_isAuthored(Spec, &Spec->option.timestep), 1);
	TestEqual(TEXT("an <option> field the document never wrote is not flagged"),
		mjs_isAuthored(Spec, &Spec->option.impratio), 0);
	TestEqual(TEXT("an authored <visual> field is flagged"),
		mjs_isAuthored(Spec, &Spec->visual.map.znear), 1);
	TestEqual(TEXT("a <visual> field the document never wrote is not flagged"),
		mjs_isAuthored(Spec, &Spec->visual.map.zfar), 0);

	// Deliberately unflagged: the resolver never visits the compiler block, so a
	// flag there would have no consumer and would only invite the belief that
	// mixed compiler settings are resolved.
	TestEqual(TEXT("the compiler block is left unflagged"),
		mjs_isAuthored(Spec, &Spec->compiler.degree), 0);
	TestEqual(TEXT("and the authored value still reached it"),
		static_cast<int32>(Spec->compiler.degree), 1);

	return !HasAnyErrors();
}

// ============================================================================
// URLab.MuJoCo.AttachPolicy.ResolutionFollowsThePolicy
//   The end-to-end case, on the conflict only the flags can see: a participant
//   that authored MuJoCo's default against a scene that authored something
//   else. Each policy, and then the same attach with the flag cleared, which is
//   what shows the flag is doing the work.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAttachPolicyResolutionFollowsThePolicy,
	"URLab.MuJoCo.AttachPolicy.ResolutionFollowsThePolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAttachPolicyResolutionFollowsThePolicy::RunTest(const FString& Parameters)
{
	using namespace MjAttachPolicyTests;

	const FAttachOutcome Warned = Compose(*this, mjCONFLICT_WARNING, /*bParticipantAuthored=*/true);
	if (!Warned.bRan)
	{
		return false;
	}
	TestEqual(TEXT("the spec build flagged the participant's timestep"),
		Warned.ParticipantFlagBefore, 1);
	TestTrue(TEXT("warning: the attach succeeds"), Warned.bAttached);
	TestEqual(TEXT("warning: the scene keeps its own timestep"), Warned.SceneTimestep, 0.01,
		UE_DOUBLE_KINDA_SMALL_NUMBER);
	TestTrue(TEXT("warning: the conflict is reported"), Warned.Warnings > 0);

	const FAttachOutcome Merged = Compose(*this, mjCONFLICT_MERGE, /*bParticipantAuthored=*/true);
	if (Merged.bRan)
	{
		TestTrue(TEXT("merge: the attach succeeds"), Merged.bAttached);
		TestEqual(TEXT("merge: the smaller timestep wins"), Merged.SceneTimestep, 0.002,
			UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	const FAttachOutcome Refused = Compose(*this, mjCONFLICT_ERROR, /*bParticipantAuthored=*/true);
	if (Refused.bRan)
	{
		TestFalse(TEXT("error: the attach is refused"), Refused.bAttached);
		TestTrue(TEXT("error: the scene carries the reason"), !Refused.Error.IsEmpty());
	}

	// The same documents, the same values, the flag cleared: the conflict is
	// invisible again and even the error policy lets the attach through. This
	// is the blind spot the flags exist to close, and it is what makes the two
	// undeclared symbols load-bearing rather than decorative.
	const FAttachOutcome Unflagged = Compose(*this, mjCONFLICT_ERROR, /*bParticipantAuthored=*/false);
	if (Unflagged.bRan)
	{
		TestTrue(TEXT("unflagged: the same attach succeeds under the error policy"),
			Unflagged.bAttached);
		TestEqual(TEXT("unflagged: nothing is reported"), Unflagged.Warnings, 0);
		TestEqual(TEXT("unflagged: the scene keeps its own timestep"), Unflagged.SceneTimestep, 0.01,
			UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
