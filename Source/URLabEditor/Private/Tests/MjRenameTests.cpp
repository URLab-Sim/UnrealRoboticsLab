// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What happens to the rest of the model when one element is renamed.
//
// Every cross-reference in MuJoCo is a name resolved at compile time: an
// actuator names the joint it drives, a geom names its material. Renaming the
// joint in the details panel corrects every referrer in the same transaction,
// so nothing is left pointing at a name nothing answers to, and a name that
// resolves to nothing is reported on the element carrying it.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Editor.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "IMessageLogListing.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "MessageLogModule.h"
#include "Misc/Guid.h"
#include "Modules/ModuleManager.h"
#include "ScopedTransaction.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjRenameTests
{

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjRename_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		TArray<FString> Lines;
		for (const FMjSpecDiagnostic& Diagnostic : Parsed.Errors)
		{
			Lines.Add(Diagnostic.ToString());
		}
		Test.AddError(FString::Printf(TEXT("parse failed: %s"), *FString::Join(Lines, TEXT("; "))));
		return nullptr;
	}
	return Blueprint;
}

/** The element template whose MJCF name is `MjName`. */
UMjNodeComponent* ElementNamed(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Element;
		}
	}
	return nullptr;
}

/** Rename `Element` the way the details panel does, hooks and all. */
void RenameInPanel(UMjNodeComponent& Element, const TCHAR* NewName)
{
	FProperty* Property = Element.GetClass()->FindPropertyByName(FName(TEXT("MjName")));
	Element.PreEditChange(Property);
	Element.Modify();
	Element.MjName = FString(NewName);
	FPropertyChangedEvent Event(Property);
	Element.PostEditChangeProperty(Event);
}

const TCHAR* const DrivenJoint = TEXT(R"(<mujoco model="rename">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="shape" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="hinge"/>
  </actuator>
</mujoco>
)");

} // namespace MjRenameTests

// ============================================================================
// URLab.Spec.RenamingAnElementFollowsItsReferrers
//   The actuator drives the joint by name. Rename the joint and the actuator
//   has to come with it, or the model stops compiling for a reason the user
//   never sees.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenameFollowsReferrersTest, "URLab.Spec.RenamingAnElementFollowsItsReferrers",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenameFollowsReferrersTest::RunTest(const FString& Parameters)
{
	using namespace MjRenameTests;

	UBlueprint* Blueprint = ParseScratch(*this, DrivenJoint);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* Joint = ElementNamed(*Blueprint, TEXT("hinge"));
	if (!TestNotNull(TEXT("the imported joint is in the construction script"), Joint))
	{
		return false;
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	if (!TestTrue(TEXT("the actuator starts out driving 'hinge'"),
			Spec.WriteMjcf().Contains(TEXT("joint=\"hinge\""), ESearchCase::CaseSensitive)))
	{
		return false;
	}

	RenameInPanel(*Joint, TEXT("elbow"));

	const FString Mjcf = Spec.WriteMjcf();
	TestTrue(TEXT("the joint is written under its new name"),
		Mjcf.Contains(TEXT("name=\"elbow\""), ESearchCase::CaseSensitive));
	TestTrue(TEXT("and the actuator drives it under that name"),
		Mjcf.Contains(TEXT("joint=\"elbow\""), ESearchCase::CaseSensitive));
	// Spelled out per attribute rather than as a bare search for the old string:
	// the joint's TYPE is also `hinge`, and `type="hinge"` is not a reference to
	// anything. A search for the word alone asserts that a MuJoCo keyword
	// disappeared, which is not what a rename does or should do.
	TestFalse(TEXT("no element is left named by the old name"),
		Mjcf.Contains(TEXT("name=\"hinge\""), ESearchCase::CaseSensitive));
	TestFalse(TEXT("nothing is left pointing at the old name"),
		Mjcf.Contains(TEXT("joint=\"hinge\""), ESearchCase::CaseSensitive));

	// And nothing is reported dangling, because nothing is.
	const UMjNodeComponent* Motor = ElementNamed(*Blueprint, TEXT("drive"));
	if (TestNotNull(TEXT("the actuator is in the construction script"), Motor))
	{
		TestEqual(TEXT("the corrected reference resolves"), Motor->DanglingReferences.Num(), 0);
	}

	return true;
}

// ============================================================================
// URLab.Spec.ARenameAndItsFixupUndoTogether
//   The referrers are corrected inside the transaction the panel opened, so one
//   undo puts the whole model back. Two undos, or one that leaves the actuator
//   on the new name, would be a model the user cannot restore by hand.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenameUndoTest, "URLab.Spec.ARenameAndItsFixupUndoTogether",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenameUndoTest::RunTest(const FString& Parameters)
{
	using namespace MjRenameTests;

	if (!TestNotNull(TEXT("the undo test needs an editor"), GEditor))
	{
		return false;
	}

	UBlueprint* Blueprint = ParseScratch(*this, DrivenJoint);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjNodeComponent* Joint = ElementNamed(*Blueprint, TEXT("hinge"));
	if (!TestNotNull(TEXT("the imported joint is in the construction script"), Joint))
	{
		return false;
	}

	{
		FScopedTransaction Transaction(FText::FromString(TEXT("Rename MuJoCo element")));
		RenameInPanel(*Joint, TEXT("elbow"));
	}

	const FSpecRef Spec = FSpecRef::OverBlueprint(*Blueprint);
	if (!TestTrue(TEXT("the rename and its fixup landed"),
			Spec.WriteMjcf().Contains(TEXT("joint=\"elbow\""), ESearchCase::CaseSensitive)))
	{
		return false;
	}

	GEditor->UndoTransaction();

	const FString Mjcf = Spec.WriteMjcf();
	TestTrue(TEXT("undo restores the element's own name"),
		Mjcf.Contains(TEXT("name=\"hinge\""), ESearchCase::CaseSensitive));
	TestTrue(TEXT("and the referrer with it"),
		Mjcf.Contains(TEXT("joint=\"hinge\""), ESearchCase::CaseSensitive));

	return true;
}

// ============================================================================
// URLab.Spec.ANameThatResolvesToNothingIsReported
//   A reference is a name, and a name that answers to nothing is silent until
//   MuJoCo refuses the model. The element carrying it says so instead, from the
//   moment the model is read.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDanglingReferenceTest, "URLab.Spec.ANameThatResolvesToNothingIsReported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDanglingReferenceTest::RunTest(const FString& Parameters)
{
	using namespace MjRenameTests;

	const TCHAR* const Xml = TEXT(R"(<mujoco model="dangling">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="shape" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="knee"/>
  </actuator>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const UMjNodeComponent* Motor = ElementNamed(*Blueprint, TEXT("drive"));
	if (!TestNotNull(TEXT("the actuator is in the construction script"), Motor))
	{
		return false;
	}

	if (TestEqual(TEXT("the actuator reports its one unresolvable reference"), Motor->DanglingReferences.Num(), 1))
	{
		TestTrue(TEXT("and the report names the name that failed"),
			Motor->DanglingReferences[0].Contains(TEXT("knee")));
	}

	// The elements whose references do resolve say nothing, or the report is
	// noise rather than a diagnostic.
	const UMjNodeComponent* Geom = ElementNamed(*Blueprint, TEXT("shape"));
	if (TestNotNull(TEXT("the geom is in the construction script"), Geom))
	{
		TestEqual(TEXT("an element with no bad reference reports nothing"), Geom->DanglingReferences.Num(), 0);
	}

	// Pointing it at something real clears the report, at edit time.
	UMjNodeComponent* Joint = ElementNamed(*Blueprint, TEXT("hinge"));
	if (TestNotNull(TEXT("the imported joint is in the construction script"), Joint))
	{
		RenameInPanel(*Joint, TEXT("knee"));
		TestEqual(TEXT("a name that now resolves stops being reported"), Motor->DanglingReferences.Num(), 0);
	}

	return true;
}

// ============================================================================
// URLab.Spec.ADanglingNameReachesTheMessageLog
//   The component row is where the answer is, and it is only read by someone
//   who already suspects that element. The user who has just broken a reference
//   does not know which element to select, so the diagnostic also goes to the
//   editor's own Messages panel, naming the referrer and the missing target.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDanglingReferenceIsLoudTest, "URLab.Spec.ADanglingNameReachesTheMessageLog",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDanglingReferenceIsLoudTest::RunTest(const FString& Parameters)
{
	using namespace MjRenameTests;

	const TCHAR* const Xml = TEXT(R"(<mujoco model="loud">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="shape" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="knee"/>
  </actuator>
</mujoco>
)");

	// Read back from the listing rather than trusting that the call was made:
	// the panel is where a user goes looking, and a message that never reaches
	// it is invisible in exactly the case it exists for.
	FMessageLogModule& MessageLogModule = FModuleManager::LoadModuleChecked<FMessageLogModule>(TEXT("MessageLog"));
	if (!TestTrue(TEXT("the \"URLab\" listing is registered"),
			MessageLogModule.IsRegisteredLogListing(TEXT("URLab"))))
	{
		return false;
	}
	const TSharedRef<IMessageLogListing> Listing = MessageLogModule.GetLogListing(TEXT("URLab"));
	Listing->ClearMessages();

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	const FString Listed = Listing->GetAllMessagesAsString();
	TestTrue(FString::Printf(TEXT("the referring element is named, got '%s'"), *Listed),
		Listed.Contains(TEXT("drive")));
	TestTrue(FString::Printf(TEXT("and the name that answers to nothing, got '%s'"), *Listed),
		Listed.Contains(TEXT("knee")));

	// A model whose references all resolve says nothing at all, or the panel
	// fills with lines that mean a user must read every one to find a real one.
	Listing->ClearMessages();
	const TCHAR* const Clean = TEXT(R"(<mujoco model="quiet">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="shape" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="hinge"/>
  </actuator>
</mujoco>
)");
	if (ParseScratch(*this, Clean) != nullptr)
	{
		TestEqual(TEXT("a model whose references resolve writes nothing to the log"),
			Listing->GetAllMessagesAsString(), FString());
	}

	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
