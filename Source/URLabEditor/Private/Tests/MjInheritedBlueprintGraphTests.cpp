// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/InheritableComponentHandler.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "Misc/PackageName.h"
#include "UObject/Linker.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectHash.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjSpatial.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjSpatialGeom.gen.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace MjInheritedBlueprintGraphTests
{
const TCHAR* const PackageRoot = TEXT("/Game/URLabInheritedGraphTemp/");

const TCHAR* const ParentXml = TEXT(R"(<mujoco model="inherited_graph">
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="parent_geom" type="sphere" size="0.1"/>
    </body>
  </worldbody>
  <actuator><motor name="drive" joint="hinge"/></actuator>
  <tendon>
    <spatial name="route">
      <geom geom="parent_geom"/>
    </spatial>
  </tendon>
</mujoco>)");

UBlueprint* NewBlueprint(UClass* ParentClass, const TCHAR* Prefix)
{
	const FString Name = FString::Printf(TEXT("%s_%s"), Prefix,
		*FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(FString(PackageRoot) + Name));
	return FKismetEditorUtilities::CreateBlueprint(ParentClass, Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

FString PackageFilename(UPackage& Package, const FString& Extension)
{
	FString Filename;
	FPackageName::TryConvertLongPackageNameToFilename(Package.GetName(), Filename, Extension);
	return Filename;
}

bool SaveBlueprint(UBlueprint& Blueprint, FString& OutFilename)
{
	OutFilename = PackageFilename(*Blueprint.GetPackage(), FPackageName::GetAssetPackageExtension());
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutFilename), true);
	FSavePackageArgs Args;
	Args.TopLevelFlags = RF_Public | RF_Standalone;
	return UPackage::SavePackage(Blueprint.GetPackage(), &Blueprint, *OutFilename, Args);
}

void UnloadPackageForReload(UPackage* Package)
{
	if (Package == nullptr) return;
	ResetLoaders(Package);
	const FString TrashName = Package->GetName() + TEXT("_TRASH_") +
		FGuid::NewGuid().ToString(EGuidFormats::Digits);
	Package->Rename(*TrashName, nullptr, REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
	ForEachObjectWithPackage(Package, [](UObject* Object) {
		Object->ClearFlags(RF_Standalone | RF_Public);
		return true;
	});
	Package->ClearFlags(RF_Standalone | RF_Public);
	CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);
}

void UnloadAndDelete(UPackage* Package, const FString& Filename)
{
	UnloadPackageForReload(Package);
	IFileManager::Get().Delete(*Filename, false, true, true);
}

bool HasGeomSize(const FString& Xml, const TCHAR* Name, double Expected)
{
	const FString OpenTag = FString::Printf(TEXT("<geom name=\"%s\""), Name);
	const int32 GeomAt = Xml.Find(OpenTag, ESearchCase::CaseSensitive, ESearchDir::FromStart);
	if (GeomAt == INDEX_NONE) return false;
	const int32 SizeAt = Xml.Find(TEXT("size=\""), ESearchCase::CaseSensitive,
		ESearchDir::FromStart, GeomAt + OpenTag.Len());
	if (SizeAt == INDEX_NONE) return false;
	const int32 ValueAt = SizeAt + FCString::Strlen(TEXT("size=\""));
	const int32 EndAt = Xml.Find(TEXT("\""), ESearchCase::CaseSensitive,
		ESearchDir::FromStart, ValueAt);
	if (EndAt == INDEX_NONE) return false;
	const FString Value = Xml.Mid(ValueAt, EndAt - ValueAt);
	const int32 SpaceAt = Value.Find(TEXT(" "));
	const FString First = SpaceAt == INDEX_NONE ? Value : Value.Left(SpaceAt);
	return FMath::IsNearlyEqual(FCString::Atod(*First), Expected, 1e-12);
}

USCS_Node* FindNode(UBlueprint& Blueprint, const UClass* ComponentClass)
{
	if (Blueprint.SimpleConstructionScript == nullptr) return nullptr;
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
		if (Node && Node->ComponentTemplate && Node->ComponentTemplate->IsA(ComponentClass)) return Node;
	return nullptr;
}

void RenameMjNameInEditor(UMjNodeComponent& Node, const TCHAR* NewName)
{
	FProperty* const Property = Node.GetClass()->FindPropertyByName(
		GET_MEMBER_NAME_CHECKED(UMjNodeComponent, MjName));
	Node.PreEditChange(Property);
	Node.Modify();
	Node.MjName = FString(NewName);
	FPropertyChangedEvent Event(Property);
	Node.PostEditChangeProperty(Event);
}

void EditMotorJointInEditor(UMjMotor& Motor, const TCHAR* NewTarget)
{
	FProperty* const Property = Motor.GetClass()->FindPropertyByName(
		GET_MEMBER_NAME_CHECKED(UMjMotor, Joint));
	Motor.PreEditChange(Property);
	Motor.Modify();
	Motor.Joint = FString(NewTarget);
	FPropertyChangedEvent Event(Property);
	Motor.PostEditChangeProperty(Event);
}
} // namespace MjInheritedBlueprintGraphTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjInheritedBlueprintGraphTest,
	"URLab.Authoring.InheritedBlueprintGraphIsEditableAndExportable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjInheritedBlueprintGraphTest::RunTest(const FString&)
{
	using namespace MjInheritedBlueprintGraphTests;
	UBlueprint* Parent = NewBlueprint(AActor::StaticClass(), TEXT("MjInheritedParent"));
	if (!TestNotNull(TEXT("parent Blueprint created"), Parent)) return false;
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Parent, ParentXml, TEXT("<inherited-test>"));
	if (!TestTrue(TEXT("parent MJCF imports"), Parsed.IsOk())) return false;
	FKismetEditorUtilities::CompileBlueprint(Parent);

	const FString ParentBefore = FSpecRef::OverBlueprint(*Parent).WriteMjcf();
	USCS_Node* ParentGeomNode = FindNode(*Parent, UMjGeom::StaticClass());
	USCS_Node* ParentSpatialNode = FindNode(*Parent, UMjSpatial::StaticClass());
	USCS_Node* ParentJointNode = FindNode(*Parent, UMjJoint::StaticClass());
	USCS_Node* ParentMotorNode = FindNode(*Parent, UMjMotor::StaticClass());
	if (!TestNotNull(TEXT("base geom SCS node"), ParentGeomNode) ||
		!TestNotNull(TEXT("base tendon SCS node"), ParentSpatialNode) ||
		!TestNotNull(TEXT("base joint SCS node"), ParentJointNode) ||
		!TestNotNull(TEXT("base motor SCS node"), ParentMotorNode)) return false;
	UMjMotor* ParentMotor = Cast<UMjMotor>(ParentMotorNode->ComponentTemplate);
	if (!TestNotNull(TEXT("base motor component template"), ParentMotor)) return false;
	const FName JointField(TEXT("Joint"));
	TestTrue(TEXT("export binds a uniquely resolved authored reference field"),
		ParentMotor->ReferenceTargets.Contains(JointField));
	if (const FComponentKey* BoundJoint = ParentMotor->ReferenceTargets.Find(JointField))
		TestTrue(TEXT("reference binding points to the base joint identity"), BoundJoint->Match(FComponentKey(ParentJointNode)));
	EditMotorJointInEditor(*ParentMotor, TEXT("unresolved_macro_joint"));
	TestFalse(TEXT("manual unresolved text clears stale component binding"),
		ParentMotor->ReferenceTargets.Contains(JointField));
	EditMotorJointInEditor(*ParentMotor, TEXT("hinge"));
	TestTrue(TEXT("manual text edit rebinds a unique target"),
		ParentMotor->ReferenceTargets.Contains(JointField));
	const FComponentKey BoundJointBeforeComponentRename = ParentMotor->ReferenceTargets.FindChecked(JointField);
	ParentJointNode->SetVariableName(FName(TEXT("JointComponentRenamed")));
	TestTrue(TEXT("UE component-variable rename preserves the stable SCS target"),
		BoundJointBeforeComponentRename.FindSCSNode() == ParentJointNode);
	TestTrue(TEXT("UE component-variable rename retains key identity by GUID"),
		BoundJointBeforeComponentRename.Match(FComponentKey(ParentJointNode)));
	FKismetEditorUtilities::CompileBlueprint(Parent);
	const int32 ParentSpatialChildCount = ParentSpatialNode->GetChildNodes().Num();

	UBlueprint* Child = NewBlueprint(Parent->GeneratedClass, TEXT("MjInheritedChild"));
	if (!TestNotNull(TEXT("derived Blueprint created"), Child)) return false;
	FKismetEditorUtilities::CompileBlueprint(Child);
	UBlueprintGeneratedClass* ChildClass = Cast<UBlueprintGeneratedClass>(Child->GeneratedClass);
	if (!TestNotNull(TEXT("derived generated class"), ChildClass)) return false;

	// This is the same UE-owned override path used by the Components panel. The
	// authored base template is never copied into the child's SCS or edited.
	UInheritableComponentHandler* Handler = Child->GetInheritableComponentHandler(true);
	if (!TestNotNull(TEXT("UE inheritable component handler"), Handler)) return false;
	UActorComponent* OverrideObject = Handler->CreateOverridenComponentTemplate(FComponentKey(ParentGeomNode));
	UMjGeom* OverrideGeom = Cast<UMjGeom>(OverrideObject);
	if (!TestNotNull(TEXT("UE override template for inherited geom"), OverrideGeom)) return false;
	OverrideGeom->Size = TArray<double>{0.2};
	TestTrue(TEXT("inherited geom override authors a size"), OverrideGeom->Size.IsSet());

	UMjSpatial* EffectiveSpatial = nullptr;
	UMjGeom* EffectiveGeom = nullptr;
	UMjSpatialGeom* AddedWrap = nullptr;
	FString ChildXml;
	{
		urlab::spec::FMjScsScope Scope(*Child);
		EffectiveSpatial = Cast<UMjSpatial>(Scope.TemplateFor(*ParentSpatialNode));
		EffectiveGeom = Cast<UMjGeom>(Scope.TemplateFor(*ParentGeomNode));
		if (!TestNotNull(TEXT("effective inherited tendon template"), EffectiveSpatial) ||
			!TestNotNull(TEXT("effective inherited geom override"), EffectiveGeom)) return false;
		TestTrue(TEXT("effective geom template is UE's override"), EffectiveGeom == OverrideGeom);

		// A newly authored child of an inherited SCS component is a local root
		// whose UE parent key names the ancestor component.
		AddedWrap = urlab::spec::FScsNodeFactory::Construct<UMjSpatialGeom>(nullptr);
		if (!TestNotNull(TEXT("derived tendon geom authored"), AddedWrap)) return false;
		AddedWrap->Geom = TEXT("parent_geom");
		urlab::spec::FMjScsAdapter::Attach(*EffectiveSpatial, *AddedWrap);

		const TArray<FString> RefOptions = AddedWrap->GetGeomOptions();
		TestTrue(TEXT("derived reference dropdown sees inherited geom"),
			RefOptions.Contains(TEXT("parent_geom")));
		const FSpecRef ChildSpec = FSpecRef::OverBlueprint(*Child);
		TestTrue(TEXT("derived effective root exists"), ChildSpec.IsValid());
		TestTrue(TEXT("derived traversal includes inherited geom"),
			ChildSpec.NamesOfType(UMjGeom::StaticClass()).Contains(TEXT("parent_geom")));
		ChildXml = ChildSpec.WriteMjcf();
	}
	TestTrue(TEXT("derived export contains local inherited-parent child"),
		ChildXml.Contains(TEXT("<geom geom=\"parent_geom\"")));
	TestTrue(TEXT("derived export contains the child's inherited geom override"),
		HasGeomSize(ChildXml, TEXT("parent_geom"), 0.2));
	UMjSpatialGeom* DuplicatedWrap = nullptr;
	{
		urlab::spec::FMjScsScope Scope(*Child);
		EffectiveSpatial = Cast<UMjSpatial>(Scope.TemplateFor(*ParentSpatialNode));
		if (!TestNotNull(TEXT("effective tendon remains available for copied referrer"), EffectiveSpatial)) return false;
		DuplicatedWrap = urlab::spec::FScsNodeFactory::Construct<UMjSpatialGeom>(nullptr);
		if (!TestNotNull(TEXT("source-only reference component copy created"), DuplicatedWrap)) return false;
		DuplicatedWrap->Geom = AddedWrap->Geom;
		DuplicatedWrap->ReferenceTargets = AddedWrap->ReferenceTargets;
		urlab::spec::FMjScsAdapter::Attach(*EffectiveSpatial, *DuplicatedWrap);
	}
	const FString WithCopiedReferrerXml = FSpecRef::OverBlueprint(*Child).WriteMjcf();
	TestTrue(TEXT("source-only duplicated referrer still exports its raw name"),
		WithCopiedReferrerXml.Contains(TEXT("<geom geom=\"parent_geom\"")));
	if (const FComponentKey* CopiedTarget = DuplicatedWrap->ReferenceTargets.Find(FName(TEXT("Geom"))))
		TestTrue(TEXT("source-only duplicated referrer keeps its original target identity"),
			CopiedTarget->Match(FComponentKey(ParentGeomNode)));
	else
		TestTrue(TEXT("source-only duplicated referrer retains copied target identity"), false);
	TestEqual(TEXT("base template child list was not mutated"),
		ParentSpatialNode->GetChildNodes().Num(), ParentSpatialChildCount);
	TestEqual(TEXT("base authored export is unchanged"),
		FSpecRef::OverBlueprint(*Parent).WriteMjcf(), ParentBefore);

	// Renaming an inherited target must create a UE-owned override for its
	// inherited referrer before the reference is rewritten. This catches the
	// failure where child graph traversal mutates the parent's motor template.
	UActorComponent* JointOverrideObject = Handler->CreateOverridenComponentTemplate(
		FComponentKey(ParentJointNode));
	UMjJoint* JointOverride = Cast<UMjJoint>(JointOverrideObject);
	if (!TestNotNull(TEXT("UE override template for inherited joint"), JointOverride)) return false;
	UMjNodeComponent* const ParentJointTemplate = Cast<UMjNodeComponent>(ParentJointNode->ComponentTemplate);
	if (!TestNotNull(TEXT("base joint component template"), ParentJointTemplate)) return false;
	TestEqual(TEXT("inherited override preserves authored Serial"), JointOverride->Serial, ParentJointTemplate->Serial);
	RenameMjNameInEditor(*JointOverride, TEXT("elbow"));
	const FString RenamedChildXml = FSpecRef::OverBlueprint(*Child).WriteMjcf();
	TestTrue(TEXT("renamed child joint exports its new name"),
		RenamedChildXml.Contains(TEXT("<joint name=\"elbow\"")));
	TestTrue(TEXT("inherited motor reference follows renamed joint in child"),
		RenamedChildXml.Contains(TEXT("joint=\"elbow\"")));
	TestEqual(TEXT("renaming child joint leaves parent references and target unchanged"),
		FSpecRef::OverBlueprint(*Parent).WriteMjcf(), ParentBefore);
	UActorComponent* MotorOverrideObject = Handler->GetOverridenComponentTemplate(FComponentKey(ParentMotorNode));
	UMjMotor* MotorOverride = Cast<UMjMotor>(MotorOverrideObject);
	if (TestNotNull(TEXT("UE override template for inherited motor referrer"), MotorOverride))
	{
		TestTrue(TEXT("renamed inherited motor keeps stable target identity"), MotorOverride->ReferenceTargets.Contains(JointField));
		if (const FComponentKey* BoundJoint = MotorOverride->ReferenceTargets.Find(JointField))
			TestTrue(TEXT("child motor identity still targets the inherited joint component"),
				BoundJoint->Match(FComponentKey(ParentJointNode)));
	}

	// Save the base and derived assets, then place the child in a level and save
	// that level too. A fresh package load is the persistence boundary this graph
	// is intended to support.
	FKismetEditorUtilities::CompileBlueprint(Parent);
	FKismetEditorUtilities::CompileBlueprint(Child);
	ChildClass = Cast<UBlueprintGeneratedClass>(Child->GeneratedClass);
	if (!TestNotNull(TEXT("compiled derived generated class"), ChildClass)) return false;
	Parent->SetFlags(RF_Public | RF_Standalone | RF_Transactional);
	Child->SetFlags(RF_Public | RF_Standalone | RF_Transactional);
	FString ParentFilename;
	FString ChildFilename;
	if (!TestTrue(TEXT("save parent Blueprint"), SaveBlueprint(*Parent, ParentFilename)) ||
		!TestTrue(TEXT("save derived Blueprint"), SaveBlueprint(*Child, ChildFilename))) return false;
	UPackage* const ParentPackage = Parent->GetPackage();
	UPackage* const ChildPackage = Child->GetPackage();
	const FString ParentPackageName = ParentPackage->GetName();
	const FString ParentBlueprintName = Parent->GetName();
	const FString ChildPackageName = ChildPackage->GetName();
	const FString ChildBlueprintName = Child->GetName();

	const FString MapPackageName = FString(PackageRoot) + TEXT("Placed_") +
		FGuid::NewGuid().ToString(EGuidFormats::Digits);
	UPackage* MapPackage = CreatePackage(*MapPackageName);
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false,
		*(TEXT("MjInheritedPlaced_") + FGuid::NewGuid().ToString(EGuidFormats::Digits)),
		MapPackage, false);
	if (!TestNotNull(TEXT("level world created"), World)) return false;
	AActor* ChildInstance = World->SpawnActor<AActor>(ChildClass);
	if (!TestNotNull(TEXT("derived Blueprint placed in transient world"), ChildInstance))
	{
		World->DestroyWorld(false);
		return false;
	}
	TArray<UMjJoint*> InstanceJoints;
	ChildInstance->GetComponents<UMjJoint>(InstanceJoints);
	UMjJoint* InstanceJoint = nullptr;
	for (UMjJoint* Joint : InstanceJoints)
		if (Joint && Joint->MjName.IsSet() && Joint->MjName.GetValue() == TEXT("elbow")) InstanceJoint = Joint;
	if (!TestNotNull(TEXT("placed inherited joint component"), InstanceJoint)) return false;
	RenameMjNameInEditor(*InstanceJoint, TEXT("placed_elbow"));
	// Editing an instance property may reconstruct its Blueprint actor. Spec
	// refs are short-lived graph views, so resolve the actor graph after edits.
	const FSpecRef RuntimeSpec = FSpecRef::OverActor(*ChildInstance);
	const FString RuntimeBeforeSave = RuntimeSpec.WriteMjcf();
	TestTrue(TEXT("placed actor uses child inherited override"),
		HasGeomSize(RuntimeBeforeSave, TEXT("parent_geom"), 0.2));
	TestTrue(TEXT("placed actor exports its instance-level joint name override"),
		RuntimeBeforeSave.Contains(TEXT("<joint name=\"placed_elbow\"")));
	TestTrue(TEXT("placed actor rename immediately repoints the motor by stable identity"),
		RuntimeBeforeSave.Contains(TEXT("joint=\"placed_elbow\"")));
	TestTrue(TEXT("instance rename leaves child Blueprint target name unchanged"),
		FSpecRef::OverBlueprint(*Child).WriteMjcf().Contains(TEXT("<joint name=\"elbow\"")));
	TestTrue(TEXT("placed actor uses local inherited-parent child"),
		RuntimeBeforeSave.Contains(TEXT("<geom geom=\"parent_geom\"")));
	TArray<UMjGeom*> InstanceGeoms;
	ChildInstance->GetComponents<UMjGeom>(InstanceGeoms);
	UMjGeom* InstanceOverrideGeom = nullptr;
	for (UMjGeom* Geom : InstanceGeoms)
		if (Geom && Geom->MjName.IsSet() && Geom->MjName.GetValue() == TEXT("parent_geom"))
			InstanceOverrideGeom = Geom;
	if (TestNotNull(TEXT("placed inherited geom component"), InstanceOverrideGeom))
		InstanceOverrideGeom->Size = TArray<double>{0.3};

	const FString MapFilename = PackageFilename(*MapPackage, FPackageName::GetMapPackageExtension());
	FSavePackageArgs MapSaveArgs;
	MapSaveArgs.TopLevelFlags = RF_NoFlags;
	if (!TestTrue(TEXT("save placed-instance level"),
		UPackage::SavePackage(MapPackage, World, *MapFilename, MapSaveArgs))) return false;
	World->DestroyWorld(false);
	World = nullptr;

	// Move all loaded copies out of the way; LoadPackage must read the saved
	// parent, child, and level data from disk.
	UnloadPackageForReload(MapPackage);
	UnloadPackageForReload(ChildPackage);
	UnloadPackageForReload(ParentPackage);
	UPackage* ReloadedMapPackage = LoadPackage(nullptr, *MapPackageName, LOAD_None);
	if (!TestNotNull(TEXT("reloaded level package"), ReloadedMapPackage)) return false;
	UWorld* ReloadedWorld = UWorld::FindWorldInPackage(ReloadedMapPackage);
	if (!TestNotNull(TEXT("reloaded placed-instance world"), ReloadedWorld)) return false;
	AActor* ReloadedInstance = nullptr;
	for (AActor* Actor : ReloadedWorld->PersistentLevel->Actors)
		if (Actor && Actor->GetClass()->ClassGeneratedBy != nullptr &&
			Actor->GetClass()->ClassGeneratedBy->GetName().StartsWith(TEXT("MjInheritedChild_")))
			ReloadedInstance = Actor;
	if (!TestNotNull(TEXT("reloaded derived Blueprint instance"), ReloadedInstance)) return false;
	const FSpecRef ReloadedSpec = FSpecRef::OverActor(*ReloadedInstance);
	const FString ReloadedXml = ReloadedSpec.WriteMjcf();
	TestTrue(TEXT("reloaded placed export retains local child"),
		ReloadedXml.Contains(TEXT("<geom geom=\"parent_geom\"")));
	TestTrue(TEXT("reloaded placed export retains the child Blueprint override"),
		HasGeomSize(ReloadedXml, TEXT("parent_geom"), 0.3));
	TestTrue(TEXT("reloaded placed actor retains its instance target name override"),
		ReloadedXml.Contains(TEXT("<joint name=\"placed_elbow\"")));
	TestTrue(TEXT("reloaded placed actor retains stable-reference rename fixup"),
		ReloadedXml.Contains(TEXT("joint=\"placed_elbow\"")));

	UPackage* ReloadedChildPackage = FindPackage(nullptr, *ChildPackageName);
	UBlueprint* ReloadedChild = ReloadedChildPackage
		? FindObject<UBlueprint>(ReloadedChildPackage, *ChildBlueprintName) : nullptr;
	UPackage* ReloadedParentPackage = FindPackage(nullptr, *ParentPackageName);
	UBlueprint* ReloadedParent = ReloadedParentPackage
		? FindObject<UBlueprint>(ReloadedParentPackage, *ParentBlueprintName) : nullptr;
	if (TestNotNull(TEXT("reloaded child Blueprint asset"), ReloadedChild))
	{
		const FSpecRef ReloadedChildSpec = FSpecRef::OverBlueprint(*ReloadedChild);
		USCS_Node* ReloadedParentMotorNode = ReloadedParent
			? FindNode(*ReloadedParent, UMjMotor::StaticClass()) : nullptr;
		if (ReloadedParentMotorNode != nullptr)
		{
			urlab::spec::FMjScsScope ReloadedScope(*ReloadedChild);
			UMjMotor* ReloadedEffectiveMotor = Cast<UMjMotor>(ReloadedScope.TemplateFor(*ReloadedParentMotorNode));
			if (TestNotNull(TEXT("reloaded effective inherited motor"), ReloadedEffectiveMotor))
			{
				TestTrue(TEXT("reloaded reference sidecar keeps joint identity"),
					ReloadedEffectiveMotor->ReferenceTargets.Contains(JointField));
				if (const FComponentKey* BoundJoint = ReloadedEffectiveMotor->ReferenceTargets.Find(JointField))
				{
					USCS_Node* const ReloadedParentJointNode = FindNode(*ReloadedParent, UMjJoint::StaticClass());
					if (TestNotNull(TEXT("reloaded parent joint SCS node"), ReloadedParentJointNode))
						TestTrue(TEXT("reloaded sidecar target key resolves to original parent SCS node"),
							BoundJoint->Match(FComponentKey(ReloadedParentJointNode)));
				}
			}
		}
		TestTrue(TEXT("reloaded child asset exports inherited override"),
			HasGeomSize(ReloadedChildSpec.WriteMjcf(), TEXT("parent_geom"), 0.2));
		TestTrue(TEXT("reloaded child asset keeps renamed inherited joint"),
			ReloadedChildSpec.WriteMjcf().Contains(TEXT("<joint name=\"elbow\"")));
		TestTrue(TEXT("reloaded child asset keeps inherited motor reference fixup"),
			ReloadedChildSpec.WriteMjcf().Contains(TEXT("joint=\"elbow\"")));
	}
	if (TestNotNull(TEXT("reloaded parent Blueprint asset"), ReloadedParent))
		TestEqual(TEXT("parent Blueprint export remains unchanged after child override and reload"),
			FSpecRef::OverBlueprint(*ReloadedParent).WriteMjcf(), ParentBefore);

	if (ReloadedParent && ReloadedChild)
	{
		USCS_Node* ReloadedJointNode = FindNode(*ReloadedParent, UMjJoint::StaticClass());
		if (ReloadedJointNode != nullptr)
		{
			const FSpecRef BeforeDelete = FSpecRef::OverBlueprint(*ReloadedChild);
			ReloadedParent->SimpleConstructionScript->RemoveNode(ReloadedJointNode);
			FString IdentityError;
			TestFalse(TEXT("deleting a bound target makes identity validation fail"),
				urlab::spec::MjSynchronizeReferenceBindings(BeforeDelete, IdentityError));
			TestTrue(TEXT("deleted bound target produces explicit identity error"),
				IdentityError.Contains(TEXT("deleted or incompatible component")));
			TArray<FMjSpecDiagnostic> ExportErrors;
			TestTrue(TEXT("export is blocked after deleting a bound reference target"),
				BeforeDelete.WriteMjcf(&ExportErrors).IsEmpty());
			TestTrue(TEXT("blocked export reports the stable-reference failure"), ExportErrors.Num() > 0);
			UMjMotor* EffectiveMotor = nullptr;
			{
				urlab::spec::FMjScsScope Scope(*ReloadedChild);
				USCS_Node* MotorNode = FindNode(*ReloadedParent, UMjMotor::StaticClass());
				if (MotorNode) EffectiveMotor = Cast<UMjMotor>(Scope.TemplateFor(*MotorNode));
			}
			if (TestNotNull(TEXT("effective inherited motor after target deletion"), EffectiveMotor))
			{
				TestTrue(TEXT("deleted target marker is retained on reference field"),
					EffectiveMotor->UnresolvedReferenceFields.Contains(JointField));
				EditMotorJointInEditor(*EffectiveMotor, TEXT("manual_raw_joint"));
				TestFalse(TEXT("deliberate raw edit clears deleted-target marker"),
					EffectiveMotor->UnresolvedReferenceFields.Contains(JointField));
				TestFalse(TEXT("manual unresolved raw reference does not get guessed a target"),
					EffectiveMotor->ReferenceTargets.Contains(JointField));
			}
		}
	}

	ReloadedWorld->DestroyWorld(false);
	UnloadAndDelete(ReloadedMapPackage, MapFilename);
	UnloadAndDelete(ReloadedChildPackage, ChildFilename);
	UnloadAndDelete(ReloadedParentPackage, ParentFilename);
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
