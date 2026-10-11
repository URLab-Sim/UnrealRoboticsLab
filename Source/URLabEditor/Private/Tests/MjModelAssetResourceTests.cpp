// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/Blueprint.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjModelAsset.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "MuJoCo/Spec/MjModelAssetResources.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MjChildModelImport.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if URLAB_MJ_GEN && WITH_DEV_AUTOMATION_TESTS
namespace
{
UBlueprint* NewModelBlueprint(const TCHAR* Label)
{
	const FString Name = FString::Printf(TEXT("%s_%s"), Label,
		*FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(FString(TEXT("/Temp/URLabModelAssetTests/")) + Name));
	return Package != nullptr
		? FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package, FName(*Name),
			BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass())
		: nullptr;
}

bool ImportSpec(UBlueprint& Blueprint, const TCHAR* Xml)
{
	return MjParseIntoBlueprint(Blueprint, Xml, TEXT("<model-asset-test>")).IsOk();
}

template <typename T>
T* FindElement(UBlueprint& Blueprint)
{
	const FSpecRef Spec = FSpecRef::OverBlueprint(Blueprint);
	if (!Spec.IsValid()) return nullptr;
	TArray<UMjNodeComponent*> Stack{Spec.GetRoot()};
	while (!Stack.IsEmpty())
	{
		UMjNodeComponent* Parent = Stack.Pop(EAllowShrinking::No);
		if (T* Found = Cast<T>(Parent)) return Found;
		for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *Parent))
			if (Child.Node != nullptr) Stack.Add(Child.Node);
	}
	return nullptr;
}

template <typename T>
TArray<T*> FindElements(UBlueprint& Blueprint)
{
	TArray<T*> Found;
	const FSpecRef Spec = FSpecRef::OverBlueprint(Blueprint);
	if (!Spec.IsValid()) return Found;
	TArray<UMjNodeComponent*> Stack{Spec.GetRoot()};
	while (!Stack.IsEmpty())
	{
		UMjNodeComponent* Parent = Stack.Pop(EAllowShrinking::No);
		if (T* Element = Cast<T>(Parent)) Found.Add(Element);
		for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *Parent))
			if (Child.Node != nullptr) Stack.Add(Child.Node);
	}
	return Found;
}

bool HasNoImportProvenance(UBlueprint& Blueprint)
{
	const FSpecRef Spec = FSpecRef::OverBlueprint(Blueprint);
	if (!Spec.IsValid()) return false;
	TArray<UMjNodeComponent*> Stack{Spec.GetRoot()};
	while (!Stack.IsEmpty())
	{
		UMjNodeComponent* Parent = Stack.Pop(EAllowShrinking::No);
		if (Parent == nullptr || !Parent->SourceFile.IsEmpty()) return false;
		for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *Parent))
			if (Child.Node != nullptr) Stack.Add(Child.Node);
	}
	return true;
}

FString AsUtf8(const TArray<uint8>& Bytes)
{
	if (Bytes.IsEmpty()) return FString();
	const FUTF8ToTCHAR Decoded(reinterpret_cast<const ANSICHAR*>(Bytes.GetData()), Bytes.Num());
	return FString(Decoded.Length(), Decoded.Get());
}

struct FNativeModelGuard
{
	mjModel* Model = nullptr;
	~FNativeModelGuard() { if (Model != nullptr) mj_deleteModel(Model); }
};

bool SaveFixture(const FString& Path, const TCHAR* Xml)
{
	return FFileHelper::SaveStringToFile(Xml, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

mjModel* CompileBundle(FAutomationTestBase& Test, const FMjSourceBundle& Bundle)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FMjGeneratedBundleResource& Resource : Bundle.Resources)
	{
		if (mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Resource.Key), Resource.Bytes.GetData(), Resource.Bytes.Num()) != 0)
		{
			Test.AddError(FString::Printf(TEXT("could not mount generated child resource '%s'"), *Resource.Key));
			mj_deleteVFS(&Vfs);
			return nullptr;
		}
	}
	char Error[2048]{};
	mjSpec* Spec = mj_parseXMLString(TCHAR_TO_UTF8(*Bundle.Xml), &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("MuJoCo rejected the generated parent and child MJCF: %s"), UTF8_TO_TCHAR(Error)));
		mj_deleteVFS(&Vfs);
		return nullptr;
	}
	mjModel* Model = mj_compile(Spec, &Vfs);
	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("MuJoCo could not compile the generated parent and child payload: %s"),
			UTF8_TO_TCHAR(mjs_getError(Spec))));
	}
	mj_deleteSpec(Spec);
	mj_deleteVFS(&Vfs);
	return Model;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelAssetChildBlueprintResourceTest,
	"URLab.Authoring.ModelAssetChildBlueprintGeneratesLiveNativePayload",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelAssetChildBlueprintResourceTest::RunTest(const FString&)
{
	UBlueprint* const Child = NewModelBlueprint(TEXT("MjChildModel"));
	UBlueprint* const Parent = NewModelBlueprint(TEXT("MjParentModel"));
	if (!TestNotNull(TEXT("child Blueprint"), Child) || !TestNotNull(TEXT("parent Blueprint"), Parent)) return false;
	TestTrue(TEXT("child authored tree parses"), ImportSpec(*Child,
		TEXT("<mujoco model=\"child\"><worldbody><body name=\"child_body\"><geom name=\"child_geom\" type=\"sphere\" size=\"0.1\"/></body></worldbody></mujoco>")));
	TestTrue(TEXT("parent authored tree parses"), ImportSpec(*Parent,
		TEXT("<mujoco model=\"parent\"><asset><model name=\"child_left\" file=\"import-left.xml\"/><model name=\"child_right\" file=\"import-right.xml\"/></asset><worldbody><attach model=\"child_left\" prefix=\"left_\"/><attach model=\"child_right\" prefix=\"right_\"/></worldbody></mujoco>")));
	TArray<UMjModelAsset*> ModelAssets = FindElements<UMjModelAsset>(*Parent);
	UMjGeom* const ChildGeom = FindElement<UMjGeom>(*Child);
	if (!TestEqual(TEXT("parent has two child model refs"), ModelAssets.Num(), 2) ||
		!TestNotNull(TEXT("child authored geom"), ChildGeom) ||
		!TestNotNull(TEXT("child generated class"), Child->GeneratedClass.Get())) return false;
	for (UMjModelAsset* ModelAsset : ModelAssets) ModelAsset->ChildModelClass = Child->GeneratedClass;

	TArray<FMjPreparedAssetResource> BeforeRows;
	FString Error;
	if (!TestTrue(TEXT("child model generates a native compiler closure"),
		MjBuildChildModelResources(*ModelAssets[0], BeforeRows, Error)))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("first row is the generated model MJCF"), BeforeRows.Num() > 0 &&
		BeforeRows[0].Attribute == TEXT("file") && BeforeRows[0].Bytes.Num() > 0);
	const FString BeforeXml = BeforeRows.IsEmpty() ? FString() : AsUtf8(BeforeRows[0].Bytes);
	TestTrue(TEXT("generated XML contains live child body"), BeforeXml.Contains(TEXT("child_body")));
	TArray<FMjPreparedAssetResource> ReusedRows;
	TestTrue(TEXT("same child Blueprint can be attached a second time"),
		MjBuildChildModelResources(*ModelAssets[1], ReusedRows, Error));
	TestTrue(TEXT("reused child class has the same deterministic native mount name"),
		!BeforeRows.IsEmpty() && !ReusedRows.IsEmpty() && BeforeRows[0].VfsName == ReusedRows[0].VfsName);
	const FString ParentXml = FSpecRef::OverBlueprint(*Parent).WriteMjcf();
	TestTrue(TEXT("native attach prefixes remain authored for both references"),
		ParentXml.Contains(TEXT("prefix=\"left_\"")) && ParentXml.Contains(TEXT("prefix=\"right_\"")));
	TestTrue(TEXT("model filename is derived from Blueprint identity"),
		BeforeRows.Num() > 0 && BeforeRows[0].File.StartsWith(TEXT("__urlab_models/")) &&
		!BeforeRows[0].File.Contains(TEXT("import-only.xml")));

	ChildGeom->Size = TArray<double>{0.25};
	TArray<FMjPreparedAssetResource> AfterRows;
	TestTrue(TEXT("edited child model regenerates its compiler closure"),
		MjBuildChildModelResources(*ModelAssets[0], AfterRows, Error));
	const FString AfterXml = AfterRows.IsEmpty() ? FString() : AsUtf8(AfterRows[0].Bytes);
	TestTrue(TEXT("parent payload reflects current child Blueprint edits"),
		AfterXml.Contains(TEXT("0.25")) && AfterXml != BeforeXml);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelAssetBlueprintCycleTest,
	"URLab.Authoring.ModelAssetBlueprintCycleFailsWithChain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelAssetBlueprintCycleTest::RunTest(const FString&)
{
	UBlueprint* const A = NewModelBlueprint(TEXT("MjCycleA"));
	UBlueprint* const B = NewModelBlueprint(TEXT("MjCycleB"));
	if (!TestNotNull(TEXT("first Blueprint"), A) || !TestNotNull(TEXT("second Blueprint"), B)) return false;
	TestTrue(TEXT("first cycle spec parses"), ImportSpec(*A,
		TEXT("<mujoco><asset><model name=\"b\" file=\"b.xml\"/></asset><worldbody><attach model=\"b\" prefix=\"a_\"/></worldbody></mujoco>")));
	TestTrue(TEXT("second cycle spec parses"), ImportSpec(*B,
		TEXT("<mujoco><asset><model name=\"a\" file=\"a.xml\"/></asset><worldbody><attach model=\"a\" prefix=\"b_\"/></worldbody></mujoco>")));
	UMjModelAsset* const AToB = FindElement<UMjModelAsset>(*A);
	UMjModelAsset* const BToA = FindElement<UMjModelAsset>(*B);
	if (!TestNotNull(TEXT("A to B model reference"), AToB) ||
		!TestNotNull(TEXT("B to A model reference"), BToA) ||
		!TestNotNull(TEXT("A generated class"), A->GeneratedClass.Get()) ||
		!TestNotNull(TEXT("B generated class"), B->GeneratedClass.Get())) return false;
	AToB->ChildModelClass = B->GeneratedClass;
	BToA->ChildModelClass = A->GeneratedClass;
	TArray<FMjPreparedAssetResource> Rows;
	FString Error;
	TestFalse(TEXT("cyclic Blueprint graph is rejected"), MjBuildChildModelResources(*AToB, Rows, Error));
	TestTrue(TEXT("cycle diagnostic names the reference chain"),
		Error.Contains(TEXT("child model Blueprint cycle")) && Error.Contains(TEXT("MjCycleA")) && Error.Contains(TEXT("MjCycleB")));
	TestTrue(TEXT("failed preparation clears partial rows"), Rows.IsEmpty());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelAssetRecursiveImportCompileTest,
	"URLab.Authoring.ModelAssetRecursiveImportUsesOnlyGeneratedPayload",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelAssetRecursiveImportCompileTest::RunTest(const FString&)
{
	const FString FixtureDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab/ChildModelTests/"),
		FGuid::NewGuid().ToString(EGuidFormats::Digits));
	IFileManager::Get().MakeDirectory(*FixtureDir, true);
	const FString ParentPath = FPaths::Combine(FixtureDir, TEXT("parent.xml"));
	const FString ChildPath = FPaths::Combine(FixtureDir, TEXT("child.xml"));
	const FString GrandchildPath = FPaths::Combine(FixtureDir, TEXT("grandchild.xml"));
	const TCHAR* const ParentXml = TEXT(R"(<mujoco model="parent">
  <default><geom size="0.05" friction="0.1 0.01 0.001"/></default>
  <asset><model name="child_left" file="child.xml"/><model name="child_right" file="child.xml"/></asset>
  <worldbody><attach model="child_left" prefix="left_"/><attach model="child_right" prefix="right_"/></worldbody>
</mujoco>)");
	const TCHAR* const ChildXml = TEXT(R"(<mujoco model="child">
  <default><default class="child_geom"><geom size="0.2" friction="0.8 0.1 0.01"/></default></default>
  <asset><model name="grand" file="grandchild.xml"/></asset>
  <worldbody><body name="child_root" pos="1 0 0" quat="0.7071067811865476 0 0 0.7071067811865475">
    <geom name="child_sphere" type="sphere" class="child_geom"/>
  </body><attach model="grand" prefix="inner_"/></worldbody>
</mujoco>)");
	const TCHAR* const GrandchildXml = TEXT(R"(<mujoco model="grandchild">
  <worldbody><body name="grand_root" pos="0 2 0"><geom name="grand_sphere" type="sphere" size="0.1"/></body></worldbody>
</mujoco>)");
	if (!TestTrue(TEXT("parent fixture saved"), SaveFixture(ParentPath, ParentXml)) ||
		!TestTrue(TEXT("child fixture saved"), SaveFixture(ChildPath, ChildXml)) ||
		!TestTrue(TEXT("grandchild fixture saved"), SaveFixture(GrandchildPath, GrandchildXml)))
	{
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}

	UBlueprint* const Parent = NewModelBlueprint(TEXT("MjRecursiveParent"));
	if (!TestNotNull(TEXT("parent Blueprint"), Parent))
	{
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}
	const FMjSpecParseResult ParseResult = MjParseIntoBlueprint(*Parent, ParentXml, ParentPath);
	if (!TestTrue(TEXT("parent source parses into editable authoring"), ParseResult.IsOk()))
	{
		for (const FMjSpecDiagnostic& Diagnostic : ParseResult.Errors) AddError(Diagnostic.ToString());
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}
	FMjDocParseOptions ParseOptions;
	FString ImportError;
	if (!TestTrue(TEXT("child and grandchild import recursively to separate Blueprints"),
		MjImportChildModels(*Parent, ParentPath, ParseOptions, ImportError)))
	{
		AddError(ImportError);
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}

	TArray<UMjModelAsset*> ParentModels = FindElements<UMjModelAsset>(*Parent);
	if (!TestEqual(TEXT("parent retains two native model references"), ParentModels.Num(), 2))
	{
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}
	UClass* const ChildClass = ParentModels[0]->ChildModelClass.Get();
	TestNotNull(TEXT("child class imported"), ChildClass);
	TestTrue(TEXT("reused child source maps to one child Blueprint"),
		ChildClass != nullptr && ParentModels[1]->ChildModelClass.Get() == ChildClass);
	TestTrue(TEXT("original XML filename is not left on model nodes"),
		!ParentModels[0]->File.IsSet() && !ParentModels[1]->File.IsSet());
	UBlueprint* const Child = ChildClass != nullptr ? Cast<UBlueprint>(ChildClass->ClassGeneratedBy) : nullptr;
	if (!TestNotNull(TEXT("child remains an editable Blueprint"), Child))
	{
		IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
		return false;
	}
	UMjModelAsset* const GrandchildRef = FindElement<UMjModelAsset>(*Child);
	UClass* const GrandchildClass = GrandchildRef != nullptr ? GrandchildRef->ChildModelClass.Get() : nullptr;
	TestNotNull(TEXT("grandchild Blueprint imported recursively"), GrandchildClass);
	TestTrue(TEXT("grandchild reference uses a derived compiler filename, not its source path"),
		GrandchildRef != nullptr && GrandchildRef->File.IsSet() &&
		GrandchildRef->GetFile().StartsWith(TEXT("__urlab_models/")) &&
		GrandchildRef->GetFile() != TEXT("grandchild.xml"));

	// Import has copied all editable child components and source paths are transient.
	MjFinalizeImportedAuthoredResources(FSpecRef::OverBlueprint(*Parent));
	TestTrue(TEXT("root import provenance cleared"), HasNoImportProvenance(*Parent));
	TestTrue(TEXT("child import provenance cleared"), HasNoImportProvenance(*Child));
	IFileManager::Get().DeleteDirectory(*FixtureDir, false, true);
	TestFalse(TEXT("parent source removed"), FPaths::FileExists(ParentPath));
	TestFalse(TEXT("child source removed"), FPaths::FileExists(ChildPath));
	TestFalse(TEXT("grandchild source removed"), FPaths::FileExists(GrandchildPath));

	// Editing the imported child after deleting all source documents must change the
	// generated bundle that the same native parser/compiler consumes.
	UMjGeom* const ChildGeom = FindElement<UMjGeom>(*Child);
	if (!TestNotNull(TEXT("editable child geom"), ChildGeom)) return false;
	ChildGeom->Size = TArray<double>{0.3};
	FMjSourceBundle Bundle;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!TestTrue(TEXT("current Blueprint state prepares a cooked compiler bundle"),
		MjBuildSourceBundle(FSpecRef::OverBlueprint(*Parent), Bundle, Diagnostics)))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics) AddError(Diagnostic.ToString());
		return false;
	}
	TestTrue(TEXT("bundle contains generated child and grandchild model payloads"), Bundle.Resources.Num() >= 2);
	TestTrue(TEXT("parent file references are generated, not original source filenames"),
		Bundle.Xml.Contains(TEXT("__urlab_models/")) && !Bundle.Xml.Contains(TEXT("child.xml")));
	TestFalse(TEXT("generated XML does not retain an original source path"), Bundle.Xml.Contains(FixtureDir));

	// After preparation, discard soft class refs to prove native compilation uses
	// only the generated model XML/VFS payload included in the bundle.
	for (UMjModelAsset* ModelAsset : ParentModels) ModelAsset->ChildModelClass.Reset();
	if (GrandchildRef != nullptr) GrandchildRef->ChildModelClass.Reset();
	FNativeModelGuard Native;
	Native.Model = CompileBundle(*this, Bundle);
	if (!TestNotNull(TEXT("generated parent bundle compiles with native MuJoCo"), Native.Model)) return false;

	const int LeftBody = mj_name2id(Native.Model, mjOBJ_BODY, "left_child_root");
	const int RightBody = mj_name2id(Native.Model, mjOBJ_BODY, "right_child_root");
	const int LeftGeom = mj_name2id(Native.Model, mjOBJ_GEOM, "left_child_sphere");
	const int RightGeom = mj_name2id(Native.Model, mjOBJ_GEOM, "right_child_sphere");
	const int LeftGrandchild = mj_name2id(Native.Model, mjOBJ_BODY, "left_inner_grand_root");
	const int RightGrandchild = mj_name2id(Native.Model, mjOBJ_BODY, "right_inner_grand_root");
	TestTrue(TEXT("left attach prefix is applied to child body and geom"), LeftBody >= 0 && LeftGeom >= 0);
	TestTrue(TEXT("right attach prefix is applied to child body and geom"), RightBody >= 0 && RightGeom >= 0);
	TestTrue(TEXT("nested attach prefixes compose through grandchild"), LeftGrandchild >= 0 && RightGrandchild >= 0);
	if (LeftGeom >= 0 && RightGeom >= 0)
	{
		TestTrue(TEXT("child defaults override parent defaults after native attach"),
			FMath::IsNearlyEqual(Native.Model->geom_size[3 * LeftGeom], 0.3, 1e-6) &&
			FMath::IsNearlyEqual(Native.Model->geom_size[3 * RightGeom], 0.3, 1e-6));
		TestTrue(TEXT("child friction defaults survive native attach"),
			FMath::IsNearlyEqual(Native.Model->geom_friction[3 * LeftGeom], 0.8, 1e-6) &&
			FMath::IsNearlyEqual(Native.Model->geom_friction[3 * RightGeom], 0.8, 1e-6));
	}
	if (LeftBody >= 0 && RightBody >= 0)
	{
		TestTrue(TEXT("native child frame transforms remain authored"),
			FMath::IsNearlyEqual(Native.Model->body_pos[3 * LeftBody], 1.0, 1e-6) &&
			FMath::IsNearlyEqual(Native.Model->body_pos[3 * RightBody], 1.0, 1e-6) &&
			FMath::IsNearlyEqual(Native.Model->body_quat[4 * LeftBody], 0.7071067812, 1e-6));
	}
	return true;
}
#endif
