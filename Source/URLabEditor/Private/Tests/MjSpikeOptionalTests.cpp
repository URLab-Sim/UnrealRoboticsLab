// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// SPIKE CODE. TOptional UPROPERTY lifecycle spike (URLab.Spike.TOptional.*).
// Proves whether UPROPERTY(EditAnywhere) TOptional<T> survives Blueprint SCS
// template save/load, level instance save/load, template-vs-instance delta
// serialization, undo/redo, duplication, component copy/paste, and the
// details-panel property-handle API. Findings in docs/spike_0b_toptional.md.
// Delete together with MjSpikeOptionalComponent.h/.cpp.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/PackageName.h"
#include "HAL/FileManager.h"

#include "Editor.h"
#include "EdGraphSchema_K2.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/Level.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/Actor.h"
#include "IDetailTreeNode.h"
#include "IPropertyRowGenerator.h"
#include "Kismet2/ComponentEditorUtils.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "ScopedTransaction.h"
#include "UObject/Linker.h"
#include "UObject/Package.h"
#include "UObject/PropertyOptional.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectHash.h"

#include "MjSpikeOptionalComponent.h"

namespace MjSpikeOptional
{

const TCHAR* const SpikeRoot = TEXT("/Game/URLabSpikeTemp/");

const TCHAR* const OptionalNames[] = {
	TEXT("OptDouble"), TEXT("OptVector"), TEXT("OptQuat"), TEXT("OptArray"),
	TEXT("OptString"), TEXT("OptEnum"), TEXT("OptBlueprintInt") };

FString UniqueSuffix()
{
	return FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(12);
}

FString PackageFilename(const FString& PkgName, const FString& Ext)
{
	FString Filename;
	FPackageName::TryConvertLongPackageNameToFilename(PkgName, Filename, Ext);
	return Filename;
}

FProperty* Prop(const TCHAR* Name)
{
	return UMjSpikeOptionalComponent::StaticClass()->FindPropertyByName(Name);
}

// Authored state used across tests: five optionals set, two left unset.
void SetAuthoredValues(UMjSpikeOptionalComponent* C)
{
	C->OptDouble = 1.5;
	C->OptVector = FVector(1.0, 2.0, 3.0);
	C->OptQuat = FQuat(0.5, 0.5, 0.5, 0.5);
	C->OptArray = TArray<double>{0.25, -4.0, 9.75};
	C->OptEnum = EMjSpikeOptEnum::Beta;
	// OptString and OptBlueprintInt stay unset.
}

bool VerifyAuthoredValues(FAutomationTestBase& T, const FString& Ctx, const UMjSpikeOptionalComponent* C)
{
	bool bOk = true;
	bOk &= T.TestNotNull(*(Ctx + TEXT(": component")), C);
	if (!C)
		return false;

	bOk &= T.TestTrue(*(Ctx + TEXT(": OptDouble set")), C->OptDouble.IsSet());
	if (C->OptDouble.IsSet())
		bOk &= T.TestEqual(*(Ctx + TEXT(": OptDouble value")), C->OptDouble.GetValue(), 1.5);

	bOk &= T.TestTrue(*(Ctx + TEXT(": OptVector set")), C->OptVector.IsSet());
	if (C->OptVector.IsSet())
		bOk &= T.TestEqual(*(Ctx + TEXT(": OptVector value")), C->OptVector.GetValue(), FVector(1.0, 2.0, 3.0));

	bOk &= T.TestTrue(*(Ctx + TEXT(": OptQuat set")), C->OptQuat.IsSet());
	if (C->OptQuat.IsSet())
	{
		const FQuat& Q = C->OptQuat.GetValue();
		bOk &= T.TestTrue(*(Ctx + TEXT(": OptQuat value")),
			Q.X == 0.5 && Q.Y == 0.5 && Q.Z == 0.5 && Q.W == 0.5);
	}

	bOk &= T.TestTrue(*(Ctx + TEXT(": OptArray set")), C->OptArray.IsSet());
	if (C->OptArray.IsSet())
	{
		const TArray<double>& A = C->OptArray.GetValue();
		bOk &= T.TestEqual(*(Ctx + TEXT(": OptArray num")), A.Num(), 3);
		if (A.Num() == 3)
			bOk &= T.TestTrue(*(Ctx + TEXT(": OptArray values")),
				A[0] == 0.25 && A[1] == -4.0 && A[2] == 9.75);
	}

	bOk &= T.TestTrue(*(Ctx + TEXT(": OptEnum set")), C->OptEnum.IsSet());
	if (C->OptEnum.IsSet())
		bOk &= T.TestTrue(*(Ctx + TEXT(": OptEnum value")), C->OptEnum.GetValue() == EMjSpikeOptEnum::Beta);

	// Unset must reload unset, not default-constructed.
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptString unset")), C->OptString.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptBlueprintInt unset")), C->OptBlueprintInt.IsSet());
	return bOk;
}

bool VerifyAllUnset(FAutomationTestBase& T, const FString& Ctx, const UMjSpikeOptionalComponent* C)
{
	bool bOk = true;
	bOk &= T.TestNotNull(*(Ctx + TEXT(": component")), C);
	if (!C)
		return false;
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptDouble unset")), C->OptDouble.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptVector unset")), C->OptVector.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptQuat unset")), C->OptQuat.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptArray unset")), C->OptArray.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptEnum unset")), C->OptEnum.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptString unset")), C->OptString.IsSet());
	bOk &= T.TestFalse(*(Ctx + TEXT(": OptBlueprintInt unset")), C->OptBlueprintInt.IsSet());
	return bOk;
}

// Rename the in-memory package aside and release everything so that a
// subsequent LoadPackage(PkgName) is forced to deserialize from disk.
void UnloadPackageForReload(UPackage* Pkg)
{
	ResetLoaders(Pkg);
	const FString TrashName = Pkg->GetName() + TEXT("_TRASH_") + UniqueSuffix();
	Pkg->Rename(*TrashName, nullptr, REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
	ForEachObjectWithPackage(Pkg, [](UObject* Obj)
	{
		Obj->ClearFlags(RF_Standalone | RF_Public);
		return true;
	});
	Pkg->ClearFlags(RF_Standalone | RF_Public);
	CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);
}

// Release a package's linker before removing its file, otherwise the still-open
// read handle makes the delete fail and leaves the asset behind in Content.
void UnloadAndDelete(UPackage* Pkg, const FString& Filename)
{
	if (Pkg)
		UnloadPackageForReload(Pkg);
	IFileManager::Get().Delete(*Filename, /*bRequireExists=*/false, /*bEvenReadOnly=*/true, /*bQuiet=*/true);

	FString Dir;
	if (FPackageName::TryConvertLongPackageNameToFilename(FString(SpikeRoot), Dir))
		IFileManager::Get().DeleteDirectory(*Dir, /*bRequireExists=*/false, /*bTree=*/false);
}

UMjSpikeOptionalComponent* FindSpikeComponentIn(UObject* Outer)
{
	TArray<UObject*> Subs;
	GetObjectsWithOuter(Outer, Subs, /*bIncludeNestedObjects=*/true);
	for (UObject* O : Subs)
		if (UMjSpikeOptionalComponent* C = Cast<UMjSpikeOptionalComponent>(O))
			return C;
	return nullptr;
}

struct FSpikeBlueprint
{
	UPackage* Pkg = nullptr;
	UBlueprint* Blueprint = nullptr;
	UMjSpikeOptionalComponent* Template = nullptr;
	FString PkgName;
	FString BPName;
	FString Filename;
};

// Create a saveable AActor blueprint with one SCS node of the spike component.
bool CreateSpikeBlueprint(const FString& Tag, const FString& Sfx, FSpikeBlueprint& Out)
{
	Out.BPName = TEXT("Spike") + Tag + TEXT("_") + Sfx;
	Out.PkgName = FString(SpikeRoot) + Out.BPName;
	Out.Pkg = CreatePackage(*Out.PkgName);
	Out.Blueprint = FKismetEditorUtilities::CreateBlueprint(
		AActor::StaticClass(), Out.Pkg, *Out.BPName,
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (!Out.Blueprint)
		return false;
	Out.Blueprint->SetFlags(RF_Public | RF_Standalone | RF_Transactional);

	USimpleConstructionScript* SCS = Out.Blueprint->SimpleConstructionScript;
	USCS_Node* Node = SCS->CreateNode(UMjSpikeOptionalComponent::StaticClass(), TEXT("SpikeComp"));
	SCS->AddNode(Node);
	Out.Template = Cast<UMjSpikeOptionalComponent>(Node->ComponentTemplate);
	Out.Filename = PackageFilename(Out.PkgName, FPackageName::GetAssetPackageExtension());
	return Out.Template != nullptr;
}

bool SaveBlueprint(const FSpikeBlueprint& BP)
{
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	return UPackage::SavePackage(BP.Pkg, BP.Blueprint, *BP.Filename, SaveArgs);
}

UMjSpikeOptionalComponent* FindScsTemplate(UBlueprint* BP, const TCHAR* VarName)
{
	if (!BP || !BP->SimpleConstructionScript)
		return nullptr;
	for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
		if (Node && Node->GetVariableName() == VarName)
			return Cast<UMjSpikeOptionalComponent>(Node->ComponentTemplate);
	return nullptr;
}

// True when a delta-serializing writer would emit this property for Object
// against Archetype. This is the oracle for what lands in a .umap/.uasset
// instance record and in T3D copy-paste text.
bool WouldEmitDelta(const TCHAR* PropName, const UObject* Object, const UObject* Archetype, FString& OutText)
{
	OutText.Reset();
	FProperty* P = Prop(PropName);
	if (!P)
		return false;
	return P->ExportText_InContainer(0, OutText, Object, Archetype,
		const_cast<UObject*>(Object), PPF_Copy);
}

void SpikeLog(FAutomationTestBase& T, const FString& Msg)
{
	T.AddInfo(Msg);
	UE_LOG(LogTemp, Display, TEXT("[MjSpikeOpt] %s"), *Msg);
}

enum class ENodeKind : uint8
{
	Any,          // first node carrying the name, whatever its field class
	OptionalOnly, // only a node whose property is the FOptionalProperty itself
	ValueOnly     // only a node whose property is the optional's inner value
};

bool NodeMatches(const TSharedRef<IDetailTreeNode>& Node, FName PropName, ENodeKind Kind)
{
	TSharedPtr<IPropertyHandle> Handle = Node->CreatePropertyHandle();
	FProperty* P = Handle.IsValid() ? Handle->GetProperty() : nullptr;
	if (!P || P->GetFName() != PropName)
		return false;
	const bool bIsOptional = CastField<FOptionalProperty>(P) != nullptr;
	switch (Kind)
	{
	case ENodeKind::OptionalOnly: return bIsOptional;
	case ENodeKind::ValueOnly:    return !bIsOptional && P->GetOwner<FOptionalProperty>() != nullptr;
	default:                      return true;
	}
}

TSharedPtr<IDetailTreeNode> FindNodeByPropertyName(
	const TArray<TSharedRef<IDetailTreeNode>>& Nodes, FName PropName, ENodeKind Kind = ENodeKind::Any)
{
	for (const TSharedRef<IDetailTreeNode>& Node : Nodes)
	{
		if (NodeMatches(Node, PropName, Kind))
			return Node;

		TArray<TSharedRef<IDetailTreeNode>> Children;
		Node->GetChildren(Children);
		if (TSharedPtr<IDetailTreeNode> Found = FindNodeByPropertyName(Children, PropName, Kind))
			return Found;
	}
	return nullptr;
}

void DumpNodes(FAutomationTestBase& T, const TArray<TSharedRef<IDetailTreeNode>>& Nodes, int32 Depth)
{
	for (const TSharedRef<IDetailTreeNode>& Node : Nodes)
	{
		TSharedPtr<IPropertyHandle> Handle = Node->CreatePropertyHandle();
		FProperty* P = Handle.IsValid() ? Handle->GetProperty() : nullptr;
		SpikeLog(T, FString::Printf(TEXT("%*srow '%s' prop=%s class=%s optionalHandle=%s"),
			Depth * 2, TEXT(""), *Node->GetNodeName().ToString(),
			P ? *P->GetName() : TEXT("<none>"),
			P ? *P->GetClass()->GetName() : TEXT("<none>"),
			(Handle.IsValid() && Handle->AsOptional().IsValid()) ? TEXT("yes") : TEXT("no")));

		TArray<TSharedRef<IDetailTreeNode>> Children;
		Node->GetChildren(Children);
		DumpNodes(T, Children, Depth + 1);
	}
}

} // namespace MjSpikeOptional

using namespace MjSpikeOptional;

// ---------------------------------------------------------------------------
// Baseline: fresh component has every optional unset; FOptionalProperty is
// the reflected type; Identical distinguishes set from unset.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptDefaults,
	"URLab.Spike.TOptional.Defaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptDefaults::RunTest(const FString& Parameters)
{
	UMjSpikeOptionalComponent* A = NewObject<UMjSpikeOptionalComponent>(GetTransientPackage());
	VerifyAllUnset(*this, TEXT("fresh component"), A);

	for (const TCHAR* Name : OptionalNames)
	{
		FProperty* P = Prop(Name);
		TestNotNull(*FString::Printf(TEXT("%s reflected"), Name), P);
		if (P)
		{
			FOptionalProperty* OP = CastField<FOptionalProperty>(P);
			TestTrue(*FString::Printf(TEXT("%s is FOptionalProperty"), Name), OP != nullptr);
			if (OP)
				TestNotNull(*FString::Printf(TEXT("%s has a value property"), Name), OP->GetValueProperty());
		}
	}

	UMjSpikeOptionalComponent* B = NewObject<UMjSpikeOptionalComponent>(GetTransientPackage());
	FProperty* PD = Prop(TEXT("OptDouble"));
	if (PD)
	{
		TestTrue(TEXT("Identical: unset == unset"), PD->Identical_InContainer(A, B));
		B->OptDouble = 0.0;
		TestFalse(TEXT("Identical: unset != set(default value)"), PD->Identical_InContainer(A, B));
		A->OptDouble = 0.0;
		TestTrue(TEXT("Identical: set(x) == set(x)"), PD->Identical_InContainer(A, B));
		A->OptDouble = 1.0;
		TestFalse(TEXT("Identical: set(x) != set(y)"), PD->Identical_InContainer(A, B));
	}
	return true;
}

// ---------------------------------------------------------------------------
// Property text format. A set optional exports as "(Value)" and an unset one
// as "()", but "()" is only ever written when the baseline it is exported
// against has the optional set. Against a null baseline an unset optional is
// treated as "same as default" and nothing is written at all.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptTextFormat,
	"URLab.Spike.TOptional.TextFormat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptTextFormat::RunTest(const FString& Parameters)
{
	UMjSpikeOptionalComponent* Src = NewObject<UMjSpikeOptionalComponent>(GetTransientPackage());
	SetAuthoredValues(Src);

	auto ExportOne = [Src](const TCHAR* Name, FString& OutTxt)
	{
		OutTxt.Reset();
		return Prop(Name)->ExportText_InContainer(0, OutTxt, Src, nullptr, Src, PPF_Copy);
	};

	FString Txt;
	for (const TCHAR* Name : OptionalNames)
	{
		const bool bExported = ExportOne(Name, Txt);
		SpikeLog(*this, FString::Printf(TEXT("text[%s] exported=%s value=%s"),
			Name, bExported ? TEXT("true") : TEXT("false"), *Txt));
	}

	// Unset against a null baseline: nothing is written.
	TestFalse(TEXT("unset OptString does not export against a null baseline"),
		ExportOne(TEXT("OptString"), Txt));
	TestEqual(TEXT("unset OptString leaves the text buffer empty"), Txt, FString());
	TestFalse(TEXT("unset OptBlueprintInt does not export against a null baseline"),
		ExportOne(TEXT("OptBlueprintInt"), Txt));

	// Set values are parenthesised, one level around the inner value's own text.
	TestTrue(TEXT("set OptDouble exports"), ExportOne(TEXT("OptDouble"), Txt));
	TestEqual(TEXT("set OptDouble text"), Txt, FString(TEXT("(1.500000)")));
	TestTrue(TEXT("set OptVector exports"), ExportOne(TEXT("OptVector"), Txt));
	TestEqual(TEXT("set OptVector text"), Txt, FString(TEXT("((X=1.000000,Y=2.000000,Z=3.000000))")));
	TestTrue(TEXT("set OptEnum exports"), ExportOne(TEXT("OptEnum"), Txt));
	TestEqual(TEXT("set OptEnum text"), Txt, FString(TEXT("(Beta)")));
	TestTrue(TEXT("set OptArray exports"), ExportOne(TEXT("OptArray"), Txt));
	TestEqual(TEXT("set OptArray text"), Txt, FString(TEXT("((0.250000,-4.000000,9.750000))")));

	// Unset DOES export as "()" when the baseline has it set.
	UMjSpikeOptionalComponent* Baseline = NewObject<UMjSpikeOptionalComponent>(GetTransientPackage());
	Baseline->OptString = FString(TEXT("baseline"));
	Txt.Reset();
	TestTrue(TEXT("unset OptString exports against a set baseline"),
		Prop(TEXT("OptString"))->ExportText_InContainer(0, Txt, Src, Baseline, Src, PPF_Copy));
	TestEqual(TEXT("unset OptString text"), Txt, FString(TEXT("()")));

	// Import must actively clear as well as set: the target starts in the
	// opposite state for both unset properties.
	UMjSpikeOptionalComponent* Dst = NewObject<UMjSpikeOptionalComponent>(GetTransientPackage());
	Dst->OptString = FString(TEXT("sentinel"));
	Dst->OptBlueprintInt = 42;

	for (const TCHAR* Name : OptionalNames)
	{
		FProperty* P = Prop(Name);
		if (!ExportOne(Name, Txt))
			Txt = TEXT("()");
		P->ImportText_InContainer(*Txt, Dst, Dst, PPF_Copy);
		TestTrue(*FString::Printf(TEXT("%s text round-trip identical"), Name),
			P->Identical_InContainer(Src, Dst));
	}
	VerifyAuthoredValues(*this, TEXT("text-imported component"), Dst);
	return true;
}

// ---------------------------------------------------------------------------
// Delta emission oracle: with an archetype whose optionals are set, an
// instance must emit a delta for a changed value AND for an explicit clear,
// and must emit nothing for an inherited value.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptDeltaEmission,
	"URLab.Spike.TOptional.DeltaEmission",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptDeltaEmission::RunTest(const FString& Parameters)
{
	UMjSpikeOptionalComponent* Arch = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), *(TEXT("SpikeDeltaArch_") + UniqueSuffix()));
	SetAuthoredValues(Arch);

	UMjSpikeOptionalComponent* Inst = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), UMjSpikeOptionalComponent::StaticClass(),
		*(TEXT("SpikeDeltaInst_") + UniqueSuffix()), RF_NoFlags, Arch);

	VerifyAuthoredValues(*this, TEXT("NewObject from archetype"), Inst);

	Inst->OptVector = FVector(9.0, 9.0, 9.0);   // changed
	Inst->OptEnum.Reset();                       // explicitly cleared
	Inst->OptString = FString(TEXT("added"));    // set where archetype is unset

	FString Txt;
	TestFalse(TEXT("inherited OptDouble emits no delta"), WouldEmitDelta(TEXT("OptDouble"), Inst, Arch, Txt));
	TestFalse(TEXT("inherited OptQuat emits no delta"), WouldEmitDelta(TEXT("OptQuat"), Inst, Arch, Txt));
	TestFalse(TEXT("inherited OptArray emits no delta"), WouldEmitDelta(TEXT("OptArray"), Inst, Arch, Txt));
	TestFalse(TEXT("inherited-unset OptBlueprintInt emits no delta"),
		WouldEmitDelta(TEXT("OptBlueprintInt"), Inst, Arch, Txt));

	TestTrue(TEXT("changed OptVector emits a delta"), WouldEmitDelta(TEXT("OptVector"), Inst, Arch, Txt));
	SpikeLog(*this, FString::Printf(TEXT("delta[OptVector] = %s"), *Txt));

	const bool bClearEmits = WouldEmitDelta(TEXT("OptEnum"), Inst, Arch, Txt);
	SpikeLog(*this, FString::Printf(TEXT("delta[OptEnum cleared] = %s"), *Txt));
	TestTrue(TEXT("explicit clear against a set archetype emits a delta"), bClearEmits);
	TestEqual(TEXT("explicit clear serialises as ()"), Txt, FString(TEXT("()")));

	TestTrue(TEXT("newly-set OptString emits a delta"), WouldEmitDelta(TEXT("OptString"), Inst, Arch, Txt));
	SpikeLog(*this, FString::Printf(TEXT("delta[OptString] = %s"), *Txt));

	// The clear delta must import back as a clear, not as a no-op.
	UMjSpikeOptionalComponent* Round = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), UMjSpikeOptionalComponent::StaticClass(),
		*(TEXT("SpikeDeltaRound_") + UniqueSuffix()), RF_NoFlags, Arch);
	TestTrue(TEXT("round target inherits OptEnum before import"), Round->OptEnum.IsSet());
	Prop(TEXT("OptEnum"))->ImportText_InContainer(TEXT("()"), Round, Round, PPF_Copy);
	TestFalse(TEXT("importing () clears an inherited set value"), Round->OptEnum.IsSet());
	return true;
}

// ---------------------------------------------------------------------------
// 1. Blueprint SCS template save / disk reload.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptScsTemplateSaveLoad,
	"URLab.Spike.TOptional.ScsTemplateSaveLoad",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptScsTemplateSaveLoad::RunTest(const FString& Parameters)
{
	const FString Sfx = UniqueSuffix();
	FSpikeBlueprint BP;
	if (!TestTrue(TEXT("CreateSpikeBlueprint"), CreateSpikeBlueprint(TEXT("ScsBP"), Sfx, BP)))
		return false;

	SetAuthoredValues(BP.Template);
	FKismetEditorUtilities::CompileBlueprint(BP.Blueprint);
	VerifyAuthoredValues(*this, TEXT("template post-compile"), FindScsTemplate(BP.Blueprint, TEXT("SpikeComp")));

	if (!TestTrue(TEXT("SavePackage(BP)"), SaveBlueprint(BP)))
		return false;

	UnloadPackageForReload(BP.Pkg);

	UPackage* Reloaded = LoadPackage(nullptr, *BP.PkgName, LOAD_None);
	if (!TestNotNull(TEXT("LoadPackage(BP)"), Reloaded))
		return false;
	UBlueprint* BP2 = FindObject<UBlueprint>(Reloaded, *BP.BPName);
	if (!TestNotNull(TEXT("reloaded UBlueprint"), BP2))
		return false;

	VerifyAuthoredValues(*this, TEXT("reloaded SCS template"), FindScsTemplate(BP2, TEXT("SpikeComp")));

	UnloadAndDelete(Reloaded, BP.Filename);
	return true;
}

// ---------------------------------------------------------------------------
// 2. Level instance save / disk reload. One authored component, one fully
//    unset component on a second actor.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptLevelInstanceSaveLoad,
	"URLab.Spike.TOptional.LevelInstanceSaveLoad",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptLevelInstanceSaveLoad::RunTest(const FString& Parameters)
{
	const FString Sfx = UniqueSuffix();
	const FString PkgName = FString(SpikeRoot) + TEXT("SpikeMap_") + Sfx;
	UPackage* Pkg = CreatePackage(*PkgName);
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false,
		*(TEXT("SpikeWorld_") + Sfx), Pkg, /*bAddToRoot=*/false);
	if (!TestNotNull(TEXT("CreateWorld"), World))
		return false;

	AActor* AuthoredActor = World->SpawnActor<AActor>();
	UMjSpikeOptionalComponent* Authored =
		NewObject<UMjSpikeOptionalComponent>(AuthoredActor, TEXT("SpikeInstComp"));
	Authored->SetFlags(RF_Transactional);
	AuthoredActor->AddInstanceComponent(Authored);
	Authored->RegisterComponent();
	SetAuthoredValues(Authored);

	AActor* UnsetActor = World->SpawnActor<AActor>();
	UMjSpikeOptionalComponent* Unset =
		NewObject<UMjSpikeOptionalComponent>(UnsetActor, TEXT("SpikeUnsetComp"));
	Unset->SetFlags(RF_Transactional);
	UnsetActor->AddInstanceComponent(Unset);
	Unset->RegisterComponent();

	const FString Filename = PackageFilename(PkgName, FPackageName::GetMapPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_NoFlags;
	if (!TestTrue(TEXT("SavePackage(map)"), UPackage::SavePackage(Pkg, World, *Filename, SaveArgs)))
		return false;

	World->DestroyWorld(false);
	UnloadPackageForReload(Pkg);

	UPackage* Reloaded = LoadPackage(nullptr, *PkgName, LOAD_None);
	if (!TestNotNull(TEXT("LoadPackage(map)"), Reloaded))
		return false;
	UWorld* World2 = UWorld::FindWorldInPackage(Reloaded);
	if (!TestNotNull(TEXT("reloaded world"), World2))
		return false;

	UMjSpikeOptionalComponent* Authored2 = nullptr;
	UMjSpikeOptionalComponent* Unset2 = nullptr;
	for (AActor* Actor : World2->PersistentLevel->Actors)
	{
		if (!Actor)
			continue;
		if (UMjSpikeOptionalComponent* C = FindSpikeComponentIn(Actor))
		{
			if (C->GetFName() == TEXT("SpikeInstComp"))
				Authored2 = C;
			else if (C->GetFName() == TEXT("SpikeUnsetComp"))
				Unset2 = C;
		}
	}

	VerifyAuthoredValues(*this, TEXT("reloaded level instance"), Authored2);
	VerifyAllUnset(*this, TEXT("reloaded all-unset instance"), Unset2);

	World2->DestroyWorld(false);
	UnloadAndDelete(Reloaded, Filename);
	return true;
}

// ---------------------------------------------------------------------------
// 3. Template-vs-instance delta serialization through disk. Instance
//    overrides one optional, explicitly clears a template-set optional,
//    inherits the rest. The template is edited AFTER the map is saved, so an
//    inherited value must track the template while the override and the
//    explicit clear both survive.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptTemplateInstanceDelta,
	"URLab.Spike.TOptional.TemplateInstanceDelta",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptTemplateInstanceDelta::RunTest(const FString& Parameters)
{
	const FString Sfx = UniqueSuffix();
	FSpikeBlueprint BP;
	if (!TestTrue(TEXT("CreateSpikeBlueprint"), CreateSpikeBlueprint(TEXT("DeltaBP"), Sfx, BP)))
		return false;

	BP.Template->OptDouble = 1.5;
	BP.Template->OptVector = FVector(1.0, 2.0, 3.0);
	BP.Template->OptEnum = EMjSpikeOptEnum::Beta;
	FKismetEditorUtilities::CompileBlueprint(BP.Blueprint);
	if (!TestTrue(TEXT("SavePackage(BP)"), SaveBlueprint(BP)))
		return false;

	const FString MapPkgName = FString(SpikeRoot) + TEXT("SpikeDeltaMap_") + Sfx;
	UPackage* MapPkg = CreatePackage(*MapPkgName);
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false,
		*(TEXT("SpikeDeltaWorld_") + Sfx), MapPkg, /*bAddToRoot=*/false);
	if (!TestNotNull(TEXT("CreateWorld"), World))
		return false;

	AActor* Inst = World->SpawnActor<AActor>(BP.Blueprint->GeneratedClass);
	if (!TestNotNull(TEXT("spawned BP instance"), Inst))
		return false;
	UMjSpikeOptionalComponent* IC = Cast<UMjSpikeOptionalComponent>(
		Inst->GetComponentByClass(UMjSpikeOptionalComponent::StaticClass()));
	if (!TestNotNull(TEXT("SCS component on instance"), IC))
		return false;

	TestTrue(TEXT("instance inherits OptDouble at spawn"),
		IC->OptDouble.IsSet() && IC->OptDouble.GetValue() == 1.5);

	IC->Modify();
	IC->OptVector = FVector(9.0, 9.0, 9.0);
	IC->OptEnum.Reset();

	// The delta vs the archetype must be exactly {OptVector, OptEnum}.
	UMjSpikeOptionalComponent* Arch = Cast<UMjSpikeOptionalComponent>(IC->GetArchetype());
	if (TestNotNull(TEXT("instance archetype is spike template"), Arch))
	{
		FString Txt;
		TestFalse(TEXT("delta: OptDouble not emitted"), WouldEmitDelta(TEXT("OptDouble"), IC, Arch, Txt));
		TestFalse(TEXT("delta: OptString not emitted"), WouldEmitDelta(TEXT("OptString"), IC, Arch, Txt));
		TestTrue(TEXT("delta: OptVector emitted"), WouldEmitDelta(TEXT("OptVector"), IC, Arch, Txt));
		TestTrue(TEXT("delta: cleared OptEnum emitted"), WouldEmitDelta(TEXT("OptEnum"), IC, Arch, Txt));
	}

	const FString MapFilename = PackageFilename(MapPkgName, FPackageName::GetMapPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_NoFlags;
	if (!TestTrue(TEXT("SavePackage(map)"), UPackage::SavePackage(MapPkg, World, *MapFilename, SaveArgs)))
		return false;

	World->DestroyWorld(false);
	UnloadPackageForReload(MapPkg);

	// Edit the template AFTER the map was saved: an inherited property must
	// pick this up on reload, proving it was not baked into the instance.
	UMjSpikeOptionalComponent* Tmpl = FindScsTemplate(BP.Blueprint, TEXT("SpikeComp"));
	if (!TestNotNull(TEXT("template still reachable"), Tmpl))
		return false;
	Tmpl->Modify();
	Tmpl->OptDouble = 2.5;
	FKismetEditorUtilities::CompileBlueprint(BP.Blueprint);

	UPackage* Reloaded = LoadPackage(nullptr, *MapPkgName, LOAD_None);
	if (!TestNotNull(TEXT("LoadPackage(map)"), Reloaded))
		return false;
	UWorld* World2 = UWorld::FindWorldInPackage(Reloaded);
	if (!TestNotNull(TEXT("reloaded world"), World2))
		return false;

	UMjSpikeOptionalComponent* IC2 = nullptr;
	for (AActor* Actor : World2->PersistentLevel->Actors)
		if (Actor)
			if (UMjSpikeOptionalComponent* C = FindSpikeComponentIn(Actor))
				IC2 = C;
	if (!TestNotNull(TEXT("reloaded instance component"), IC2))
		return false;

	TestTrue(TEXT("inherited OptDouble tracks edited template (2.5)"),
		IC2->OptDouble.IsSet() && IC2->OptDouble.GetValue() == 2.5);
	TestTrue(TEXT("overridden OptVector survives reload"),
		IC2->OptVector.IsSet() && IC2->OptVector.GetValue() == FVector(9.0, 9.0, 9.0));
	TestFalse(TEXT("explicitly cleared OptEnum stays unset despite set template"),
		IC2->OptEnum.IsSet());
	TestFalse(TEXT("never-set OptString stays unset"), IC2->OptString.IsSet());

	World2->DestroyWorld(false);
	UnloadAndDelete(Reloaded, MapFilename);
	UnloadAndDelete(BP.Pkg, BP.Filename);
	return true;
}

// ---------------------------------------------------------------------------
// 4. Undo/redo across set, value change, and clear.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptUndoRedo,
	"URLab.Spike.TOptional.UndoRedo",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptUndoRedo::RunTest(const FString& Parameters)
{
	UMjSpikeOptionalComponent* C = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), *(TEXT("SpikeUndo_") + UniqueSuffix()));
	C->SetFlags(RF_Transactional);

	{
		FScopedTransaction Tx(NSLOCTEXT("MjSpike", "SetOpt", "Spike: set optionals"));
		C->Modify();
		C->OptDouble = 5.0;
		C->OptVector = FVector(1.0, 1.0, 1.0);
		C->OptArray = TArray<double>{2.0, 4.0};
	}
	TestTrue(TEXT("undo(set)"), GEditor->UndoTransaction());
	TestFalse(TEXT("after undo(set): OptDouble unset"), C->OptDouble.IsSet());
	TestFalse(TEXT("after undo(set): OptVector unset"), C->OptVector.IsSet());
	TestFalse(TEXT("after undo(set): OptArray unset"), C->OptArray.IsSet());

	TestTrue(TEXT("redo(set)"), GEditor->RedoTransaction());
	TestTrue(TEXT("after redo(set): OptDouble == 5"),
		C->OptDouble.IsSet() && C->OptDouble.GetValue() == 5.0);
	TestTrue(TEXT("after redo(set): OptVector == (1,1,1)"),
		C->OptVector.IsSet() && C->OptVector.GetValue() == FVector(1.0, 1.0, 1.0));
	TestTrue(TEXT("after redo(set): OptArray == {2,4}"),
		C->OptArray.IsSet() && C->OptArray.GetValue() == TArray<double>({2.0, 4.0}));

	{
		FScopedTransaction Tx(NSLOCTEXT("MjSpike", "ChangeOpt", "Spike: change optional value"));
		C->Modify();
		C->OptDouble = 7.0;
	}
	TestTrue(TEXT("undo(change)"), GEditor->UndoTransaction());
	TestTrue(TEXT("after undo(change): OptDouble == 5"),
		C->OptDouble.IsSet() && C->OptDouble.GetValue() == 5.0);
	TestTrue(TEXT("redo(change)"), GEditor->RedoTransaction());
	TestTrue(TEXT("after redo(change): OptDouble == 7"),
		C->OptDouble.IsSet() && C->OptDouble.GetValue() == 7.0);

	{
		FScopedTransaction Tx(NSLOCTEXT("MjSpike", "ClearOpt", "Spike: clear optional"));
		C->Modify();
		C->OptDouble.Reset();
		C->OptArray.Reset();
	}
	TestFalse(TEXT("after clear: OptDouble unset"), C->OptDouble.IsSet());
	TestTrue(TEXT("undo(clear)"), GEditor->UndoTransaction());
	TestTrue(TEXT("after undo(clear): OptDouble == 7"),
		C->OptDouble.IsSet() && C->OptDouble.GetValue() == 7.0);
	TestTrue(TEXT("after undo(clear): OptArray == {2,4}"),
		C->OptArray.IsSet() && C->OptArray.GetValue() == TArray<double>({2.0, 4.0}));
	TestTrue(TEXT("redo(clear)"), GEditor->RedoTransaction());
	TestFalse(TEXT("after redo(clear): OptDouble unset"), C->OptDouble.IsSet());
	TestFalse(TEXT("after redo(clear): OptArray unset"), C->OptArray.IsSet());

	GEditor->ResetTransaction(NSLOCTEXT("MjSpike", "ResetTx", "Spike test cleanup"));
	return true;
}

// ---------------------------------------------------------------------------
// 5. Duplication (object and actor) and component copy/paste, which is the
//    T3D text path the details and SCS panels use.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptDuplicateAndCopyPaste,
	"URLab.Spike.TOptional.DuplicateAndCopyPaste",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptDuplicateAndCopyPaste::RunTest(const FString& Parameters)
{
	UMjSpikeOptionalComponent* Src = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), *(TEXT("SpikeDupSrc_") + UniqueSuffix()));
	SetAuthoredValues(Src);

	UMjSpikeOptionalComponent* Dup = DuplicateObject<UMjSpikeOptionalComponent>(
		Src, GetTransientPackage(), *(TEXT("SpikeDupDst_") + UniqueSuffix()));
	VerifyAuthoredValues(*this, TEXT("DuplicateObject copy"), Dup);

	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false,
		*(TEXT("SpikeDupWorld_") + UniqueSuffix()), nullptr, /*bAddToRoot=*/false);
	if (!TestNotNull(TEXT("CreateWorld"), World))
		return false;

	AActor* Actor = World->SpawnActor<AActor>();
	UMjSpikeOptionalComponent* Comp =
		NewObject<UMjSpikeOptionalComponent>(Actor, TEXT("SpikeInstComp"));
	Actor->AddInstanceComponent(Comp);
	Comp->RegisterComponent();
	SetAuthoredValues(Comp);

	AActor* Actor2 = Cast<AActor>(StaticDuplicateObject(Actor, World->PersistentLevel));
	if (TestNotNull(TEXT("duplicated actor"), Actor2))
		VerifyAuthoredValues(*this, TEXT("actor-duplicate component"), FindSpikeComponentIn(Actor2));

	// Copy/paste: one authored component, one all-unset component, both in the
	// same clipboard payload.
	UMjSpikeOptionalComponent* UnsetComp =
		NewObject<UMjSpikeOptionalComponent>(Actor, TEXT("SpikeUnsetComp"));
	Actor->AddInstanceComponent(UnsetComp);
	UnsetComp->RegisterComponent();

	TArray<UActorComponent*> ToCopy{Comp, UnsetComp};
	TestTrue(TEXT("CanCopyComponents"), FComponentEditorUtils::CanCopyComponents(ToCopy));

	FString Clipboard;
	FComponentEditorUtils::CopyComponents(ToCopy, &Clipboard);
	SpikeLog(*this, FString::Printf(TEXT("component clipboard T3D:\n%s"), *Clipboard));
	TestTrue(TEXT("clipboard payload non-empty"), !Clipboard.IsEmpty());
	TestTrue(TEXT("clipboard carries the set optional"), Clipboard.Contains(TEXT("OptDouble=(")));
	// Both components' archetype is the all-unset CDO, so the unset optionals
	// are delta-identical to it and are simply omitted from the payload.
	TestFalse(TEXT("clipboard omits optionals that are unset in the archetype too"),
		Clipboard.Contains(TEXT("OptString=")));

	AActor* PasteTarget = World->SpawnActor<AActor>();
	TArray<UActorComponent*> Pasted;
	FComponentEditorUtils::PasteComponents(Pasted, PasteTarget, nullptr, &Clipboard);
	TestEqual(TEXT("pasted component count"), Pasted.Num(), 2);

	UMjSpikeOptionalComponent* PastedAuthored = nullptr;
	UMjSpikeOptionalComponent* PastedUnset = nullptr;
	for (UActorComponent* AC : Pasted)
	{
		UMjSpikeOptionalComponent* SC = Cast<UMjSpikeOptionalComponent>(AC);
		if (!SC)
			continue;
		if (SC->OptDouble.IsSet())
			PastedAuthored = SC;
		else
			PastedUnset = SC;
	}
	VerifyAuthoredValues(*this, TEXT("pasted authored component"), PastedAuthored);
	VerifyAllUnset(*this, TEXT("pasted all-unset component"), PastedUnset);

	World->DestroyWorld(false);
	return true;
}

// ---------------------------------------------------------------------------
// 6. Details panel, via the same property-handle API the panel widgets drive.
//    Covers the Set/None state, inner FVector X/Y/Z children, the TArray
//    inner handle, and multi-select with a mixed set/unset selection.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptDetailsHandles,
	"URLab.Spike.TOptional.DetailsHandles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptDetailsHandles::RunTest(const FString& Parameters)
{
	if (!FSlateApplication::IsInitialized())
	{
		AddWarning(TEXT("Slate not initialized; details-handle coverage skipped in this run."));
		return true;
	}

	FPropertyEditorModule& PEM = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	FPropertyRowGeneratorArgs Args;
	Args.bShouldShowHiddenProperties = true;
	TSharedRef<IPropertyRowGenerator> Gen = PEM.CreatePropertyRowGenerator(Args);

	UMjSpikeOptionalComponent* A = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), *(TEXT("SpikeDetailsA_") + UniqueSuffix()));
	A->SetFlags(RF_Transactional);
	SetAuthoredValues(A);

	Gen->SetObjects(TArray<UObject*>{A});
	DumpNodes(*this, Gen->GetRootTreeNodes(), 0);

	// A SET optional collapses to its inner value row (DetailPropertyRow sets
	// bForceShowOnlyChildren), so the row's own handle is the inner value and
	// the optional handle is reached through the parent.
	TSharedPtr<IDetailTreeNode> DoubleNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptDouble"), ENodeKind::ValueOnly);
	TestFalse(TEXT("a set optional exposes no FOptionalProperty row of its own"),
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptDouble"), ENodeKind::OptionalOnly).IsValid());
	if (TestTrue(TEXT("set OptDouble surfaces as its inner value row"), DoubleNode.IsValid()))
	{
		TSharedPtr<IPropertyHandle> H = DoubleNode->CreatePropertyHandle();
		double Read = 0.0;
		TestEqual(TEXT("inner value row reads the optional's value"),
			(int32)H->GetValue(Read), (int32)FPropertyAccess::Success);
		TestEqual(TEXT("inner value row value"), Read, 1.5);

		TSharedPtr<IPropertyHandle> Parent = H->GetParentHandle();
		TSharedPtr<IPropertyHandleOptional> Opt = Parent.IsValid() ? Parent->AsOptional() : nullptr;
		if (TestTrue(TEXT("parent handle of a set optional's value row is the optional handle"), Opt.IsValid()))
		{
			FProperty* Inner = nullptr;
			TestEqual(TEXT("GetOptionalValue(set) succeeds"),
				(int32)Opt->GetOptionalValue(Inner), (int32)FPropertyAccess::Success);
			TestNotNull(TEXT("set optional reports an inner value property"), Inner);

			// This is what the row's 'X' (EPropertyButton::OptionalClear) does.
			TestEqual(TEXT("ClearOptionalValue succeeds"),
				(int32)Opt->ClearOptionalValue(), (int32)FPropertyAccess::Success);
			TestFalse(TEXT("clear through the handle unsets the property"), A->OptDouble.IsSet());
		}
	}

	// An UNSET optional keeps its own row: this is the "None" state carrying
	// the Set button (EPropertyButton::OptionalSet).
	Gen->SetObjects(TArray<UObject*>{A});
	TSharedPtr<IDetailTreeNode> StringNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptString"), ENodeKind::OptionalOnly);
	if (TestTrue(TEXT("unset OptString keeps its own optional row"), StringNode.IsValid()))
	{
		TSharedPtr<IPropertyHandleOptional> Opt = StringNode->CreatePropertyHandle()->AsOptional();
		if (TestTrue(TEXT("OptString handle is an optional handle"), Opt.IsValid()))
		{
			FProperty* Inner = nullptr;
			TestEqual(TEXT("GetOptionalValue(unset) succeeds"),
				(int32)Opt->GetOptionalValue(Inner), (int32)FPropertyAccess::Success);
			TestNull(TEXT("unset optional reports no inner value property"), Inner);

			TestEqual(TEXT("SetOptionalValue(nullptr) default-initialises"),
				(int32)Opt->SetOptionalValue(nullptr, nullptr), (int32)FPropertyAccess::Success);
			TestTrue(TEXT("set through the handle sets the property"), A->OptString.IsSet());
			if (A->OptString.IsSet())
				TestEqual(TEXT("handle-set value is default-constructed"), A->OptString.GetValue(), FString());
			A->OptString.Reset();
		}
	}

	// Inner FVector must expose the usual X/Y/Z children.
	A->OptDouble = 1.5;
	Gen->SetObjects(TArray<UObject*>{A});
	TSharedPtr<IDetailTreeNode> VecNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptVector"), ENodeKind::ValueOnly);
	if (TestTrue(TEXT("set OptVector surfaces as its inner FVector row"), VecNode.IsValid()))
	{
		TSharedPtr<IPropertyHandle> H = VecNode->CreatePropertyHandle();
		TSharedPtr<IPropertyHandle> XHandle = H->GetChildHandle(TEXT("X"), /*bRecurse=*/true);
		if (TestTrue(TEXT("inner FVector exposes an X child handle"), XHandle.IsValid()))
		{
			TestEqual(TEXT("X writes through the optional"),
				(int32)XHandle->SetValue(42.0), (int32)FPropertyAccess::Success);
			TestTrue(TEXT("editing X writes through to the optional value"),
				A->OptVector.IsSet() && A->OptVector.GetValue().X == 42.0);
			A->OptVector = FVector(1.0, 2.0, 3.0);
		}
	}

	// The inner FQuat row must look like a plain FQuat row, so that anything
	// odd about it is FQuat's own details customization and not TOptional's.
	Gen->SetObjects(TArray<UObject*>{A});
	TSharedPtr<IDetailTreeNode> OptQuatNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptQuat"), ENodeKind::ValueOnly);
	TSharedPtr<IDetailTreeNode> PlainQuatNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("PlainQuat"), ENodeKind::Any);
	if (TestTrue(TEXT("set OptQuat surfaces as its inner FQuat row"), OptQuatNode.IsValid()) &&
		TestTrue(TEXT("PlainQuat control row exists"), PlainQuatNode.IsValid()))
	{
		TArray<TSharedRef<IDetailTreeNode>> OptQuatChildren;
		TArray<TSharedRef<IDetailTreeNode>> PlainQuatChildren;
		OptQuatNode->GetChildren(OptQuatChildren);
		PlainQuatNode->GetChildren(PlainQuatChildren);
		SpikeLog(*this, FString::Printf(TEXT("OptQuat value row children = %d, PlainQuat row children = %d"),
			OptQuatChildren.Num(), PlainQuatChildren.Num()));
		TestEqual(TEXT("inner FQuat row has the same child count as a plain FQuat row"),
			OptQuatChildren.Num(), PlainQuatChildren.Num());
	}

	// Inner TArray must be the normal array UI.
	Gen->SetObjects(TArray<UObject*>{A});
	TSharedPtr<IDetailTreeNode> ArrNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptArray"), ENodeKind::ValueOnly);
	if (TestTrue(TEXT("set OptArray surfaces as its inner array row"), ArrNode.IsValid()))
	{
		TSharedPtr<IPropertyHandle> H = ArrNode->CreatePropertyHandle();
		TSharedPtr<IPropertyHandleArray> ArrHandle = H->AsArray();
		if (TestTrue(TEXT("inner TArray exposes an array handle"), ArrHandle.IsValid()))
		{
			uint32 NumElems = 0;
			ArrHandle->GetNumElements(NumElems);
			TestEqual(TEXT("array handle sees 3 elements"), (int32)NumElems, 3);
			TestEqual(TEXT("AddItem succeeds"), (int32)ArrHandle->AddItem(), (int32)FPropertyAccess::Success);
			TestTrue(TEXT("AddItem grew the optional's array"),
				A->OptArray.IsSet() && A->OptArray.GetValue().Num() == 4);
			A->OptArray = TArray<double>{0.25, -4.0, 9.75};
		}
	}

	// Multi-select with mixed set/unset.
	UMjSpikeOptionalComponent* B = NewObject<UMjSpikeOptionalComponent>(
		GetTransientPackage(), *(TEXT("SpikeDetailsB_") + UniqueSuffix()));
	B->SetFlags(RF_Transactional);
	// B leaves OptDouble unset while A has it set.

	Gen->SetObjects(TArray<UObject*>{A, B});
	DumpNodes(*this, Gen->GetRootTreeNodes(), 0);
	TSharedPtr<IDetailTreeNode> MixedNode =
		FindNodeByPropertyName(Gen->GetRootTreeNodes(), TEXT("OptDouble"), ENodeKind::OptionalOnly);
	if (TestTrue(TEXT("mixed selection keeps the optional row"), MixedNode.IsValid()))
	{
		TSharedPtr<IPropertyHandleOptional> Opt = MixedNode->CreatePropertyHandle()->AsOptional();
		if (TestTrue(TEXT("multi-select handle is an optional handle"), Opt.IsValid()))
		{
			FProperty* Inner = nullptr;
			const FPropertyAccess::Result R = Opt->GetOptionalValue(Inner);
			SpikeLog(*this, FString::Printf(TEXT("mixed-selection GetOptionalValue = %d (0=MultipleValues,1=Fail,2=Success)"), (int32)R));
			TestEqual(TEXT("mixed set/unset selection reports MultipleValues"),
				(int32)R, (int32)FPropertyAccess::MultipleValues);

			// A blanket set across a mixed selection must land on both.
			TestEqual(TEXT("SetOptionalValue across mixed selection succeeds"),
				(int32)Opt->SetOptionalValue(nullptr, nullptr), (int32)FPropertyAccess::Success);
			TestTrue(TEXT("mixed-selection set reaches A"), A->OptDouble.IsSet());
			TestTrue(TEXT("mixed-selection set reaches B"), B->OptDouble.IsSet());
		}
	}

	GEditor->ResetTransaction(NSLOCTEXT("MjSpike", "ResetTxDetails", "Spike details cleanup"));
	return true;
}

// ---------------------------------------------------------------------------
// UHT hazard: BlueprintReadWrite TOptional<int32> compiles, but the K2 schema
// cannot form a pin type for it, so it yields a broken pin rather than a
// build error. The `Replicated` hazard is a hard UHT error and cannot be
// asserted from a compiled test; see docs/spike_0b_toptional.md.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSpikeOptBlueprintPinHazard,
	"URLab.Spike.TOptional.BlueprintPinHazard",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSpikeOptBlueprintPinHazard::RunTest(const FString& Parameters)
{
	FProperty* P = Prop(TEXT("OptBlueprintInt"));
	if (!TestNotNull(TEXT("OptBlueprintInt reflected"), P))
		return false;

	TestTrue(TEXT("OptBlueprintInt carries CPF_BlueprintVisible"),
		P->HasAnyPropertyFlags(CPF_BlueprintVisible));

	FEdGraphPinType PinType;
	const bool bConverts = GetDefault<UEdGraphSchema_K2>()->ConvertPropertyToPinType(P, PinType);
	SpikeLog(*this, FString::Printf(
		TEXT("ConvertPropertyToPinType(OptBlueprintInt) = %s, PinCategory = %s"),
		bConverts ? TEXT("true") : TEXT("false"), *PinType.PinCategory.ToString()));
	TestFalse(TEXT("K2 schema cannot form a pin type (broken pin hazard confirmed)"), bConverts);
	return true;
}
