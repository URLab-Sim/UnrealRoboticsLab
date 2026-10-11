// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The round-trip and differential-compile gates for the generated MuJoCo spec profile.
//
// Round-trip fixpoint proves the writer emits exactly what the reader accepted,
// and the differential compile proves that what it emits compiles to the same
// mjModel the original did. The second is the anchor: a round trip can be
// self-consistently wrong, a byte-identical compiled model cannot.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Editor.h"
#include "ScopedTransaction.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MjModelSemanticCompare.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include <type_traits>
#include <cstring>
#include <cmath>
#include <limits>

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "UObject/Package.h"
#include "UObject/Linker.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectHash.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MjEffectiveDetails.h"
#include "Tests/MjModelSemanticCompare.h"
#include "MjNativeDependencyHelper.h"
#include "MujocoGenerationAction.h"
#include "MujocoImportFactory.h"
#include "MuJoCo/Core/MjArticulation.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include <tinyxml2.h>
THIRD_PARTY_INCLUDES_END

namespace MjGenProfileTests
{

/**
 * A model exercising the shapes the profile has to get right: presence, fixed
 * arities including the quaternion permutation, sequences, enums, references,
 * a default class, and sibling order that the qpos layout depends on.
 */
const TCHAR* const InlineCorpus = TEXT(R"(<mujoco model="seam">
  <compiler angle="radian"/>
  <default>
    <default class="robot">
      <geom friction="1.5 0.005 0.0001" rgba="0.2 0.4 0.6 1"/>
    </default>
  </default>
  <asset>
    <material name="steel" rgba="0.5 0.5 0.55 1"/>
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.1" quat="0.7071067811865476 0 0 0.7071067811865476">
      <geom name="plinth" type="box" size="0.1 0.2 0.05" material="steel"/>
      <body name="link" childclass="robot" pos="0 0 0.2">
        <joint name="hinge" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom name="shin" type="capsule" size="0.02 0.15"/>
        <site name="tip" pos="0 0 0.3"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="drive" joint="hinge" kp="120" kv="8"/>
  </actuator>
  <sensor>
    <gyro name="imu" site="tip"/>
  </sensor>
</mujoco>
)");

// Imported fromto becomes canonical class pose and half-extents.
const TCHAR* const InlineInheritedFromto = TEXT(R"(<mujoco model="inherited_fromto_edit">
  <default><default class="limb"><geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.03"/></default></default>
  <worldbody><body name="link" pos="0 0 0.2"><geom name="limb_geom" class="limb"/><geom name="limb_geom_2" class="limb"/></body></worldbody>
</mujoco>)");

/** A throwaway Blueprint in the transient package; nothing reaches disk. */
UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjGenProfile_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

FString DiagnosticsToString(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	return FString::Join(Lines, TEXT("; "));
}

/** Ignore formatting-only text nodes and comments while preserving child order. */
const tinyxml2::XMLNode* NextContentNode(const tinyxml2::XMLNode* Node)
{
	while (Node != nullptr)
	{
		if (Node->ToComment() != nullptr)
		{
			Node = Node->NextSibling();
			continue;
		}
		if (const tinyxml2::XMLText* Text = Node->ToText())
		{
			if (FString(UTF8_TO_TCHAR(Text->Value())).TrimStartAndEnd().IsEmpty())
			{
				Node = Node->NextSibling();
				continue;
			}
		}
		break;
	}
	return Node;
}

bool XmlElementsMatch(const tinyxml2::XMLElement& A, const tinyxml2::XMLElement& B)
{
	if (std::strcmp(A.Name(), B.Name()) != 0)
	{
		return false;
	}
	int32 CountA = 0;
	int32 CountB = 0;
	for (const tinyxml2::XMLAttribute* Attribute = A.FirstAttribute(); Attribute != nullptr;
		Attribute = Attribute->Next())
	{
		++CountA;
		const char* Other = B.Attribute(Attribute->Name());
		if (Other == nullptr || std::strcmp(Attribute->Value(), Other) != 0)
		{
			return false;
		}
	}
	for (const tinyxml2::XMLAttribute* Attribute = B.FirstAttribute(); Attribute != nullptr;
		Attribute = Attribute->Next())
	{
		++CountB;
	}
	if (CountA != CountB)
	{
		return false;
	}
	const tinyxml2::XMLNode* ChildA = NextContentNode(A.FirstChild());
	const tinyxml2::XMLNode* ChildB = NextContentNode(B.FirstChild());
	while (ChildA != nullptr && ChildB != nullptr)
	{
		if (const tinyxml2::XMLElement* ElementA = ChildA->ToElement())
		{
			const tinyxml2::XMLElement* ElementB = ChildB->ToElement();
			if (ElementB == nullptr || !XmlElementsMatch(*ElementA, *ElementB))
			{
				return false;
			}
		}
		else if (ChildA->ToText() == nullptr || ChildB->ToText() == nullptr
			|| std::strcmp(ChildA->Value(), ChildB->Value()) != 0)
		{
			return false;
		}
		ChildA = NextContentNode(ChildA->NextSibling());
		ChildB = NextContentNode(ChildB->NextSibling());
	}
	return ChildA == nullptr && ChildB == nullptr;
}

bool XmlStructuresMatch(const FString& A, const FString& B)
{
	tinyxml2::XMLDocument ParsedA;
	tinyxml2::XMLDocument ParsedB;
	if (ParsedA.Parse(TCHAR_TO_UTF8(*A)) != tinyxml2::XML_SUCCESS
		|| ParsedB.Parse(TCHAR_TO_UTF8(*B)) != tinyxml2::XML_SUCCESS
		|| ParsedA.RootElement() == nullptr || ParsedB.RootElement() == nullptr)
	{
		return false;
	}
	return XmlElementsMatch(*ParsedA.RootElement(), *ParsedB.RootElement());
}

UMjGeomBase* FindGeomTemplate(UBlueprint& Blueprint, const TCHAR* Name)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (UMjGeomBase* Geom = Cast<UMjGeomBase>(Node->ComponentTemplate))
		{
			if (Geom->MjName.Get(FString()) == Name)
			{
				return Geom;
			}
		}
	}
	return nullptr;
}

UMjGeomBase* FindClassGeomTemplate(UBlueprint& Blueprint)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (UMjGeomBase* Geom = Cast<UMjGeomBase>(Node->ComponentTemplate))
		{
			if (Geom->IsClassPartial())
			{
				return Geom;
			}
		}
	}
	return nullptr;
}

/** Parse `Xml` into a fresh Blueprint and write it straight back out. */
bool RoundTrip(FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Filename,
	FString& OutWritten, bool* OutUnsupported = nullptr, const FMjDocParseOptions& Options = {})
{
	if (OutUnsupported != nullptr)
	{
		*OutUnsupported = false;
	}
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return false;
	}

	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Filename, Options);
	if (!Parsed.IsOk())
	{
		if (Parsed.IsUnsupportedOnly())
		{
			if (OutUnsupported != nullptr)
			{
				*OutUnsupported = true;
			}
			Test.AddInfo(FString::Printf(TEXT("%s: skipped, unsupported element families (%s)"), *Label,
				*DiagnosticsToString(Parsed.Errors)));
			return false;
		}
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	TArray<FMjSpecDiagnostic> WriteErrors;
	OutWritten = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	if (WriteErrors.Num() > 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: write failed: %s"), *Label, *DiagnosticsToString(WriteErrors)));
		return false;
	}
	if (OutWritten.IsEmpty())
	{
		Test.AddError(FString::Printf(TEXT("%s: writer produced nothing"), *Label));
		return false;
	}
	return true;
}

FString CorpusRoot()
{
	FString Root = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"),
		TEXT("Content"), TEXT("TestData"));
	FPaths::NormalizeDirectoryName(Root);
	return Root;
}

/** Named packaged fixtures that must be exercised, regardless of optional external corpora. */
TArray<FString> RequiredCorpusFiles()
{
	return {
		FPaths::Combine(CorpusRoot(), TEXT("humanoid.xml")),
		FPaths::Combine(CorpusRoot(), TEXT("parity"), TEXT("defaults_nested.xml")),
		FPaths::Combine(CorpusRoot(), TEXT("parity"), TEXT("actuators_all.xml")),
		FPaths::Combine(CorpusRoot(), TEXT("parity"), TEXT("assets_meshes.xml")),
		FPaths::Combine(CorpusRoot(), TEXT("parity"), TEXT("keyframes_options.xml")),
		FPaths::Combine(CorpusRoot(), TEXT("parity"), TEXT("trossen_vx300s"), TEXT("vx300s.xml")),
	};
}

void CheckRequiredCorpusFiles(FAutomationTestBase& Test, const TSet<FString>& Compared)
{
	for (const FString& File : RequiredCorpusFiles())
	{
		if (!IFileManager::Get().FileExists(*File))
		{
			Test.AddError(FString::Printf(TEXT("required packaged fixture is missing: %s"), *File));
		}
		else if (!Compared.Contains(File))
		{
			Test.AddError(FString::Printf(TEXT("required packaged fixture was not compared: %s"), *File));
		}
	}
}

/**
 * Every MJCF file the run has to work with.
 *
 * Include fragments are excluded: a <mujocoinclude> is not a spec and
 * only means anything spliced into the file that includes it, which the reader
 * does during that file's own parse. A plain <mujoco> wrapper with no
 * <worldbody> is the same thing in disguise -- an <include> target that cannot
 * compile standalone -- so a <worldbody> is required as well.
 */
TArray<FString> CorpusFiles()
{
	TArray<FString> Found;

	const FString TestData = CorpusRoot();
	IFileManager::Get().FindFilesRecursive(Found, *TestData, TEXT("*.xml"), true, false);

	// Two subdirectories are outputs or non-inputs, never fixtures. `goldens`
	// holds recorded answers: re-reading one as input would test the run
	// against itself, and its asset references are relative to the fixture it
	// was recorded from, so they do not resolve from there. `boundary` holds
	// models that exercise the reader and writer but cannot compile in URLab,
	// which is the property that puts them in that directory.
	const FString Goldens = FPaths::Combine(TestData, TEXT("goldens")) + TEXT("/");
	const FString Boundary = FPaths::Combine(TestData, TEXT("boundary")) + TEXT("/");
	Found.RemoveAll([&Goldens, &Boundary](const FString& File) {
		FString Path = File;
		FPaths::NormalizeFilename(Path);
		return Path.StartsWith(Goldens) || Path.StartsWith(Boundary);
	});

	const FString Menagerie = FPlatformMisc::GetEnvironmentVariable(TEXT("MUJOCO_MENAGERIE_PATH"));
	if (!Menagerie.IsEmpty())
	{
		TArray<FString> Extra;
		IFileManager::Get().FindFilesRecursive(Extra, *Menagerie, TEXT("*.xml"), true, false);
		Found.Append(Extra);
	}

	TArray<FString> Files;
	for (const FString& File : Found)
	{
		FString Text;
		if (FFileHelper::LoadFileToString(Text, *File) && Text.Contains(TEXT("<mujoco"))
			&& !Text.Contains(TEXT("<mujocoinclude")) && Text.Contains(TEXT("<worldbody")))
		{
			Files.Add(File);
		}
	}
	return Files;
}

/**
 * Where two arrays first disagree, and by how much; empty when they agree.
 *
 * The element type decides the test. Float fields (mjtNum/float) compare with a
 * relative tolerance, because a model authored with euler angles or an
 * unnormalized quat recompiles to floats that differ in the last bits (~1e-13 in
 * geom_quat, bvh_aabb) without any real disagreement. Integer and byte fields
 * compare exactly: a size, index or flag that moved is a structural regression
 * and has to fail.
 *
 * The report names the field's first differing element, since "compiled models
 * differ at geom_quat" names three hundred numbers and leaves the reader to
 * guess which body moved.
 */
template <class T>
FString FieldDifference(const T* A, const T* B, int32 Rows, int32 Cols, int32* OutRow = nullptr)
{
	for (int32 Index = 0; Index < Rows * Cols; ++Index)
	{
		if (MjModelSemanticCompare::SameFieldValue(A[Index], B[Index])) continue;
		const FString At = Cols > 1 ? FString::Printf(TEXT("[%d][%d]"), Index / Cols, Index % Cols)
									: FString::Printf(TEXT("[%d]"), Index);
		if (OutRow != nullptr)
		{
			*OutRow = Index / Cols;
		}
		if constexpr (std::is_floating_point_v<T>)
		{
			return FString::Printf(TEXT(", first at %s: %.17g vs %.17g"), *At, static_cast<double>(A[Index]),
				static_cast<double>(B[Index]));
		}
		else if constexpr (std::is_integral_v<T>)
		{
			return FString::Printf(TEXT(", first at %s: %lld vs %lld"), *At, static_cast<long long>(A[Index]),
				static_cast<long long>(B[Index]));
		}
		else
		{
			return FString::Printf(TEXT(", first at %s"), *At);
		}
	}
	return FString();
}

bool IsBodyBvhNode(const mjModel& Model, int Node)
{
	for (int Body = 0; Body < Model.nbody; ++Body)
	{
		const int Start = Model.body_bvhadr[Body];
		const int Count = Model.body_bvhnum[Body];
		if (Start >= 0 && Node >= Start && Node < Start + Count) return true;
	}
	return false;
}

bool SameBodyBvhRanges(const mjModel& A, const mjModel& B)
{
	if (A.nbody != B.nbody) return false;
	for (int Body = 0; Body < A.nbody; ++Body)
		if (A.body_bvhadr[Body] != B.body_bvhadr[Body] || A.body_bvhnum[Body] != B.body_bvhnum[Body]) return false;
	return true;
}

template <class T>
FString FieldDifferenceOutsideBodyBvh(const T* A, const T* B, int32 Rows, int32 Cols,
	const mjModel& ModelA, const mjModel& ModelB, int32* OutRow = nullptr)
{
	for (int32 Row = 0; Row < Rows; ++Row)
	{
		if (IsBodyBvhNode(ModelA, Row) && IsBodyBvhNode(ModelB, Row)) continue;
		for (int32 Column = 0; Column < Cols; ++Column)
		{
			const int32 Index = Row * Cols + Column;
			if (MjModelSemanticCompare::SameFieldValue(A[Index], B[Index])) continue;
			if (OutRow != nullptr) *OutRow = Row;
			const FString At = Cols > 1 ? FString::Printf(TEXT("[%d][%d]"), Row, Column)
				: FString::Printf(TEXT("[%d]"), Row);
			if constexpr (std::is_floating_point_v<T>)
				return FString::Printf(TEXT(", first at %s: %.17g vs %.17g"), *At,
					static_cast<double>(A[Index]), static_cast<double>(B[Index]));
			else
				return FString::Printf(TEXT(", first at %s: %lld vs %lld"), *At,
					static_cast<long long>(A[Index]), static_cast<long long>(B[Index]));
		}
	}
	return FString();
}

bool BodyBvhRepresentationDiffers(const mjModel& A, const mjModel& B)
{
	for (int Node = 0; Node < A.nbvh; ++Node)
		if (IsBodyBvhNode(A, Node) && IsBodyBvhNode(B, Node) && A.bvh_nodeid[Node] != B.bvh_nodeid[Node]) return true;
	for (int Node = 0; Node < A.nbvhstatic; ++Node)
	{
		if (!IsBodyBvhNode(A, Node) || !IsBodyBvhNode(B, Node)) continue;
		for (int Axis = 0; Axis < 6; ++Axis)
			if (!MjModelSemanticCompare::SameFieldValue(A.bvh_aabb[6 * Node + Axis], B.bvh_aabb[6 * Node + Axis])) return true;
	}
	return false;
}

/** Name the affected object when a pointer array has one row per object. */
FString ModelObjectAtRow(const mjModel& Model, const FString& Field, int32 Row, int32 Rows)
{
	struct FObjectFamily
	{
		const TCHAR* Prefix;
		int32 Count;
		mjtObj Type;
	};
	const FObjectFamily Families[] = {
		{TEXT("body_"), Model.nbody, mjOBJ_BODY},
		{TEXT("geom_"), Model.ngeom, mjOBJ_GEOM},
		{TEXT("jnt_"), Model.njnt, mjOBJ_JOINT},
		{TEXT("site_"), Model.nsite, mjOBJ_SITE},
		{TEXT("cam_"), Model.ncam, mjOBJ_CAMERA},
		{TEXT("light_"), Model.nlight, mjOBJ_LIGHT},
		{TEXT("tendon_"), Model.ntendon, mjOBJ_TENDON},
		{TEXT("actuator_"), Model.nu, mjOBJ_ACTUATOR},
		{TEXT("sensor_"), Model.nsensor, mjOBJ_SENSOR},
	};
	for (const FObjectFamily& Family : Families)
	{
		if (Rows == Family.Count && Field.StartsWith(Family.Prefix) && Row < Rows)
		{
			const char* Name = mj_id2name(&Model, Family.Type, Row);
			return Name != nullptr ? FString::Printf(TEXT(" (object '%s')"), UTF8_TO_TCHAR(Name)) : FString();
		}
	}
	return FString();
}

/**
 * Compile both texts and compare the resulting models field by field.
 *
 * The comparison is driven by MuJoCo's own mjxmacro tables rather than a hand
 * list, so a field added by an engine bump is covered without an edit here --
 * which is the only way a differential gate stays honest across bumps.
 *
 * All four tables, not just the two that describe the buffer: `mjModel::opt`,
 * `::stat` and `::vis` are struct members rather than sizes or pointers, so a
 * gate built from MJMODEL_SIZES and MJMODEL_POINTERS alone cannot see a
 * regression in <option>, <statistic> or <visual> at all.
 */
bool CompiledModelsAgree(FAutomationTestBase& Test, const FString& Label, const FString& OriginalXml,
	const FString& WrittenXml, const FString& BaseDir, const FString& WrittenBaseDir = FString(),
	bool bDifferentResourceLocation = false)
{
	// Stage each text beside its own dependencies. Ordinary source-backed tests
	// use one directory; portable bundle tests use the original and output trees.
	const FString OriginalStageDir = BaseDir.IsEmpty() ? FPaths::ProjectIntermediateDir() : BaseDir;
	const FString WrittenStageDir = WrittenBaseDir.IsEmpty() ? OriginalStageDir : WrittenBaseDir;

	auto Load = [&](const FString& Text, const TCHAR* Which) -> mjModel* {
		char Error[1024] = {0};
		const FString Path = FPaths::Combine(FCString::Strcmp(Which, TEXT("written")) == 0
			? WrittenStageDir : OriginalStageDir,
			FString::Printf(TEXT("MjGenProfile_%s_%s.xml"), Which, *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
		if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			Test.AddError(FString::Printf(TEXT("%s: could not stage %s MJCF"), *Label, Which));
			return nullptr;
		}
		mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		if (Model == nullptr)
		{
			Test.AddError(FString::Printf(TEXT("%s: %s MJCF did not compile: %s"), *Label, Which, UTF8_TO_TCHAR(Error)));
			if (FCString::Strcmp(Which, TEXT("written")) == 0)
			{
				TArray<FString> Lines;
				Text.ParseIntoArrayLines(Lines);
				int32 Reported = 0;
				for (const FString& Line : Lines)
				{
					if (Line.Contains(TEXT("<geom")) && Line.Contains(TEXT("size=\"0")) && Reported++ < 8)
					{
						Test.AddError(FString::Printf(TEXT("%s: suspect emitted geom: %s"), *Label, *Line.TrimStartAndEnd()));
					}
				}
			}
		}
		IFileManager::Get().Delete(*Path);
		return Model;
	};

	mjModel* A = Load(OriginalXml, TEXT("original"));
	if (A == nullptr)
	{
		return false;
	}
	mjModel* B = Load(WrittenXml, TEXT("written"));
	if (B == nullptr)
	{
		mj_deleteModel(A);
		return false;
	}

	int32 Mismatches = 0;
	auto Report = [&](const TCHAR* Field, const FString& Where) {
		// The cap bounds the log, not the verdict; every mismatch still counts.
		if (Mismatches < 32)
		{
			Test.AddError(FString::Printf(TEXT("%s: compiled models differ at %s%s"), *Label, Field, *Where));
		}
		++Mismatches;
	};

#define X(name)                                                     \
	if (!(bDifferentResourceLocation && (FCString::Strcmp(TEXT(#name), TEXT("npaths")) == 0 || \
		(FCString::Strcmp(TEXT(#name), TEXT("nbuffer")) == 0 && A->npaths != B->npaths))) && A->name != B->name) \
	{                                                               \
		Report(TEXT(#name),                                         \
			FString::Printf(TEXT(": %d vs %d"), A->name, B->name)); \
	}
	MJMODEL_SIZES
	#undef X

	const bool bModelSizesMatch = Mismatches == 0;
	std::string BodyBvhErrorA;
	std::string BodyBvhErrorB;
	const bool bBodyBvhValidA = MjModelSemanticCompare::ValidateBodyBvh(*A, &BodyBvhErrorA);
	const bool bBodyBvhValidB = MjModelSemanticCompare::ValidateBodyBvh(*B, &BodyBvhErrorB);
	if (!bBodyBvhValidA)
		Report(TEXT("original body BVH validity"), FString::Printf(TEXT(": %s"), UTF8_TO_TCHAR(BodyBvhErrorA.c_str())));
	if (!bBodyBvhValidB)
		Report(TEXT("written body BVH validity"), FString::Printf(TEXT(": %s"), UTF8_TO_TCHAR(BodyBvhErrorB.c_str())));
	const bool bBodyBvhSemantic = bModelSizesMatch && bBodyBvhValidA && bBodyBvhValidB
		&& SameBodyBvhRanges(*A, *B);
	if (bModelSizesMatch)
	{
		MJMODEL_POINTERS_PREAMBLE(A)
#define X(type, name, nr, nc)                                                                                 \
	{                                                                                                         \
		if (FCString::Strcmp(TEXT(#name), TEXT("body_inertia")) != 0 &&                            \
			FCString::Strcmp(TEXT(#name), TEXT("body_iquat")) != 0 &&                             \
			!(bDifferentResourceLocation && (FCString::Strcmp(TEXT(#name), TEXT("paths")) == 0 || \
				FCString::Strcmp(TEXT(#name), TEXT("mesh_pathadr")) == 0 ||                    \
				FCString::Strcmp(TEXT(#name), TEXT("skin_pathadr")) == 0 ||                    \
				FCString::Strcmp(TEXT(#name), TEXT("hfield_pathadr")) == 0 ||                  \
				FCString::Strcmp(TEXT(#name), TEXT("tex_pathadr")) == 0)))                    \
		{                                                                                                         \
			int32 FirstRow = INDEX_NONE;                                                                         \
			const bool bBodyBvhField = FCString::Strcmp(TEXT(#name), TEXT("bvh_nodeid")) == 0                \
				|| FCString::Strcmp(TEXT(#name), TEXT("bvh_aabb")) == 0;                                       \
			const FString Where = bBodyBvhField && bBodyBvhSemantic                                           \
				? FieldDifferenceOutsideBodyBvh<type>(A->name, B->name, A->nr, nc, *A, *B, &FirstRow)           \
				: FieldDifference<type>(A->name, B->name, A->nr, nc, &FirstRow);                                \
			if (!Where.IsEmpty())                                                                                 \
			{                                                                                                     \
				Report(TEXT(#name), Where + ModelObjectAtRow(*A, TEXT(#name), FirstRow, A->nr));                   \
			}                                                                                                     \
		}                                                                                                         \
	}
		MJMODEL_POINTERS
#undef X
		// MuJoCo may choose another eigensolver basis when principal moments
		// coincide. Compare the body-frame inertia tensor instead of its basis.
		for (int32 Body = 0; Body < A->nbody; ++Body)
		{
			double MaxError = 0;
			if (!MjModelSemanticCompare::EquivalentBodyInertia(
				A->body_inertia + 3 * Body, A->body_iquat + 4 * Body,
				B->body_inertia + 3 * Body, B->body_iquat + 4 * Body, &MaxError))
			{
				Report(TEXT("body_inertia_tensor"), FString::Printf(TEXT("[%d], max component difference %.17g%s"),
					Body, MaxError, *ModelObjectAtRow(*A, TEXT("body_iquat"), Body, A->nbody)));
			}
		}

		// mjOption, mjStatistic and mjVisual: struct members, so neither of the
		// tables above reaches them. `X` names a scalar and `XVEC` an array, and
		// the two need different address expressions -- which is the whole reason
		// mjxmacro spells them apart.
#define MJ_COMPARE(FieldLabel, APtr, BPtr, Type, Num)                          \
	{                                                                          \
		const FString Where = FieldDifference<Type>((APtr), (BPtr), 1, (Num)); \
		if (!Where.IsEmpty())                                                  \
		{                                                                      \
			Report(FieldLabel, Where);                                         \
		}                                                                      \
	}

#define X(type, name, num) MJ_COMPARE(TEXT("opt.") TEXT(#name), &A->opt.name, &B->opt.name, type, num)
#define XVEC(type, name, num) MJ_COMPARE(TEXT("opt.") TEXT(#name), A->opt.name, B->opt.name, type, num)
		MJOPTION_FIELDS
#undef XVEC
#undef X

#define X(name, num) MJ_COMPARE(TEXT("stat.") TEXT(#name), &A->stat.name, &B->stat.name, mjtNum, num)
#define XVEC(name, num) MJ_COMPARE(TEXT("stat.") TEXT(#name), A->stat.name, B->stat.name, mjtNum, num)
		MJSTATISTIC_FIELDS
#undef XVEC
#undef X

		// Every mjVisual group. They share one X signature, so only the group
		// varies -- and the group has to appear in the label, because `haze`
		// names a field in two of them.
#define X(type, name, num) \
	MJ_COMPARE(TEXT("vis.") TEXT(VISLABEL) TEXT(".") TEXT(#name), &A->vis.VISGROUP.name, &B->vis.VISGROUP.name, type, num)
#define XVEC(type, name, num) \
	MJ_COMPARE(TEXT("vis.") TEXT(VISLABEL) TEXT(".") TEXT(#name), A->vis.VISGROUP.name, B->vis.VISGROUP.name, type, num)
#define VISGROUP global
#define VISLABEL "global"
		MJVISUAL_GLOBAL_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP quality
#define VISLABEL "quality"
		MJVISUAL_QUALITY_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP headlight
#define VISLABEL "headlight"
		MJVISUAL_HEADLIGHT_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP map
#define VISLABEL "map"
		MJVISUAL_MAP_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP scale
#define VISLABEL "scale"
		MJVISUAL_SCALE_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP rgba
#define VISLABEL "rgba"
		MJVISUAL_RGBA_FIELDS
#undef VISLABEL
#undef VISGROUP
#undef XVEC
#undef X
#undef MJ_COMPARE
	}
	if (bBodyBvhSemantic && Mismatches == 0 && BodyBvhRepresentationDiffers(*A, *B))
	{
		std::string ContactError;
		if (!MjModelSemanticCompare::EquivalentContactPhysics(*A, *B, &ContactError))
			Report(TEXT("body BVH physical behavior"), FString::Printf(TEXT(": %s"), UTF8_TO_TCHAR(ContactError.c_str())));
	}

	mj_deleteModel(A);
	mj_deleteModel(B);
	return Mismatches == 0;
}

/** `mj_saveXMLString` of a freshly parsed spec: the shape the handshake used to ship. */
bool SaveSpecXml(FAutomationTestBase& Test, const FString& Label, const FString& File, const FString& Text,
	FString& Out)
{
	char Error[1024] = {0};
	mjSpec* Spec = File.IsEmpty() ? mj_parseXMLString(TCHAR_TO_UTF8(*Text), nullptr, Error, sizeof(Error))
								  : mj_parseXML(TCHAR_TO_UTF8(*File), nullptr, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: mj_parseXML declined (%s)"), *Label, UTF8_TO_TCHAR(Error)));
		return false;
	}

	// MuJoCo refuses to write a spec it has not compiled ("Only compiled model
	// can be written"), which is why the engine's own handshake takes this text
	// after a compile rather than after a parse. The model is not wanted here,
	// only the side effect on the spec.
	if (mjModel* Compiled = mj_compile(Spec, nullptr))
	{
		mj_deleteModel(Compiled);
	}
	else
	{
		const char* SpecError = mjs_getError(Spec);
		Test.AddError(FString::Printf(TEXT("%s: mj_compile declined (%s)"), *Label,
			SpecError != nullptr ? UTF8_TO_TCHAR(SpecError) : TEXT("unknown")));
		mj_deleteSpec(Spec);
		return false;
	}

	bool bSaved = false;
	TArray<uint8> Buffer;
	for (int32 Capacity = 256 * 1024; Capacity <= 32 * 1024 * 1024; Capacity *= 2)
	{
		Buffer.SetNumUninitialized(Capacity);
		FMemory::Memzero(Buffer.GetData(), Capacity);
		char SaveError[1024] = "";
		if (mj_saveXMLString(Spec, reinterpret_cast<char*>(Buffer.GetData()), Capacity, SaveError,
				sizeof(SaveError))
			== 0)
		{
			Out = UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer.GetData()));
			bSaved = true;
			break;
		}
		if (!FString(UTF8_TO_TCHAR(SaveError)).Contains(TEXT("buffer"), ESearchCase::IgnoreCase))
		{
			Test.AddError(FString::Printf(TEXT("%s: mj_saveXMLString declined (%s)"), *Label,
				UTF8_TO_TCHAR(SaveError)));
			break;
		}
	}
	mj_deleteSpec(Spec);
	return bSaved;
}

/** Exercise the direct mjSpec build alongside the MJCF writer route. */
bool BuiltSpecAgrees(FAutomationTestBase& Test, const FString& Label, UBlueprint& Blueprint,
	const FString& NativeXml)
{
	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(FSpecRef::OverBlueprint(Blueprint), Diagnostics);
	if (Built.Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: direct BuildSpec failed: %s"), *Label,
			*DiagnosticsToString(Diagnostics)));
		return false;
	}
	mjModel* Model = mj_compile(Built.Spec, nullptr);
	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: direct BuildSpec did not compile: %s"), *Label,
			UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		return false;
	}
	mj_deleteModel(Model);
	TArray<uint8> Buffer;
	Buffer.SetNumZeroed(256 * 1024);
	char Error[1024] = {0};
	if (mj_saveXMLString(Built.Spec, reinterpret_cast<char*>(Buffer.GetData()), Buffer.Num(), Error,
		sizeof(Error)) != 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: direct BuildSpec save failed: %s"), *Label, UTF8_TO_TCHAR(Error)));
		return false;
	}
	return CompiledModelsAgree(Test, Label + TEXT(" (direct BuildSpec)"), NativeXml,
		UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer.GetData())), FString());
}

/** A one-line account of how two MJCF texts differ, for the record rather than the verdict. */
FString DescribeTextDelta(const FString& A, const FString& B)
{
	TArray<FString> LinesA;
	TArray<FString> LinesB;
	A.ParseIntoArrayLines(LinesA);
	B.ParseIntoArrayLines(LinesB);

	auto Trimmed = [](const TArray<FString>& Lines) {
		TArray<FString> Out;
		for (const FString& Line : Lines)
		{
			FString T = Line.TrimStartAndEnd();
			if (!T.IsEmpty())
			{
				Out.Add(MoveTemp(T));
			}
		}
		Out.Sort();
		return Out;
	};

	TArray<FString> SortedA = Trimmed(LinesA);
	TArray<FString> SortedB = Trimmed(LinesB);
	int32 Common = 0;
	for (int32 I = 0, J = 0; I < SortedA.Num() && J < SortedB.Num();)
	{
		const int32 Order = SortedA[I].Compare(SortedB[J]);
		if (Order == 0)
		{
			++Common;
			++I;
			++J;
		}
		else if (Order < 0)
		{
			++I;
		}
		else
		{
			++J;
		}
	}
	return FString::Printf(TEXT("%d lines saved / %d written, %d identical"), SortedA.Num(), SortedB.Num(), Common);
}

void UnloadCorpusPackage(UPackage* Package)
{
	if (Package == nullptr) return;
	ResetLoaders(Package);
	const FString Trash = Package->GetName() + TEXT("_TRASH_") + FGuid::NewGuid().ToString(EGuidFormats::Digits);
	Package->Rename(*Trash, nullptr, REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
	ForEachObjectWithPackage(Package, [](UObject* Object) {
		Object->ClearFlags(RF_Standalone | RF_Public);
		return true;
	});
	Package->ClearFlags(RF_Standalone | RF_Public);
	CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);
}

bool CorpusXmlHasExternalAbsoluteReference(const FString& XmlPath, const FString& BundleRoot, const FString& SourceRoot,
	const FString& IsolatedRoot, const FString& PreparedRoot, FString& Detail)
{
	TArray<FString> XmlFiles;
	IFileManager::Get().FindFilesRecursive(XmlFiles, *BundleRoot, TEXT("*.xml"), true, false);
	for (const FString& File : XmlFiles)
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *File))
		{
			Detail = FString::Printf(TEXT("cannot audit bundled XML '%s'"), *File);
			return true;
		}
		if (Text.Contains(SourceRoot) || Text.Contains(IsolatedRoot) || Text.Contains(PreparedRoot))
		{
			Detail = FString::Printf(TEXT("bundled XML '%s' retains an original absolute source path"), *File);
			return true;
		}
		tinyxml2::XMLDocument Parsed;
		if (Parsed.Parse(TCHAR_TO_UTF8(*Text)) != tinyxml2::XML_SUCCESS)
		{
			Detail = FString::Printf(TEXT("bundled XML '%s' is malformed"), *File);
			return true;
		}
		TArray<const tinyxml2::XMLElement*> Pending;
		if (Parsed.RootElement() != nullptr) Pending.Add(Parsed.RootElement());
		while (!Pending.IsEmpty())
		{
			const tinyxml2::XMLElement* Element = Pending.Pop();
			for (const tinyxml2::XMLAttribute* Attr = Element->FirstAttribute(); Attr != nullptr; Attr = Attr->Next())
			{
				const FString Name = UTF8_TO_TCHAR(Attr->Name());
				const bool bResourcePath = Name.StartsWith(TEXT("file")) ||
					(FStringView(UTF8_TO_TCHAR(Element->Name())).Equals(TEXT("compiler")) && Name.EndsWith(TEXT("dir")));
				if (!bResourcePath) continue;
				const FString Value = UTF8_TO_TCHAR(Attr->Value());
				if (Value.IsEmpty()) continue;
				if (!FPaths::IsRelative(Value) || Value.Contains(TEXT("://")))
				{
					Detail = FString::Printf(TEXT("bundled XML '%s' has external absolute resource %s='%s'"),
						*File, *Name, *Value);
					return true;
				}
				const FString Resolved = FPaths::ConvertRelativePathToFull(FPaths::GetPath(File), Value);
				if (!FPaths::IsUnderDirectory(Resolved, BundleRoot))
				{
					Detail = FString::Printf(TEXT("bundled XML '%s' has escaping resource %s='%s'"),
						*File, *Name, *Value);
					return true;
				}
			}
			for (const tinyxml2::XMLElement* Child = Element->FirstChildElement(); Child != nullptr;
				Child = Child->NextSiblingElement()) Pending.Add(Child);
		}
	}
	return false;
}

bool CorpusConvertedAssetsOnDisk(UBlueprint& Blueprint, int32& OutCount,
	int32& OutUnsupportedPreviewCount, FString& Detail)
{
	OutCount = 0;
	OutUnsupportedPreviewCount = 0;
	int32 FileBacked = 0;
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		Detail = TEXT("the imported Blueprint has no construction script");
		return false;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UObject* Asset = nullptr;
		if (const UMjMesh* Mesh = Cast<UMjMesh>(Node->ComponentTemplate))
		{
			if (!Mesh->HasFile() || Mesh->GetFile().IsEmpty()) continue;
			++FileBacked;
			Asset = Mesh->MeshAsset.Get();
		}
		else if (const UMjTexture* Texture = Cast<UMjTexture>(Node->ComponentTemplate))
		{
			if (!Texture->HasFile() || Texture->GetFile().IsEmpty()) continue;
			++FileBacked;
			Asset = Texture->TextureAsset.Get();
		}
		else continue;
		if (Asset == nullptr)
		{
		// MuJoCo consumes KTX as an opaque texture resource. The editor has no
		// KTX preview importer, but the portable bundle must retain its bytes.
		if (const UMjTexture* Texture = Cast<UMjTexture>(Node->ComponentTemplate);
			Texture != nullptr && FPaths::GetExtension(Texture->GetFile()).Equals(TEXT("ktx"), ESearchCase::IgnoreCase))
		{
			++OutUnsupportedPreviewCount;
			--FileBacked;
			Detail = FString::Printf(TEXT("%d KTX texture(s) retain native resource bytes without an editor preview asset"),
				OutUnsupportedPreviewCount);
			continue;
		}
			Detail = FString::Printf(TEXT("only %d of %d file-backed mesh/texture assets converted"),
				OutCount, FileBacked);
			return false;
		}
		const FString AssetPackage = Asset->GetOutermost()->GetName();
		const FString AssetFile = FPackageName::LongPackageNameToFilename(AssetPackage,
			FPackageName::GetAssetPackageExtension());
		if (!IFileManager::Get().FileExists(*AssetFile))
		{
			Detail = FString::Printf(TEXT("converted asset '%s' has no saved package '%s'"),
				*Asset->GetPathName(), *AssetFile);
			return false;
		}
		++OutCount;
	}
	return OutCount == FileBacked;
}

bool RunPortableCorpusCase(FAutomationTestBase& Test, const FString& Path, int32 Index,
	const FString& Phase, const FString& RunId,
	const FString& ArtifactsDir, FString& Status, FString& Detail, FString& OutXml,
	int32& OutConvertedAssets, bool& bOutSourceHidden)
{
	Status = TEXT("source_read_failed");
	OutConvertedAssets = 0;
	int32 UnsupportedPreviewCount = 0;
	bOutSourceHidden = false;
	FString Original;
	if (!FFileHelper::LoadFileToString(Original, *Path)) return false;
	const FString Unique = FString::Printf(TEXT("MjCorpus_%s_%06d"), *RunId, Index);
	const FString InputDir = FPaths::Combine(FPaths::ProjectIntermediateDir(), TEXT("MjCorpusInput"), Unique);
	const FString PreparedDir = UMujocoImportFactory::ImportPrepDir(FPaths::Combine(InputDir, Unique + TEXT(".xml")));
	const FString OutputDir = FPaths::Combine(ArtifactsDir, Unique);
	const FString PackageName = TEXT("/Game/URLabCorpusVerification/BP_") + Unique;
	const FString BlueprintName = TEXT("BP_") + Unique;
	const FString PackageFile = FPackageName::LongPackageNameToFilename(PackageName,
		FPackageName::GetAssetPackageExtension());
	const FString ImportedAssetsDir = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("MuJoCoImports"),
		Unique + TEXT("_ue_Assets"));
	UPackage* Package = nullptr;
	UPackage* ReloadedPackage = nullptr;
	bool bPassed = false;
	do
	{
		if (Phase == TEXT("export"))
		{
			if (!IFileManager::Get().FileExists(*PackageFile))
			{
				Status = TEXT("phase1_blueprint_missing"); break;
			}
			ReloadedPackage = LoadPackage(nullptr, *PackageName, LOAD_None);
			UBlueprint* Reloaded = ReloadedPackage != nullptr
				? FindObject<UBlueprint>(ReloadedPackage, *BlueprintName) : nullptr;
			if (Reloaded == nullptr) { Status = TEXT("blueprint_reload_failed"); break; }
			if (!CorpusConvertedAssetsOnDisk(*Reloaded, OutConvertedAssets, UnsupportedPreviewCount, Detail))
			{
				Status = TEXT("reloaded_asset_missing"); break;
			}
			if (!IFileManager::Get().DeleteDirectory(*InputDir, false, true)
				|| IFileManager::Get().DirectoryExists(*InputDir))
			{
				Status = TEXT("source_hide_failed"); break;
			}
			if (IFileManager::Get().DirectoryExists(*PreparedDir)
				&& (!IFileManager::Get().DeleteDirectory(*PreparedDir, false, true)
					|| IFileManager::Get().DirectoryExists(*PreparedDir)))
			{
				Status = TEXT("prepared_source_hide_failed"); break;
			}
			bOutSourceHidden = true;
			FMjSourceBundle PersistedBundle;
			TArray<FMjSpecDiagnostic> BundleDiagnostics;
			if (!MjBuildSourceBundle(FSpecRef::OverBlueprint(*Reloaded), PersistedBundle, BundleDiagnostics))
			{
				Status = TEXT("persisted_bundle_failed"); Detail = DiagnosticsToString(BundleDiagnostics); break;
			}
			FString Error;
			if (!PersistedBundle.WriteToDirectory(OutputDir, OutXml, Error))
			{
				Status = TEXT("portable_write_failed"); Detail = Error; break;
			}
			if (CorpusXmlHasExternalAbsoluteReference(OutXml, OutputDir, FPaths::GetPath(Path), InputDir, PreparedDir, Detail))
			{
				Status = TEXT("source_reference_leaked"); break;
			}
			FString Written;
			if (!FFileHelper::LoadFileToString(Written, *OutXml))
			{
				Status = TEXT("portable_read_failed"); break;
			}
			// MuJoCo stores resolved filenames in model.paths. Its byte count,
			// allocation, and four *_pathadr offsets depend on staging directories;
			// every physical/semantic field remains compared.
			const bool bModelsAgree = CompiledModelsAgree(Test, Path, Original, Written, FPaths::GetPath(Path),
				FPaths::GetPath(OutXml), true);
			Status = bModelsAgree
				? (UnsupportedPreviewCount > 0 ? TEXT("passed_preview_unsupported") : TEXT("passed"))
				: TEXT("model_mismatch");
			bPassed = bModelsAgree;
			break;
		}
		if (Phase != TEXT("import")) { Status = TEXT("invalid_phase"); break; }
		FMjDocParseOptions Options;
		Options.bAllowExternalIncludes = true;
		Package = CreatePackage(*PackageName);
		UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
			FName(*BlueprintName), BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
		if (Blueprint == nullptr) { Status = TEXT("editor_blueprint_failed"); break; }
		Blueprint->SetFlags(RF_Public | RF_Standalone | RF_Transactional);
		FMjImportSettings ImportSettings;
		ImportSettings.Parse = Options;
		ImportSettings.bAllowPrompts = false;
		UBlueprint* ImportedBlueprint = nullptr;
		bool bCancelled = false;
		FString Error;
		if (!UMujocoImportFactory::ImportModel(Path, ImportSettings,
			[Blueprint]() { return Blueprint; }, ImportedBlueprint, Error, bCancelled)
			|| ImportedBlueprint != Blueprint)
		{
			Status = TEXT("editor_import_failed"); Detail = Error; break;
		}
		if (!CorpusConvertedAssetsOnDisk(*Blueprint, OutConvertedAssets, UnsupportedPreviewCount, Detail))
		{
			Status = TEXT("editor_asset_not_persisted");
			break;
		}
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(PackageFile), true);
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		if (!UPackage::SavePackage(Package, Blueprint, *PackageFile, SaveArgs))
		{
			Status = TEXT("blueprint_save_failed"); break;
		}
		Status = UnsupportedPreviewCount > 0 ? TEXT("passed_preview_unsupported") : TEXT("passed");
		bPassed = true;
	} while (false);
	if (ReloadedPackage != nullptr) UnloadCorpusPackage(ReloadedPackage);
	if (Package != nullptr) UnloadCorpusPackage(Package);
	if (Phase == TEXT("import") && bPassed) return true;
	IFileManager::Get().Delete(*PackageFile, false, true, true);
	IFileManager::Get().DeleteDirectory(*InputDir, false, true);
	IFileManager::Get().DeleteDirectory(*PreparedDir, false, true);
	IFileManager::Get().DeleteDirectory(*ImportedAssetsDir, false, true);
	return bPassed;
}

} // namespace MjGenProfileTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileFullCorpusTest,
	"URLab.Gen.Profile.FullCorpusDifferential",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileFullCorpusTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;
	const FString ManifestPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_MANIFEST"));
	const FString ResultsPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_RESULTS"));
	const FString ArtifactsDir = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_ARTIFACTS"));
	if (ManifestPath.IsEmpty() || ResultsPath.IsEmpty() || ArtifactsDir.IsEmpty())
	{
		AddError(TEXT("full corpus requires URLAB_MJCF_CORPUS_MANIFEST, _RESULTS and _ARTIFACTS"));
		return false;
	}
	FString ManifestText;
	TSharedPtr<FJsonObject> Manifest;
	if (!FFileHelper::LoadFileToString(ManifestText, *ManifestPath)
		|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(ManifestText), Manifest)
		|| !Manifest.IsValid())
	{
		AddError(FString::Printf(TEXT("cannot read corpus manifest %s"), *ManifestPath));
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Paths = nullptr;
	if (!Manifest->TryGetArrayField(TEXT("native_valid_paths"), Paths) || Paths == nullptr || Paths->IsEmpty())
	{
		AddError(TEXT("corpus manifest has no native-valid paths"));
		return false;
	}
	int32 NativeVersion = 0;
	if (!Manifest->TryGetNumberField(TEXT("mj_version"), NativeVersion) || NativeVersion != mj_version())
	{
		AddError(FString::Printf(TEXT("corpus native version mismatch: manifest=%d runtime=%d"),
			NativeVersion, mj_version()));
		return false;
	}
	const FString StartText = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_START"));
	const FString CountText = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_COUNT"));
	const int32 Start = StartText.IsEmpty() ? 0 : FCString::Atoi(*StartText);
	const int32 Count = CountText.IsEmpty() ? Paths->Num() : FCString::Atoi(*CountText);
	if (Start < 0 || Count <= 0 || Start >= Paths->Num())
	{
		AddError(FString::Printf(TEXT("invalid corpus slice start=%d count=%d total=%d"),
			Start, Count, Paths->Num()));
		return false;
	}
	const int32 End = FMath::Min(Start + Count, Paths->Num());
	IFileManager::Get().MakeDirectory(*ArtifactsDir, true);
	if (!FFileHelper::SaveStringToFile(FString(), *ResultsPath))
	{
		AddError(FString::Printf(TEXT("cannot create corpus result file %s"), *ResultsPath));
		return false;
	}
	int32 Passed = 0;
	int32 Failed = 0;
	for (int32 Index = Start; Index < End; ++Index)
	{
		const FString Path = (*Paths)[Index]->AsString();
		const double Began = FPlatformTime::Seconds();
		FAutomationTestExecutionInfo Before;
		GetExecutionInfo(Before);
		FString Status = TEXT("read_failed");
		FString WrittenPath;
		FString Source;
		if (!FFileHelper::LoadFileToString(Source, *Path))
		{
			AddError(FString::Printf(TEXT("%s: could not read native-valid corpus file"), *Path));
		}
		else
		{
			FString Written;
			bool bUnsupported = false;
			FMjDocParseOptions ParseOptions;
			ParseOptions.bAllowExternalIncludes = true;
			if (!RoundTrip(*this, Path, Source, Path, Written, &bUnsupported, ParseOptions))
			{
				Status = bUnsupported ? TEXT("unsupported_import") : TEXT("import_failed");
				if (bUnsupported)
				{
					AddError(FString::Printf(TEXT("%s: native-valid model uses an unsupported import family"), *Path));
				}
			}
			else
			{
				WrittenPath = FPaths::Combine(ArtifactsDir,
					FString::Printf(TEXT("%06d.export.xml"), Index));
				if (!FFileHelper::SaveStringToFile(Written, *WrittenPath,
					FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
				{
					Status = TEXT("artifact_failed");
					AddError(FString::Printf(TEXT("%s: could not save exported XML %s"), *Path, *WrittenPath));
				}
				else
				{
					Status = CompiledModelsAgree(*this, Path, Source, Written, FPaths::GetPath(Path))
						? TEXT("passed") : TEXT("model_mismatch");
				}
			}
		}
		FAutomationTestExecutionInfo After;
		GetExecutionInfo(After);
		TArray<TSharedPtr<FJsonValue>> Diagnostics;
		for (int32 EventIndex = Before.GetEntries().Num(); EventIndex < After.GetEntries().Num(); ++EventIndex)
		{
			const FAutomationEvent& Event = After.GetEntries()[EventIndex].Event;
			if (Event.Type == EAutomationEventType::Error || Event.Type == EAutomationEventType::Warning)
			{
				Diagnostics.Add(MakeShared<FJsonValueString>(Event.Message));
			}
		}
		TSharedRef<FJsonObject> Row = MakeShared<FJsonObject>();
		Row->SetStringField(TEXT("source"), Path);
		Row->SetNumberField(TEXT("index"), Index);
		Row->SetStringField(TEXT("status"), Status);
		Row->SetStringField(TEXT("export_xml"), WrittenPath);
		Row->SetBoolField(TEXT("allow_external_includes"), true);
		Row->SetNumberField(TEXT("seconds"), FPlatformTime::Seconds() - Began);
		Row->SetArrayField(TEXT("diagnostics"), Diagnostics);
		FString Json;
		FJsonSerializer::Serialize(Row,
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json));
		if (!FFileHelper::SaveStringToFile(Json + TEXT("\n"), *ResultsPath,
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append))
		{
			AddError(FString::Printf(TEXT("cannot append corpus result for %s"), *Path));
			return false;
		}
		Status == TEXT("passed") ? ++Passed : ++Failed;
		AddInfo(FString::Printf(TEXT("full corpus [%d/%d] %s: %s"), Index + 1,
			Paths->Num(), *Status, *Path));
	}
	AddInfo(FString::Printf(TEXT("full corpus slice start=%d end=%d passed=%d failed=%d total_native_valid=%d results=%s"),
		Start, End, Passed, Failed, Paths->Num(), *ResultsPath));
	return Failed == 0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfilePortableCorpusTest,
	"URLab.Gen.Profile.FullCorpusPortableEditor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfilePortableCorpusTest::RunTest(const FString&)
{
	using namespace MjGenProfileTests;
	const FString ManifestPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_MANIFEST"));
	const FString ResultsPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_PORTABLE_RESULTS"));
	const FString ArtifactsDir = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_PORTABLE_ARTIFACTS"));
	const FString Phase = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_PHASE"));
	const FString RunId = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_RUN_ID"));
	if (ManifestPath.IsEmpty() || ResultsPath.IsEmpty() || ArtifactsDir.IsEmpty()
		|| (Phase != TEXT("import") && Phase != TEXT("export")) || RunId.IsEmpty() || RunId.Len() > 24)
	{
		AddError(TEXT("portable corpus requires manifest, portable results/artifacts, phase=import|export, and run ID <=24 characters"));
		return false;
	}
	for (const TCHAR C : RunId)
	{
		if (!FChar::IsAlnum(C) && C != TEXT('_'))
		{
			AddError(TEXT("portable corpus run ID may contain only letters, digits, and underscores"));
			return false;
		}
	}
	FString ManifestText;
	TSharedPtr<FJsonObject> Manifest;
	if (!FFileHelper::LoadFileToString(ManifestText, *ManifestPath)
		|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(ManifestText), Manifest)
		|| !Manifest.IsValid())
	{
		AddError(FString::Printf(TEXT("cannot read corpus manifest %s"), *ManifestPath));
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Paths = nullptr;
	if (!Manifest->TryGetArrayField(TEXT("native_valid_paths"), Paths) || Paths == nullptr || Paths->IsEmpty())
	{
		AddError(TEXT("corpus manifest has no native-valid paths"));
		return false;
	}
	int32 NativeVersion = 0;
	if (!Manifest->TryGetNumberField(TEXT("mj_version"), NativeVersion) || NativeVersion != mj_version())
	{
		AddError(FString::Printf(TEXT("corpus native version mismatch: manifest=%d runtime=%d"),
			NativeVersion, mj_version()));
		return false;
	}
	const FString StartText = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_START"));
	const FString CountText = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MJCF_CORPUS_COUNT"));
	const int32 Start = StartText.IsEmpty() ? 0 : FCString::Atoi(*StartText);
	const int32 Count = CountText.IsEmpty() ? Paths->Num() : FCString::Atoi(*CountText);
	if (Start < 0 || Count <= 0 || Start >= Paths->Num())
	{
		AddError(FString::Printf(TEXT("invalid portable corpus slice start=%d count=%d total=%d"),
			Start, Count, Paths->Num()));
		return false;
	}
	const int32 End = FMath::Min(Start + Count, Paths->Num());
	IFileManager::Get().MakeDirectory(*ArtifactsDir, true);
	if (!FFileHelper::SaveStringToFile(FString(), *ResultsPath))
	{
		AddError(FString::Printf(TEXT("cannot create portable corpus result file %s"), *ResultsPath));
		return false;
	}
	int32 Passed = 0;
	for (int32 Index = Start; Index < End; ++Index)
	{
		const FString Path = (*Paths)[Index]->AsString();
		const double Began = FPlatformTime::Seconds();
		FAutomationTestExecutionInfo Before;
		GetExecutionInfo(Before);
		FString Status, Detail, ExportXml;
		int32 ConvertedAssets = 0;
		bool bSourceHidden = false;
		const bool bPassed = RunPortableCorpusCase(*this, Path, Index, Phase, RunId, ArtifactsDir,
			Status, Detail, ExportXml, ConvertedAssets, bSourceHidden);
		if (bPassed) ++Passed;
		else AddError(FString::Printf(TEXT("portable corpus [%d/%d] %s: %s%s"),
			Index + 1, Paths->Num(), *Status, *Path, Detail.IsEmpty() ? TEXT("") : *FString(TEXT("; ") + Detail)));
		FAutomationTestExecutionInfo After;
		GetExecutionInfo(After);
		TArray<TSharedPtr<FJsonValue>> Diagnostics;
		for (int32 EventIndex = Before.GetEntries().Num(); EventIndex < After.GetEntries().Num(); ++EventIndex)
		{
			const FAutomationEvent& Event = After.GetEntries()[EventIndex].Event;
			if (Event.Type == EAutomationEventType::Error || Event.Type == EAutomationEventType::Warning)
				Diagnostics.Add(MakeShared<FJsonValueString>(Event.Message));
		}
		TSharedRef<FJsonObject> Row = MakeShared<FJsonObject>();
		Row->SetStringField(TEXT("source"), Path);
		Row->SetNumberField(TEXT("index"), Index);
		Row->SetStringField(TEXT("phase"), Phase);
		Row->SetStringField(TEXT("run_id"), RunId);
		Row->SetStringField(TEXT("status"), Status);
		Row->SetStringField(TEXT("detail"), Detail);
		Row->SetStringField(TEXT("export_xml"), ExportXml);
		Row->SetNumberField(TEXT("converted_mesh_texture_assets"), ConvertedAssets);
		Row->SetBoolField(TEXT("source_hidden_before_export"), bSourceHidden);
		Row->SetNumberField(TEXT("seconds"), FPlatformTime::Seconds() - Began);
		Row->SetArrayField(TEXT("diagnostics"), Diagnostics);
		FString Json;
		FJsonSerializer::Serialize(Row,
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json));
		if (!FFileHelper::SaveStringToFile(Json + TEXT("\n"), *ResultsPath,
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append))
		{
			AddError(FString::Printf(TEXT("cannot append portable corpus result for %s"), *Path));
			return false;
		}
		AddInfo(FString::Printf(TEXT("portable corpus [%d/%d] %s: %s"), Index + 1,
			Paths->Num(), *Status, *Path));
	}
	AddInfo(FString::Printf(TEXT("portable corpus slice start=%d end=%d passed=%d failed=%d results=%s"),
		Start, End, Passed, End - Start - Passed, *ResultsPath));
	return Passed == End - Start;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileFixpointTest, "URLab.Gen.Profile.RoundTripFixpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileFixpointTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	FString First;
	if (!RoundTrip(*this, TEXT("inline"), InlineCorpus, TEXT("<inline>"), First))
	{
		return false;
	}

	FString Second;
	if (!RoundTrip(*this, TEXT("inline (second pass)"), First, TEXT("<inline>"), Second))
	{
		return false;
	}

	TestEqual(TEXT("the second write reaches a fixpoint"), Second, First);

	// As the differential gate: a corpus that has stopped being found must fail
	// this rather than sail through on the inline model alone.
	const TArray<FString> Files = CorpusFiles();
	TSet<FString> ComparedFiles;
	int32 Compared = 0;
	int32 Skipped = 0;
	int32 Failed = 0;
	for (const FString& File : Files)
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *File))
		{
			AddError(FString::Printf(TEXT("%s: could not load corpus file"), *File));
			++Failed;
			continue;
		}
		const FString Label = File;

		FString Written;
		bool bUnsupported = false;
		if (!RoundTrip(*this, Label, Text, File, Written, &bUnsupported))
		{
			bUnsupported ? ++Skipped : ++Failed;
			continue;
		}
		FString Rewritten;
		if (!RoundTrip(*this, Label + TEXT(" (second pass)"), Written, File, Rewritten, &bUnsupported))
		{
			bUnsupported ? ++Skipped : ++Failed;
			continue;
		}
		if (!TestTrue(*FString::Printf(TEXT("%s reaches a structural XML fixpoint"), *Label),
			XmlStructuresMatch(Rewritten, Written)))
		{
			++Failed;
		}
		ComparedFiles.Add(File);
		++Compared;
	}

	CheckRequiredCorpusFiles(*this, ComparedFiles);
	AddInfo(FString::Printf(TEXT("fixpoint corpus: discovered=%d compared=%d skipped_unsupported=%d failed=%d"),
		Files.Num(), Compared, Skipped, Failed));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileDifferentialTest, "URLab.Gen.Profile.DifferentialCompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileDifferentialTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	// Every path out of the loop below is a `continue`, so without a floor this
	// test passes on an empty corpus -- which is to say it passes when the thing
	// it gates has stopped running at all.
	int32 Compared = 0;
	int32 Skipped = 0;
	int32 Failed = 0;
	TSet<FString> ComparedFiles;

	FString Written;
	if (RoundTrip(*this, TEXT("inline"), InlineCorpus, TEXT("<inline>"), Written))
	{
		if (!CompiledModelsAgree(*this, TEXT("inline"), InlineCorpus, Written, FString()))
		{
			++Failed;
		}
		++Compared;
	}
	else
	{
		++Failed;
	}

	const TArray<FString> Files = CorpusFiles();
	for (const FString& File : Files)
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *File))
		{
			AddError(FString::Printf(TEXT("%s: could not load corpus file"), *File));
			++Failed;
			continue;
		}
		const FString Label = File;
		FString Out;
		bool bUnsupported = false;
		if (!RoundTrip(*this, Label, Text, File, Out, &bUnsupported))
		{
			bUnsupported ? ++Skipped : ++Failed;
			continue;
		}
		if (!CompiledModelsAgree(*this, Label, Text, Out, FPaths::GetPath(File)))
		{
			++Failed;
		}
		ComparedFiles.Add(File);
		++Compared;
	}

	CheckRequiredCorpusFiles(*this, ComparedFiles);
	AddInfo(FString::Printf(TEXT("differential corpus: discovered=%d compared=%d skipped_unsupported=%d failed=%d; inline_compared=%d"),
		Files.Num(), ComparedFiles.Num(), Skipped, Failed, Compared - ComparedFiles.Num()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileInheritedFromtoEditTest,
	"URLab.Gen.Profile.InheritedFromtoAfterBodyEdit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileInheritedFromtoEditTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;
	using namespace urlab::spec;

	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, InlineInheritedFromto, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		AddError(FString::Printf(TEXT("inherited fromto import failed: %s"), *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	UMjBody* Link = nullptr;
	FMjScsScope Scope(*Blueprint);
	TArray<UMjNodeComponent*> Pending{Parsed.Root};
	while (Pending.Num() > 0)
	{
		UMjNodeComponent* Node = Pending.Pop();
		if (UMjBody* Body = Cast<UMjBody>(Node))
		{
			if (Body->MjName.Get(FString()) == TEXT("link"))
			{
				Link = Body;
				break;
			}
		}
		for (const FMjOrderedChild& Child : FMjScsAdapter::OrderedChildren(*Node))
		{
			Pending.Add(Child.Node);
		}
	}
	if (Link == nullptr)
	{
		AddError(TEXT("imported Blueprint has no link body"));
		return false;
	}

	// Edit the stored Blueprint field, then ask native MuJoCo to compile the
	// corresponding source edit. The geom's inherited fromto still has to yield
	// the same compiled shape and position after this unrelated body edit.
	Link->SetPos(FMjPosition3(0.0, 0.0, 0.4));
	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	if (WriteErrors.Num() > 0 || Written.IsEmpty())
	{
		AddError(FString::Printf(TEXT("edited Blueprint write failed: %s"), *DiagnosticsToString(WriteErrors)));
		return false;
	}
	const FString NativeEdited = FString(InlineInheritedFromto).Replace(TEXT("pos=\"0 0 0.2\""),
		TEXT("pos=\"0 0 0.4\""));
	TestTrue(TEXT("native fixture contains the body edit"), NativeEdited != FString(InlineInheritedFromto));
	return CompiledModelsAgree(*this, TEXT("inherited fromto after Blueprint body edit"),
		NativeEdited, Written, FString());
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileLiveDefaultFromtoTest,
    "URLab.Gen.Profile.NormalizedDefaultSpatialEdits",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileLiveDefaultFromtoTest::RunTest(const FString& Parameters)
{
    using namespace MjGenProfileTests;
    using namespace urlab::spec;

    UBlueprint* Blueprint = MakeScratchBlueprint();
    if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) return false;
    const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, InlineInheritedFromto, TEXT("<inline>"));
    if (!Parsed.IsOk())
    {
        AddError(FString::Printf(TEXT("import failed: %s"), *DiagnosticsToString(Parsed.Errors)));
        return false;
    }
    UMjGeomBase* ClassGeom = FindClassGeomTemplate(*Blueprint);
    UMjGeomBase* First = FindGeomTemplate(*Blueprint, TEXT("limb_geom"));
    UMjGeomBase* Second = FindGeomTemplate(*Blueprint, TEXT("limb_geom_2"));
    if (!TestNotNull(TEXT("class geom"), ClassGeom) || !TestNotNull(TEXT("first consumer"), First)
        || !TestNotNull(TEXT("second consumer"), Second)) return false;
    TestFalse(TEXT("class endpoints normalized away"), ClassGeom->HasFromto());
    TestFalse(TEXT("class raw size normalized away"), ClassGeom->HasSize());
    TestTrue(TEXT("class has canonical position"), ClassGeom->HasPos());
    TestTrue(TEXT("class has persistent half-extents"), ClassGeom->MjSpatialHalfExtentsCm.IsSet());
    TestFalse(TEXT("consumer endpoints absent"), First->HasFromto() || Second->HasFromto());
    TestFalse(TEXT("consumers inherit pose and size"), First->HasPos() || First->HasSize()
        || Second->HasPos() || Second->HasSize() || First->MjSpatialHalfExtentsCm.IsSet()
        || Second->MjSpatialHalfExtentsCm.IsSet());

    auto WriteAndCompare = [&](const TCHAR* Label, const FString& Native) {
        TArray<FMjSpecDiagnostic> Errors;
        const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
        if (Errors.Num() || Written.IsEmpty())
        {
            AddError(FString::Printf(TEXT("%s: write failed: %s"), Label, *DiagnosticsToString(Errors)));
            return false;
        }
        TestFalse(*FString::Printf(TEXT("%s: no fromto survives export"), Label), Written.Contains(TEXT("fromto=")));
        TestFalse(*FString::Printf(TEXT("%s: no fromto returns to stored class"), Label), ClassGeom->HasFromto());
        return CompiledModelsAgree(*this, Label, Native, Written, FString())
            && BuiltSpecAgrees(*this, Label, *Blueprint, Native);
    };
    WriteAndCompare(TEXT("original native model"), InlineInheritedFromto);

    FTransform BeforeFirst, BeforeSecond, AfterFirst, AfterSecond;
    TestTrue(TEXT("first initial preview"), First->ComputePreviewTransform(BeforeFirst));
    TestTrue(TEXT("second initial preview"), Second->ComputePreviewTransform(BeforeSecond));
    TestTrue(TEXT("initial consumers share pose"), BeforeFirst.Equals(BeforeSecond));
    ClassGeom->SetFlags(RF_Transactional);
    {
        FScopedTransaction Transaction(NSLOCTEXT("MjGenProfile", "EditNormalizedClassSpatial", "Edit normalized class spatial"));
        ClassGeom->Modify();
        ClassGeom->SetPos(FMjPosition3(0.0, 0.0, 0.2));
        ClassGeom->MjSpatialHalfExtentsCm = FVector(5.0, 5.0, 20.0);
    }
    TestTrue(TEXT("first updated preview"), First->ComputePreviewTransform(AfterFirst));
    TestTrue(TEXT("second updated preview"), Second->ComputePreviewTransform(AfterSecond));
    TestTrue(TEXT("class edit reaches both consumers"), AfterFirst.Equals(AfterSecond));
    TestFalse(TEXT("class edit changes preview"), BeforeFirst.Equals(AfterFirst));
    TestNearlyEqual(TEXT("edited midpoint is 20 cm"), (float)AfterFirst.GetLocation().Z, 20.0f, 1e-4f);

    const FString NativeEdited = TEXT(R"(<mujoco model="inherited_fromto_edit">
  <default><default class="limb"><geom type="capsule" pos="0 0 0.2" quat="0 1 0 0" size="0.05 0.2"/></default></default>
  <worldbody><body name="link" pos="0 0 0.2"><geom name="limb_geom" class="limb"/><geom name="limb_geom_2" class="limb"/></body></worldbody>
</mujoco>)");
    WriteAndCompare(TEXT("edited canonical class"), NativeEdited);
    TestTrue(TEXT("undo class edit"), GEditor->UndoTransaction());
    FTransform UndoFirst;
    TestTrue(TEXT("first preview after undo"), First->ComputePreviewTransform(UndoFirst));
    TestTrue(TEXT("undo restores inherited preview"), UndoFirst.Equals(BeforeFirst));
    TestTrue(TEXT("redo class edit"), GEditor->RedoTransaction());
    FTransform RedoFirst;
    TestTrue(TEXT("first preview after redo"), First->ComputePreviewTransform(RedoFirst));
    TestTrue(TEXT("redo restores edited preview"), RedoFirst.Equals(AfterFirst));

    First->SetPos(FMjPosition3(0.0, 0.0, 0.6));
    First->SetQuat(FMjQuatRot(0.0, 1.0, 0.0, 0.0));
    First->MjSpatialHalfExtentsCm = FVector(8.0, 8.0, 10.0);
    TestTrue(TEXT("first authors local pose and vector size"), First->HasPos() && First->HasQuat()
        && First->MjSpatialHalfExtentsCm.IsSet());
    TestFalse(TEXT("second still inherits"), Second->HasPos() || Second->HasQuat()
        || Second->HasSize() || Second->MjSpatialHalfExtentsCm.IsSet());
    const FString NativeOverride = TEXT(R"(<mujoco model="inherited_fromto_edit">
  <default><default class="limb"><geom type="capsule" pos="0 0 0.2" quat="0 1 0 0" size="0.05 0.2"/></default></default>
  <worldbody><body name="link" pos="0 0 0.2">
    <geom name="limb_geom" class="limb" pos="0 0 0.6" quat="0 1 0 0" size="0.08 0.1"/>
    <geom name="limb_geom_2" class="limb"/>
  </body></worldbody>
</mujoco>)");
    WriteAndCompare(TEXT("local pose and atomic vector override"), NativeOverride);
    TestTrue(TEXT("restore inherited position control"),
        FMjEffectiveDetails::RestoreSpatialInheritance(*First, FName(TEXT("Pos"))));
    TestTrue(TEXT("restore inherited rotation control"),
        FMjEffectiveDetails::RestoreSpatialInheritance(*First, FName(TEXT("Quat"))));
    TestTrue(TEXT("restore inherited size control"),
        FMjEffectiveDetails::RestoreSpatialInheritance(*First, FName(TEXT("Size"))));
    TestFalse(TEXT("restore clears local canonical override"), First->MjSpatialHalfExtentsCm.IsSet()
        || First->HasPos() || First->HasQuat() || First->HasSize());
    FTransform ResetFirst;
    TestTrue(TEXT("restored preview"), First->ComputePreviewTransform(ResetFirst));
    TestTrue(TEXT("restore resumes class"), ResetFirst.Equals(AfterSecond));
    WriteAndCompare(TEXT("restored class inheritance"), NativeEdited);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileUntypedDefaultFromtoTest,
    "URLab.Gen.Profile.UntypedDefaultFromtoAcrossShapes",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileUntypedDefaultFromtoTest::RunTest(const FString& Parameters)
{
    using namespace MjGenProfileTests;
    using namespace urlab::spec;
    const FString Xml = TEXT(R"(<mujoco model="untyped_fromto">
  <default>
    <default class="shape"><geom fromto="0 0 0 0 0 0.4" size="0.05"/></default>
    <default class="unused"><geom fromto="0 0 0 0 0 0.2" size="0.02"/></default>
  </default>
  <worldbody><body name="b">
    <geom name="capsule" type="capsule" class="shape"/>
    <geom name="box" type="box" class="shape"/>
  </body></worldbody>
</mujoco>)");
    UBlueprint* Blueprint = MakeScratchBlueprint();
    if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) return false;
    const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
    if (!Parsed.IsOk())
    {
        AddError(FString::Printf(TEXT("untyped default import failed: %s"), *DiagnosticsToString(Parsed.Errors)));
        return false;
    }
    UMjGeomBase* ClassGeom = FindClassGeomTemplate(*Blueprint);
    UMjGeomBase* Capsule = FindGeomTemplate(*Blueprint, TEXT("capsule"));
    UMjGeomBase* Box = FindGeomTemplate(*Blueprint, TEXT("box"));
    if (!TestNotNull(TEXT("class geom"), ClassGeom) || !TestNotNull(TEXT("capsule"), Capsule)
        || !TestNotNull(TEXT("box"), Box)) return false;
    TestFalse(TEXT("untyped class has no raw endpoints or size"), ClassGeom->HasFromto() || ClassGeom->HasSize());
    TestTrue(TEXT("untyped class has editable half-extents"), ClassGeom->MjSpatialHalfExtentsCm.IsSet());
    TestFalse(TEXT("consumers have no source endpoints"), Capsule->HasFromto() || Box->HasFromto());
    TArray<FMjSpecDiagnostic> Errors;
    const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
    if (!TestEqual(TEXT("write diagnostics"), Errors.Num(), 0) || Written.IsEmpty()) return false;
    TestFalse(TEXT("export is canonical"), Written.Contains(TEXT("fromto=")));
    const bool bInitialParity = CompiledModelsAgree(*this, TEXT("untyped default consumed as capsule and box"), Xml, Written, FString())
        && BuiltSpecAgrees(*this, TEXT("untyped default native BuildSpec"), *Blueprint, Xml);

    // A class edit changes the canonical shape for existing consumers and a
    // newly added consumer alike, with no replay of the imported spelling.
    ClassGeom->SetPos(FMjPosition3(0.0, 0.0, 0.25));
    ClassGeom->MjSpatialHalfExtentsCm = FVector(8.0, 8.0, 25.0);
    USCS_Node* BodyNode = nullptr;
    for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
        if (UMjBody* Body = Cast<UMjBody>(Node->ComponentTemplate))
            if (Body->MjName.Get(FString()) == TEXT("b")) BodyNode = Node;
    if (!TestNotNull(TEXT("body SCS node"), BodyNode)) return false;
    USCS_Node* NewNode = Blueprint->SimpleConstructionScript->CreateNode(Capsule->GetClass(), TEXT("new_capsule"));
    BodyNode->AddChildNode(NewNode);
    UMjGeomBase* NewCapsule = Cast<UMjGeomBase>(NewNode->ComponentTemplate);
    if (!TestNotNull(TEXT("new capsule"), NewCapsule)) return false;
    NewCapsule->MjName = FString(TEXT("new_capsule"));
    NewCapsule->Dclass = FString(TEXT("shape"));
    NewCapsule->Type = EMjGeomType::capsule;
    TestFalse(TEXT("new consumer has no local size"), NewCapsule->HasSize()
        || NewCapsule->MjSpatialHalfExtentsCm.IsSet());
    const FString NativeEdited = TEXT(R"(<mujoco model="untyped_fromto">
  <default>
    <default class="shape"><geom fromto="0 0 0 0 0 0.5" size="0.08"/></default>
    <default class="unused"><geom fromto="0 0 0 0 0 0.2" size="0.02"/></default>
  </default>
  <worldbody><body name="b">
    <geom name="capsule" type="capsule" class="shape"/>
    <geom name="box" type="box" class="shape"/>
    <geom name="new_capsule" type="capsule" class="shape"/>
  </body></worldbody>
</mujoco>)");
    Errors.Reset();
    const FString Edited = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
    if (!TestEqual(TEXT("edited write diagnostics"), Errors.Num(), 0) || Edited.IsEmpty()) return false;
    TestFalse(TEXT("edited export remains canonical"), Edited.Contains(TEXT("fromto=")));
    return bInitialParity && CompiledModelsAgree(*this, TEXT("edited untyped class plus new consumer"),
        NativeEdited, Edited, FString());
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileMixedFromtoProjectionTest,
	"URLab.Gen.Profile.MixedFromtoProjection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileMixedFromtoProjectionTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;
	const FString Xml = TEXT(R"(<mujoco model="mixed_fromto">
  <default><default class="limb">
    <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.03"/>
    <site type="capsule" fromto="0 0 0 0 0 0.1" size="0.01"/>
  </default></default>
  <worldbody><body name="link">
    <geom name="g0" class="limb"/>
    <site name="s0" class="limb"/>
    <geom name="g1" type="sphere" size="0.02" pos="0.2 0 0"/>
    <site name="s1" class="limb"/>
  </body></worldbody>
  <tendon><spatial name="wire"><site site="s0"/><site site="s1"/></spatial></tendon>
</mujoco>)");
	FString Written;
	if (!RoundTrip(*this, TEXT("mixed geom/site/default/tendon"), Xml, TEXT("<inline>"), Written))
	{
		return false;
	}
	TestFalse(TEXT("mixed export projects fromto"), Written.Contains(TEXT("fromto=")));
	return CompiledModelsAgree(*this, TEXT("mixed geom/site/default/tendon"), Xml, Written, FString());
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileHandshakeMjcfTest, "URLab.Gen.Profile.HandshakeMjcfEquivalence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileHandshakeMjcfTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	// The bridge handshake's `mjcf_compiled` is MJCF text a client reloads
	// offline. It came from mj_saveXMLString over the engine's root spec and
	// comes from the spec writer once no such spec exists, so what has to
	// hold is not that the two texts match -- they cannot, one is MuJoCo's
	// canonical form and the other is the authored spec -- but that they
	// compile to the same model. The texts' shapes are recorded either way.
	int32 Compared = 0;
	TArray<TPair<FString, FString>> Cases;
	Cases.Emplace(FString(), InlineCorpus);
	for (const FString& File : CorpusFiles())
	{
		Cases.Emplace(File, FString());
	}

	for (const TPair<FString, FString>& Case : Cases)
	{
		const FString& File = Case.Key;
		const FString Label = File.IsEmpty() ? TEXT("inline") : FPaths::GetCleanFilename(File);

		FString SavedXml;
		if (!SaveSpecXml(*this, Label, File, Case.Value, SavedXml))
		{
			continue;
		}

		FString Written;
		if (!RoundTrip(*this, Label + TEXT(" (saved form)"), SavedXml, File, Written))
		{
			AddError(FString::Printf(TEXT("%s: the saved MJCF did not round-trip"), *Label));
			continue;
		}

		AddInfo(FString::Printf(TEXT("%s: %s"), *Label, *DescribeTextDelta(SavedXml, Written)));
		++Compared;
		CompiledModelsAgree(*this, Label, SavedXml, Written, FPaths::GetPath(File));
	}

	TestTrue(TEXT("at least one payload was compared"), Compared > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileSiblingOrderTest, "URLab.Gen.Profile.SiblingOrderSurvivesReorder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileSiblingOrderTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;
	using namespace urlab::spec;

	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}

	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, InlineCorpus, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		AddError(FString::Printf(TEXT("parse failed: %s"), *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	// Joint order under a body determines the qpos layout, so the spec's
	// order has to survive being restated by the adapter rather than being an
	// accident of whichever list the components happen to sit in.
	FMjScsScope Scope(*Blueprint);
	TArray<UMjNodeComponent*> BeforeOrder;
	TArray<UMjNodeComponent*> Pending{Parsed.Root};
	while (Pending.Num() > 0)
	{
		UMjNodeComponent* Node = Pending.Pop();
		BeforeOrder.Add(Node);
		for (const FMjOrderedChild& Child : FMjScsAdapter::OrderedChildren(*Node))
		{
			Pending.Add(Child.Node);
		}
	}

	for (UMjNodeComponent* Node : BeforeOrder)
	{
		FMjScsAdapter::Renumber(*Node);
	}

	TArray<FMjSpecDiagnostic> Errors;
	const FString AfterRenumber = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
	TestEqual(TEXT("renumbering is idempotent on a freshly read spec"), Errors.Num(), 0);
	TestFalse(TEXT("the spec still writes"), AfterRenumber.IsEmpty());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileNonfiniteComparisonTest,
	"URLab.Gen.Profile.NonfiniteCompiledFields",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileNonfiniteComparisonTest::RunTest(const FString&)
{
	using MjGenProfileTests::FieldDifference;
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	const double Infinity = std::numeric_limits<double>::infinity();
	const double SameNaN[1] = {NaN};
	const double OneSided[1] = {1.0};
	const double SameInfinity[1] = {Infinity};
	const double OppositeInfinity[1] = {-Infinity};
	TestTrue(TEXT("matching native NaNs compare equal"),
		FieldDifference(SameNaN, SameNaN, 1, 1).IsEmpty());
	TestFalse(TEXT("a NaN and a finite value differ"),
		FieldDifference(SameNaN, OneSided, 1, 1).IsEmpty());
	TestTrue(TEXT("matching infinities compare equal"),
		FieldDifference(SameInfinity, SameInfinity, 1, 1).IsEmpty());
	TestFalse(TEXT("opposite infinities differ"),
		FieldDifference(SameInfinity, OppositeInfinity, 1, 1).IsEmpty());
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
