// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MujocoImportFactory.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{
struct FMjNativeModelGuard
{
	mjModel* Model = nullptr;
	~FMjNativeModelGuard() { if (Model != nullptr) mj_deleteModel(Model); }
};

bool WriteMeshFixture(const FString& Path)
{
	// Tetrahedron with nonzero origin and unequal extents exposes translation,
	// axis, and unit changes. Triangles avoid importing a different quad split.
	const FString Obj = TEXT(
		"v 1.25 2.5 -0.75\n"
		"v 2.75 2.5 -0.75\n"
		"v 1.5 3.25 -0.75\n"
		"v 1.0 2.75 0.5\n"
		"vt 0.1 0.2\nvt 0.8 0.15\nvt 0.35 0.95\nvt 0.65 0.55\n"
		"f 1/1 2/2 3/3\n"
		"f 1/1 4/4 2/2\n"
		"f 2/2 4/4 3/3\n"
		"f 3/3 4/4 1/1\n");
	return FFileHelper::SaveStringToFile(Obj, *Path);
}

bool SaveFixture(const FString& Path, const FString& Text)
{
	return FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

bool CompileXmlFile(const FString& Path, FMjNativeModelGuard& OutModel, FString& OutError)
{
	char Error[2048]{};
	OutModel.Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
	if (OutModel.Model != nullptr) return true;
	OutError = UTF8_TO_TCHAR(Error);
	return false;
}

bool CompileGeneratedBundle(FAutomationTestBase& Test, const FMjSourceBundle& Bundle,
	FMjNativeModelGuard& OutModel)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FMjGeneratedBundleResource& Resource : Bundle.Resources)
	{
		if (mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Resource.Key),
			Resource.Bytes.GetData(), Resource.Bytes.Num()) != 0)
		{
			Test.AddError(FString::Printf(TEXT("cannot mount authored resource '%s'"), *Resource.Key));
			mj_deleteVFS(&Vfs);
			return false;
		}
	}
	char Error[2048]{};
	mjSpec* Spec = mj_parseXMLString(TCHAR_TO_UTF8(*Bundle.Xml), &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("generated authored MJCF failed native parse: %s"), UTF8_TO_TCHAR(Error)));
		mj_deleteVFS(&Vfs);
		return false;
	}
	OutModel.Model = mj_compile(Spec, &Vfs);
	if (OutModel.Model == nullptr)
		Test.AddError(FString::Printf(TEXT("generated authored MJCF failed native compile: %s"), UTF8_TO_TCHAR(mjs_getError(Spec))));
	mj_deleteSpec(Spec);
	mj_deleteVFS(&Vfs);
	return OutModel.Model != nullptr;
}

TArray<FVector> WorldTriangleCorners(const mjModel& Model, mjData& Data, int32 GeomId)
{
	TArray<FVector> Out;
	if (GeomId < 0) return Out;
	const int32 MeshId = Model.geom_dataid[GeomId];
	if (MeshId < 0) return Out;
	const int32 FaceAddress = Model.mesh_faceadr[MeshId];
	const int32 VertexAddress = Model.mesh_vertadr[MeshId];
	const int32 FaceCount = Model.mesh_facenum[MeshId];
	const mjtNum* Rotation = Data.geom_xmat + 9 * GeomId;
	const mjtNum* Translation = Data.geom_xpos + 3 * GeomId;
	Out.Reserve(FaceCount * 3);
	for (int32 Face = 0; Face < FaceCount; ++Face)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const int32 VertexId = Model.mesh_face[3 * (FaceAddress + Face) + Corner];
			const mjtNum* Vertex = Model.mesh_vert + 3 * (VertexAddress + VertexId);
			Out.Emplace(
				(float)(Rotation[0] * Vertex[0] + Rotation[1] * Vertex[1] + Rotation[2] * Vertex[2] + Translation[0]),
				(float)(Rotation[3] * Vertex[0] + Rotation[4] * Vertex[1] + Rotation[5] * Vertex[2] + Translation[1]),
				(float)(Rotation[6] * Vertex[0] + Rotation[7] * Vertex[1] + Rotation[8] * Vertex[2] + Translation[2]));
		}
	}
	return Out;
}

void SortCorners(TArray<FVector>& Corners)
{
	Corners.Sort([](const FVector& A, const FVector& B)
	{
		if (!FMath::IsNearlyEqual(A.X, B.X, 1e-6)) return A.X < B.X;
		if (!FMath::IsNearlyEqual(A.Y, B.Y, 1e-6)) return A.Y < B.Y;
		return A.Z < B.Z;
	});
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMeshNativeImportExportParityTest,
	"URLab.Import.NativeMeshAssetRoundTripParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjMeshNativeImportExportParityTest::RunTest(const FString&)
{
	const FString Unique = FGuid::NewGuid().ToString(EGuidFormats::Digits);
	const FString Scratch = FPaths::ProjectSavedDir() / TEXT("URLabTest/MeshNativeParity/") / Unique;
	const FString SourceDir = Scratch / TEXT("source");
	const FString HiddenSourceDir = Scratch / TEXT("source_hidden");
	IFileManager::Get().MakeDirectory(*SourceDir, true);
	struct FCleanup
	{
		FString Source;
		FString Hidden;
		bool bHidden = false;
		~FCleanup()
		{
			if (bHidden) IFileManager::Get().Move(*Hidden, *Source);
			IFileManager::Get().DeleteDirectory(*Source, false, true);
			IFileManager::Get().DeleteDirectory(*Hidden, false, true);
		}
	} Cleanup{SourceDir, HiddenSourceDir};

	const FString SourceMeshPath = SourceDir / TEXT("asymmetric.obj");
	const FString SourceXmlPath = SourceDir / TEXT("source.xml");
	if (!TestTrue(TEXT("write asymmetric OBJ fixture"), WriteMeshFixture(SourceMeshPath))) return false;
	const FString SourceXml = TEXT(
		"<mujoco model='mesh_roundtrip'><compiler angle='radian'/>"
		"<asset><mesh name='asymmetric' file='asymmetric.obj' scale='1.5 0.75 2.0'/></asset>"
		"<worldbody><body name='offset_body' pos='0.7 -0.4 1.2' quat='0.9238795325 0 0.3826834324 0'>"
		"<freejoint name='free'/><geom name='mesh_geom' type='mesh' mesh='asymmetric' pos='0.1 0.2 -0.3' "
		"quat='0.9659258263 0.2588190451 0 0' density='400'/></body></worldbody></mujoco>");
	if (!TestTrue(TEXT("write original MJCF"), SaveFixture(SourceXmlPath, SourceXml))) return false;

	FMjNativeModelGuard SourceModel;
	FString Error;
	if (!TestTrue(TEXT("original native model compiles"), CompileXmlFile(SourceXmlPath, SourceModel, Error)))
	{
		AddError(Error);
		return false;
	}

	const FString BlueprintName = TEXT("MeshNativeParity_") + Unique;
	UPackage* Package = CreatePackage(*(TEXT("/Temp/URLabMeshNativeParity/") + BlueprintName));
	UBlueprint* ImportedBlueprint = nullptr;
	FString ImportError;
	bool bCancelled = false;
	FMjImportSettings Settings;
	Settings.bAllowPrompts = false;
	const bool bImported = UMujocoImportFactory::ImportModel(SourceXmlPath, Settings,
		[&]() -> UBlueprint*
		{
			return FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
				*BlueprintName, BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
		}, ImportedBlueprint, ImportError, bCancelled);
	if (!TestTrue(TEXT("actual import factory and PrepareMeshes create editable authored Blueprint"), bImported))
	{
		AddError(ImportError);
		return false;
	}
	if (!TestNotNull(TEXT("factory returned imported Blueprint"), ImportedBlueprint)) return false;

	if (!TestTrue(TEXT("hide all original XML and OBJ source inputs"), IFileManager::Get().Move(*SourceDir, *HiddenSourceDir)))
		return false;
	Cleanup.bHidden = true;
	TestFalse(TEXT("original mesh source is unavailable before authored export"), FPaths::FileExists(SourceMeshPath));

	FMjSourceBundle Bundle;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!TestTrue(TEXT("current imported UE Blueprint prepares a native compiler bundle"),
		MjBuildSourceBundle(FSpecRef::OverBlueprint(*ImportedBlueprint), Bundle, Diagnostics)))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics) AddError(Diagnostic.ToString());
		return false;
	}
	TestFalse(TEXT("generated XML does not retain source path"), Bundle.Xml.Contains(SourceDir));
	FMjNativeModelGuard GeneratedModel;
	if (!TestTrue(TEXT("generated Blueprint bundle installs through native compiler"),
		CompileGeneratedBundle(*this, Bundle, GeneratedModel))) return false;

	const int32 SourceGeom = mj_name2id(SourceModel.Model, mjOBJ_GEOM, "mesh_geom");
	const int32 GeneratedGeom = mj_name2id(GeneratedModel.Model, mjOBJ_GEOM, "mesh_geom");
	TestTrue(TEXT("authored mesh geom exists in both native models"), SourceGeom >= 0 && GeneratedGeom >= 0);
	if (SourceGeom < 0 || GeneratedGeom < 0) return false;
	mjData* SourceData = mj_makeData(SourceModel.Model);
	mjData* GeneratedData = mj_makeData(GeneratedModel.Model);
	if (!TestNotNull(TEXT("original model data"), SourceData) ||
		!TestNotNull(TEXT("generated model data"), GeneratedData))
	{
		if (SourceData != nullptr) mj_deleteData(SourceData);
		if (GeneratedData != nullptr) mj_deleteData(GeneratedData);
		return false;
	}
	mj_forward(SourceModel.Model, SourceData);
	mj_forward(GeneratedModel.Model, GeneratedData);
	TArray<FVector> SourceCorners = WorldTriangleCorners(*SourceModel.Model, *SourceData, SourceGeom);
	TArray<FVector> GeneratedCorners = WorldTriangleCorners(*GeneratedModel.Model, *GeneratedData, GeneratedGeom);
	TestEqual(TEXT("expanded triangle corner counts match"), GeneratedCorners.Num(), SourceCorners.Num());
	SortCorners(SourceCorners);
	SortCorners(GeneratedCorners);
	if (GeneratedCorners.Num() == SourceCorners.Num())
	{
		for (int32 Index = 0; Index < SourceCorners.Num(); ++Index)
		{
			TestTrue(FString::Printf(TEXT("native world-space mesh corner %d matches"), Index),
				SourceCorners[Index].Equals(GeneratedCorners[Index], 1e-5));
		}
	}
	const int32 SourceBody = SourceModel.Model->geom_bodyid[SourceGeom];
	const int32 GeneratedBody = GeneratedModel.Model->geom_bodyid[GeneratedGeom];
	TestTrue(TEXT("body mass is preserved by mesh round trip"), FMath::IsNearlyEqual(
		SourceModel.Model->body_mass[SourceBody], GeneratedModel.Model->body_mass[GeneratedBody], 1e-6));
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		TestTrue(TEXT("body inertia is preserved by mesh round trip"), FMath::IsNearlyEqual(
			SourceModel.Model->body_inertia[3 * SourceBody + Axis],
			GeneratedModel.Model->body_inertia[3 * GeneratedBody + Axis], 1e-6));
	}
	mj_deleteData(SourceData);
	mj_deleteData(GeneratedData);
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
