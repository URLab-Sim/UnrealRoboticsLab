// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/StaticMesh.h"
#include "HAL/FileManager.h"
#include "MeshDescription.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "StaticMeshAttributes.h"

#include "MuJoCo/Elements/MjMesh.h"
#include "MujocoMeshImporter.h"

namespace
{
bool WriteTriangleObj(const FString& Path, bool bWithNormals)
{
	FString Obj = TEXT("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\n");
	if (bWithNormals)
	{
		// Deliberately orthogonal to this triangle's geometric normal (+Z).
		Obj += TEXT("vn 1 0 0\n");
		Obj += TEXT("f 1/1/1 2/2/1 3/3/1\n");
	}
	else
	{
		Obj += TEXT("f 1/1 2/2 3/3\n");
	}
	return FFileHelper::SaveStringToFile(Obj, *Path);
}

int32 CountObjLines(const FString& Path, const TCHAR* Prefix)
{
	FString Obj;
	if (!FFileHelper::LoadFileToString(Obj, *Path)) return INDEX_NONE;
	TArray<FString> Lines;
	Obj.ParseIntoArrayLines(Lines);
	int32 Count = 0;
	for (const FString& Line : Lines)
	{
		Count += Line.StartsWith(Prefix) ? 1 : 0;
	}
	return Count;
}

bool HasMeshDescriptionNormalAlongX(UStaticMesh* Mesh)
{
	if (!Mesh || !Mesh->GetMeshDescription(0)) return false;
	const FMeshDescription* Description = Mesh->GetMeshDescription(0);
	FStaticMeshConstAttributes Attributes(*Description);
	const TVertexInstanceAttributesConstRef<FVector3f> Normals = Attributes.GetVertexInstanceNormals();
	for (const FVertexInstanceID InstanceId : Description->VertexInstances().GetElementIDs())
	{
		const FVector3f Normal = Normals[InstanceId].GetSafeNormal();
		if (!Normal.IsNearlyZero())
		{
			if (FMath::Abs(Normal.X) > 0.9f) return true;
		}
	}
	return false;
}

} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportNormalsTest,
	"URLab.Import.ObjNormalsPreserveExplicitAndEditedMeshData",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportNormalsTest::RunTest(const FString& Parameters)
{
	const FString Scratch = FPaths::ProjectSavedDir() / TEXT("URLabTest/ImportNormals") /
		FGuid::NewGuid().ToString(EGuidFormats::Digits);
	IFileManager::Get().MakeDirectory(*Scratch, true);
	struct FCleanup
	{
		FString Path;
		~FCleanup() { IFileManager::Get().DeleteDirectory(*Path, false, true); }
	} Cleanup{Scratch};

	const FString ExplicitObj = Scratch / TEXT("explicit.obj");
	const FString MissingObj = Scratch / TEXT("missing.obj");
	if (!TestTrue(TEXT("write explicit-normal OBJ"), WriteTriangleObj(ExplicitObj, true)) ||
		!TestTrue(TEXT("write no-normal OBJ"), WriteTriangleObj(MissingObj, false))) return false;

	const FString Destination = FString::Printf(TEXT("/Game/URLabImportNormalTests/%s"),
		*FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UStaticMesh* ExplicitMesh = urlab::editor::ImportMeshAsset(ExplicitObj, Destination, TEXT("explicit"));
	if (!TestNotNull(TEXT("real importer imports explicit-normal OBJ"), ExplicitMesh)) return false;

	TestTrue(TEXT("imported MeshDescription retains explicit source normal direction"),
		HasMeshDescriptionNormalAlongX(ExplicitMesh));
	const FString ExplicitExportDir = Scratch / TEXT("explicit_export");
	IFileManager::Get().MakeDirectory(*(ExplicitExportDir / TEXT("urlab_assets")), true);
	UMjMesh* ExplicitElement = NewObject<UMjMesh>(GetTransientPackage(), NAME_None, RF_Transient);
	ExplicitElement->MjName = TEXT("explicit");
	ExplicitElement->MeshAsset = ExplicitMesh;
	if (TestTrue(TEXT("export explicitly normalled imported mesh"), ExplicitElement->DumpAssetToFile(ExplicitExportDir)))
	{
		TestTrue(TEXT("re-export includes source normal stream"),
			CountObjLines(ExplicitExportDir / TEXT("urlab_assets/explicit.obj"), TEXT("vn ")) > 0);
	}

	UStaticMesh* MissingMesh = urlab::editor::ImportMeshAsset(MissingObj, Destination, TEXT("missing"));
	if (!TestNotNull(TEXT("real importer imports OBJ without normals"), MissingMesh)) return false;
	const FString MissingExportDir = Scratch / TEXT("missing_export");
	IFileManager::Get().MakeDirectory(*(MissingExportDir / TEXT("urlab_assets")), true);
	UMjMesh* MissingElement = NewObject<UMjMesh>(GetTransientPackage(), NAME_None, RF_Transient);
	MissingElement->MjName = TEXT("missing");
	MissingElement->MeshAsset = MissingMesh;
	if (TestTrue(TEXT("export imported OBJ without normals"), MissingElement->DumpAssetToFile(MissingExportDir)))
	{
		TestEqual(TEXT("missing source normals remain absent in export"),
			CountObjLines(MissingExportDir / TEXT("urlab_assets/missing.obj"), TEXT("vn ")), 0);
	}

	// Editing the imported mesh description should make that authored value the
	// value exported on the next package conversion, even when import had no vn.
	FMeshDescription* EditedDescription = MissingMesh->GetMeshDescription(0);
	if (!TestNotNull(TEXT("missing-normal import has editable MeshDescription"), EditedDescription)) return false;
	FStaticMeshAttributes EditedAttributes(*EditedDescription);
	TVertexInstanceAttributesRef<FVector3f> EditedNormals = EditedAttributes.GetVertexInstanceNormals();
	for (const FVertexInstanceID InstanceId : EditedDescription->VertexInstances().GetElementIDs())
	{
		EditedNormals[InstanceId] = FVector3f(1.0f, 0.0f, 0.0f);
	}
	MissingMesh->CommitMeshDescription(0);
	const FString EditedExportDir = Scratch / TEXT("edited_export");
	IFileManager::Get().MakeDirectory(*(EditedExportDir / TEXT("urlab_assets")), true);
	if (TestTrue(TEXT("export edited imported mesh"), MissingElement->DumpAssetToFile(EditedExportDir)))
	{
		TestEqual(TEXT("edited mesh exports one explicit normal per corner"),
			CountObjLines(EditedExportDir / TEXT("urlab_assets/missing.obj"), TEXT("vn ")), 3);
	}
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
