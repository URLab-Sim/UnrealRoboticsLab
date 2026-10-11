// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// A portable export must use the authored UE asset when its content changes,
// even when its object path and the MJCF file attribute stay the same.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "HAL/FileManager.h"
#include "Interfaces/IPluginManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "MeshDescription.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "StaticMeshAttributes.h"
#include "UObject/Package.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MujocoGenerationAction.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{
template <typename T>
T* FirstTemplate(UBlueprint& Blueprint)
{
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
		if (T* Element = Node != nullptr ? Cast<T>(Node->ComponentTemplate) : nullptr)
			return Element;
	return nullptr;
}

struct FOwnedSourceDir
{
	FString Path;
	FString Hidden;
	FString ImportedAssets;
	bool bHidden = false;
	~FOwnedSourceDir()
	{
		if (bHidden) IFileManager::Get().Move(*Path, *Hidden);
		IFileManager::Get().DeleteDirectory(*Path, false, true);
		IFileManager::Get().DeleteDirectory(*Hidden, false, true);
		if (!ImportedAssets.IsEmpty())
			IFileManager::Get().DeleteDirectory(*ImportedAssets, false, true);
	}
};

bool ReadBytes(const FString& Path, TArray<uint8>& Out)
{
	return FFileHelper::LoadFileToArray(Out, *Path);
}

struct FNativeModelGuard
{
	mjModel* Model = nullptr;
	~FNativeModelGuard() { if (Model != nullptr) mj_deleteModel(Model); }
};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEditedAssetBundleTest,
	"URLab.Export.EditedMeshAndTexturePortableBundle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjEditedAssetBundleTest::RunTest(const FString& Parameters)
{
	const FString Unique = FGuid::NewGuid().ToString(EGuidFormats::Digits);
	const FString Scratch = FPaths::ProjectSavedDir() / TEXT("URLabTest/EditedBundle") / Unique;
	FOwnedSourceDir Source{Scratch / TEXT("source"), Scratch / TEXT("source_hidden")};
	const FString BundleDir = Scratch / TEXT("bundle");
	IFileManager::Get().MakeDirectory(*Source.Path, true);
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!TestTrue(TEXT("URLab plugin is mounted"), Plugin.IsValid())) return false;
	const FString FixtureDir = Plugin->GetContentDir() / TEXT("TestData/parity");
	const FString MeshSource = Source.Path / TEXT("part.obj");
	const FString TextureSource = Source.Path / TEXT("paint.png");
	const FString XmlPath = Source.Path / (TEXT("MjEdited_") + Unique + TEXT(".xml"));
	Source.ImportedAssets = FPaths::ProjectContentDir() / TEXT("MuJoCoImports") /
		(FPaths::GetBaseFilename(XmlPath) + TEXT("_Assets"));
	if (!TestTrue(TEXT("copy mesh fixture"), IFileManager::Get().Copy(*MeshSource,
		*(FixtureDir / TEXT("meshA/base.obj"))) == 0) ||
		!TestTrue(TEXT("copy texture fixture"), IFileManager::Get().Copy(*TextureSource,
			*(FixtureDir / TEXT("checker.png"))) == 0)) return false;

	const FString Xml = TEXT("<mujoco model='edited_assets'><asset>")
		TEXT("<mesh name='part' file='part.obj'/>")
		TEXT("<texture name='paint' type='2d' file='paint.png'/>")
		TEXT("<material name='mat' texture='paint'/>")
		TEXT("</asset><worldbody><body name='body'><freejoint/>")
		TEXT("<geom name='shape' type='mesh' mesh='part' material='mat'/>")
		TEXT("</body></worldbody></mujoco>");
	if (!TestTrue(TEXT("write source model"), FFileHelper::SaveStringToFile(Xml, *XmlPath))) return false;
	TArray<uint8> OriginalMesh, OriginalTexture;
	if (!TestTrue(TEXT("read source mesh"), ReadBytes(MeshSource, OriginalMesh)) ||
		!TestTrue(TEXT("read source texture"), ReadBytes(TextureSource, OriginalTexture))) return false;

	UPackage* Package = CreatePackage(*(TEXT("/Temp/MjEditedBundle_") + Unique));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
		FName(*(TEXT("EditedBundle_") + Unique)), BPTYPE_Normal, UBlueprint::StaticClass(),
		UBlueprintGeneratedClass::StaticClass());
	if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) return false;
	UMujocoGenerationAction* Action = NewObject<UMujocoGenerationAction>();
	if (!TestTrue(TEXT("real editor asset import"), Action->GenerateFromXml(Blueprint, Xml, XmlPath))) return false;
	UMjMesh* Mesh = FirstTemplate<UMjMesh>(*Blueprint);
	UMjTexture* Texture = FirstTemplate<UMjTexture>(*Blueprint);
	if (!TestNotNull(TEXT("imported mesh element"), Mesh) ||
		!TestNotNull(TEXT("imported texture element"), Texture) ||
		!TestNotNull(TEXT("imported mesh asset"), Mesh->MeshAsset.Get()) ||
		!TestNotNull(TEXT("imported texture asset"), Texture->TextureAsset.Get())) return false;
	// A selected StaticMesh supersedes old inline data in the compile/export
	// view, while those authored values themselves remain untouched.
	Mesh->Vertex = TArray<float>({0, 0, 0, 1, 0, 0, 0, 1, 0});
	Mesh->Face = TArray<int32>({0, 1, 2});
	const FString AuthoredMeshFile = Mesh->GetFile();
	const FString AuthoredTextureFile = Texture->GetFile();
	FMeshDescription* Description = Mesh->MeshAsset->GetMeshDescription(0);
	if (!TestNotNull(TEXT("mesh has editable description"), Description)) return false;
	FStaticMeshAttributes Attributes(*Description);
	TVertexAttributesRef<FVector3f> Positions = Attributes.GetVertexPositions();
	bool bMovedVertex = false;
	for (const FVertexID Vertex : Description->Vertices().GetElementIDs())
	{
		Positions[Vertex] += FVector3f(5.0f, 0.0f, 0.0f);
		bMovedVertex = true;
		break;
	}
	if (!TestTrue(TEXT("changed a real mesh vertex"), bMovedVertex)) return false;
	Mesh->MeshAsset->CommitMeshDescription(0);
	Mesh->MeshAsset->Build();

	FImage Pixels;
	if (!TestTrue(TEXT("texture has source pixels"), Texture->TextureAsset->Source.GetMipImage(Pixels, 0)) ||
		!TestTrue(TEXT("texture has pixels to edit"), Pixels.RawData.Num() >= 4)) return false;
	Pixels.RawData[0] ^= 0xff;
	Texture->TextureAsset->Source.Init(Pixels.SizeX, Pixels.SizeY, 1, 1,
		Texture->TextureAsset->Source.GetFormat(), Pixels.RawData.GetData());
	Texture->TextureAsset->UpdateResource();

	FMjSourceBundle Bundle;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!TestTrue(TEXT("edited assets build a bundle"),
		MjBuildSourceBundle(FSpecRef::OverBlueprint(*Blueprint), Bundle, Diagnostics))) return false;
	TestTrue(TEXT("portable MJCF uses the selected StaticMesh file"), Bundle.Xml.Contains(TEXT("file=")));
	TestFalse(TEXT("portable MJCF omits superseded inline vertices"), Bundle.Xml.Contains(TEXT("vertex=")));
	TestFalse(TEXT("portable MJCF omits superseded inline faces"), Bundle.Xml.Contains(TEXT("face=")));
	TestEqual(TEXT("inline vertex authoring data remains unchanged"), Mesh->Vertex.GetValue().Num(), 9);
	TestEqual(TEXT("inline face authoring data remains unchanged"), Mesh->Face.GetValue().Num(), 3);
	int32 EditedMeshCount = 0, EditedTextureCount = 0;
	for (const FMjGeneratedBundleResource& Resource : Bundle.Resources)
	{
		if (Resource.Key.StartsWith(TEXT("__urlab_mesh_")) && Resource.Key.EndsWith(TEXT(".obj")))
		{
			++EditedMeshCount;
			TestTrue(TEXT("edited mesh bytes differ from source"), Resource.Bytes != OriginalMesh);
		}
		if (Resource.Key.StartsWith(TEXT("__urlab_texture_")) && Resource.Key.EndsWith(TEXT(".png")))
		{
			++EditedTextureCount;
			TestTrue(TEXT("edited texture bytes differ from source"), Resource.Bytes != OriginalTexture);
		}
	}
	TestEqual(TEXT("one edited mesh resource"), EditedMeshCount, 1);
	TestEqual(TEXT("one edited texture resource"), EditedTextureCount, 1);
	TestEqual(TEXT("source mesh attribute stays authored"), Mesh->GetFile(), AuthoredMeshFile);
	TestEqual(TEXT("source texture attribute stays authored"), Texture->GetFile(), AuthoredTextureFile);
	TArray<uint8> SourceMeshAfter, SourceTextureAfter;
	TestTrue(TEXT("read unchanged source mesh"), ReadBytes(MeshSource, SourceMeshAfter));
	TestTrue(TEXT("read unchanged source texture"), ReadBytes(TextureSource, SourceTextureAfter));
	TestTrue(TEXT("source OBJ unchanged"), SourceMeshAfter == OriginalMesh);
	TestTrue(TEXT("source PNG unchanged"), SourceTextureAfter == OriginalTexture);
	if (EditedMeshCount != 1 || EditedTextureCount != 1) return false;
	char NativeError[1024] = {};
	FNativeModelGuard SourceModel;
	SourceModel.Model = mj_loadXML(TCHAR_TO_UTF8(*XmlPath), nullptr, NativeError, sizeof(NativeError));
	if (!TestNotNull(TEXT("source XML compiles before hiding"), SourceModel.Model))
	{
		AddError(UTF8_TO_TCHAR(NativeError));
		return false;
	}
	if (!TestTrue(TEXT("hide original source directory"),
		IFileManager::Get().Move(*Source.Hidden, *Source.Path))) return false;
	Source.bHidden = true;
	// Regenerated compiler bytes must serve the edited-asset path after sources disappear.
	Bundle = FMjSourceBundle();
	Diagnostics.Reset();
	if (!TestTrue(TEXT("edited bundle builds after source is hidden"),
		MjBuildSourceBundle(FSpecRef::OverBlueprint(*Blueprint), Bundle, Diagnostics))) return false;
	FString PortableXml, Error;
	if (!TestTrue(TEXT("write portable bundle"), Bundle.WriteToDirectory(BundleDir, PortableXml, Error)))
	{
		AddError(Error);
		return false;
	}
	NativeError[0] = 0;
	mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*PortableXml), nullptr, NativeError, sizeof(NativeError));
	if (!TestNotNull(TEXT("portable edited XML compiles with source hidden"), Model))
		AddError(UTF8_TO_TCHAR(NativeError));
	if (Model != nullptr)
	{
		const bool bMeshChanged = Model->nmeshvert != SourceModel.Model->nmeshvert ||
			FMemory::Memcmp(Model->mesh_vert, SourceModel.Model->mesh_vert,
				Model->nmeshvert * 3 * sizeof(float)) != 0;
		const bool bTextureChanged = Model->ntexdata != SourceModel.Model->ntexdata ||
			FMemory::Memcmp(Model->tex_data, SourceModel.Model->tex_data, Model->ntexdata) != 0;
		TestTrue(TEXT("compiled mesh vertices reflect the UE edit"), bMeshChanged);
		TestTrue(TEXT("compiled texture pixels reflect the UE edit"), bTextureChanged);
	}
	if (Model != nullptr) mj_deleteModel(Model);
	return Model != nullptr;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompMeshAssetParityTest,
	"URLab.Export.FlexcompMeshAssetNativeParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompMeshAssetParityTest::RunTest(const FString& Parameters)
{
	const FString Unique = FGuid::NewGuid().ToString(EGuidFormats::Digits);
	const FString Scratch = FPaths::ProjectSavedDir() / TEXT("URLabTest/FlexcompAsset") / Unique;
	FOwnedSourceDir Source{Scratch / TEXT("source"), Scratch / TEXT("source_hidden")};
	const FString BundleDir = Scratch / TEXT("bundle");
	IFileManager::Get().MakeDirectory(*Source.Path, true);
	const FString ObjPath = Source.Path / TEXT("surface.obj");
	const FString ModelName = FString(TEXT("MjFlexcomp_")) + Unique;
	const FString XmlPath = Source.Path / (ModelName + TEXT(".xml"));
	Source.ImportedAssets = FPaths::ProjectContentDir() / TEXT("MuJoCoImports") / (ModelName + TEXT("_Assets"));
	const FString Obj = TEXT(
		"v 0 0 0\n"
		"v 1.0 0.0 0.0\n"
		"v 1.0 1.0 0.0\n"
		"v 0.0 1.0 0.0\n"
		"v 0.4 0.3 0.9\n"
		"vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
		"vt 0.1 0.1\nvt 0.9 0.2\nvt 0.8 0.9\n"
		"vt 0.2 0.1\nvt 0.7 0.2\nvt 0.5 0.8\n"
		"vt 0.1 0.3\nvt 0.8 0.4\nvt 0.3 0.9\n"
		"vt 0.6 0.1\nvt 0.7 0.7\nvt 0.2 0.8\n"
		"f 1/1 2/2 3/3 4/4\n"
		"f 1/5 5/6 2/7\n"
		"f 2/8 5/9 3/10\n"
		"f 3/11 5/12 4/13\n"
		"f 4/14 5/15 1/16\n");
	const FString Xml = TEXT(
		"<mujoco model='flexcomp_mesh_asset'>"
		"<worldbody><body name='parent' pos='0.3 -0.2 0.4' quat='0.9238795325 0.1022764494 -0.2045528988 0.3068293482'>"
		"<flexcomp name='soft' type='mesh' file='surface.obj' dim='2' pos='0.2 -0.1 0.3' euler='90 0 0' scale='1.2 0.8 1.1' radius='0.001' mass='0.4' dof='trilinear'>"
		"<pin id='0'/><contact selfcollide='none' internal='false'/>"
		"</flexcomp></body></worldbody></mujoco>");
	if (!TestTrue(TEXT("write UV-seamed source OBJ"), FFileHelper::SaveStringToFile(Obj, *ObjPath)) ||
		!TestTrue(TEXT("write source flexcomp model"), FFileHelper::SaveStringToFile(Xml, *XmlPath))) return false;

	TArray<uint8> OriginalObj;
	if (!TestTrue(TEXT("read original flex mesh bytes"), ReadBytes(ObjPath, OriginalObj))) return false;
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + ModelName));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AMjArticulation::StaticClass(), Package,
		FName(*ModelName), BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (!TestNotNull(TEXT("scratch flexcomp Blueprint"), Blueprint)) return false;
	UMujocoGenerationAction* Action = NewObject<UMujocoGenerationAction>();
	if (!TestTrue(TEXT("import source model through the editor path"),
		Action->GenerateFromXml(Blueprint, Xml, XmlPath, {}, XmlPath))) return false;
	UMjFlexcomp* Flexcomp = FirstTemplate<UMjFlexcomp>(*Blueprint);
	if (!TestNotNull(TEXT("imported flexcomp element"), Flexcomp) ||
		!TestNotNull(TEXT("imported editable flexcomp UStaticMesh"), Flexcomp->MeshAsset.Get())) return false;
	TestTrue(TEXT("file attribute now names its generated mesh resource"),
		Flexcomp->GetFile().StartsWith(TEXT("__urlab_flexcomp_")) && Flexcomp->GetFile().EndsWith(TEXT(".obj")));
	TestTrue(TEXT("generated flex resource row exists"), Flexcomp->PreparedAssetResources.ContainsByPredicate(
		[&](const FMjPreparedAssetResource& Resource)
		{
			return Resource.Attribute == TEXT("file") && Resource.File == Flexcomp->GetFile()
				&& Resource.VfsName == Resource.File && !Resource.Bytes.IsEmpty()
				&& Resource.AssetPath == FSoftObjectPath(Flexcomp->MeshAsset.Get());
		}));
	TestTrue(TEXT("import source path was cleared"), Flexcomp->SourceFile.IsEmpty());

	char NativeError[2048] = {};
	FNativeModelGuard SourceModel;
	SourceModel.Model = mj_loadXML(TCHAR_TO_UTF8(*XmlPath), nullptr, NativeError, sizeof(NativeError));
	if (!TestNotNull(TEXT("native source flexcomp compiles"), SourceModel.Model))
	{
		AddError(UTF8_TO_TCHAR(NativeError));
		return false;
	}
	mjData* SourceData = mj_makeData(SourceModel.Model);
	if (!TestNotNull(TEXT("native source qpos0 data"), SourceData)) return false;
	mj_forward(SourceModel.Model, SourceData);

	FString Error;
	// Hide source XML/OBJ before requesting any authored export. All compiler
	// bytes must now come from the imported UStaticMesh row.
	if (!TestTrue(TEXT("hide original source directory"), IFileManager::Get().Move(*Source.Hidden, *Source.Path)))
	{
		mj_deleteData(SourceData);
		return false;
	}
	Source.bHidden = true;
	FString BuiltXml;
	FMjSourceBundle Bundle;
	TArray<FMjSpecDiagnostic> Diagnostics;
	if (!TestTrue(TEXT("authored flexcomp builds source bundle after source is hidden"),
		MjBuildSourceBundle(FSpecRef::OverBlueprint(*Blueprint), Bundle, Diagnostics)))
	{
		for (const FMjSpecDiagnostic& Diagnostic : Diagnostics) AddError(Diagnostic.ToString());
		mj_deleteData(SourceData);
		return false;
	}
	if (!TestTrue(TEXT("write generated flex resource bundle"), Bundle.WriteToDirectory(BundleDir, BuiltXml, Error)))
	{
		AddError(Error);
		mj_deleteData(SourceData);
		return false;
	}
	FNativeModelGuard AuthoredModel;
	AuthoredModel.Model = mj_loadXML(TCHAR_TO_UTF8(*BuiltXml), nullptr, NativeError, sizeof(NativeError));
	if (!TestNotNull(TEXT("authored model compiles with original files hidden"), AuthoredModel.Model))
	{
		AddError(UTF8_TO_TCHAR(NativeError));
		mj_deleteData(SourceData);
		return false;
	}
	mjData* AuthoredData = mj_makeData(AuthoredModel.Model);
	if (!TestNotNull(TEXT("authored qpos0 data"), AuthoredData))
	{
		mj_deleteData(SourceData);
		return false;
	}
	mj_forward(AuthoredModel.Model, AuthoredData);

	const mjModel& A = *SourceModel.Model;
	const mjModel& B = *AuthoredModel.Model;
	auto SameNumbers = [](const auto* Left, const auto* Right, int64 Count, double Tolerance = 1.e-6)
	{
		if ((Left == nullptr || Right == nullptr) && Count > 0) return false;
		for (int64 Index = 0; Index < Count; ++Index)
		{
			if (!FMath::IsFinite(static_cast<double>(Left[Index]))
				|| !FMath::IsFinite(static_cast<double>(Right[Index]))
				|| FMath::Abs(static_cast<double>(Left[Index]) - static_cast<double>(Right[Index])) > Tolerance)
			{
				return false;
			}
		}
		return true;
	};
	TestEqual(TEXT("same generated bodies"), (int32)B.nbody, (int32)A.nbody);
	TestEqual(TEXT("same generated joints"), (int32)B.njnt, (int32)A.njnt);
	TestEqual(TEXT("same flex vertices"), (int32)B.nflexvert, (int32)A.nflexvert);
	TestEqual(TEXT("same flex elements"), (int32)B.nflexelem, (int32)A.nflexelem);
	TestTrue(TEXT("both models retain flex UV data"), A.nflextexcoord > 0 && B.nflextexcoord > 0);
	TestEqual(TEXT("same flex element data"), (int32)B.nflexelemdata, (int32)A.nflexelemdata);
	if (A.nbody == B.nbody && A.nflexvert == B.nflexvert && A.nflexelemdata == B.nflexelemdata)
	{
		TestTrue(TEXT("same body positions under transformed parent"),
			SameNumbers(A.body_pos, B.body_pos, A.nbody * 3));
		TestTrue(TEXT("same body orientations"),
			SameNumbers(A.body_quat, B.body_quat, A.nbody * 4));
		TestTrue(TEXT("same body masses"),
			SameNumbers(A.body_mass, B.body_mass, A.nbody));
		TestTrue(TEXT("same flex vertex ownership and pinned identity"),
			FMemory::Memcmp(A.flex_vertbodyid, B.flex_vertbodyid, A.nflexvert * sizeof(int)) == 0);
		TestTrue(TEXT("same flex element topology"),
			FMemory::Memcmp(A.flex_elem, B.flex_elem, A.nflexelemdata * sizeof(int)) == 0);
		const int32 FlexA = mj_name2id(&A, mjOBJ_FLEX, "soft");
		const int32 FlexB = mj_name2id(&B, mjOBJ_FLEX, "soft");
		bool bSameExpandedUvs = FlexA >= 0 && FlexB >= 0 && A.nflexelemdata == B.nflexelemdata
			&& A.flex_texcoordadr[FlexA] >= 0 && B.flex_texcoordadr[FlexB] >= 0;
		if (bSameExpandedUvs)
		{
			const int32 AddressA = A.flex_elemdataadr[FlexA];
			const int32 AddressB = B.flex_elemdataadr[FlexB];
			for (int32 Corner = 0; Corner < A.nflexelemdata; ++Corner)
			{
				const int32 TexcoordA = A.flex_texcoordadr[FlexA] + A.flex_elemtexcoord[AddressA + Corner];
				const int32 TexcoordB = B.flex_texcoordadr[FlexB] + B.flex_elemtexcoord[AddressB + Corner];
				if (TexcoordA < A.flex_texcoordadr[FlexA] || TexcoordA >= A.flex_texcoordadr[FlexA] + A.nflextexcoord
					|| TexcoordB < B.flex_texcoordadr[FlexB] || TexcoordB >= B.flex_texcoordadr[FlexB] + B.nflextexcoord
					|| !SameNumbers(A.flex_texcoord + 2 * TexcoordA, B.flex_texcoord + 2 * TexcoordB, 2))
				{
					bSameExpandedUvs = false;
					break;
				}
			}
		}
		TestTrue(TEXT("same expanded per-element-corner UV values and seams"), bSameExpandedUvs);
		TestTrue(TEXT("same rest positions"),
			SameNumbers(A.flex_vert0, B.flex_vert0, A.nflexvert * 3));
		TestTrue(TEXT("same qpos0 world vertex positions"),
			SameNumbers(SourceData->flexvert_xpos, AuthoredData->flexvert_xpos, A.nflexvert * 3));
	}
	if (AuthoredData != nullptr) mj_deleteData(AuthoredData);
	if (SourceData != nullptr) mj_deleteData(SourceData);
	TestFalse(TEXT("original input path is gone"), FPaths::FileExists(ObjPath));
	TestTrue(TEXT("original source bytes were captured"), !OriginalObj.IsEmpty());
	return !HasAnyErrors();
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
