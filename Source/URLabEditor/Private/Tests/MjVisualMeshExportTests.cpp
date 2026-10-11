// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// A user-selected static mesh exports its visual LOD0 attributes even when
// there is no Chaos triangle mesh. The exported OBJ must retain the corner
// data MuJoCo compiles and the renderer maps back into Unreal coordinates.

#include "CoreMinimal.h"
#include "Engine/Texture2D.h"
#include "Misc/AutomationTest.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/StaticMesh.h"
#include "HAL/FileManager.h"
#include "MeshDescription.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshAttributes.h"
#include "TextureResource.h"

#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Render/MjRenderResourceShim.h"
#include "Render/RenderAssetBuilder.h"
#include "Render/RenderSceneResources.h"
#include "Utils/CoordinateConversion.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjVisualMeshExportTest,
	"URLab.Export.SelectedVisualMeshPreservesNativeAttributesWithoutCollision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjVisualMeshExportTest::RunTest(const FString& Parameters)
{
	const FString Scratch = FPaths::ProjectSavedDir() / TEXT("URLabTest/VisualMeshExport") /
		FGuid::NewGuid().ToString(EGuidFormats::Digits);
	IFileManager::Get().MakeDirectory(*(Scratch / TEXT("urlab_assets")), true);
	struct FCleanup
	{
		FString Path;
		// Keep this small fixture when a test fails so the emitted OBJ and MJCF
		// can be inspected alongside MuJoCo's compiled mesh arrays.
		bool bPreserve = true;
		~FCleanup() { if (!bPreserve) IFileManager::Get().DeleteDirectory(*Path, false, true); }
	} Cleanup{Scratch};
	AddInfo(FString::Printf(TEXT("Visual mesh diagnostic fixture: %s"), *Scratch));

	UStaticMesh* Asset = NewObject<UStaticMesh>(GetTransientPackage(), NAME_None, RF_Transient);
	Asset->SetNumSourceModels(1);
	FMeshDescription* Description = Asset->CreateMeshDescription(0);
	if (!TestNotNull(TEXT("transient mesh has an editable LOD0 description"), Description)) return false;
	FStaticMeshAttributes Attributes(*Description);
	Attributes.Register();
	Attributes.GetVertexInstanceUVs().SetNumChannels(1);
	TVertexAttributesRef<FVector3f> Positions = Attributes.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> Normals = Attributes.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector2f> UVs = Attributes.GetVertexInstanceUVs();

	const FVector3f SourcePositions[] = {
		FVector3f(0.0f, 0.0f, 0.0f), FVector3f(200.0f, 0.0f, 0.0f),
		FVector3f(0.0f, 100.0f, 0.0f), FVector3f(30.0f, 20.0f, 150.0f)};
	TArray<FVertexID> VertexIds;
	for (const FVector3f& Position : SourcePositions)
	{
		const FVertexID VertexId = Description->CreateVertex();
		Positions[VertexId] = Position;
		VertexIds.Add(VertexId);
	}

	struct FFace
	{
		int32 Vertices[3];
		FVector3f Normals[3];
		FVector2f UVs[3];
	};
	const FFace Faces[] = {
		{{0, 2, 1}, {{0.02f, 0.0f, -1.0f}, {-0.01f, 0.03f, -1.0f}, {0.01f, -0.02f, -1.0f}},
		 {{0.125f, 0.25f}, {0.875f, 0.25f}, {0.125f, 0.75f}}},
		{{0, 1, 3}, {{0.02f, -1.0f, 0.0f}, {-0.01f, -1.0f, 0.03f}, {0.01f, -1.0f, -0.02f}},
		 {{0.2f, 0.3f}, {0.8f, 0.35f}, {0.4f, 0.95f}}},
		{{0, 3, 2}, {{-1.0f, 0.02f, 0.0f}, {-1.0f, -0.01f, 0.03f}, {-1.0f, 0.01f, -0.02f}},
		 {{0.3f, 0.15f}, {0.7f, 0.85f}, {0.15f, 0.65f}}},
		{{1, 2, 3}, {{0.22f, 0.3f, 0.9f}, {0.19f, 0.33f, 0.9f}, {0.21f, 0.28f, 0.94f}},
		 {{0.4f, 0.2f}, {0.9f, 0.6f}, {0.6f, 0.8f}}},
	};
	TArray<FVector> ExpectedSourceNormals;
	TArray<FVector2D> ExpectedUVs;
	const FPolygonGroupID Group = Description->CreatePolygonGroup();
	for (const FFace& Face : Faces)
	{
		TArray<FVertexInstanceID> Instances;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const FVertexInstanceID Instance = Description->CreateVertexInstance(VertexIds[Face.Vertices[Corner]]);
			Normals[Instance] = Face.Normals[Corner];
			UVs.Set(Instance, 0, Face.UVs[Corner]);
			ExpectedSourceNormals.Add(FVector(Face.Normals[Corner]).GetSafeNormal());
			ExpectedUVs.Emplace(Face.UVs[Corner].X, Face.UVs[Corner].Y);
			Instances.Add(Instance);
		}
		Description->CreatePolygon(Group, Instances);
	}
	Asset->CommitMeshDescription(0);

	UBodySetup* BodySetup = Asset->GetBodySetup();
	TestTrue(TEXT("visual asset has no Chaos triangle collision"),
		BodySetup == nullptr || BodySetup->TriMeshGeometries.IsEmpty());

	UMjMesh* Element = NewObject<UMjMesh>(GetTransientPackage(), NAME_None, RF_Transient);
	Element->MjName = TEXT("visual_source");
	Element->MeshAsset = Asset;
	Element->Vertex = TArray<float>{1.0f, 2.0f, 3.0f};
	Element->Normal = TArray<float>{0.0f, 0.0f, 1.0f};
	Element->Texcoord = TArray<float>{0.0f, 0.0f};
	Element->Face = TArray<int32>{0, 1, 2};
	if (!TestTrue(TEXT("selected visual mesh exports without collision"),
		Element->DumpAssetToFile(Scratch))) return false;

	// Four distinct source pixels make a row flip visible at every corner.
	const FColor SourcePixels[4] = {
		FColor(255, 0, 0, 255), FColor(0, 255, 0, 255),
		FColor(0, 0, 255, 255), FColor(255, 255, 0, 255)};
	UTexture2D* TextureAsset = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
	TextureAsset->Source.Init(2, 2, 1, 1, TSF_BGRA8,
		reinterpret_cast<const uint8*>(SourcePixels));
	UMjTexture* TextureElement = NewObject<UMjTexture>(GetTransientPackage(), NAME_None, RF_Transient);
	TextureElement->MjName = TEXT("asymmetric_pixels");
	TextureElement->TextureAsset = TextureAsset;
	if (!TestTrue(TEXT("asymmetric UE texture exports as PNG"),
		TextureElement->DumpAssetToFile(Scratch))) return false;
	TestTrue(TEXT("inline mesh arrays are cleared after selecting the visual asset"),
		!Element->Vertex.IsSet() && !Element->Normal.IsSet() &&
		!Element->Texcoord.IsSet() && !Element->Face.IsSet());

	const FString ObjPath = Scratch / TEXT("urlab_assets/visual_source.obj");
	FString Obj;
	if (!TestTrue(TEXT("visual OBJ was written"), FFileHelper::LoadFileToString(Obj, *ObjPath))) return false;
	TArray<FString> ObjLines;
	Obj.ParseIntoArrayLines(ObjLines);
	for (int32 LineIndex = 0; LineIndex < FMath::Min(ObjLines.Num(), 16); ++LineIndex)
	{
		AddInfo(FString::Printf(TEXT("OBJ[%d] %s"), LineIndex, *ObjLines[LineIndex]));
	}
	TArray<FVector> ObjPositions;
	for (const FString& Line : ObjLines)
	{
		if (Line.StartsWith(TEXT("v ")))
		{
			TArray<FString> VertexTokens;
			Line.ParseIntoArrayWS(VertexTokens);
			if (VertexTokens.Num() == 4)
			{
				ObjPositions.Emplace(FCString::Atof(*VertexTokens[1]),
					FCString::Atof(*VertexTokens[2]), FCString::Atof(*VertexTokens[3]));
			}
		}
	}
	TestEqual(TEXT("OBJ contains every LOD0 visual vertex"), ObjPositions.Num(),
		static_cast<int32>(UE_ARRAY_COUNT(SourcePositions)));
	for (const FVector3f& SourcePosition : SourcePositions)
	{
		double Expected[3];
		URLabCoordinateConversion::UnrealCentimetresPositionToRightHandedMetres(
			FVector(SourcePosition), Expected);
		bool bFound = false;
		for (const FVector& ObjPosition : ObjPositions)
		{
			bFound |= FMath::IsNearlyEqual(ObjPosition.X, Expected[0], 1.e-6) &&
				FMath::IsNearlyEqual(ObjPosition.Y, Expected[1], 1.e-6) &&
				FMath::IsNearlyEqual(ObjPosition.Z, Expected[2], 1.e-6);
		}
		TestTrue(TEXT("OBJ preserves each vertex in metres with reflected Y"), bFound);
	}
	TArray<FString> UVTokens;
	for (const FString& Line : ObjLines)
	{
		if (Line.StartsWith(TEXT("vt ")))
		{
			Line.ParseIntoArrayWS(UVTokens);
			break;
		}
	}
	TestTrue(TEXT("OBJ writes inverse MuJoCo V mapping"), UVTokens.Num() == 3 &&
		FMath::IsNearlyEqual(FCString::Atof(*UVTokens[1]), 0.125f, 1.e-6f) &&
		FMath::IsNearlyEqual(FCString::Atof(*UVTokens[2]), 0.75f, 1.e-6f));
	TestTrue(TEXT("OBJ has per-corner normals"), Obj.Contains(TEXT("\nvn ")));
	TestTrue(TEXT("OBJ face reverses winding with split corner indices"),
		Obj.Contains(TEXT("f 2/3/3 3/2/2 1/1/1")));

	const FString XmlPath = Scratch / TEXT("scene.xml");
	const FString Xml = TEXT("<mujoco><asset><texture name='asymmetric_pixels' type='2d' nchannel='4' file='urlab_assets/asymmetric_pixels.png'/><mesh name='visual_source' file='urlab_assets/visual_source.obj' inertia='shell'/></asset>")
		TEXT("<worldbody><geom name='visual_only' type='mesh' mesh='visual_source' contype='0' conaffinity='0'/></worldbody></mujoco>");
	if (!TestTrue(TEXT("native test model was written"), FFileHelper::SaveStringToFile(Xml, *XmlPath))) return false;
	char Error[2048] = {};
	mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*XmlPath), nullptr, Error, sizeof(Error));
	if (!TestNotNull(TEXT("MuJoCo compiles visual OBJ while collision masks are disabled"), Model))
	{
		AddError(UTF8_TO_TCHAR(Error));
		return false;
	}
	struct FModelCleanup
	{
		mjModel* Model;
		~FModelCleanup() { if (Model) mj_deleteModel(Model); }
	} ModelCleanup{Model};
	const int32 MeshId = mj_name2id(Model, mjOBJ_MESH, "visual_source");
	if (!TestTrue(TEXT("compiled mesh has visual faces and normals"), MeshId >= 0 &&
		Model->mesh_facenum[MeshId] == UE_ARRAY_COUNT(Faces) && Model->nmeshnormal >= ExpectedSourceNormals.Num())) return false;
	{
		const int32 VertexAddress = Model->mesh_vertadr[MeshId];
		const int32 NormalAddress = Model->mesh_normaladr[MeshId];
		const int32 TexcoordAddress = Model->mesh_texcoordadr[MeshId];
		AddInfo(FString::Printf(TEXT("compiled mesh counts: vertices=%d normals=%d texcoords=%d faces=%d; addresses=%d/%d/%d"),
			Model->mesh_vertnum[MeshId], Model->nmeshnormal, Model->nmeshtexcoord,
			Model->mesh_facenum[MeshId], VertexAddress, NormalAddress, TexcoordAddress));
		for (int32 Index = 0; Index < FMath::Min(Model->mesh_vertnum[MeshId], 4); ++Index)
		{
			const float* V = Model->mesh_vert + 3 * (VertexAddress + Index);
			AddInfo(FString::Printf(TEXT("native vertex[%d] %.6f %.6f %.6f"), Index, V[0], V[1], V[2]));
		}
		for (int32 Index = 0; Index < FMath::Min(Model->nmeshnormal - NormalAddress, 4); ++Index)
		{
			const float* N = Model->mesh_normal + 3 * (NormalAddress + Index);
			AddInfo(FString::Printf(TEXT("native normal[%d] %.6f %.6f %.6f"), Index, N[0], N[1], N[2]));
		}
		for (int32 Index = 0; Index < FMath::Min(Model->nmeshtexcoord - TexcoordAddress, 4); ++Index)
		{
			const float* UV = Model->mesh_texcoord + 2 * (TexcoordAddress + Index);
			AddInfo(FString::Printf(TEXT("native uv[%d] %.6f %.6f"), Index, UV[0], UV[1]));
		}
	}
	TestTrue(TEXT("MuJoCo retained all authored UV corners"), Model->nmeshtexcoord >= ExpectedUVs.Num());
	const int32 TextureId = mj_name2id(Model, mjOBJ_TEXTURE, "asymmetric_pixels");
	if (!TestTrue(TEXT("native model contains the exported 2x2 texture"), TextureId >= 0 &&
		Model->tex_width[TextureId] == 2 && Model->tex_height[TextureId] == 2 &&
		Model->tex_nchannel[TextureId] == 4)) return false;
	for (int32 Pixel = 0; Pixel < UE_ARRAY_COUNT(SourcePixels); ++Pixel)
	{
		const uint8* NativePixel = reinterpret_cast<const uint8*>(Model->tex_data +
			Model->tex_adr[TextureId] + 4 * Pixel);
		const FColor& Expected = SourcePixels[Pixel];
		TestTrue(FString::Printf(TEXT("PNG/native texel keeps authored row-major pixel %d"), Pixel),
			NativePixel[0] == Expected.R && NativePixel[1] == Expected.G &&
			NativePixel[2] == Expected.B && NativePixel[3] == Expected.A);
	}
	FRenderSceneResources Resources;
	FString ShimError;
	if (!TestTrue(TEXT("native mesh maps through the preview/render resource adapter"),
		MjRenderResourceShim::Fill(Resources, Model, TEXT("visual-export-test"), ShimError)))
	{
		AddError(ShimError);
		return false;
	}
	if (!TestTrue(TEXT("neutral texture adapter retains the native 2x2 image"),
		Resources.Textures.IsValidIndex(TextureId) && Resources.Textures[TextureId].Width == 2 &&
		Resources.Textures[TextureId].Height == 2 && Resources.Textures[TextureId].Rgba8.Num() == 16)) return false;
	for (int32 Pixel = 0; Pixel < UE_ARRAY_COUNT(SourcePixels); ++Pixel)
	{
		const uint8* NeutralPixel = Resources.Textures[TextureId].Rgba8.GetData() + 4 * Pixel;
		const FColor& Expected = SourcePixels[Pixel];
		TestTrue(FString::Printf(TEXT("neutral adapter keeps row-major pixel %d"), Pixel),
			NeutralPixel[0] == Expected.R && NeutralPixel[1] == Expected.G &&
			NeutralPixel[2] == Expected.B && NeutralPixel[3] == Expected.A);
	}
	const int32 NormalAddress = Model->mesh_normaladr[MeshId];
	for (int32 ExpectedIndex = 0; ExpectedIndex < ExpectedSourceNormals.Num(); ++ExpectedIndex)
	{
		double Expected[3];
		URLabCoordinateConversion::UnrealDirectionToRightHanded(
			ExpectedSourceNormals[ExpectedIndex], Expected);
		bool bFound = false;
		for (int32 NativeIndex = 0; NativeIndex < Model->nmeshnormal - NormalAddress; ++NativeIndex)
		{
			const float* CompiledNormal = Model->mesh_normal + 3 * (NormalAddress + NativeIndex);
			double CompiledNormalDouble[3] = {
				CompiledNormal[0], CompiledNormal[1], CompiledNormal[2]};
			double SourceNormal[3];
		// MuJoCo centers and rotates mesh vertices/normals into principal-axis
		// space at compile time. mesh_quat maps that compiled frame back to the
		// source OBJ frame; mesh_pos restores the removed center for positions.
		mju_rotVecQuat(SourceNormal, CompiledNormalDouble, Model->mesh_quat + 4 * MeshId);
			if (FMath::IsNearlyEqual(SourceNormal[0], Expected[0], 1.e-4) &&
				FMath::IsNearlyEqual(SourceNormal[1], Expected[1], 1.e-4) &&
				FMath::IsNearlyEqual(SourceNormal[2], Expected[2], 1.e-4))
			{
				bFound = true;
				break;
			}
		}
		TestTrue(FString::Printf(TEXT("native mesh retains per-corner normal %d"), ExpectedIndex), bFound);
	}
	const int32 FaceAddress = Model->mesh_faceadr[MeshId];
	for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); ++Face)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const int32 NativeNormalIndex = Model->mesh_facenormal[
				3 * (FaceAddress + Face) + Corner] + NormalAddress;
			const float* NativeNormal = Model->mesh_normal + 3 * NativeNormalIndex;
			double Compiled[3] = {NativeNormal[0], NativeNormal[1], NativeNormal[2]};
			double Source[3];
			mju_rotVecQuat(Source, Compiled, Model->mesh_quat + 4 * MeshId);
			double Expected[3];
			URLabCoordinateConversion::UnrealDirectionToRightHanded(
				ExpectedSourceNormals[3 * Face + (2 - Corner)], Expected);
			TestTrue(FString::Printf(TEXT("native face %d corner %d keeps its authored normal"), Face, Corner),
				FMath::IsNearlyEqual(Source[0], Expected[0], 1.e-4) &&
				FMath::IsNearlyEqual(Source[1], Expected[1], 1.e-4) &&
				FMath::IsNearlyEqual(Source[2], Expected[2], 1.e-4));
		}
	}
	const int32 TexcoordAddress = Model->mesh_texcoordadr[MeshId];
	for (int32 ExpectedIndex = 0; ExpectedIndex < ExpectedUVs.Num(); ++ExpectedIndex)
	{
		bool bFound = false;
		for (int32 NativeIndex = 0; NativeIndex < Model->nmeshtexcoord - TexcoordAddress; ++NativeIndex)
		{
			const float* NativeUV = Model->mesh_texcoord + 2 * (TexcoordAddress + NativeIndex);
			if (FMath::IsNearlyEqual(NativeUV[0], ExpectedUVs[ExpectedIndex].X, 1.e-5) &&
				FMath::IsNearlyEqual(NativeUV[1], ExpectedUVs[ExpectedIndex].Y, 1.e-5))
			{
				bFound = true;
				break;
			}
		}
		TestTrue(FString::Printf(TEXT("native mesh retains raw UV %d"), ExpectedIndex), bFound);
	}
	for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); ++Face)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const int32 NativeUVIndex = Model->mesh_facetexcoord[
				3 * (FaceAddress + Face) + Corner] + TexcoordAddress;
			const float* NativeUV = Model->mesh_texcoord + 2 * NativeUVIndex;
			const FVector2D& Expected = ExpectedUVs[3 * Face + (2 - Corner)];
			TestTrue(FString::Printf(TEXT("native face %d corner %d keeps its authored UV"), Face, Corner),
				FMath::IsNearlyEqual(NativeUV[0], Expected.X, 1.e-5) &&
				FMath::IsNearlyEqual(NativeUV[1], Expected.Y, 1.e-5));
		}
	}

	const FRenderMeshResource& RenderMesh = Resources.Meshes[MeshId];
	for (int32 Index = 0; Index < FMath::Min(RenderMesh.Positions.Num(), 3); ++Index)
	{
		const FVector& P = RenderMesh.Positions[Index];
		const FVector2D& UV = RenderMesh.UVs[Index];
		AddInfo(FString::Printf(TEXT("render corner[%d] pos %.6f %.6f %.6f uv %.6f %.6f"),
			Index, P.X, P.Y, P.Z, UV.X, UV.Y));
	}
	TestEqual(TEXT("render adapter keeps every visual triangle corner"), RenderMesh.Positions.Num(), 12);
	TestEqual(TEXT("render adapter retains MuJoCo UV corner data"), RenderMesh.UVs.Num(), 12);
	for (int32 Face = 0; Face < Model->mesh_facenum[MeshId]; ++Face)
	{
		const int32 RenderFaceAddress = Model->mesh_faceadr[MeshId] + Face;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const int32 UVIndex = Model->mesh_facetexcoord[3 * RenderFaceAddress + Corner] + TexcoordAddress;
			const FVector2D NativeUV(Model->mesh_texcoord[2 * UVIndex], Model->mesh_texcoord[2 * UVIndex + 1]);
			const FVector2D RenderUV = RenderMesh.UVs[3 * Face + Corner];
			TestTrue(FString::Printf(TEXT("render adapter preserves native UV at face %d corner %d"), Face, Corner),
				RenderUV.Equals(NativeUV, 1.e-6));
			TestTrue(FString::Printf(TEXT("render face %d corner %d keeps authored UV"), Face, Corner),
				RenderUV.Equals(ExpectedUVs[3 * Face + (2 - Corner)], 1.e-5));
		}
	}
	for (int32 Corner = 0; Corner < RenderMesh.Positions.Num(); ++Corner)
	{
		const double Compiled[3] = {
			static_cast<double>(RenderMesh.Positions[Corner].X) / 100.0,
			-static_cast<double>(RenderMesh.Positions[Corner].Y) / 100.0,
			static_cast<double>(RenderMesh.Positions[Corner].Z) / 100.0};
		double Source[3];
		mju_rotVecQuat(Source, Compiled, Model->mesh_quat + 4 * MeshId);
		for (int32 Axis = 0; Axis < 3; ++Axis) Source[Axis] += Model->mesh_pos[3 * MeshId + Axis];
		const FVector UESourcePosition = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Source);
		bool bMatchesSourceVertex = false;
		for (const FVector3f& Expected : SourcePositions)
		{
			bMatchesSourceVertex |= UESourcePosition.Equals(FVector(Expected), 1.e-3);
		}
		TestTrue(FString::Printf(TEXT("render corner %d inverse-maps to visual LOD0"), Corner),
			bMatchesSourceVertex);
	}
	// The runtime builder converts RGBA to UE's BGRA byte order without
	// changing rows, then installs that buffer as the top mip.
	URenderAssetBuilder* TextureBuilder = NewObject<URenderAssetBuilder>();
	FString BuilderError;
	if (!TestTrue(TEXT("renderer accepts native mesh and image resources"),
		TextureBuilder->InitResources(MoveTemp(Resources), false, BuilderError)))
	{
		AddError(BuilderError);
		return false;
	}
	UTexture2D* RuntimeTexture = TextureBuilder->GetOrBuildTexture(TextureId, true);
	if (!TestNotNull(TEXT("renderer builds the imported texture"), RuntimeTexture)) return false;
	FTexturePlatformData* PlatformData = RuntimeTexture->GetPlatformData();
	if (!TestNotNull(TEXT("renderer texture has platform data"), PlatformData) ||
		!TestTrue(TEXT("renderer texture has its 2x2 top mip"), PlatformData->Mips.Num() > 0 &&
			PlatformData->Mips[0].SizeX == 2 && PlatformData->Mips[0].SizeY == 2)) return false;
	FTexture2DMipMap& TopMip = PlatformData->Mips[0];
	const uint8* RuntimeBgra = static_cast<const uint8*>(TopMip.BulkData.LockReadOnly());
	if (!TestNotNull(TEXT("renderer top mip pixels are readable"), RuntimeBgra))
	{
		TopMip.BulkData.Unlock();
		return false;
	}
	for (int32 Pixel = 0; Pixel < UE_ARRAY_COUNT(SourcePixels); ++Pixel)
	{
		const FColor& Expected = SourcePixels[Pixel];
		TestTrue(FString::Printf(TEXT("UE runtime texture keeps row-major pixel %d"), Pixel),
			RuntimeBgra[4 * Pixel + 0] == Expected.B &&
			RuntimeBgra[4 * Pixel + 1] == Expected.G &&
			RuntimeBgra[4 * Pixel + 2] == Expected.R &&
			RuntimeBgra[4 * Pixel + 3] == Expected.A);
	}
	TopMip.BulkData.Unlock();
	Cleanup.bPreserve = false;
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
