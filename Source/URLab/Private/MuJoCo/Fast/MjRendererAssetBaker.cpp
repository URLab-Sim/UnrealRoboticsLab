// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjRendererAssetBaker.h"

#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"
#include "TextureResource.h"

#include "MuJoCo/Fast/MjAssetSource.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Entity/MjModelMaterial.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"
#if WITH_EDITOR
#include "Misc/PackageName.h"
#include "PackageTools.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "AssetRegistry/AssetRegistryModule.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// Root of the cached, content-hash-keyed fast-path assets (SM_<id> / T_<id> live
// under <root>/<hash>/). Also the folder a saved fast-path level reloads from.
constexpr const TCHAR* kFastPathAssetRoot = TEXT("/Game/URLabFastPath");

// Crease-split mesh geometry (per-face-corner verts/normals/uvs/tris) from the
// mesh pool the asset source hands out. Pure math over that view -- shared by the
// editor static-mesh baker and the packaged procedural path. File-local free
// function (no actor state).
void BuildMeshArrays(const IMjAssetSource& Source, int32 MeshId, TArray<FVector>& Verts,
	TArray<FVector>& Normals, TArray<FVector2D>& UVs, TArray<int32>& Tris)
{
	Verts.Reset();
	Normals.Reset();
	UVs.Reset();
	Tris.Reset();
	const FMjMeshDataView Mesh = Source.GetMeshData(MeshId);
	if (!Mesh.bValid)
	{
		return;
	}
	const int32 FaceAdr = Mesh.FaceAdr;
	const int32 FaceNum = Mesh.FaceNum;
	const bool bHasUV = Mesh.TexcoordAdr >= 0;
	// Face indices are LOCAL to each mesh (0-based); add the per-mesh base
	// addresses to reach this mesh's slice of the shared vert/normal/uv pools.
	const int32 VertAdr = Mesh.VertAdr;
	const int32 NormalAdr = Mesh.NormalAdr;
	const int32 TexAdr = bHasUV ? Mesh.TexcoordAdr : 0;

	// Expand per face-corner (each corner its own vertex + normal + texcoord). The
	// static-mesh build later welds coincident positions while keeping the crease
	// normals; the procedural path uses the expansion directly.
	Verts.Reserve(FaceNum * 3);
	Normals.Reserve(FaceNum * 3);
	UVs.Reserve(FaceNum * 3);
	Tris.Reserve(FaceNum * 3);

	// Single-sided, one triangle per face. MuJoCo winds faces CCW-from-outside in
	// its right-handed frame; MjPositionToUe negates Y, a reflection that flips
	// the winding sense, so MuJoCo's own order (0,1,2) is the front-facing (outward)
	// order in Unreal. Keep the outward normal as-is. (An earlier reversed order
	// culled the visible faces -- the "see-through" holes -- and duplicating faces
	// to hide that introduced coplanar shadow acne / dark self-shadowing; a single
	// correctly-wound face is both hole-free and correctly lit.)
	// MuJoCo stores ONE averaged normal per vertex (mjCMesh::MakeNormal), so every
	// hard edge shades soft. Recompute per-corner normals with MuJoCo's own crease
	// threshold (acos(0.8)) -- the same split clean_meshes.py does on the import
	// path -- so box/extrusion edges stay sharp while cylinders stay round.
	// Faces meeting at a vertex are clustered greedily: a face joins the first group
	// whose running-mean normal it agrees with (dot >= 0.8), else it starts a group;
	// a corner's normal is its group's averaged normal.
	constexpr double kCreaseDot = 0.8;
	const int32 VertNum = Mesh.VertNum;
	TArray<FVector> FaceGeoN;
	FaceGeoN.SetNumUninitialized(FaceNum);
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Mesh.Faces + 3 * (FaceAdr + F);
		const FVector P0 = URLabAxisConv::MjPositionToUe(Mesh.Verts + 3 * (FV[0] + VertAdr));
		const FVector P1 = URLabAxisConv::MjPositionToUe(Mesh.Verts + 3 * (FV[1] + VertAdr));
		const FVector P2 = URLabAxisConv::MjPositionToUe(Mesh.Verts + 3 * (FV[2] + VertAdr));
		FVector Gn = FVector::CrossProduct(P1 - P0, P2 - P0).GetSafeNormal();
		// Align outward using MuJoCo's per-vertex normal (sign only); fall back to it
		// for a degenerate (zero-area) face.
		const int32* FN = Mesh.FaceNormals + 3 * (FaceAdr + F);
		const int32 Ni0 = FN[0] + NormalAdr;
		const double Nm[3] = {Mesh.Normals[3 * Ni0], Mesh.Normals[3 * Ni0 + 1],
			Mesh.Normals[3 * Ni0 + 2]};
		const FVector Ref = URLabAxisConv::MjDirectionToUe(Nm).GetSafeNormal();
		if (Gn.IsNearlyZero())
		{
			Gn = Ref;
		}
		else if (FVector::DotProduct(Gn, Ref) < 0.0)
		{
			Gn = -Gn;
		}
		FaceGeoN[F] = Gn;
	}
	// Incident faces per local vertex.
	TArray<TArray<int32, TInlineAllocator<8>>> Incident;
	Incident.SetNum(FMath::Max(VertNum, 0));
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Mesh.Faces + 3 * (FaceAdr + F);
		for (int32 K = 0; K < 3; ++K)
		{
			if (FV[K] >= 0 && FV[K] < VertNum)
			{
				Incident[FV[K]].Add(F);
			}
		}
	}
	// Per-corner crease-averaged normal, indexed [3*F + K].
	TArray<FVector> CornerN;
	CornerN.SetNumUninitialized(FaceNum * 3);
	for (int32 V = 0; V < VertNum; ++V)
	{
		const TArray<int32, TInlineAllocator<8>>& Faces = Incident[V];
		if (Faces.Num() == 0)
		{
			continue;
		}
		TArray<FVector, TInlineAllocator<8>> GroupSum; // running summed normal per group
		TArray<int32, TInlineAllocator<16>> FaceGroup; // group index, parallel to Faces
		FaceGroup.SetNumUninitialized(Faces.Num());
		for (int32 i = 0; i < Faces.Num(); ++i)
		{
			const FVector Fn = FaceGeoN[Faces[i]];
			int32 GroupIdx = INDEX_NONE;
			for (int32 g = 0; g < GroupSum.Num(); ++g)
			{
				if (FVector::DotProduct(Fn, GroupSum[g].GetSafeNormal()) >= kCreaseDot)
				{
					GroupSum[g] += Fn;
					GroupIdx = g;
					break;
				}
			}
			FaceGroup[i] = (GroupIdx != INDEX_NONE) ? GroupIdx : GroupSum.Add(Fn);
		}
		for (int32 i = 0; i < Faces.Num(); ++i)
		{
			const int32 F = Faces[i];
			const FVector Gn = GroupSum[FaceGroup[i]].GetSafeNormal();
			const int32* FV = Mesh.Faces + 3 * (FaceAdr + F);
			for (int32 K = 0; K < 3; ++K)
			{
				if (FV[K] == V)
				{
					CornerN[3 * F + K] = Gn;
				}
			}
		}
	}

	const int32 Order[3] = {0, 1, 2};
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Mesh.Faces + 3 * (FaceAdr + F);
		const int32* FT = bHasUV ? Mesh.FaceTexcoords + 3 * (FaceAdr + F) : nullptr;
		for (int32 C = 0; C < 3; ++C)
		{
			const int32 K = Order[C];
			const int32 Vi = FV[K] + VertAdr;
			Verts.Add(URLabAxisConv::MjPositionToUe(Mesh.Verts + 3 * Vi));
			Normals.Add(CornerN[3 * F + K].GetSafeNormal());
			if (FT)
			{
				const int32 Ti = FT[K] + TexAdr;
				// No V flip: MuJoCo stores tex_data bottom-row-first (OpenGL), and
				// GetOrBuildTexture uploads it row-0-first, so the texture is already
				// oriented to sample the raw MuJoCo texcoord directly. Flipping V here
				// (1 - v) double-flips and samples the wrong band of the atlas.
				UVs.Add(FVector2D(Mesh.Texcoords[2 * Ti], Mesh.Texcoords[2 * Ti + 1]));
			}
			else
			{
				UVs.Add(FVector2D::ZeroVector);
			}
			Tris.Add(Verts.Num() - 1);
		}
	}
}

} // namespace

void UMjRendererAssetBaker::Init(mjModel_* InModel, const FString& InContentHash, bool bInForceRebuild)
{
	Model = InModel;
	ContentHash = InContentHash;
	bForceRebuildAssets = bInForceRebuild;
	Master = MjLoadMasterMaterial();
	if (!Master)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjRenderer] master material not found; geoms will be default-lit"));
	}
}

void UMjRendererAssetBaker::Reset()
{
	TextureCache.Reset();
	StaticMeshCache.Reset();
	Master = nullptr;
	ContentHash.Empty();
	Model = nullptr;
}

#if WITH_EDITOR
UStaticMesh* UMjRendererAssetBaker::GetOrBuildStaticMesh(int32 MeshId)
{
	if (const TObjectPtr<UStaticMesh>* Found = StaticMeshCache.Find(MeshId))
	{
		return *Found;
	}
	// Persistent, content-hashed cache: reuse the on-disk asset if present, so an
	// identical model doesn't rebuild and a saved level keeps its geometry.
	FString PackageName;
	if (!ContentHash.IsEmpty())
	{
		PackageName = UPackageTools::SanitizePackageName(
			FString::Printf(TEXT("%s/%s/SM_%d"), kFastPathAssetRoot, *ContentHash, MeshId));
		if (!bForceRebuildAssets)
		{
			if (UStaticMesh* Existing = LoadObject<UStaticMesh>(nullptr, *PackageName))
			{
				StaticMeshCache.Add(MeshId, Existing);
				return Existing;
			}
		}
	}

	TArray<FVector> Verts;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Tris;
	const FMjModelAssetSource AssetSource(Model);
	BuildMeshArrays(AssetSource, MeshId, Verts, Normals, UVs, Tris);
	if (Verts.Num() < 3 || Tris.Num() < 3)
	{
		return nullptr;
	}

	// UStaticMesh via a MeshDescription: the render build welds coincident positions
	// while splitting by our crease normals, so many geoms share one pointer-
	// referenced asset and the PIE-world duplication copies pointers, not verts.
	FMeshDescription MeshDesc;
	FStaticMeshAttributes Attrs(MeshDesc);
	Attrs.Register();
	Attrs.GetVertexInstanceUVs().SetNumChannels(1);
	TVertexAttributesRef<FVector3f> Positions = Attrs.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> InstNormals = Attrs.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector2f> InstUVs = Attrs.GetVertexInstanceUVs();

	const int32 NumVerts = Verts.Num();
	MeshDesc.ReserveNewVertices(NumVerts);
	TArray<FVertexID> VertIDs;
	VertIDs.SetNumUninitialized(NumVerts);
	for (int32 v = 0; v < NumVerts; ++v)
	{
		VertIDs[v] = MeshDesc.CreateVertex();
		Positions[VertIDs[v]] = FVector3f(Verts[v]);
	}
	const FPolygonGroupID PolyGroup = MeshDesc.CreatePolygonGroup();
	MeshDesc.ReserveNewVertexInstances(Tris.Num());
	MeshDesc.ReserveNewPolygons(Tris.Num() / 3);
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		FVertexInstanceID Inst[3];
		for (int32 K = 0; K < 3; ++K)
		{
			const int32 Vi = Tris[t + K];
			Inst[K] = MeshDesc.CreateVertexInstance(VertIDs[Vi]);
			InstNormals[Inst[K]] = FVector3f(Normals[Vi].GetSafeNormal());
			InstUVs.Set(Inst[K], 0, FVector2f(UVs[Vi]));
		}
		MeshDesc.CreatePolygon(PolyGroup, TArray<FVertexInstanceID>{Inst[0], Inst[1], Inst[2]});
	}

	// Persistent when we have a content hash (saved into /Game/URLabFastPath/<hash>/
	// so a saved level reloads and a re-connect skips the rebuild); transient
	// otherwise.
	UStaticMesh* Mesh = nullptr;
	UPackage* Package = nullptr;
	if (!PackageName.IsEmpty())
	{
		Package = CreatePackage(*PackageName);
		Package->FullyLoad();
		Mesh = NewObject<UStaticMesh>(Package, FName(*FString::Printf(TEXT("SM_%d"), MeshId)),
			RF_Public | RF_Standalone);
	}
	else
	{
		Mesh = NewObject<UStaticMesh>(this, NAME_None, RF_Transient);
	}
	Mesh->GetStaticMaterials().Add(FStaticMaterial());
	// No mesh distance field: these are puppet-render meshes (no Lumen GI/DFAO), and
	// the distance-field scene update ensure-spams on their transforms, stalling
	// ~2.5s per ensure (FDistanceFieldSceneData::UpdateDistanceFieldObjectBuffers).
	Mesh->bGenerateMeshDistanceField = false;
	UStaticMesh::FBuildMeshDescriptionsParams Params;
	Params.bBuildSimpleCollision = false;
	Params.bFastBuild = true;
	Mesh->NeverStream = true;
	Mesh->BuildFromMeshDescriptions({&MeshDesc}, Params);

	if (Package)
	{
		FAssetRegistryModule::AssetCreated(Mesh);
		Mesh->MarkPackageDirty();
		const FString FileName =
			FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		UPackage::SavePackage(Package, Mesh, *FileName, SaveArgs);
	}
	StaticMeshCache.Add(MeshId, Mesh);
	return Mesh;
}
#endif // WITH_EDITOR

UProceduralMeshComponent* UMjRendererAssetBaker::BuildMesh(int32 G, AActor* Body)
{
	TArray<FVector> Verts;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Tris;
	const FMjModelAssetSource AssetSource(Model);
	BuildMeshArrays(AssetSource, Model->geom_dataid[G], Verts, Normals, UVs, Tris);
	if (Verts.Num() < 3)
	{
		return nullptr;
	}

	// Per-face tangents from the UV gradient (the packaged-game path builds render
	// data at runtime, so it supplies them); assigned to all three corners.
	TArray<FProcMeshTangent> Tangents;
	Tangents.SetNum(Verts.Num());
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		const int32 I0 = Tris[t], I1 = Tris[t + 1], I2 = Tris[t + 2];
		const FVector E1 = Verts[I1] - Verts[I0];
		const FVector E2 = Verts[I2] - Verts[I0];
		const FVector2D D1 = UVs[I1] - UVs[I0];
		const FVector2D D2 = UVs[I2] - UVs[I0];
		const double Det = D1.X * D2.Y - D2.X * D1.Y;
		FVector Tan = FMath::Abs(Det) > SMALL_NUMBER ? ((E1 * D2.Y - E2 * D1.Y) / Det) : E1;
		Tan = Tan.GetSafeNormal();
		if (Tan.IsNearlyZero())
		{
			Tan = FVector::ForwardVector;
		}
		Tangents[I0] = Tangents[I1] = Tangents[I2] = FProcMeshTangent(Tan, /*bFlipTangentY=*/false);
	}

	UProceduralMeshComponent* Pmc = NewObject<UProceduralMeshComponent>(Body);
	Pmc->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Pmc->RegisterComponent();
	Pmc->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Pmc->CreateMeshSection(0, Verts, Tris, Normals, UVs, TArray<FColor>(), Tangents, /*bCreateCollision=*/false);
	return Pmc;
}

UTexture2D* UMjRendererAssetBaker::GetOrBuildTexture(int32 TexId, bool bSRGB, bool bNormal)
{
	if (!Model || TexId < 0 || TexId >= static_cast<int32>(Model->ntex))
	{
		return nullptr;
	}
	if (const TObjectPtr<UTexture2D>* Found = TextureCache.Find(TexId))
	{
		return *Found;
	}
	const FMjModelAssetSource AssetSource(Model);
	const FMjTextureDataView TexView = AssetSource.GetTextureData(TexId);
	const int32 W = TexView.Width;
	const int32 H = TexView.Height;
	const int32 NC = TexView.NumChannels;
	if (W <= 0 || H <= 0 || NC < 1)
	{
		return nullptr;
	}

#if WITH_EDITOR
	// Persistent, content-hashed cache (same scheme as the meshes).
	FString PackageName;
	if (!ContentHash.IsEmpty())
	{
		PackageName = UPackageTools::SanitizePackageName(
			FString::Printf(TEXT("%s/%s/T_%d"), kFastPathAssetRoot, *ContentHash, TexId));
		if (!bForceRebuildAssets)
		{
			if (UTexture2D* Existing = LoadObject<UTexture2D>(nullptr, *PackageName))
			{
				TextureCache.Add(TexId, Existing);
				return Existing;
			}
		}
	}
#endif

	// Build a BGRA8 buffer from the source's pixel bytes.
	const uint8* Src = TexView.Data;
	const int32 Pixels = W * H;
	TArray<uint8> Bgra;
	Bgra.SetNumUninitialized(Pixels * 4);
	for (int32 i = 0; i < Pixels; ++i)
	{
		uint8 R, Gc, B, A;
		if (NC >= 3)
		{
			R = Src[i * NC + 0];
			Gc = Src[i * NC + 1];
			B = Src[i * NC + 2];
			A = (NC >= 4) ? Src[i * NC + 3] : 255;
		}
		else
		{
			R = Gc = B = Src[i * NC]; // grayscale replicated
			A = 255;
		}
		Bgra[i * 4 + 0] = B;
		Bgra[i * 4 + 1] = Gc;
		Bgra[i * 4 + 2] = R;
		Bgra[i * 4 + 3] = A;
	}

	UTexture2D* Tex = nullptr;
#if WITH_EDITOR
	if (!PackageName.IsEmpty())
	{
		// Persistent: a real UTexture2D with source data, saved to the cache folder.
		UPackage* Package = CreatePackage(*PackageName);
		Package->FullyLoad();
		Tex = NewObject<UTexture2D>(Package, FName(*FString::Printf(TEXT("T_%d"), TexId)),
			RF_Public | RF_Standalone);
		Tex->Source.Init(W, H, 1, 1, TSF_BGRA8, Bgra.GetData());
		Tex->SRGB = bSRGB;
		// A normal-role map needs the normal-map codec so a SAMPLERTYPE_Normal
		// sampler decodes it (and so UE stops warning it is not a normal map).
		Tex->CompressionSettings = bNormal
			? TextureCompressionSettings::TC_Normalmap
			: TextureCompressionSettings::TC_Default;
		Tex->MipGenSettings = TextureMipGenSettings::TMGS_FromTextureGroup;
		Tex->UpdateResource();
		FAssetRegistryModule::AssetCreated(Tex);
		Tex->MarkPackageDirty();
		const FString FileName =
			FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		UPackage::SavePackage(Package, Tex, *FileName, SaveArgs);
	}
	else
#endif
	{
		// Transient (packaged, or no content hash): upload straight into the mip.
		Tex = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
		if (!Tex)
		{
			return nullptr;
		}
		Tex->SRGB = bSRGB;
		FTexturePlatformData* PD = Tex->GetPlatformData();
		uint8* Dst = static_cast<uint8*>(PD->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
		FMemory::Memcpy(Dst, Bgra.GetData(), Pixels * 4);
		PD->Mips[0].BulkData.Unlock();
		Tex->UpdateResource();
	}

	TextureCache.Add(TexId, Tex);
	return Tex;
}

void UMjRendererAssetBaker::ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 G)
{
	if (!Master || !Comp)
	{
		return;
	}
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);

	// Base colour, the scalar PBR terms and the texrepeat/texuniform mapping are read
	// off the compiled model as the same FMjMaterialValues the spec path produces, and
	// written through the one shared material-parameter writer -- so the wire path and
	// the authored path drive the master material identically. Every texture slot is
	// bound to its neutral stand-in there (the MJB carries no resolvable texture
	// names); the model's own images override the roles it fills below.
	FLinearColor BaseColor;
	FVector2D GeomSize;
	const FMjMaterialValues Values = MjMaterialValuesFromModel(Model, G, BaseColor, GeomSize);
	MjApplyMaterialParameters(*Mid, Values, BaseColor, FSpecRef(), GeomSize);

	// Bind the real MJB textures for the roles this material fills. mat_texid is
	// (nmat x mjNTEXROLE), role order matching EMjMaterialRole after the unused
	// USER slot (offset +1). Colour roles sample sRGB; data roles linear.
	const int32 MatId = Model->geom_matid[G];
	if (MatId >= 0)
	{
		for (int32 R = 0; R < static_cast<int32>(EMjMaterialRole::Count); ++R)
		{
			const int32 TexId = Model->mat_texid[MatId * mjNTEXROLE + R + 1];
			if (TexId < 0)
			{
				continue;
			}
			const EMjMaterialRole MatRole = static_cast<EMjMaterialRole>(R);
			const bool bSRGB = (MatRole == EMjMaterialRole::Rgb || MatRole == EMjMaterialRole::Rgba
				|| MatRole == EMjMaterialRole::Emissive);
			const bool bNormal = (MatRole == EMjMaterialRole::Normal);
			if (UTexture2D* Tex = GetOrBuildTexture(TexId, bSRGB, bNormal))
			{
				Mid->SetTextureParameterValue(MjMaterialRoleParameter(MatRole), Tex);
			}
		}
	}
}
