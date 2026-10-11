// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "Render/RenderAssetBuilder.h"

#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PhysicsEngine/BodySetup.h"
#include "Engine/Texture2D.h"
#include "TextureResource.h"

#include "Render/RenderMaterial.h"
#include "Render/RenderScene.h"
#include "Utils/URLabLogging.h"
#if WITH_EDITOR
#include "Misc/PackageName.h"
#include "PackageTools.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "AssetRegistry/AssetRegistryModule.h"
#endif

namespace
{
// Root of the cached, content-hash-keyed render assets (SM_<id> / T_<id> live
// under <root>/<hash>/). Also the folder a saved render level reloads from.
constexpr const TCHAR* kRenderAssetRoot = TEXT("/Game/URLabRender");

void BuildFaceTangents(const TArray<FVector>& Verts, const TArray<FVector2D>& UVs,
	const TArray<int32>& Tris, TArray<FVector>& OutTangents)
{
	OutTangents.Reset();
	OutTangents.SetNum(Verts.Num());
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		const int32 I0 = Tris[t], I1 = Tris[t + 1], I2 = Tris[t + 2];
		const FVector E1 = Verts[I1] - Verts[I0];
		const FVector E2 = Verts[I2] - Verts[I0];
		const FVector2D D1 = UVs[I1] - UVs[I0];
		const FVector2D D2 = UVs[I2] - UVs[I0];
		const double Det = D1.X * D2.Y - D2.X * D1.Y;
		// A degenerate UV triangle (no gradient) has no defined tangent; fall back
		// to the first edge so the basis stays finite rather than zero.
		FVector Tan = FMath::Abs(Det) > SMALL_NUMBER ? ((E1 * D2.Y - E2 * D1.Y) / Det) : E1;
		Tan = Tan.GetSafeNormal();
		if (Tan.IsNearlyZero())
		{
			Tan = FVector::ForwardVector;
		}
		OutTangents[I0] = OutTangents[I1] = OutTangents[I2] = Tan;
	}
}

/** sRGB electro-optical transfer, the same curve UE's texture build uses when it
 *  filters an sRGB texture's mips in linear space. */
float SrgbToLinear(uint8 Byte)
{
	const float V = static_cast<float>(Byte) / 255.0f;
	return (V <= 0.04045f) ? (V / 12.92f) : FMath::Pow((V + 0.055f) / 1.055f, 2.4f);
}

uint8 LinearToSrgb(float Linear)
{
	const float C = FMath::Clamp(Linear, 0.0f, 1.0f);
	const float V = (C <= 0.0031308f) ? (C * 12.92f) : (1.055f * FMath::Pow(C, 1.0f / 2.4f) - 0.055f);
	return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(V * 255.0f), 0, 255));
}

/**
 * Box-filter one BGRA8 image down to half size, honouring what the channels mean.
 *
 * Colour (sRGB) channels are averaged in linear light -- averaging the encoded
 * bytes darkens every mip. A normal map is averaged as a vector and renormalised,
 * because the mean of two unit normals is not a unit normal and a shortened
 * tangent-space normal flattens the shading. Alpha and linear data channels
 * (roughness, metallic, occlusion) are averaged directly.
 */
void DownsampleBgra(const TArray<uint8>& Src, int32 SrcW, int32 SrcH, bool bSRGB, bool bNormal,
	TArray<uint8>& Dst, int32& DstW, int32& DstH)
{
	DstW = FMath::Max(1, SrcW / 2);
	DstH = FMath::Max(1, SrcH / 2);
	Dst.SetNumUninitialized(DstW * DstH * 4);
	for (int32 Y = 0; Y < DstH; ++Y)
	{
		// Clamp so an odd source dimension still samples inside the image.
		const int32 Y0 = FMath::Min(Y * 2, SrcH - 1);
		const int32 Y1 = FMath::Min(Y * 2 + 1, SrcH - 1);
		for (int32 X = 0; X < DstW; ++X)
		{
			const int32 X0 = FMath::Min(X * 2, SrcW - 1);
			const int32 X1 = FMath::Min(X * 2 + 1, SrcW - 1);
			const int32 Taps[4] = {(Y0 * SrcW + X0) * 4, (Y0 * SrcW + X1) * 4,
				(Y1 * SrcW + X0) * 4, (Y1 * SrcW + X1) * 4};
			uint8* Out = Dst.GetData() + (Y * DstW + X) * 4;
			if (bNormal)
			{
				// BGRA layout: index 2 is R (normal x), 1 is G (y), 0 is B (z).
				FVector N = FVector::ZeroVector;
				for (int32 T = 0; T < 4; ++T)
				{
					N += FVector(Src[Taps[T] + 2] / 255.0f * 2.0f - 1.0f,
						Src[Taps[T] + 1] / 255.0f * 2.0f - 1.0f,
						Src[Taps[T] + 0] / 255.0f * 2.0f - 1.0f);
				}
				N = N.GetSafeNormal();
				if (N.IsNearlyZero())
				{
					N = FVector::UpVector; // flat normal
				}
				auto Encode = [](double C) {
					return static_cast<uint8>(FMath::Clamp(
						FMath::RoundToInt((C * 0.5 + 0.5) * 255.0), 0, 255));
				};
				Out[2] = Encode(N.X);
				Out[1] = Encode(N.Y);
				Out[0] = Encode(N.Z);
				Out[3] = 255;
			}
			else
			{
				for (int32 C = 0; C < 4; ++C)
				{
					// Alpha is always linear, even on an sRGB texture.
					if (bSRGB && C < 3)
					{
						float Sum = 0.0f;
						for (int32 T = 0; T < 4; ++T)
						{
							Sum += SrgbToLinear(Src[Taps[T] + C]);
						}
						Out[C] = LinearToSrgb(Sum * 0.25f);
					}
					else
					{
						int32 Sum = 0;
						for (int32 T = 0; T < 4; ++T)
						{
							Sum += Src[Taps[T] + C];
						}
						Out[C] = static_cast<uint8>((Sum + 2) / 4);
					}
				}
			}
		}
	}
}

} // namespace

bool URenderAssetBuilder::ValidateResources(
	const FRenderSceneResources& InResources, FString& OutError)
{
	if (!InResources.IsValid(OutError))
	{
		return false;
	}
	if (!LoadRenderMasterMaterial())
	{
		OutError = TEXT("required renderer master material is unavailable");
		return false;
	}
	return true;
}

bool URenderAssetBuilder::InitResources(FRenderSceneResources&& InResources,
	bool bInForceRebuild, FString& OutError)
{
	if (!ValidateResources(InResources, OutError))
	{
		return false;
	}
	UMaterialInterface* CandidateMaster = LoadRenderMasterMaterial();
	const FString CandidateHash = InResources.ContentHash();

	// Nothing live is changed until the complete candidate has passed validation.
	TextureCache.Reset();
	StaticMeshCache.Reset();
	ResolvedObjects.Reset();
	Master = CandidateMaster;
	ContentHash = CandidateHash;
	bForceRebuildAssets = bInForceRebuild;
	bBakeToProject = false;
	Resources = MoveTemp(InResources);
	return true;
}

void URenderAssetBuilder::Reset()
{
	TextureCache.Reset();
	StaticMeshCache.Reset();
	ResolvedObjects.Reset();
	Master = nullptr;
	ContentHash.Empty();
	bBakeToProject = false;
	Resources = FRenderSceneResources();
}

#if WITH_EDITOR
UStaticMesh* URenderAssetBuilder::GetOrBuildStaticMesh(int32 MeshId)
{
	if (const TObjectPtr<UStaticMesh>* Found = StaticMeshCache.Find(MeshId))
	{
		return *Found;
	}
	// Persistent, content-hashed cache: reuse the on-disk asset if present, so an
	// identical resource content doesn't rebuild and a saved level keeps its geometry.
	// Only the explicit bake-to-project action takes this route. Runtime scene installation
	// always builds transient assets, because a cooked build cannot
	// write (or read back) /Game at all: doing it during play made the editor and
	// the package render two different asset sets, and stalled the first connect
	// on package saves.
	FString PackageName;
	if (bBakeToProject && !ContentHash.IsEmpty())
	{
		PackageName = UPackageTools::SanitizePackageName(
			FString::Printf(TEXT("%s/%s/SM_%d"), kRenderAssetRoot, *ContentHash, MeshId));
		if (!bForceRebuildAssets)
		{
			if (UStaticMesh* Existing = LoadObject<UStaticMesh>(nullptr, *PackageName))
			{
				StaticMeshCache.Add(MeshId, Existing);
				return Existing;
			}
		}
	}

	if (!Resources.Meshes.IsValidIndex(MeshId)) return nullptr;
	const FRenderMeshResource& Resource = Resources.Meshes[MeshId];
	const TArray<FVector>& Verts = Resource.Positions;
	const TArray<FVector>& Normals = Resource.Normals;
	const TArray<FVector2D>& UVs = Resource.UVs;
	const TArray<int32>& Tris = Resource.Triangles;
	if (Verts.Num() < 3 || Tris.Num() < 3)
	{
		return nullptr;
	}

	// UStaticMesh via a MeshDescription: the render build welds coincident positions
	// while splitting by our crease normals, so many geoms share one referenced asset.
	FMeshDescription MeshDesc;
	FStaticMeshAttributes Attrs(MeshDesc);
	Attrs.Register();
	Attrs.GetVertexInstanceUVs().SetNumChannels(1);
	TVertexAttributesRef<FVector3f> Positions = Attrs.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> InstNormals = Attrs.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector3f> InstTangents = Attrs.GetVertexInstanceTangents();
	TVertexInstanceAttributesRef<float> InstBinormalSigns = Attrs.GetVertexInstanceBinormalSigns();
	TVertexInstanceAttributesRef<FVector2f> InstUVs = Attrs.GetVertexInstanceUVs();

	// The same UV-gradient tangents the procedural route supplies. A fast build
	// copies these attributes verbatim into the vertex buffer and computes nothing,
	// so leaving them at their zero default is what made normal-mapped geoms shade
	// differently here than in a cooked build.
	TArray<FVector> Tangents;
	BuildFaceTangents(Verts, UVs, Tris, Tangents);

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
			InstTangents[Inst[K]] = FVector3f(Tangents[Vi]);
			// Right-handed basis, matching FProcMeshTangent(Tan, bFlipTangentY=false)
			// on the procedural route so both meshes shade a normal map alike.
			InstBinormalSigns[Inst[K]] = 1.0f;
			InstUVs.Set(Inst[K], 0, FVector2f(UVs[Vi]));
		}
		MeshDesc.CreatePolygon(PolyGroup, TArray<FVertexInstanceID>{Inst[0], Inst[1], Inst[2]});
	}

	// Package-owned only under the explicit bake (saved into
	// /Game/URLabRender/<hash>/ so a saved level reloads its geometry);
	// transient during play, in both builds.
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
	// These are visual puppet meshes. BuildFromMeshDescriptions always creates a
	// BodySetup, so seed its explicit no-cook policy before the fast build reuses it.
	Mesh->CreateBodySetup();
	Mesh->GetBodySetup()->bNeverNeedsCookedCollisionData = true;
	Mesh->MarkAsNotHavingNavigationData();
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

int32 URenderAssetBuilder::BakeLoadedModelToProject()
{
	if (Resources.SceneRevision.IsEmpty() || ContentHash.IsEmpty())
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjRenderer] bake to project skipped: no loaded resources or no content hash"));
		return 0;
	}

	// The caches hold the transient assets this session is rendering with. Drop
	// them so the bake builds package-owned ones, and drop them again afterwards
	// so the session goes back to rendering from transient assets -- the bake
	// writes a cache for a saved level to reference, it does not change what the
	// running scene draws.
	TextureCache.Reset();
	StaticMeshCache.Reset();

	int32 Written = 0;
	{
		TGuardValue<bool> BakeScope(bBakeToProject, true);
		for (int32 MeshId = 0; MeshId < Resources.Meshes.Num(); ++MeshId)
		{
			if (GetOrBuildStaticMesh(MeshId) != nullptr)
			{
				++Written;
			}
		}
		// Textures are reached through the materials that use them, because a
		// texture's role decides how it is sampled (and there is no role on the
		// texture itself).
		for (int32 MatId = 0; MatId < Resources.Materials.Num(); ++MatId)
		{
			const FRenderMaterialResource& Material = Resources.Materials[MatId];
			for (int32 R = 0; R < static_cast<int32>(ERenderMaterialRole::Count); ++R)
			{
				const FRenderTextureBinding& Binding = Material.Textures[R];
				if (Binding.TextureIndex == INDEX_NONE)
				{
					continue;
				}
				if (GetOrBuildTexture(Binding.TextureIndex, Binding.bSRGB, Binding.bNormal) != nullptr)
				{
					++Written;
				}
			}
		}
	}

	TextureCache.Reset();
	StaticMeshCache.Reset();
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] baked %d render assets into %s/%s"),
		Written, kRenderAssetRoot, *ContentHash);
	return Written;
}
#endif // WITH_EDITOR

UProceduralMeshComponent* URenderAssetBuilder::BuildMesh(int32 MeshId, AActor* Body)
{
	if (!Resources.Meshes.IsValidIndex(MeshId)) return nullptr;
	const FRenderMeshResource& Resource = Resources.Meshes[MeshId];
	const TArray<FVector>& Verts = Resource.Positions;
	const TArray<FVector>& Normals = Resource.Normals;
	const TArray<FVector2D>& UVs = Resource.UVs;
	const TArray<int32>& Tris = Resource.Triangles;
	if (Verts.Num() < 3)
	{
		return nullptr;
	}

	// Per-face tangents from the UV gradient, the same ones the static-mesh route
	// writes into its mesh description.
	TArray<FVector> FaceTangents;
	BuildFaceTangents(Verts, UVs, Tris, FaceTangents);
	TArray<FProcMeshTangent> Tangents;
	Tangents.SetNum(FaceTangents.Num());
	for (int32 i = 0; i < FaceTangents.Num(); ++i)
	{
		Tangents[i] = FProcMeshTangent(FaceTangents[i], /*bFlipTangentY=*/false);
	}

	UProceduralMeshComponent* Pmc = NewObject<UProceduralMeshComponent>(Body);
	Pmc->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Pmc->RegisterComponent();
	Pmc->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Pmc->CreateMeshSection(0, Verts, Tris, Normals, UVs, TArray<FColor>(), Tangents, /*bCreateCollision=*/false);
	return Pmc;
}

UTexture2D* URenderAssetBuilder::GetOrBuildTexture(int32 TexId, bool bSRGB, bool bNormal)
{
	if (!Resources.Textures.IsValidIndex(TexId))
	{
		return nullptr;
	}
	const uint64 CacheKey = (static_cast<uint64>(static_cast<uint32>(TexId)) << 2)
		| (bSRGB ? 1ull : 0ull) | (bNormal ? 2ull : 0ull);
	if (const TObjectPtr<UTexture2D>* Found = TextureCache.Find(CacheKey))
	{
		return *Found;
	}
	const FRenderTextureResource& Resource = Resources.Textures[TexId];
	const int32 W = Resource.Width, H = Resource.Height;

#if WITH_EDITOR
	// Persistent, content-hashed cache (same scheme as the meshes), reached only
	// by the explicit bake-to-project action. Runtime installs in editor and cooked builds
	// sample the same transient texture construction path.
	FString PackageName;
	if (bBakeToProject && !ContentHash.IsEmpty())
	{
		PackageName = UPackageTools::SanitizePackageName(
			FString::Printf(TEXT("%s/%s/T_%d_s%d_n%d"), kRenderAssetRoot, *ContentHash, TexId, bSRGB ? 1 : 0, bNormal ? 1 : 0));
		if (!bForceRebuildAssets)
		{
			if (UTexture2D* Existing = LoadObject<UTexture2D>(nullptr, *PackageName))
			{
				TextureCache.Add(CacheKey, Existing);
				return Existing;
			}
		}
	}
#endif

	// Build a BGRA8 buffer from the source's pixel bytes.
	const uint8* Src = Resource.Rgba8.GetData();
	const int32 Pixels = W * H;
	TArray<uint8> Bgra;
	Bgra.SetNumUninitialized(Pixels * 4);
	for (int32 i = 0; i < Pixels; ++i)
	{
		uint8 R, Gc, B, A;
		R = Src[i * 4 + 0]; Gc = Src[i * 4 + 1]; B = Src[i * 4 + 2]; A = Src[i * 4 + 3];
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
		// The one route every play session takes, editor and cooked alike: a
		// transient BGRA8 texture built here, with its own mip chain.
		//
		// UTexture2D::CreateTransient allocates a single mip, and a mip-less
		// texture aliases badly the moment a camera looks at a surface at an
		// angle or from a distance -- which is precisely what a rendered camera
		// image is made of. Build the chain down to 1x1 so a runtime texture
		// filters like an editor-built one.
		Tex = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
		if (!Tex)
		{
			return nullptr;
		}
		FTexturePlatformData* PD = new FTexturePlatformData();
		PD->SizeX = W;
		PD->SizeY = H;
		PD->SetNumSlices(1);
		PD->PixelFormat = PF_B8G8R8A8;
		Tex->SetPlatformData(PD);
		// Pixels are already in the runtime format; nothing tiles or recompresses them.
		Tex->bNotOfflineProcessed = true;

		TArray<uint8> Level = MoveTemp(Bgra);
		int32 LevelW = W;
		int32 LevelH = H;
		for (;;)
		{
			FTexture2DMipMap* Mip = new FTexture2DMipMap(LevelW, LevelH, 1);
			PD->Mips.Add(Mip);
			Mip->BulkData.Lock(LOCK_READ_WRITE);
			void* Dst = Mip->BulkData.Realloc(Level.Num());
			FMemory::Memcpy(Dst, Level.GetData(), Level.Num());
			Mip->BulkData.Unlock();
			if (LevelW <= 1 && LevelH <= 1)
			{
				break;
			}
			TArray<uint8> Next;
			int32 NextW = 0;
			int32 NextH = 0;
			DownsampleBgra(Level, LevelW, LevelH, bSRGB, bNormal, Next, NextW, NextH);
			Level = MoveTemp(Next);
			LevelW = NextW;
			LevelH = NextH;
		}

		Tex->SRGB = bSRGB;
		// Runtime textures are never compressed, so this only records the role --
		// but it is the role the material's sampler is validated against, and it
		// is what tells the engine (and a reader of this texture) that the RG
		// channels are a tangent-space normal rather than a colour.
		Tex->CompressionSettings = bNormal
			? TextureCompressionSettings::TC_Normalmap
			: TextureCompressionSettings::TC_Default;
		// Every mip lives in memory here; there is no bulk data on disk for the
		// streamer to page in, so the whole chain must stay resident.
		Tex->NeverStream = true;
		Tex->UpdateResource();
	}

	TextureCache.Add(CacheKey, Tex);
	return Tex;
}

void URenderAssetBuilder::ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 G)
{
	if (!Master || !Comp || !Resources.Geoms.IsValidIndex(G))
	{
		return;
	}
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);

	const FRenderSurfaceBinding& Surface = Resources.Geoms[G];
	const FRenderMaterialResource DefaultMaterial;
	const FRenderMaterialResource& Material = Resources.Materials.IsValidIndex(Surface.MaterialIndex)
		? Resources.Materials[Surface.MaterialIndex] : DefaultMaterial;
	ApplyRenderMaterialParameters(*Mid, Material, Surface.Rgba, Surface.TexUniformScale);
	BindMaterialRoleTextures(*Mid, Surface.MaterialIndex);
}

void URenderAssetBuilder::ApplyResolvedMaterialOverride(UPrimitiveComponent* Comp,
	const FRenderResolvedMaterialOverride& Override)
{
	if (!Master || !Comp || !Override.bEnabled)
	{
		return;
	}
	for (UTexture2D* Texture : Override.RoleTextures)
	{
		RetainResolvedObject(Texture);
	}
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);
	ApplyRenderMaterialParameters(*Mid, Override.Material, Override.BaseColor,
		Override.TexUniformScale);
	for (int32 RoleIndex = 0;
		RoleIndex < static_cast<int32>(ERenderMaterialRole::Count); ++RoleIndex)
	{
		UTexture2D* Texture = Override.RoleTextures.IsValidIndex(RoleIndex)
			? Override.RoleTextures[RoleIndex].Get() : nullptr;
		if (Texture)
		{
			Mid->SetTextureParameterValue(
				RenderMaterialRoleParameter(static_cast<ERenderMaterialRole>(RoleIndex)), Texture);
		}
	}
}

void URenderAssetBuilder::RetainResolvedObject(UObject* Object)
{
	if (Object)
	{
		ResolvedObjects.AddUnique(Object);
	}
}

void URenderAssetBuilder::BindMaterialRoleTextures(UMaterialInstanceDynamic& Mid, int32 MatId)
{
	if (!Resources.Materials.IsValidIndex(MatId))
	{
		return;
	}
	const FRenderMaterialResource& Material = Resources.Materials[MatId];
	for (int32 R = 0; R < static_cast<int32>(ERenderMaterialRole::Count); ++R)
	{
		const FRenderTextureBinding& Binding = Material.Textures[R];
		if (Binding.TextureIndex == INDEX_NONE)
		{
			continue;
		}
		const ERenderMaterialRole Role = static_cast<ERenderMaterialRole>(R);
		if (UTexture2D* Tex = GetOrBuildTexture(
			Binding.TextureIndex, Binding.bSRGB, Binding.bNormal))
		{
			Mid.SetTextureParameterValue(RenderMaterialRoleParameter(Role), Tex);
		}
	}
}

void URenderAssetBuilder::ApplyDeformableMaterial(UPrimitiveComponent* Comp, int32 SurfaceIndex)
{
	if (!Master || !Comp || !Resources.Deformables.IsValidIndex(SurfaceIndex))
	{
		return;
	}
	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);

	const FRenderSurfaceBinding& Surface = Resources.Deformables[SurfaceIndex];
	const FRenderMaterialResource DefaultMaterial;
	const FRenderMaterialResource& Material = Resources.Materials.IsValidIndex(Surface.MaterialIndex)
		? Resources.Materials[Surface.MaterialIndex] : DefaultMaterial;
	ApplyRenderMaterialParameters(*Mid, Material, Surface.Rgba, Surface.TexUniformScale);
	BindMaterialRoleTextures(*Mid, Surface.MaterialIndex);
}
