// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Asset-builder parity tests. The editor (PIE) and the cooked package are both
// supported render targets, so what the builder hands the renderer during play
// must not depend on which one is running: transient, fully mipped textures and
// a real tangent basis on every mesh, with the persistent /Game bake reachable
// only through its own explicit action.

#include "Misc/AutomationTest.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture.h"
#include "Engine/Texture2D.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpression.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshResources.h"

#include "Render/RenderSceneResources.h"
#include "Render/RenderAssetBuilder.h"
#include "Render/RenderMaterial.h"
#include "MuJoCo/Render/MjRenderResourceShim.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_EDITOR

namespace RenderBuilderTest
{
FRenderSceneResources MakeNeutralResources(const FString& Revision = TEXT("neutral-test"))
{
	FRenderSceneResources Resources;
	Resources.SceneRevision = Revision;

	FRenderMeshResource& Mesh = Resources.Meshes.AddDefaulted_GetRef();
	Mesh.Positions = {FVector(0, 0, 0), FVector(100, 0, 0), FVector(0, 100, 0)};
	Mesh.Normals = {FVector::UpVector, FVector::UpVector, FVector::UpVector};
	Mesh.UVs = {FVector2D(0, 0), FVector2D(1, 0), FVector2D(0, 1)};
	Mesh.Triangles = {0, 1, 2};

	for (int32 TextureIndex = 0; TextureIndex < 2; ++TextureIndex)
	{
		FRenderTextureResource& Texture = Resources.Textures.AddDefaulted_GetRef();
		Texture.Width = 16;
		Texture.Height = 16;
		Texture.Rgba8.SetNumUninitialized(16 * 16 * 4);
		for (int32 Pixel = 0; Pixel < 16 * 16; ++Pixel)
		{
			Texture.Rgba8[4 * Pixel] = TextureIndex == 0 ? 255 : 128;
			Texture.Rgba8[4 * Pixel + 1] = TextureIndex == 0 ? 32 : 128;
			Texture.Rgba8[4 * Pixel + 2] = TextureIndex == 0 ? 16 : 255;
			Texture.Rgba8[4 * Pixel + 3] = 255;
		}
	}

	FRenderMaterialResource& Material = Resources.Materials.AddDefaulted_GetRef();
	Material.Emission = 3.0f;
	Material.Metallic = 0.25f;
	Material.Roughness = 0.75f;
	Material.Textures[static_cast<int32>(ERenderMaterialRole::Rgb)] = {0, true, false};
	Material.Textures[static_cast<int32>(ERenderMaterialRole::Normal)] = {1, false, true};

	FRenderSurfaceBinding& Geom = Resources.Geoms.AddDefaulted_GetRef();
	Geom.MaterialIndex = 0;
	Geom.Rgba = FLinearColor(0.1f, 0.2f, 0.3f, 1.0f);
	Geom.TexUniformScale = FVector2D(2.0, 3.0);
	return Resources;
}

// A model with one colour texture, one normal-role texture and one mesh geom --
// the three things the two build routes used to disagree about.
mjModel* CompileFixture(FString& OutError)
{
	static const char* kXml =
		"<mujoco>"
		"  <asset>"
		"    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" width=\"16\" height=\"16\""
		"             rgb1=\"1 0 0\" rgb2=\"0 1 0\"/>"
		"    <texture name=\"bump\" type=\"2d\" builtin=\"flat\" width=\"16\" height=\"16\""
		"             rgb1=\"0.5 0.5 1\"/>"
		// Both textures are referenced by a material: MuJoCo compiles what is used.
		"    <material name=\"mat\" texture=\"grid\"/>"
		"    <material name=\"mat_bump\" texture=\"bump\"/>"
		"    <mesh name=\"tet\" vertex=\"0 0 0  1 0 0  0 1 0  0 0 1\"/>"
		"  </asset>"
		"  <worldbody>"
		"    <geom type=\"mesh\" mesh=\"tet\" material=\"mat\"/>"
		"  </worldbody>"
		"</mujoco>";

	char Err[1000] = "";
	mjSpec* Spec = mj_parseXMLString(kXml, nullptr, Err, sizeof(Err));
	if (Spec == nullptr)
	{
		OutError = ANSI_TO_TCHAR(Err);
		return nullptr;
	}
	mjModel* M = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	if (M == nullptr)
	{
		OutError = TEXT("mj_compile failed");
	}
	return M;
}
} // namespace RenderBuilderTest

// ============================================================================
// URLab.Render.AssetBuilder_PlayBuildsTransientMippedTextures
//   During play the builder must build the SAME texture in the editor and in a
//   cooked build: transient (a cooked build cannot read or write /Game) and with
//   a full mip chain (a single-mip texture aliases, so a camera image rendered in
//   the editor did not match one rendered from the package). A normal-role map is
//   additionally tagged with the normal-map codec.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetBuilderTransientMippedTextures,
	"URLab.Render.AssetBuilder_PlayBuildsTransientMippedTextures",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAssetBuilderTransientMippedTextures::RunTest(const FString& Parameters)
{
	FString Error;
	URenderAssetBuilder* Builder = NewObject<URenderAssetBuilder>();
	if (!TestTrue(TEXT("neutral resources accepted"), Builder->InitResources(
		RenderBuilderTest::MakeNeutralResources(), /*bForceRebuild=*/false, Error)))
	{
		AddError(Error);
		return false;
	}

	UTexture2D* Colour = Builder->GetOrBuildTexture(0, /*bSRGB=*/true, /*bNormal=*/false);
	UTexture2D* Normal = Builder->GetOrBuildTexture(1, /*bSRGB=*/false, /*bNormal=*/true);
	UTexture2D* ColourAsData = Builder->GetOrBuildTexture(0, /*bSRGB=*/false, /*bNormal=*/false);
	if (TestNotNull(TEXT("colour texture"), Colour) && TestNotNull(TEXT("normal texture"), Normal))
	{
		TestFalse(TEXT("play never saves the texture into /Game"),
			Colour->GetPathName().StartsWith(TEXT("/Game/")));

		const FTexturePlatformData* ColourData = Colour->GetPlatformData();
		if (TestNotNull(TEXT("colour platform data"), ColourData))
		{
			// 16x16 halves down to 1x1: 16, 8, 4, 2, 1.
			TestEqual(TEXT("full mip chain"), ColourData->Mips.Num(), 5);
			TestEqual(TEXT("top mip is the source size"),
				static_cast<int32>(ColourData->Mips[0].SizeX), 16);
			TestEqual(TEXT("last mip is 1x1"),
				static_cast<int32>(ColourData->Mips.Last().SizeX), 1);
		}
		TestTrue(TEXT("colour texture samples sRGB"), Colour->SRGB != 0);
		TestNotEqual(TEXT("same source has a distinct linear sampling view"), Colour, ColourAsData);
		if (ColourAsData) TestFalse(TEXT("linear view does not inherit sRGB"), ColourAsData->SRGB != 0);
		TestEqual(TEXT("colour texture is not tagged a normal map"),
			static_cast<int32>(Colour->CompressionSettings.GetValue()),
			static_cast<int32>(TextureCompressionSettings::TC_Default));

		const FTexturePlatformData* NormalData = Normal->GetPlatformData();
		if (TestNotNull(TEXT("normal platform data"), NormalData))
		{
			TestEqual(TEXT("normal map is mipped too"), NormalData->Mips.Num(), 5);
		}
		TestFalse(TEXT("normal map samples linear"), Normal->SRGB != 0);
		TestEqual(TEXT("normal map carries the normal-map codec"),
			static_cast<int32>(Normal->CompressionSettings.GetValue()),
			static_cast<int32>(TextureCompressionSettings::TC_Normalmap));
		// Both mips stay resident: nothing on disk backs them.
		TestTrue(TEXT("runtime texture is never streamed"), Colour->NeverStream != 0);
	}

	Builder->Reset();
	return true;
}

// ============================================================================
// URLab.Render.AssetBuilder_StaticMeshHasTangentBasis
//   The editor route writes its mesh description straight into the vertex buffer
//   (a fast build computes nothing), so an unset Tangent attribute reached the
//   GPU as a zero tangent -- normal-mapped geoms shaded differently there than on
//   the packaged procedural route, which always supplied tangents. Both routes
//   now derive them from the same helper.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetBuilderStaticMeshTangents,
	"URLab.Render.AssetBuilder_StaticMeshHasTangentBasis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAssetBuilderStaticMeshTangents::RunTest(const FString& Parameters)
{
	FString Error;
	URenderAssetBuilder* Builder = NewObject<URenderAssetBuilder>();
	if (!TestTrue(TEXT("neutral resources accepted"), Builder->InitResources(
		RenderBuilderTest::MakeNeutralResources(), /*bForceRebuild=*/false, Error)))
	{
		AddError(Error);
		return false;
	}

	UStaticMesh* Mesh = Builder->GetOrBuildStaticMesh(0);
	if (TestNotNull(TEXT("built static mesh"), Mesh))
	{
		TestFalse(TEXT("play never saves the mesh into /Game"),
			Mesh->GetPathName().StartsWith(TEXT("/Game/")));
		const UBodySetup* BodySetup = Mesh->GetBodySetup();
		TestTrue(TEXT("visual mesh suppresses collision cooking"),
			BodySetup && BodySetup->bNeverNeedsCookedCollisionData);

		const FStaticMeshRenderData* Render = Mesh->GetRenderData();
		if (TestNotNull(TEXT("render data"), Render) && Render->LODResources.Num() > 0)
		{
			const FStaticMeshVertexBuffer& VB =
				Render->LODResources[0].VertexBuffers.StaticMeshVertexBuffer;
			const uint32 NumVerts = VB.GetNumVertices();
			if (TestTrue(TEXT("vertices built"), NumVerts > 0)
				&& TestTrue(TEXT("tangents kept CPU-accessible"),
					VB.GetTangentData() != nullptr))
			{
				int32 Degenerate = 0;
				for (uint32 V = 0; V < NumVerts; ++V)
				{
					if (FVector3f(VB.VertexTangentX(V)).IsNearlyZero())
					{
						++Degenerate;
					}
				}
				TestEqual(TEXT("no vertex has a zero tangent"), Degenerate, 0);
			}
		}
	}

	Builder->Reset();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetBuilderNeutralMaterial,
	"URLab.Render.AssetBuilder_NeutralMaterialCreatesRealMID",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAssetBuilderNeutralMaterial::RunTest(const FString& Parameters)
{
	FString Error;
	URenderAssetBuilder* Builder = NewObject<URenderAssetBuilder>();
	if (!TestTrue(TEXT("neutral resources accepted"), Builder->InitResources(
		RenderBuilderTest::MakeNeutralResources(), false, Error)))
	{
		AddError(Error);
		return false;
	}

	UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>();
	Builder->ApplyGeomMaterial(Component, 0);
	UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(Component->GetMaterial(0));
	if (TestNotNull(TEXT("neutral resource produced a material instance"), Mid))
	{
		UMaterial* Parent = Mid->GetMaterial();
		TestNotNull(TEXT("MID resolves the loaded master material"), Parent);
		if (Parent)
		{
			TestEqual(TEXT("runtime parent is the commandlet-generated master asset"),
				Parent->GetPathName(),
				FString(TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master.M_MuJoCo_Master")));
			TArray<UMaterialExpression*> Chain;
			TestTrue(TEXT("stored master has a connected emissive property chain"),
				Parent->GetExpressionsInPropertyChain(MP_EmissiveColor, Chain, nullptr,
					ERHIFeatureLevel::SM5, EMaterialQualityLevel::High));
			bool bHasEmissionScalar = false;
			bool bHasEmissiveTexture = false;
			for (UMaterialExpression* Expression : Chain)
			{
				if (const auto* Scalar = Cast<UMaterialExpressionScalarParameter>(Expression))
				{
					bHasEmissionScalar |= Scalar->ParameterName == TEXT("Emission");
				}
				if (const auto* Texture = Cast<UMaterialExpressionTextureSampleParameter2D>(Expression))
				{
					bHasEmissiveTexture |= Texture->ParameterName == TEXT("EmissiveTexture");
				}
			}
			TestTrue(TEXT("stored emissive chain reaches the Emission scalar"), bHasEmissionScalar);
			TestTrue(TEXT("stored emissive chain reaches EmissiveTexture"), bHasEmissiveTexture);
			for (UMaterialExpression* Expression : Parent->GetExpressions())
			{
				const auto* Texture = Cast<UMaterialExpressionTextureSampleParameter2D>(Expression);
				if (!Texture) continue;
				const bool bColor = Texture->ParameterName == TEXT("RgbTexture")
					|| Texture->ParameterName == TEXT("RgbaTexture")
					|| Texture->ParameterName == TEXT("EmissiveTexture");
				const bool bNormal = Texture->ParameterName == TEXT("NormalTexture");
				TestEqual(*FString::Printf(TEXT("%s uses its role-compatible sampler"),
					*Texture->ParameterName.ToString()), Texture->SamplerType,
					bNormal ? SAMPLERTYPE_Normal
						: bColor ? SAMPLERTYPE_Color : SAMPLERTYPE_LinearColor);
			}
		}
		FLinearColor BaseColor;
		TestTrue(TEXT("base colour parameter is present"), Mid->GetVectorParameterValue(
			FMaterialParameterInfo(TEXT("BaseColor")), BaseColor));
		TestTrue(TEXT("resolved geom colour reaches the MID"),
			BaseColor.Equals(FLinearColor(0.1f, 0.2f, 0.3f, 1.0f)));
		float Metallic = 0.0f;
		TestTrue(TEXT("metallic parameter is present"), Mid->GetScalarParameterValue(
			FMaterialParameterInfo(TEXT("Metallic")), Metallic));
		TestEqual(TEXT("resolved metallic reaches the MID"), Metallic, 0.25f);
		float Emission = 0.0f;
		TestTrue(TEXT("emission parameter is present"), Mid->GetScalarParameterValue(
			FMaterialParameterInfo(TEXT("Emission")), Emission));
		TestEqual(TEXT("resolved emission reaches the MID"), Emission, 3.0f);
		UTexture* ColourTexture = nullptr;
		UTexture* NormalTexture = nullptr;
		UTexture* EmissiveTexture = nullptr;
		Mid->GetTextureParameterValue(FName(TEXT("RgbTexture")), ColourTexture);
		Mid->GetTextureParameterValue(FName(TEXT("NormalTexture")), NormalTexture);
		Mid->GetTextureParameterValue(FName(TEXT("EmissiveTexture")), EmissiveTexture);
		TestNotNull(TEXT("colour role texture is bound"), ColourTexture);
		TestNotNull(TEXT("normal role texture is bound"), NormalTexture);
		TestNotNull(TEXT("neutral emissive texture is bound"), EmissiveTexture);
		TestEqual(TEXT("emissive slot uses the canonical neutral white texture"),
			EmissiveTexture, static_cast<UTexture*>(
				RenderNeutralTexture(ERenderMaterialRole::Emissive)));
		UTexture2D* LinearWhite = RenderNeutralTexture(ERenderMaterialRole::Orm);
		TestNotNull(TEXT("linear data roles have a neutral white texture"), LinearWhite);
		if (LinearWhite)
		{
			TestFalse(TEXT("linear neutral white disables sRGB decoding"), LinearWhite->SRGB);
			TestEqual(TEXT("linear neutral white uses default data compression"),
				LinearWhite->CompressionSettings, TC_Default);
			TestEqual(TEXT("all linear data roles share the neutral identity texture"),
				RenderNeutralTexture(ERenderMaterialRole::Roughness), LinearWhite);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetBuilderAtomicResources,
	"URLab.Render.AssetBuilder_InvalidCandidatePreservesLiveBundle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAssetBuilderAtomicResources::RunTest(const FString& Parameters)
{
	FString Error;
	URenderAssetBuilder* Builder = NewObject<URenderAssetBuilder>();
	FRenderSceneResources Live = RenderBuilderTest::MakeNeutralResources(TEXT("live-revision"));
	const FString LiveHash = Live.ContentHash();
	if (!TestTrue(TEXT("live resources accepted"), Builder->InitResources(MoveTemp(Live), false, Error)))
	{
		AddError(Error);
		return false;
	}
	UTexture2D* LiveTexture = Builder->GetOrBuildTexture(0, true, false);

	FRenderSceneResources Invalid = RenderBuilderTest::MakeNeutralResources(TEXT("bad-revision"));
	Invalid.Meshes[0].Triangles[2] = 99;
	TestFalse(TEXT("invalid candidate rejected"), Builder->InitResources(MoveTemp(Invalid), false, Error));
	TestTrue(TEXT("rejection explains the bad index"), Error.Contains(TEXT("index")));
	TestEqual(TEXT("live revision survives rejection"), Builder->GetSceneRevision(), FString(TEXT("live-revision")));
	TestTrue(TEXT("live texture cache survives rejection"),
		Builder->GetOrBuildTexture(0, true, false) == LiveTexture);

	FRenderSceneResources SameContent = RenderBuilderTest::MakeNeutralResources(TEXT("another-revision"));
	TestEqual(TEXT("scene revision is excluded from persistent content identity"),
		SameContent.ContentHash(), LiveHash);
	SameContent.Geoms[0].Rgba.R = 0.9f;
	TestTrue(TEXT("surface bindings participate in persistent content identity"),
		SameContent.ContentHash() != LiveHash);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjResourceShimOwnsCopiedPools,
	"URLab.Render.ResourceShim_OwnsCopiedPools",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjResourceShimOwnsCopiedPools::RunTest(const FString& Parameters)
{
	FString Error;
	mjModel* Model = RenderBuilderTest::CompileFixture(Error);
	if (!TestNotNull(TEXT("compiled fixture"), Model))
	{
		AddError(Error);
		return false;
	}
	const int32 MaterialId = mj_name2id(Model, mjOBJ_MATERIAL, "mat");
	const int32 TextureId = mj_name2id(Model, mjOBJ_TEXTURE, "grid");
	if (!TestTrue(TEXT("fixture material and texture compiled"), MaterialId >= 0 && TextureId >= 0))
	{
		mj_deleteModel(Model);
		return false;
	}
	// Exercise the complete role table without adding file-backed test assets.
	for (int32 Role = 0; Role < static_cast<int32>(ERenderMaterialRole::Count); ++Role)
	{
		Model->mat_texid[MaterialId * mjNTEXROLE + Role + 1] = TextureId;
	}
	Model->mat_metallic[MaterialId] = -1.0f;
	Model->mat_roughness[MaterialId] = -1.0f;
	Model->mat_emission[MaterialId] = 0.0f;
	FRenderSceneResources Resources;
	const bool bFilled = MjRenderResourceShim::Fill(
		Resources, Model, TEXT("shim-revision"), Error);
	mj_deleteModel(Model);
	if (!TestTrue(TEXT("shim copied a valid complete bundle"), bFilled))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("mesh vertices remain after source model deletion"), Resources.Meshes[0].Positions.Num() >= 3);
	TestEqual(TEXT("geom binding copied"), Resources.Geoms.Num(), 1);
	TestTrue(TEXT("material binding copied"), Resources.Geoms[0].MaterialIndex >= 0);
	const FRenderMaterialResource& Material = Resources.Materials[MaterialId];
	TestTrue(TEXT("material RGBA takes precedence over geom RGBA"),
		Resources.Geoms[0].Rgba.Equals(Material.Rgba));
	for (int32 Role = 0; Role < static_cast<int32>(ERenderMaterialRole::Count); ++Role)
	{
		TestEqual(FString::Printf(TEXT("texture role %d copied"), Role),
			Material.Textures[Role].TextureIndex, TextureId);
	}
	TestEqual(TEXT("unset metallic with a map resolves to a neutral multiplier"), Material.Metallic, 1.0f);
	TestEqual(TEXT("unset roughness with a map resolves to a neutral multiplier"), Material.Roughness, 1.0f);
	TestEqual(TEXT("emissive map with zero scalar remains visible"), Material.Emission, 1.0f);

	URenderAssetBuilder* Builder = NewObject<URenderAssetBuilder>();
	TestTrue(TEXT("copied bundle installs without a model"),
		Builder->InitResources(MoveTemp(Resources), false, Error));
	TestNotNull(TEXT("copied mesh builds without a model"), Builder->GetOrBuildStaticMesh(0));
	return true;
}

#endif // WITH_EDITOR
