// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjMasterMaterial.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/Texture.h"
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionAdd.h"
#include "Materials/MaterialExpressionAppendVector.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionTextureCoordinate.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

#include "MuJoCo/Spec/MjAssetResolve.h"
#include "URLabEditorLogging.h"

namespace urlab::editor
{
namespace
{

const TCHAR* const kPackageName = TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master");
const TCHAR* const kAssetName = TEXT("M_MuJoCo_Master");
const TCHAR* const kNeutralWhite = TEXT("/Engine/EngineResources/WhiteSquareTexture.WhiteSquareTexture");
const TCHAR* const kNeutralNormal = TEXT("/Engine/EngineMaterials/FlatNormal.FlatNormal");

/** A node factory that also remembers where to put the next column. */
struct FGraphBuilder
{
	UMaterial* Material = nullptr;

	template <class T>
	T* Make(int32 X, int32 Y)
	{
		return Cast<T>(UMaterialEditingLibrary::CreateMaterialExpression(Material, T::StaticClass(), X, Y));
	}

	void Link(UMaterialExpression* From, const TCHAR* FromOutput, UMaterialExpression* To, const TCHAR* ToInput)
	{
		UMaterialEditingLibrary::ConnectMaterialExpressions(From, FromOutput, To, ToInput);
	}

	/** `A * B`, as a node. */
	UMaterialExpression* Mul(UMaterialExpression* A, const TCHAR* AOut, UMaterialExpression* B, const TCHAR* BOut,
		int32 X, int32 Y)
	{
		UMaterialExpressionMultiply* Node = Make<UMaterialExpressionMultiply>(X, Y);
		Link(A, AOut, Node, TEXT("A"));
		Link(B, BOut, Node, TEXT("B"));
		return Node;
	}
};

UMaterialExpressionScalarParameter* MakeScalar(FGraphBuilder& Graph, const TCHAR* Name, float Default, int32 X, int32 Y)
{
	UMaterialExpressionScalarParameter* Node = Graph.Make<UMaterialExpressionScalarParameter>(X, Y);
	Node->ParameterName = Name;
	Node->DefaultValue = Default;
	return Node;
}

/**
 * One always-sampled texture slot.
 *
 * The default is what makes "always sampled" harmless: white multiplies to
 * identity in every channel the graph uses it in, and a flat normal perturbs
 * nothing, so a material that fills none of its roles shades exactly as if the
 * slots were not there.
 */
UMaterialExpressionTextureSampleParameter2D* MakeSampler(
	FGraphBuilder& Graph, EMjMaterialRole Role, UMaterialExpression* Uv, int32 X, int32 Y)
{
	const bool bNormal = Role == EMjMaterialRole::Normal;
	UMaterialExpressionTextureSampleParameter2D* Node = Graph.Make<UMaterialExpressionTextureSampleParameter2D>(X, Y);
	Node->ParameterName = MjMaterialRoleParameter(Role);
	Node->Texture = LoadObject<UTexture>(nullptr, bNormal ? kNeutralNormal : kNeutralWhite);
	Node->SamplerType = bNormal ? SAMPLERTYPE_Normal : SAMPLERTYPE_Color;
	// The first input rather than a name: a texture sample calls its UV
	// input "Coordinates", and matching that by name is a spelling this file
	// should not have to know.
	Graph.Link(Uv, TEXT(""), Node, TEXT(""));
	return Node;
}

/** Load the existing master, or make the package and the asset for a new one. */
UMaterial* OpenOrCreate()
{
	if (UMaterial* Existing = LoadObject<UMaterial>(nullptr, kPackageName, nullptr, LOAD_NoWarn | LOAD_Quiet))
	{
		return Existing;
	}
	UPackage* Package = CreatePackage(kPackageName);
	if (Package == nullptr)
	{
		return nullptr;
	}
	Package->FullyLoad();
	UMaterial* Material = NewObject<UMaterial>(Package, FName(kAssetName), RF_Public | RF_Standalone);
	if (Material != nullptr)
	{
		FAssetRegistryModule::AssetCreated(Material);
	}
	return Material;
}

} // namespace

UMaterial* BuildMuJoCoMasterMaterial(bool bSave)
{
	UMaterial* Material = OpenOrCreate();
	if (Material == nullptr)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Could not open or create %s"), kPackageName);
		return nullptr;
	}

	// Wholesale replacement rather than a patch: the graph is defined here and
	// nowhere else, so whatever the asset held before is not worth merging with.
	// Deleting through the library is what breaks the property inputs too.
	const TArray<TObjectPtr<UMaterialExpression>> Existing = Material->GetExpressionCollection().Expressions;
	for (UMaterialExpression* Expression : Existing)
	{
		if (Expression != nullptr)
		{
			UMaterialEditingLibrary::DeleteMaterialExpression(Material, Expression);
		}
	}

	FGraphBuilder Graph{Material};

	// --- UVs ---------------------------------------------------------------- //
	//
	// MuJoCo scales texture coordinates by the material's `texrepeat`, and by
	// the geom's size as well when `texuniform` is set. The resolver folds both
	// into these two scalars, because how many times an image repeats across an
	// object is a per-geom answer and a MID is per geom.
	UMaterialExpressionTextureCoordinate* TexCoord = Graph.Make<UMaterialExpressionTextureCoordinate>(-1600, 0);
	UMaterialExpressionScalarParameter* RepeatU = MakeScalar(Graph, TEXT("TexRepeatU"), 1.0f, -1600, 150);
	UMaterialExpressionScalarParameter* RepeatV = MakeScalar(Graph, TEXT("TexRepeatV"), 1.0f, -1600, 250);
	UMaterialExpressionAppendVector* Repeat = Graph.Make<UMaterialExpressionAppendVector>(-1400, 200);
	Graph.Link(RepeatU, TEXT(""), Repeat, TEXT("A"));
	Graph.Link(RepeatV, TEXT(""), Repeat, TEXT("B"));
	UMaterialExpression* Uv = Graph.Mul(TexCoord, TEXT(""), Repeat, TEXT(""), -1200, 100);

	// --- Slots -------------------------------------------------------------- //
	int32 Row = -600;
	auto Slot = [&](EMjMaterialRole Role) {
		UMaterialExpressionTextureSampleParameter2D* Node = MakeSampler(Graph, Role, Uv, -900, Row);
		Row += 300;
		return Node;
	};
	UMaterialExpressionTextureSampleParameter2D* Rgb = Slot(EMjMaterialRole::Rgb);
	UMaterialExpressionTextureSampleParameter2D* Rgba = Slot(EMjMaterialRole::Rgba);
	UMaterialExpressionTextureSampleParameter2D* Normal = Slot(EMjMaterialRole::Normal);
	UMaterialExpressionTextureSampleParameter2D* Orm = Slot(EMjMaterialRole::Orm);
	UMaterialExpressionTextureSampleParameter2D* Occlusion = Slot(EMjMaterialRole::Occlusion);
	UMaterialExpressionTextureSampleParameter2D* Roughness = Slot(EMjMaterialRole::Roughness);
	UMaterialExpressionTextureSampleParameter2D* Metallic = Slot(EMjMaterialRole::Metallic);
	UMaterialExpressionTextureSampleParameter2D* Opacity = Slot(EMjMaterialRole::Opacity);
	UMaterialExpressionTextureSampleParameter2D* Emissive = Slot(EMjMaterialRole::Emissive);

	// --- Scalars ------------------------------------------------------------ //
	UMaterialExpressionVectorParameter* BaseColor = Graph.Make<UMaterialExpressionVectorParameter>(-900, Row);
	BaseColor->ParameterName = TEXT("BaseColor");
	BaseColor->DefaultValue = FLinearColor::White;

	UMaterialExpressionScalarParameter* MetallicValue = MakeScalar(Graph, TEXT("Metallic"), 0.0f, -900, Row + 200);
	UMaterialExpressionScalarParameter* RoughnessValue = MakeScalar(Graph, TEXT("Roughness"), 0.5f, -900, Row + 300);
	UMaterialExpressionScalarParameter* SpecularValue = MakeScalar(Graph, TEXT("Specular"), 0.5f, -900, Row + 400);
	UMaterialExpressionScalarParameter* ReflectanceValue =
		MakeScalar(Graph, TEXT("Reflectance"), 0.0f, -900, Row + 500);
	UMaterialExpressionScalarParameter* EmissionValue = MakeScalar(Graph, TEXT("Emission"), 0.0f, -900, Row + 600);

	// --- Outputs ------------------------------------------------------------ //
	//
	// `rgb` and `rgba` both drive base colour and both default to white, so a
	// material that fills either one gets it and a material that fills neither
	// gets its flat colour. `rgba` additionally carries alpha; `rgb` does not.
	UMaterialExpression* Tinted = Graph.Mul(BaseColor, TEXT("RGB"), Rgb, TEXT("RGB"), -600, -500);
	UMaterialExpression* Albedo = Graph.Mul(Tinted, TEXT(""), Rgba, TEXT("RGB"), -400, -500);
	UMaterialEditingLibrary::ConnectMaterialProperty(Albedo, TEXT(""), MP_BaseColor);

	// ORM is the packed workflow: occlusion in R, roughness in G, metallic in B.
	// The separate single-channel roles multiply into the same outputs, so a
	// material may use either spelling, or both.
	UMaterialExpression* MetallicMul = Graph.Mul(MetallicValue, TEXT(""), Metallic, TEXT("R"), -600, -200);
	UMaterialExpression* MetallicOut = Graph.Mul(MetallicMul, TEXT(""), Orm, TEXT("B"), -400, -200);
	UMaterialEditingLibrary::ConnectMaterialProperty(MetallicOut, TEXT(""), MP_Metallic);

	UMaterialExpression* RoughnessMul = Graph.Mul(RoughnessValue, TEXT(""), Roughness, TEXT("R"), -600, 0);
	UMaterialExpression* RoughnessOut = Graph.Mul(RoughnessMul, TEXT(""), Orm, TEXT("G"), -400, 0);
	UMaterialEditingLibrary::ConnectMaterialProperty(RoughnessOut, TEXT(""), MP_Roughness);

	UMaterialExpression* OcclusionOut = Graph.Mul(Occlusion, TEXT("R"), Orm, TEXT("R"), -400, 200);
	UMaterialEditingLibrary::ConnectMaterialProperty(OcclusionOut, TEXT(""), MP_AmbientOcclusion);

	// MuJoCo's `reflectance` has no separate Unreal channel; it is the same
	// "how mirror-like is this" knob that `specular` is, so the two add.
	UMaterialExpressionAdd* SpecularOut = Graph.Make<UMaterialExpressionAdd>(-400, 400);
	Graph.Link(SpecularValue, TEXT(""), SpecularOut, TEXT("A"));
	Graph.Link(ReflectanceValue, TEXT(""), SpecularOut, TEXT("B"));
	UMaterialEditingLibrary::ConnectMaterialProperty(SpecularOut, TEXT(""), MP_Specular);

	UMaterialEditingLibrary::ConnectMaterialProperty(Normal, TEXT("RGB"), MP_Normal);

	UMaterialExpression* EmissiveTint = Graph.Mul(BaseColor, TEXT("RGB"), Emissive, TEXT("RGB"), -600, 600);
	UMaterialExpression* EmissiveOut = Graph.Mul(EmissiveTint, TEXT(""), EmissionValue, TEXT(""), -400, 600);
	UMaterialEditingLibrary::ConnectMaterialProperty(EmissiveOut, TEXT(""), MP_EmissiveColor);

	// Wired, and unused while the blend mode is opaque: the engine drops the
	// samples. It stays connected so that switching this material to a
	// translucent blend mode is the only edit that transparency would need.
	UMaterialExpression* AlphaMul = Graph.Mul(BaseColor, TEXT("A"), Rgba, TEXT("A"), -600, 800);
	UMaterialExpression* OpacityOut = Graph.Mul(AlphaMul, TEXT(""), Opacity, TEXT("R"), -400, 800);
	UMaterialEditingLibrary::ConnectMaterialProperty(OpacityOut, TEXT(""), MP_Opacity);

	UMaterialEditingLibrary::RecompileMaterial(Material);

	if (bSave)
	{
		UPackage* Package = Material->GetPackage();
		Package->MarkPackageDirty();
		const FString FileName =
			FPackageName::LongPackageNameToFilename(kPackageName, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		if (!UPackage::SavePackage(Package, Material, *FileName, SaveArgs))
		{
			UE_LOG(LogURLabEditor, Error, TEXT("Could not save %s"), *FileName);
			return nullptr;
		}
		UE_LOG(LogURLabEditor, Display, TEXT("Wrote %s"), *FileName);
	}
	return Material;
}

} // namespace urlab::editor

int32 UMjBuildMasterMaterialCommandlet::Main(const FString& Params)
{
	return urlab::editor::BuildMuJoCoMasterMaterial(/*bSave=*/true) != nullptr ? 0 : 1;
}
