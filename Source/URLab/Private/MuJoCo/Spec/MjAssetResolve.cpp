// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjAssetResolve.h"

#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Gen/Elements/Assets/MjMaterial.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjMaterialLayer.gen.h"
#endif

#if WITH_EDITOR
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#endif

namespace
{

/** One row per role: the MJCF token and the master material's parameter. */
struct FMjRoleRow
{
	const TCHAR* Token;
	const TCHAR* Parameter;
};

// Indexed by EMjMaterialRole, in mjtTextureRole order minus the nameless
// `user` role. The parameter names are the master material's contract and are
// asserted by the tests; renaming one here without rebuilding the master
// silently drops that role.
constexpr FMjRoleRow GRoles[] = {
	/* rgb       */ {TEXT("rgb"), TEXT("RgbTexture")},
	/* occlusion */ {TEXT("occlusion"), TEXT("OcclusionTexture")},
	/* roughness */ {TEXT("roughness"), TEXT("RoughnessTexture")},
	/* metallic  */ {TEXT("metallic"), TEXT("MetallicTexture")},
	/* normal    */ {TEXT("normal"), TEXT("NormalTexture")},
	/* opacity   */ {TEXT("opacity"), TEXT("OpacityTexture")},
	/* emissive  */ {TEXT("emissive"), TEXT("EmissiveTexture")},
	/* rgba      */ {TEXT("rgba"), TEXT("RgbaTexture")},
	/* orm       */ {TEXT("orm"), TEXT("OrmTexture")},
};

static_assert(UE_ARRAY_COUNT(GRoles) == static_cast<int32>(EMjMaterialRole::Count),
	"GRoles is indexed by EMjMaterialRole and must have a row for every role");

/** The image a slot samples when the material fills it with nothing. */
const TCHAR* const kNeutralWhite = TEXT("/Engine/EngineResources/WhiteSquareTexture.WhiteSquareTexture");
const TCHAR* const kNeutralNormal = TEXT("/Engine/EngineMaterials/FlatNormal.FlatNormal");

const TCHAR* const kMasterMaterial = TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master.M_MuJoCo_Master");

#if URLAB_MJ_GEN

/**
 * Every element of the spec, template graph or actor components alike.
 *
 * The two graphs hold their elements in different places and neither is a tree
 * walk: a reference resolves by name across the whole spec exactly as MuJoCo
 * resolves it.
 */
template <class Visit>
void ForEachElement(const FSpecRef& Doc, Visit&& Visitor)
{
#if WITH_EDITOR
	if (UBlueprint* Blueprint = Doc.GetBlueprint())
	{
		if (Blueprint->SimpleConstructionScript != nullptr)
		{
			for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
			{
				if (Node != nullptr && Node->ComponentTemplate != nullptr)
				{
					Visitor(Node->ComponentTemplate);
				}
			}
		}
		return;
	}
#endif
	if (AActor* Actor = Doc.GetActor())
	{
		for (UActorComponent* Component : Actor->GetComponents())
		{
			Visitor(Component);
		}
	}
}

/** The name namespaces MuJoCo keeps apart: a mesh and a material may share one. */
enum class ENameKind : int32
{
	Material = 0,
	Texture = 1,
	Mesh = 2,
};

/**
 * `Name` in the namespace `Kind`, through the open pass index when there is one.
 *
 * Without a scope this is the scan it always was. With one, the spec is walked
 * once per kind per pass rather than once per query -- which is what turns
 * resolving a few hundred geoms' pictures from quadratic into linear.
 */
template <class T>
T* FindNamedElement(const FSpecRef& Doc, ENameKind Kind, const FString& Name,
	TFunctionRef<FString(const UMjNodeComponent&)> NameOf)
{
	if (Name.IsEmpty())
	{
		return nullptr;
	}
	auto Match = [&NameOf, &Name](UActorComponent* Component) -> T* {
		T* Element = Cast<T>(Component);
		return (Element != nullptr && NameOf(*Element) == Name) ? Element : nullptr;
	};

	const UMjModel* Root = Cast<UMjModel>(Doc.GetRoot());
	if (Root != nullptr)
	{
		if (urlab::spec::FMjEffectiveScope* Scope = urlab::spec::FMjEffectiveScope::Find(Root))
		{
			UActorComponent* Found = Scope->FindNamed(static_cast<int32>(Kind), Name, [&Doc, &NameOf](auto&& Add) {
				ForEachElement(Doc, [&NameOf, &Add](UActorComponent* Component) {
					if (T* Element = Cast<T>(Component))
					{
						Add(NameOf(*Element), Component);
					}
				});
			});
			return Cast<T>(Found);
		}
	}

	T* Result = nullptr;
	ForEachElement(Doc, [&Match, &Result](UActorComponent* Component) {
		if (Result == nullptr)
		{
			if (T* Found = Match(Component))
			{
				Result = Found;
			}
		}
	});
	return Result;
}

/**
 * The `<material>` named `Name` in `Doc`.
 *
 * Over the graph's own flat store -- the Blueprint's construction-script nodes,
 * or the actor's components -- rather than by walking the element tree. A
 * material reference is resolved by name across the whole spec exactly as
 * MuJoCo resolves it, so the tree walk would only be a longer way to the same
 * answer, and it would need an ambient tree scope on a path that has no other
 * reason to open one.
 */
UMjMaterial* FindMaterialElement(const FSpecRef& Doc, const FString& Name)
{
	return FindNamedElement<UMjMaterial>(Doc, ENameKind::Material, Name,
		[](const UMjNodeComponent& Element) { return Element.MjName.IsSet() ? Element.MjName.GetValue() : FString(); });
}

/**
 * The `<texture>` element `Name` refers to, or null when nothing does.
 *
 * The last of the four hops, and where the image comes from: the element holds
 * the asset the import produced, so a layer naming a texture the spec does
 * not have resolves to nothing rather than to whatever an earlier import left
 * lying around under a matching name.
 */
UMjTexture* FindTextureElement(const FSpecRef& Doc, const FString& Name)
{
	return FindNamedElement<UMjTexture>(Doc, ENameKind::Texture, Name,
		[](const UMjNodeComponent& Element) { return MjAssetElementName(Element); });
}

/** The `<mesh>` element `Name` refers to, by the same rule as a texture. */
const UMjMesh* FindMeshElement(const FSpecRef& Doc, const FString& Name)
{
	return FindNamedElement<UMjMesh>(Doc, ENameKind::Mesh, Name,
		[](const UMjNodeComponent& Element) { return MjAssetElementName(Element); });
}

/**
 * The `<layer>` children of `Material`, or of the nearest class that has any.
 *
 * A material inherits its layers wholesale rather than role by role: MJCF has
 * no way to say "this class's normal map but that class's base colour", and a
 * partial that authors any layer at all is the one the compiler folds in.
 */
TArray<UMjMaterialLayer*> LayersOf(const FSpecRef& Doc, const UMjMaterial& Material)
{
	TArray<UMjMaterialLayer*> Out;
	auto Gather = [&Doc, &Out](const UMjMaterial& From) {
		Out.Reset();
		for (const urlab::spec::FMjOrderedChild& Child :
			urlab::spec::MjOrderedChildrenOf(Doc, const_cast<UMjMaterial&>(From)))
		{
			if (UMjMaterialLayer* Layer = Cast<UMjMaterialLayer>(Child.Node))
			{
				Out.Add(Layer);
			}
		}
		return Out.Num() > 0;
	};

	if (Gather(Material))
	{
		return Out;
	}
	urlab::spec::WithEffectiveDoc(Material, [&](auto& Effective) {
		Effective.ForEachLayer(Material, [&](const auto& Partial) { return Gather(Partial); });
	});
	return Out;
}

#endif  // URLAB_MJ_GEN

/** A texture, or the neutral stand-in for its role when there is none. */
UTexture2D* TextureOrNeutral(UTexture2D* Texture, EMjMaterialRole Role)
{
	if (Texture != nullptr)
	{
		return Texture;
	}
	const TCHAR* Path = Role == EMjMaterialRole::Normal ? kNeutralNormal : kNeutralWhite;
	return LoadObject<UTexture2D>(nullptr, Path);
}

}  // namespace

const TCHAR* MjMaterialRoleName(EMjMaterialRole Role)
{
	const int32 Index = static_cast<int32>(Role);
	return Index >= 0 && Index < static_cast<int32>(EMjMaterialRole::Count) ? GRoles[Index].Token : TEXT("");
}

const TCHAR* MjMaterialRoleParameter(EMjMaterialRole Role)
{
	const int32 Index = static_cast<int32>(Role);
	return Index >= 0 && Index < static_cast<int32>(EMjMaterialRole::Count) ? GRoles[Index].Parameter : TEXT("");
}

EMjMaterialRole MjMaterialRoleFromName(const FString& Name)
{
	for (int32 Index = 0; Index < static_cast<int32>(EMjMaterialRole::Count); ++Index)
	{
		if (Name.Equals(GRoles[Index].Token, ESearchCase::IgnoreCase))
		{
			return static_cast<EMjMaterialRole>(Index);
		}
	}
	return EMjMaterialRole::Count;
}

float MjRoughnessFor(const FMjMaterialValues& Material)
{
	// MuJoCo's -1 means "the material does not say". Filament reads `shininess`
	// as glossiness in that case; Unreal's roughness is its complement.
	const float Value = Material.Roughness >= 0.0f ? Material.Roughness : 1.0f - Material.Shininess;
	return FMath::Clamp(Value, 0.0f, 1.0f);
}

float MjMetallicFor(const FMjMaterialValues& Material)
{
	return FMath::Clamp(Material.Metallic >= 0.0f ? Material.Metallic : 0.0f, 0.0f, 1.0f);
}

FString MjSanitizeAssetName(const FString& Name)
{
	FString Out;
	Out.Reserve(Name.Len());
	for (const TCHAR Character : Name)
	{
		const bool bKeep = (Character >= TEXT('0') && Character <= TEXT('9')) ||
			(Character >= TEXT('A') && Character <= TEXT('Z')) || (Character >= TEXT('a') && Character <= TEXT('z')) ||
			Character == TEXT('_');
		Out.AppendChar(bKeep ? Character : TEXT('_'));
	}
	return Out;
}

FString MjImportedAssetPath(const FSpecRef& Spec)
{
	const UMjNodeComponent* Root = Spec.GetRoot();
	FString Stem = Root != nullptr ? FPaths::GetBaseFilename(Root->SourceFile) : FString();
	if (Stem.IsEmpty())
	{
		// The name the import pass gives a spec that came from no file.
		Stem = TEXT("MemModel");
	}
	return FString::Printf(TEXT("/Game/MuJoCoImports/%s_Assets"), *MjSanitizeAssetName(Stem));
}

UTexture2D* MjResolveTexture(const FSpecRef& Spec, const FString& Name)
{
	if (Name.IsEmpty() || !Spec.IsValid())
	{
		return nullptr;
	}
#if URLAB_MJ_GEN
	const UMjTexture* Element = FindTextureElement(Spec, Name);
	return Element != nullptr ? Element->TextureAsset.Get() : nullptr;
#else
	return nullptr;
#endif
}

FString MjImportedMeshPath(const FSpecRef& Spec)
{
	// The subfolder the import pass writes meshes into. Textures sit beside it
	// rather than under one of their own, which is the layout that already
	// exists; both are stated once, here.
	return MjImportedAssetPath(Spec) / TEXT("Meshes");
}

FMjResolvedMesh MjResolveMesh(const FSpecRef& Spec, const FString& Name)
{
	FMjResolvedMesh Out;
	if (Name.IsEmpty() || !Spec.IsValid())
	{
		return Out;
	}
#if URLAB_MJ_GEN
	const UMjMesh* Element = FindMeshElement(Spec, Name);
	if (Element == nullptr)
	{
		return Out;
	}

	FMjVec3 Scale(1.0, 1.0, 1.0);
	bool bScaleSet = Element->Scale.IsSet();
	if (bScaleSet)
	{
		Scale = Element->Scale.GetValue();
	}
	else
	{
		urlab::spec::WithEffectiveDoc(*Element, [&](auto& Effective) {
			// The generated base: the class chain is over schema elements, and a
			// hand subclass is not one of those.
			Effective.ForEachLayer(static_cast<const UMjMeshBase&>(*Element), [&](const auto& Layer) {
				if (!Layer.Scale.IsSet())
				{
					return false;
				}
				Scale = Layer.Scale.GetValue();
				return true;
			});
		});
	}
	// `<mesh scale>` only. The import already put the asset in Unreal's units --
	// every mesh goes through the GLB conversion, and Interchange applies its own
	// metre-to-centimetre factor (GltfUnitConversionMultiplier) on the way in --
	// so applying one here as well scales the model by a hundred.
	Out.Scale = FVector(Scale.X, Scale.Y, Scale.Z);
	Out.Asset = Element->MeshAsset.Get();
#endif
	return Out;
}

UMaterialInterface* MjLoadMasterMaterial()
{
	return LoadObject<UMaterialInterface>(nullptr, kMasterMaterial);
}

bool MjResolveMaterial(const FSpecRef& Spec, const FString& Name, FMjMaterialValues& Out)
{
	Out = FMjMaterialValues();
#if URLAB_MJ_GEN
	UMjMaterial* Material = FindMaterialElement(Spec, Name);
	if (Material == nullptr)
	{
		return false;
	}
	Out.bFound = true;

	// One walk of the class chain for every attribute, taking each from the
	// nearest layer that authored it. The element itself is the first layer,
	// so an authored value never reaches the chain at all.
	auto Take = [](auto& Slot, const auto& Field, bool& bDone) {
		if (!bDone && Field.IsSet())
		{
			Slot = Field.GetValue();
			bDone = true;
		}
	};

	bool bRgba = false, bEmission = false, bSpecular = false, bShininess = false, bReflectance = false;
	bool bMetallic = false, bRoughness = false, bTexRepeat = false, bTexUniform = false;
	urlab::spec::WithEffectiveDoc(*Material, [&](auto& Effective) {
		Effective.ForEachLayer(*Material, [&](const auto& Layer) {
			Take(Out.Rgba, Layer.Rgba, bRgba);
			Take(Out.Emission, Layer.Emission, bEmission);
			Take(Out.Specular, Layer.Specular, bSpecular);
			Take(Out.Shininess, Layer.Shininess, bShininess);
			Take(Out.Reflectance, Layer.Reflectance, bReflectance);
			Take(Out.Metallic, Layer.Metallic, bMetallic);
			Take(Out.Roughness, Layer.Roughness, bRoughness);
			if (!bTexRepeat && Layer.Texrepeat.IsSet())
			{
				const TArray<float>& Values = Layer.Texrepeat.GetValue();
				if (Values.Num() >= 2)
				{
					Out.TexRepeat = FVector2D(Values[0], Values[1]);
					bTexRepeat = true;
				}
			}
			Take(Out.bTexUniform, Layer.Texuniform, bTexUniform);
			return bRgba && bEmission && bSpecular && bShininess && bReflectance && bMetallic && bRoughness &&
				bTexRepeat && bTexUniform;
		});
	});

	for (const UMjMaterialLayer* Layer : LayersOf(Spec, *Material))
	{
		if (Layer == nullptr || !Layer->Texture.IsSet())
		{
			continue;
		}
		const EMjMaterialRole Role = MjMaterialRoleFromName(Layer->Role);
		if (Role == EMjMaterialRole::Count)
		{
			continue;
		}
		Out.TextureNames[static_cast<int32>(Role)] = Layer->Texture.GetValue();
	}
	return true;
#else
	(void)Spec;
	(void)Name;
	return false;
#endif
}

void MjApplyMaterialParameters(UMaterialInstanceDynamic& Instance, const FMjMaterialValues& Material,
	const FLinearColor& BaseColor, const FSpecRef& Spec, const FVector2D& GeomSize)
{
	Instance.SetVectorParameterValue(TEXT("BaseColor"), BaseColor);
	Instance.SetScalarParameterValue(TEXT("Metallic"), MjMetallicFor(Material));
	Instance.SetScalarParameterValue(TEXT("Roughness"), MjRoughnessFor(Material));
	Instance.SetScalarParameterValue(TEXT("Specular"), FMath::Clamp(Material.Specular, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Reflectance"), FMath::Clamp(Material.Reflectance, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Emission"), FMath::Max(Material.Emission, 0.0f));

	// `settexture` in render_gl3.c: the repeat count is per object, unless the
	// material asks for it per spatial unit, in which case the geom's own size
	// multiplies it. A zero size is skipped there and skipped here.
	FVector2D Repeat = Material.TexRepeat;
	if (Material.bTexUniform)
	{
		if (GeomSize.X > 0.0)
		{
			Repeat.X *= GeomSize.X;
		}
		if (GeomSize.Y > 0.0)
		{
			Repeat.Y *= GeomSize.Y;
		}
	}
	Instance.SetScalarParameterValue(TEXT("TexRepeatU"), static_cast<float>(Repeat.X));
	Instance.SetScalarParameterValue(TEXT("TexRepeatV"), static_cast<float>(Repeat.Y));

	for (int32 Index = 0; Index < static_cast<int32>(EMjMaterialRole::Count); ++Index)
	{
		const EMjMaterialRole Role = static_cast<EMjMaterialRole>(Index);
		UTexture2D* Texture = MjResolveTexture(Spec, Material.TextureNames[Index]);
		Instance.SetTextureParameterValue(MjMaterialRoleParameter(Role), TextureOrNeutral(Texture, Role));
	}
}
