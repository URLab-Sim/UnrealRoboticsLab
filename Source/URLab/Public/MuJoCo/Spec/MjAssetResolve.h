// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What a `<material>` means, as parameters a material instance can carry.
//
// MJCF spreads one appearance over four elements. A geom names a material, the
// material carries the shading scalars, its `<layer>` children each name a
// texture and the role that texture plays, and the `<texture>` element is the
// image itself. Every hop but the last is defaultable, so in a menagerie model
// none of it is usually on the geom: MuJoCo's humanoid puts `material="body"`
// on a default class and the colour and the texture on the material that names.
//
// Two spellings reach the same place. `<material texture="x">` is the
// single-texture shorthand and the reader folds it into a `<layer texture="x"
// role="rgb"/>` child before the spec exists, so a consumer only ever sees
// layers -- which is why `UMjMaterial` has no texture field of its own.
//
// Nothing here draws. It resolves the spec and writes parameters onto a
// UMaterialInstanceDynamic, so the same resolution serves the editor preview
// today and a runtime scene builder later; the master material is entirely
// parameter-driven precisely so that one MID can express any material.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

class UMaterialInstanceDynamic;
class UMjNodeComponent;
class UStaticMesh;
class UTexture2D;
struct FSpecRef;

/**
 * The texture roles MJCF declares, in `mjtTextureRole` order.
 *
 * `Count` is the array bound, not a role. `User` (role 0 in the C enum) has no
 * MJCF spelling and no slot here.
 */
enum class EMjMaterialRole : uint8
{
	Rgb = 0,
	Occlusion,
	Roughness,
	Metallic,
	Normal,
	Opacity,
	Emissive,
	Rgba,
	Orm,
	Count
};

/** The MJCF `role` token for a slot, e.g. "rgb". */
URLAB_API const TCHAR* MjMaterialRoleName(EMjMaterialRole Role);

/** The slot an MJCF `role` token names, or `Count` when it names none. */
URLAB_API EMjMaterialRole MjMaterialRoleFromName(const FString& Name);

/** The master material's texture parameter for a slot, e.g. "RgbTexture". */
URLAB_API const TCHAR* MjMaterialRoleParameter(EMjMaterialRole Role);

/**
 * One `<material>`, with its default-class chain resolved.
 *
 * Every field is the value MuJoCo's compiler would store: authored where the
 * element authored it, inherited where a `<default>` supplied it, and the
 * schema default otherwise. `Metallic` and `Roughness` keep MuJoCo's -1
 * "unset" sentinel rather than folding it, because what -1 means is the
 * renderer's decision and not the spec's.
 */
struct URLAB_API FMjMaterialValues
{
	/** True when a `<material>` of that name was found at all. */
	bool bFound = false;

	FLinearColor Rgba = FLinearColor(1.0f, 1.0f, 1.0f, 1.0f);
	float Emission = 0.0f;
	float Specular = 0.5f;
	float Shininess = 0.5f;
	float Reflectance = 0.0f;

	/** -1 when the material does not author it; see MjRoughnessFor / MjMetallicFor. */
	float Metallic = -1.0f;
	float Roughness = -1.0f;

	FVector2D TexRepeat = FVector2D(1.0, 1.0);
	bool bTexUniform = false;

	/** The MJCF name of the `<texture>` each role's `<layer>` points at. */
	FString TextureNames[static_cast<int32>(EMjMaterialRole::Count)];

	/** The `<texture>` name for `Role`, or empty when no layer fills it. */
	const FString& TextureFor(EMjMaterialRole Role) const
	{
		return TextureNames[static_cast<int32>(Role)];
	}
};

/**
 * The `<material>` named `Name` in `Spec`, resolved.
 *
 * Lookup is by name across the whole spec, exactly as MuJoCo resolves a
 * material reference, so it goes over the graph's flat store rather than the
 * element tree. False when the spec holds no such material, in which case
 * `Out` is left at the schema defaults.
 */
URLAB_API bool MjResolveMaterial(const FSpecRef& Spec, const FString& Name, FMjMaterialValues& Out);

/**
 * MuJoCo's -1 sentinels folded into values Unreal can shade with.
 *
 * `roughness` unset falls back to `1 - shininess`, which is the glossiness
 * MuJoCo's Filament renderer reads (`model_renderables.cc`) inverted into
 * Unreal's roughness; `metallic` unset is simply not metal.
 */
URLAB_API float MjRoughnessFor(const FMjMaterialValues& Material);
URLAB_API float MjMetallicFor(const FMjMaterialValues& Material);

/**
 * Where a spec's imported assets live: `/Game/MuJoCoImports/<stem>_Assets`.
 *
 * The stem is the model file the spec's root element was read from, so an
 * import and a later preview agree on the folder without either being told.
 */
URLAB_API FString MjImportedAssetPath(const FSpecRef& Spec);

/** An MJCF name reduced to characters a package path admits. */
URLAB_API FString MjSanitizeAssetName(const FString& Name);

/**
 * The image the `<texture>` named `Name` references.
 *
 * The element's own reference, not a package path built back up out of the
 * name: where Unreal put an imported asset depends on which factory it chose,
 * so the only account of it that is always right is the one the importer wrote
 * down. Null when the spec has no such texture, when the model was never
 * imported, or when the element names a file nobody could read.
 */
URLAB_API UTexture2D* MjResolveTexture(const FSpecRef& Spec, const FString& Name);

/** Where a spec's imported meshes live, below `MjImportedAssetPath`. */
URLAB_API FString MjImportedMeshPath(const FSpecRef& Spec);

/**
 * Where a spec's convex-decomposition hulls live, beside its imported meshes.
 *
 * Their own folder because of where they come FROM, not what they are: to the
 * compiled model a hull is an ordinary `<mesh>`, but on disk it is generated
 * output that a re-decomposition replaces wholesale. Keeping them apart from
 * the meshes the document shipped with means the generated ones can be
 * recognised, and deleted, without a name convention deciding it.
 */
URLAB_API FString MjDecomposedMeshPath(const FSpecRef& Spec);

/** One `<mesh>`, resolved to the Unreal asset and the transform it needs. */
struct URLAB_API FMjResolvedMesh
{
	/** The element's own reference, or null when the model was never imported. */
	UStaticMesh* Asset = nullptr;

	/**
	 * The component scale that asset needs: MJCF `<mesh scale>`, and only that.
	 *
	 * MuJoCo applies `<mesh scale>` to the vertices at compile time and Unreal's
	 * importer does not, so the preview has to. Units are not this scale's job --
	 * the import pass converts every mesh to glTF and Interchange applies its own
	 * metre-to-centimetre factor, so the asset is already in the level's units by
	 * the time anything here sees it.
	 */
	FVector Scale = FVector::OneVector;
};

/**
 * The `<mesh>` named `Name`, resolved through its own default-class chain.
 *
 * The name is the element's, not the file's: MJCF lets `<mesh name="tabletop"
 * file="table.obj"/>` differ and a geom refers to the element.
 */
URLAB_API FMjResolvedMesh MjResolveMesh(const FSpecRef& Spec, const FString& Name);

/**
 * Write `Material` onto `Instance`, textures included.
 *
 * `BaseColor` is passed rather than read off `Material` because which of the
 * geom's `rgba` and the material's wins is the renderer's precedence rule and
 * belongs to the caller that knows both (see `UMjGeom::GetEffectiveColor`).
 *
 * `GeomSize` is the geom's MJCF `size` in metres, which decides the texture
 * scale when the material asks for `texuniform`: MuJoCo repeats the image once
 * per spatial unit rather than once per object (`render_gl3.c settexture`).
 * Pass a zero size when the caller has none; the uniform term is then dropped.
 *
 * Every parameter the master declares is written on every call, including the
 * ones this material does not use, because a MID is reused across edits and a
 * parameter left alone keeps the value the previous material gave it.
 */
URLAB_API void MjApplyMaterialParameters(UMaterialInstanceDynamic& Instance, const FMjMaterialValues& Material,
	const FLinearColor& BaseColor, const FSpecRef& Spec, const FVector2D& GeomSize);

/**
 * Bind every texture slot the master declares to its neutral stand-in (white,
 * or flat-normal for the normal role) and reset the repeat to 1.
 *
 * For callers that drive the master by BaseColor + scalar PBR terms only and
 * carry no textures (the MJB fast path). A slot left unbound keeps the master's
 * editor-default texture, which tints or darkens the result; this neutralises
 * all of them so the colour and scalars are the only inputs.
 */
URLAB_API void MjBindNeutralMaterialTextures(class UMaterialInstanceDynamic& Instance);

/** The plugin's master material, or null when the content is missing. */
URLAB_API class UMaterialInterface* MjLoadMasterMaterial();
