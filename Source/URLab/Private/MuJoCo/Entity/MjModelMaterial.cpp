// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjModelMaterial.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// A size==0 plane is drawn as a finite quad of this half-extent (metres); the
// texuniform mapping tiles per length unit, so the extent stands in for the size.
constexpr double kInfinitePlaneHalfM = 25.0;

// mat_texid is (nmat x mjNTEXROLE); EMjMaterialRole runs the same order after the
// unused USER slot (offset +1). The texture id a role carries, or -1.
int32 RoleTexId(const mjModel* Model, int32 MatId, EMjMaterialRole Role)
{
	return Model->mat_texid[MatId * mjNTEXROLE + static_cast<int32>(Role) + 1];
}
} // namespace

FMjMaterialValues MjMaterialValuesFromModel(const mjModel_* ModelPtr, int32 GeomId,
	FLinearColor& OutBaseColor, FVector2D& OutGeomSize)
{
	const mjModel* Model = ModelPtr;
	FMjMaterialValues Values;

	const int32 MatId = Model->geom_matid[GeomId];
	const float* Rgba = (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * GeomId);
	OutBaseColor = FLinearColor(Rgba[0], Rgba[1], Rgba[2], Rgba[3]);

	if (MatId < 0)
	{
		// No material: MuJoCo draws a matte, non-metallic surface. Shininess 0.2 is
		// the value that folds (1 - shininess) to the 0.8 roughness the matte look
		// wants; metallic stays unset (-1 -> 0), specular/reflectance/emission at
		// their neutral defaults. No textures, no texuniform.
		Values.Shininess = 0.2f;
		OutGeomSize = FVector2D::ZeroVector;
		return Values;
	}

	Values.bFound = true;
	Values.Rgba = OutBaseColor;
	Values.Emission = Model->mat_emission[MatId];
	Values.Specular = Model->mat_specular[MatId];
	Values.Shininess = Model->mat_shininess[MatId];
	Values.Reflectance = Model->mat_reflectance[MatId];
	// The -1 "unset" sentinel is preserved; MjMetallicFor / MjRoughnessFor decide
	// what it means against the map presence below.
	Values.Metallic = Model->mat_metallic[MatId];
	Values.Roughness = Model->mat_roughness[MatId];
	Values.TexRepeat = FVector2D(Model->mat_texrepeat[MatId * 2 + 0], Model->mat_texrepeat[MatId * 2 + 1]);
	Values.bTexUniform = Model->mat_texuniform[MatId] != 0;

	// Only presence matters: the MJB's images are bound from tex_data by the baker,
	// but the scalar-vs-map guards inside MjApplyMaterialParameters key off whether a
	// role (or the packed ORM) names a texture at all. A single marker per filled
	// role is enough to reproduce that decision.
	for (int32 R = 0; R < static_cast<int32>(EMjMaterialRole::Count); ++R)
	{
		if (RoleTexId(Model, MatId, static_cast<EMjMaterialRole>(R)) >= 0)
		{
			Values.TextureNames[R] = TEXT("*");
		}
	}

	// texuniform tiles per spatial (length) unit, so the geom's own planar size
	// multiplies the repeat. A size-0 plane is drawn as a finite quad, so report the
	// quad's half-extent rather than zero (which would drop the multiply and tile the
	// ground far too coarsely versus MuJoCo's own viewer).
	const bool bPlane = (Model->geom_type[GeomId] == mjGEOM_PLANE);
	const double Sx = Model->geom_size[3 * GeomId + 0] > 0.0 ? Model->geom_size[3 * GeomId + 0]
		: (bPlane ? kInfinitePlaneHalfM : 0.0);
	const double Sy = Model->geom_size[3 * GeomId + 1] > 0.0 ? Model->geom_size[3 * GeomId + 1]
		: (bPlane ? kInfinitePlaneHalfM : 0.0);
	OutGeomSize = FVector2D(Sx, Sy);

	return Values;
}
