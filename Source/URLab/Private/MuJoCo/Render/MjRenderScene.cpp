// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Render/MjRenderScene.h"
#include "MuJoCo/Render/MjRenderSceneShim.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
/** A model name, or the empty string when the element is anonymous. */
FString NameOf(const mjModel* M, int32 Type, int32 Id)
{
	const char* Name = (M && M->names) ? mj_id2name(M, Type, Id) : nullptr;
	return (Name && *Name) ? FString(UTF8_TO_TCHAR(Name)) : FString();
}
} // namespace

void MjRenderSceneShim::Fill(FMjRenderScene& Out, const mjModel_* ModelPtr)
{
	Out.Reset();
	const mjModel* M = ModelPtr;
	if (!M)
	{
		return;
	}

	Out.BodyNames.SetNum(M->nbody);
	for (int32 B = 0; B < M->nbody; ++B)
	{
		Out.BodyNames[B] = NameOf(M, mjOBJ_BODY, B);
	}

	Out.Geoms.SetNum(M->ngeom);
	for (int32 G = 0; G < M->ngeom; ++G)
	{
		FMjSceneGeom& Geom = Out.Geoms[G];
		Geom.BodyId = M->geom_bodyid[G];
		Geom.Group = M->geom_group[G];
		Geom.Kind = EMjSceneGeomKind::Unsupported;
		switch (M->geom_type[G])
		{
			case mjGEOM_PLANE: Geom.Kind = EMjSceneGeomKind::Plane; break;
			case mjGEOM_SPHERE: Geom.Kind = EMjSceneGeomKind::Sphere; break;
			case mjGEOM_ELLIPSOID: Geom.Kind = EMjSceneGeomKind::Ellipsoid; break;
			case mjGEOM_CYLINDER: Geom.Kind = EMjSceneGeomKind::Cylinder; break;
			case mjGEOM_CAPSULE: Geom.Kind = EMjSceneGeomKind::Capsule; break;
			case mjGEOM_BOX: Geom.Kind = EMjSceneGeomKind::Box; break;
			case mjGEOM_MESH: Geom.Kind = EMjSceneGeomKind::Mesh; break;
			default: break;
		}
		FMemory::Memcpy(Geom.Size, M->geom_size + 3 * G, sizeof(double) * 3);
		Geom.MeshId = M->geom_dataid[G];
		Geom.MaterialId = M->geom_matid[G];
		Geom.bIsMesh = (M->geom_type[G] == mjGEOM_MESH);
		FMemory::Memcpy(Geom.Pos, M->geom_pos + 3 * G, sizeof(double) * 3);
		FMemory::Memcpy(Geom.Quat, M->geom_quat + 4 * G, sizeof(double) * 4);
		// Resolve the material indirection here so the view never has to: a geom with a
		// material draws the material's rgba, otherwise its own.
		const float* Rgba = (Geom.MaterialId >= 0) ? (M->mat_rgba + 4 * Geom.MaterialId)
												   : (M->geom_rgba + 4 * G);
		FMemory::Memcpy(Geom.Rgba, Rgba, sizeof(float) * 4);
		Geom.Name = NameOf(M, mjOBJ_GEOM, G);
		if (Geom.MeshId >= 0)
		{
			Geom.MeshName = NameOf(M, mjOBJ_MESH, Geom.MeshId);
		}
	}

	Out.Cameras.SetNum(M->ncam);
	for (int32 C = 0; C < M->ncam; ++C)
	{
		FMjSceneCamera& Cam = Out.Cameras[C];
		Cam.BodyId = M->cam_bodyid[C];
		Cam.Name = NameOf(M, mjOBJ_CAMERA, C);
		Cam.ResX = M->cam_resolution ? static_cast<int32>(M->cam_resolution[2 * C]) : 0;
		Cam.ResY = M->cam_resolution ? static_cast<int32>(M->cam_resolution[2 * C + 1]) : 0;
		Cam.Fovy = M->cam_fovy[C];
		// Intrinsics only count when the sensor has a real height to scale them against;
		// this is the same test both camera builders made against the raw model.
		Cam.bHasIntrinsics = M->cam_sensorsize && M->cam_intrinsic
			&& M->cam_sensorsize[2 * C + 1] > 0.0f;
		if (Cam.bHasIntrinsics)
		{
			Cam.SensorW = M->cam_sensorsize[2 * C];
			Cam.SensorH = M->cam_sensorsize[2 * C + 1];
			const float* Intrinsic = M->cam_intrinsic + 4 * C;
			Cam.FocalX = static_cast<float>(Intrinsic[0]);
			Cam.FocalY = static_cast<float>(Intrinsic[1]);
			Cam.PrincipalX = static_cast<float>(Intrinsic[2]);
			Cam.PrincipalY = static_cast<float>(Intrinsic[3]);
		}
		FMemory::Memcpy(Cam.Pos, M->cam_pos + 3 * C, sizeof(double) * 3);
		FMemory::Memcpy(Cam.Quat, M->cam_quat + 4 * C, sizeof(double) * 4);
	}

	// MuJoCo states its clip planes as fractions of the scene extent; a UE camera wants
	// centimetres. An extent-less or unset model falls back to the range that has always
	// been used for a camera the model says nothing about.
	if (M->vis.map.znear > 0.0f && M->stat.extent > 0.0f)
	{
		Out.NearClipCm = static_cast<float>(M->vis.map.znear * M->stat.extent * 100.0);
	}
	if (M->vis.map.zfar > 0.0f && M->stat.extent > 0.0f)
	{
		Out.FarClipCm = static_cast<float>(M->vis.map.zfar * M->stat.extent * 100.0);
	}

	Out.NumFlex = M->nflex;
	Out.NumSkins = M->nskin;
}
