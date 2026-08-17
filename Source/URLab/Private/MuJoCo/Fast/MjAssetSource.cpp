// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjAssetSource.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

FMjMeshDataView FMjModelAssetSource::GetMeshData(int32 MeshId) const
{
	FMjMeshDataView View;
	if (!Model || MeshId < 0 || MeshId >= static_cast<int32>(Model->nmesh))
	{
		return View;
	}
	View.Verts = Model->mesh_vert;
	View.Normals = Model->mesh_normal;
	View.Texcoords = Model->mesh_texcoord;
	View.Faces = Model->mesh_face;
	View.FaceNormals = Model->mesh_facenormal;
	View.FaceTexcoords = Model->mesh_facetexcoord;
	View.VertAdr = Model->mesh_vertadr[MeshId];
	View.VertNum = static_cast<int32>(Model->mesh_vertnum[MeshId]);
	View.NormalAdr = Model->mesh_normaladr[MeshId];
	View.FaceAdr = Model->mesh_faceadr[MeshId];
	View.FaceNum = Model->mesh_facenum[MeshId];
	View.TexcoordAdr = Model->mesh_texcoordadr[MeshId];
	View.bValid = true;
	return View;
}

FMjTextureDataView FMjModelAssetSource::GetTextureData(int32 TexId) const
{
	FMjTextureDataView View;
	if (!Model || TexId < 0 || TexId >= static_cast<int32>(Model->ntex))
	{
		return View;
	}
	View.Width = Model->tex_width[TexId];
	View.Height = Model->tex_height[TexId];
	View.NumChannels = Model->tex_nchannel[TexId];
	View.Data = Model->tex_data + Model->tex_adr[TexId];
	View.NumBytes = static_cast<int64>(View.Width) * View.Height * View.NumChannels;
	View.bValid = true;
	return View;
}
