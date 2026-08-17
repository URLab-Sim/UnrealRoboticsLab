// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;

/**
 * A borrowed view onto one mesh's crease-split geometry inputs: the shared
 * vertex/normal/texcoord/face pools plus this mesh's per-mesh slice addresses
 * and counts. The pointers alias the provider's own storage and are never
 * owned here, so building a mesh from a view copies nothing.
 *
 * Face indices are LOCAL to each mesh; the slice addresses (`*Adr`) are the
 * base offsets that turn a local index into an absolute one in the shared pool,
 * exactly as the compiled model lays them out.
 */
struct FMjMeshDataView
{
	// Shared pools (borrowed): positions/normals stride 3, texcoords stride 2,
	// faces stride 3 of indices into the matching pool.
	const float* Verts = nullptr;
	const float* Normals = nullptr;
	const float* Texcoords = nullptr;
	const int32* Faces = nullptr;
	const int32* FaceNormals = nullptr;
	const int32* FaceTexcoords = nullptr;

	// This mesh's slice into the pools above.
	int32 VertAdr = 0;
	int32 VertNum = 0;
	int32 NormalAdr = 0;
	int32 FaceAdr = 0;
	int32 FaceNum = 0;

	// Base offset into the texcoord pool, or negative when the mesh carries no
	// texcoords (mirrors the compiled model's `mesh_texcoordadr` sentinel).
	int32 TexcoordAdr = -1;

	// False when the requested mesh id could not be resolved; a consumer should
	// produce an empty mesh, matching the model reader's early-out.
	bool bValid = false;
};

/**
 * A borrowed view onto one texture's decoded pixel buffer: its dimensions,
 * channel count, and a pointer to the first byte of its image in the provider's
 * storage. `Data` is not owned here.
 */
struct FMjTextureDataView
{
	int32 Width = 0;
	int32 Height = 0;
	int32 NumChannels = 0;

	// First byte of this texture's image (borrowed), row-0-first, NumChannels
	// bytes per pixel; NumBytes == Width * Height * NumChannels.
	const uint8* Data = nullptr;
	int64 NumBytes = 0;

	// False when the requested texture id could not be resolved.
	bool bValid = false;
};

/**
 * Where the renderer's visual asset bytes come from.
 *
 * The fast-path renderer builds its meshes and textures from views this
 * provider hands out rather than reaching into a specific storage layout, so
 * the build code is decoupled from the origin of the bytes. The only
 * implementation today reads them straight out of a compiled mjModel's mesh and
 * texture pools; a later implementation may resolve an on-disk path instead,
 * without the mesh/texture builders changing.
 *
 * Plain C++ abstract class (NOT a UE UINTERFACE), matching the module's other
 * IMj* provider seams, so a UObject-backed source can implement it freely.
 */
class URLAB_API IMjAssetSource
{
public:
	virtual ~IMjAssetSource() = default;

	/** The geometry inputs for a mesh id, or an invalid view for a bad id. */
	virtual FMjMeshDataView GetMeshData(int32 MeshId) const = 0;

	/** The pixel buffer for a texture id, or an invalid view for a bad id. */
	virtual FMjTextureDataView GetTextureData(int32 TexId) const = 0;

	// TODO: extend to mjz when mjModel can reference paths instead of bytes.
};

/**
 * The mjModel-backed asset source: returns views pointing straight into a
 * compiled model's mesh and texture pools. Borrows the model; the owner keeps
 * its lifetime.
 */
class URLAB_API FMjModelAssetSource : public IMjAssetSource
{
public:
	explicit FMjModelAssetSource(const mjModel_* InModel) : Model(InModel) {}

	virtual FMjMeshDataView GetMeshData(int32 MeshId) const override;
	virtual FMjTextureDataView GetTextureData(int32 TexId) const override;

private:
	const mjModel_* Model = nullptr;
};
