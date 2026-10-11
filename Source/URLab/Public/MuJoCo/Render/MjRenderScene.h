// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * @file MjRenderScene.h
 * @brief The neutral description of a scene's static structure, and the only thing the
 *        renderer view reads about it.
 *
 * The renderer used to read some three dozen fields straight off `mjModel` to build and
 * pose its scene graph, even though its asset layer was already abstracted behind
 * borrowed mesh/texture views and it called exactly one MuJoCo function of its own
 * (`mj_forward`, for the rest pose). That left one physics engine's memory layout wired
 * into the view for no reason other than history.
 *
 * `FMjRenderScene` is that same information in engine-neutral form: bodies, geoms,
 * cameras, and the clip planes the visual settings imply. A shim on the MuJoCo side
 * (`MjRenderSceneShim::Fill`) is the single place that knows `mjModel`; everything the
 * view does afterwards -- spawning body actors, building geom components, sizing and
 * placing cameras -- reads this struct.
 *
 * Deliberately NOT here: flex, skin and the overlay decor. Those three replay MuJoCo
 * algorithms vertex for vertex (`mj_flex`, `mjv_updateActiveSkin`, `mjv_updateScene`),
 * their headers take an `mjModel&` in their own signatures, and they belong on the
 * MuJoCo side of the boundary along with it. Marshalling their inputs through a neutral
 * struct would buy nothing and cost a copy of every skin bone weight per frame.
 *
 * This struct is pure marshalling: filling it reads the model and writes plain data, so
 * nothing about it can change what is rendered.
 */

/** One geom's static description: which body carries it, where it sits in that body, and
 *  what it draws with. World poses are not here -- they arrive per frame from a pose source. */
enum class EMjSceneGeomKind : uint8
{
	Unsupported,
	Plane,
	Sphere,
	Ellipsoid,
	Cylinder,
	Capsule,
	Box,
	Mesh,
};

struct FMjSceneGeom
{
	/** The body this geom rides on; its world pose composes with Pos/Quat below. */
	int32 BodyId = -1;
	/** MuJoCo geom group, matched against the renderer's visible-group mask. */
	int32 Group = 0;
	/** Neutral primitive/mesh kind. Unsupported kinds remain explicit. */
	EMjSceneGeomKind Kind = EMjSceneGeomKind::Unsupported;
	/** Primitive dimensions in metres, copied from the source scene's size triple. */
	double Size[3] = {0.0, 0.0, 0.0};
	/** Mesh asset index for a mesh geom, else -1 (asset resolution is separate). */
	int32 MeshId = -1;
	/** Material index, or -1 when the geom carries its own colour. */
	int32 MaterialId = -1;
	/** True for a mesh geom -- the only kind the instanced-statics collapse handles. */
	bool bIsMesh = false;
	/** Offset within the body (metres) and orientation (wxyz), both constant. */
	double Pos[3] = {0.0, 0.0, 0.0};
	double Quat[4] = {1.0, 0.0, 0.0, 0.0};
	/** The colour this geom actually draws with: its material's when it has one, else its
	 *  own. Alpha 0 is the "do not draw" convention collision proxies use. */
	float Rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	/** Authored name, empty when the geom is anonymous. */
	FString Name;
	/** Name of the mesh asset a mesh geom draws, for a readable outliner label. */
	FString MeshName;
};

/** One model camera's static description: where it is mounted and how it projects. */
struct FMjSceneCamera
{
	/** The body this camera is fixed to, or -1 for a free camera. */
	int32 BodyId = -1;
	FString Name;
	/** Authored pixel resolution, or 0 when the model left it unspecified. */
	int32 ResX = 0;
	int32 ResY = 0;
	/** Vertical field of view in degrees; used only when there are no intrinsics. */
	double Fovy = 0.0;
	/** True when the model carries a full sensor + intrinsic description, which takes
	 *  precedence over Fovy because it pins the principal point as well. */
	bool bHasIntrinsics = false;
	float SensorW = 0.0f;
	float SensorH = 0.0f;
	float FocalX = 0.0f;
	float FocalY = 0.0f;
	float PrincipalX = 0.0f;
	float PrincipalY = 0.0f;
	/** Offset within the mounting body (metres) and orientation (wxyz). */
	double Pos[3] = {0.0, 0.0, 0.0};
	double Quat[4] = {1.0, 0.0, 0.0, 0.0};
};

/**
 * @struct FMjRenderScene
 * @brief Everything the renderer view knows about the scene it is drawing.
 */
struct URLAB_API FMjRenderScene
{
	/** One entry per body, world body included, indexed by body id. Names may be empty. */
	TArray<FString> BodyNames;
	/** Indexed by geom id. Every geom is present, including the ones the view hides. */
	TArray<FMjSceneGeom> Geoms;
	/** Indexed by camera id. */
	TArray<FMjSceneCamera> Cameras;

	/** Near/far clip in UE centimetres, already resolved from the model's visual settings
	 *  and its extent (the view has no business knowing that relationship). */
	float NearClipCm = 2.0f;
	float FarClipCm = 10000.0f;

	/** Deformable counts. The view only needs to know whether there are any; the
	 *  reconstruction itself lives on the MuJoCo side with the model. */
	int32 NumFlex = 0;
	int32 NumSkins = 0;

	int32 NumBodies() const { return BodyNames.Num(); }
	int32 NumGeoms() const { return Geoms.Num(); }
	int32 NumCameras() const { return Cameras.Num(); }

	/** A scene with no bodies was never filled: MuJoCo always has a world body. */
	bool IsValid() const { return BodyNames.Num() > 0; }

	void Reset()
	{
		BodyNames.Reset();
		Geoms.Reset();
		Cameras.Reset();
		NearClipCm = 2.0f;
		FarClipCm = 10000.0f;
		NumFlex = 0;
		NumSkins = 0;
	}
};
