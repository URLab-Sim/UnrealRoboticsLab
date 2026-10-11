// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "MuJoCo/Render/MjRenderSceneAdapter.h"

#include "Utils/CoordinateConversion.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
FString NameOf(const mjModel& Model, int32 Type, int32 Index)
{
	const char* Name = Model.names ? mj_id2name(&Model, Type, Index) : nullptr;
	return Name && *Name ? FString(UTF8_TO_TCHAR(Name)) : FString();
}

bool MakeFlexVisual(const mjModel& Model, const mjData& Data, mjvScene& Visual,
	FString& OutError)
{
	FMemory::Memzero(Visual);
	mjv_makeScene(&Model, &Visual, 4096);
	mjvOption Option;
	mjv_defaultOption(&Option);
	mjvCamera Camera;
	mjv_defaultCamera(&Camera);
	mjv_updateScene(&Model, const_cast<mjData*>(&Data), &Option, nullptr,
		&Camera, mjCAT_ALL, &Visual);
	if (Visual.status == 0 && Visual.nflex == Model.nflex)
	{
		int64 TotalFaces = 0;
		for (int32 FlexId = 0; FlexId < Visual.nflex; ++FlexId)
		{
			if (Visual.flexfacenum[FlexId] < 0) break;
			TotalFaces += Visual.flexfacenum[FlexId];
		}
		bool bValid = (TotalFaces <= MAX_int32 / 9);
		for (int32 FlexId = 0; bValid && FlexId < Visual.nflex; ++FlexId)
		{
			const int64 First = Visual.flexfaceadr[FlexId];
			const int64 Used = Visual.flexfaceused[FlexId];
			bValid = First >= 0 && Used >= 0 && Used <= Visual.flexfacenum[FlexId]
				&& First + Visual.flexfacenum[FlexId] <= TotalFaces
				&& (Used == 0 || (Visual.flexface && Visual.flexnormal));
		}
		if (bValid) return true;
	}
	OutError = TEXT("MuJoCo flex visualization could not be generated");
	return false;
}

struct FScopedFlexVisual
{
	mjvScene Scene{};
	~FScopedFlexVisual() { mjv_freeScene(&Scene); }
};

ERenderGeomKind RenderKind(int32 MjKind)
{
	switch (MjKind)
	{
		case mjGEOM_PLANE: return ERenderGeomKind::Plane;
		case mjGEOM_SPHERE: return ERenderGeomKind::Sphere;
		case mjGEOM_ELLIPSOID: return ERenderGeomKind::Ellipsoid;
		case mjGEOM_CYLINDER: return ERenderGeomKind::Cylinder;
		case mjGEOM_CAPSULE: return ERenderGeomKind::Capsule;
		case mjGEOM_BOX: return ERenderGeomKind::Box;
		case mjGEOM_MESH: return ERenderGeomKind::Mesh;
		case mjGEOM_HFIELD: return ERenderGeomKind::Mesh;
		default: return ERenderGeomKind::Unsupported;
	}
}

bool RenderLight(const mjModel& Model, int32 Index, FRenderLightDesc& Out, FString& OutError)
{
	const int32 Type = Model.light_type[Index];
	if (Type == mjLIGHT_IMAGE)
	{
		OutError = FString::Printf(TEXT("MuJoCo image light %d is unsupported"), Index);
		return false;
	}
	switch (Type)
	{
		case mjLIGHT_DIRECTIONAL: Out.Kind = ERenderLightKind::Directional; break;
		case mjLIGHT_POINT: Out.Kind = ERenderLightKind::Point; break;
		case mjLIGHT_SPOT: Out.Kind = ERenderLightKind::Spot; break;
		default:
			OutError = FString::Printf(TEXT("MuJoCo light %d has unknown type %d"), Index, Type);
			return false;
	}
	Out.Name = NameOf(Model, mjOBJ_LIGHT, Index);
	if (Out.Name.IsEmpty()) Out.Name = FString::Printf(TEXT("light_%d"), Index);
	Out.bEnabled = Model.light_active[Index] != 0;
	Out.bCastShadows = Model.light_castshadow[Index] != 0;
	const float* Diffuse = Model.light_diffuse + 3 * Index;
	if (!FMath::IsFinite(Diffuse[0]) || !FMath::IsFinite(Diffuse[1])
		|| !FMath::IsFinite(Diffuse[2]) || Diffuse[0] < 0.0f || Diffuse[1] < 0.0f
		|| Diffuse[2] < 0.0f || Diffuse[0] > 1.0f || Diffuse[1] > 1.0f
		|| Diffuse[2] > 1.0f)
	{
		OutError = FString::Printf(TEXT("MuJoCo light %d has invalid diffuse colour"), Index);
		return false;
	}
	const float Luminance = 0.2126f * Diffuse[0] + 0.7152f * Diffuse[1]
		+ 0.0722f * Diffuse[2];
	const float ModelIntensity = Model.light_intensity[Index];
	if (!FMath::IsFinite(ModelIntensity) || ModelIntensity < 0.0f)
	{
		OutError = FString::Printf(TEXT("MuJoCo light %d has invalid intensity"), Index);
		return false;
	}
	if (ModelIntensity > 0.0f)
	{
		Out.Color = FLinearColor(Diffuse[0], Diffuse[1], Diffuse[2], 1.0f);
		Out.Intensity = ModelIntensity;
	}
	else if (Luminance > UE_SMALL_NUMBER)
	{
		// MuJoCo's default Phong path treats diffuse as an RGB contribution.
		// Keep that bounded colour verbatim; the compatibility intensity carries
		// the brightness approximation independently.
		Out.Color = FLinearColor(Diffuse[0], Diffuse[1], Diffuse[2], 1.0f);
		Out.Intensity = (Out.Kind == ERenderLightKind::Directional ? 6.0f : 42.0f)
			* Luminance;
	}
	else
	{
		Out.Color = FLinearColor::Black;
		Out.Intensity = 0.0f;
	}
	// Neutral descriptors keep fields irrelevant to a light kind canonical-zero.
	// Directional lights have neither a finite attenuation range nor a local source
	// radius in the UE component contract.
	Out.RangeCm = Out.Kind == ERenderLightKind::Directional
		? 0.0f : Model.light_range[Index] * 100.0f;
	Out.SourceRadiusCm = Out.Kind == ERenderLightKind::Directional
		? 0.0f : Model.light_bulbradius[Index] * 100.0f;
	if (!FMath::IsFinite(Out.RangeCm) || Out.RangeCm < 0.0f
		|| !FMath::IsFinite(Out.SourceRadiusCm) || Out.SourceRadiusCm < 0.0f)
	{
		OutError = FString::Printf(TEXT("MuJoCo light %d has invalid range or bulb radius"), Index);
		return false;
	}
	if (Out.Kind == ERenderLightKind::Spot)
	{
		Out.OuterConeDegrees = Model.light_cutoff[Index];
		Out.InnerConeDegrees = Out.OuterConeDegrees;
		if (!FMath::IsFinite(Out.OuterConeDegrees) || Out.OuterConeDegrees <= 0.0f
			|| Out.OuterConeDegrees > 90.0f)
		{
			OutError = FString::Printf(TEXT("MuJoCo spot light %d has invalid cutoff"), Index);
			return false;
		}
	}
	return true;
}

bool ConvertLightPose(const double* Position, const double* Direction, FTransform& Out)
{
	if (!Position || !Direction || !FMath::IsFinite(Position[0])
		|| !FMath::IsFinite(Position[1]) || !FMath::IsFinite(Position[2])
		|| !FMath::IsFinite(Direction[0]) || !FMath::IsFinite(Direction[1])
		|| !FMath::IsFinite(Direction[2])) return false;
	const FVector Forward =
		URLabCoordinateConversion::RightHandedDirectionToUnreal(Direction).GetSafeNormal();
	if (Forward.IsNearlyZero()) return false;
	Out = FTransform(Forward.Rotation().Quaternion(),
		URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Position));
	return true;
}

bool FiniteQuat(const double* Quat)
{
	double NormSquared = 0.0;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		if (!FMath::IsFinite(Quat[Index])) return false;
		NormSquared += Quat[Index] * Quat[Index];
	}
	return FMath::IsNearlyEqual(NormSquared, 1.0, 1.e-4);
}

bool FinitePosition(const double* Position)
{
	return FMath::IsFinite(Position[0]) && FMath::IsFinite(Position[1])
		&& FMath::IsFinite(Position[2]);
}

FTransform ConvertPose(const double* Position, const double* Quat)
{
	return FTransform(URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Quat), URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Position));
}

FTransform ConvertCameraPose(const double* Position, const double* Quat)
{
	return URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(Position, Quat);
}

FRenderCameraDesc CameraDesc(const mjModel& Model, int32 CameraIndex, int32 MaxHeight)
{
	FRenderCameraDesc Out;
	Out.CanonicalName = NameOf(Model, mjOBJ_CAMERA, CameraIndex);
	if (Out.CanonicalName.IsEmpty())
		Out.CanonicalName = FString::Printf(TEXT("camera_%d"), CameraIndex);
	int32 Width = Model.cam_resolution ? Model.cam_resolution[2 * CameraIndex] : 0;
	int32 Height = Model.cam_resolution ? Model.cam_resolution[2 * CameraIndex + 1] : 0;
	if (Width <= 1 || Height <= 1) { Width = 640; Height = 480; }
	if (MaxHeight > 0 && Height > MaxHeight)
	{
		Width = FMath::Max(1, FMath::RoundToInt(Width * (static_cast<double>(MaxHeight) / Height)));
		Height = MaxHeight;
	}
	Out.Optics.Resolution = FIntPoint(Width, Height);
	Out.Optics.Projection = Model.cam_projection
		&& Model.cam_projection[CameraIndex] == mjPROJ_ORTHOGRAPHIC
		? ERenderCameraProjection::Orthographic : ERenderCameraProjection::Perspective;
	Out.Optics.VerticalFovDegrees = Model.cam_fovy && Model.cam_fovy[CameraIndex] > 0.0
		? static_cast<float>(Model.cam_fovy[CameraIndex]) : 45.0f;
	if (Out.Optics.Projection == ERenderCameraProjection::Orthographic)
		Out.Optics.OrthographicWidthCm = Out.Optics.VerticalFovDegrees
			* (static_cast<float>(Width) / Height) * 100.0f;
	Out.Optics.bHasIntrinsics = Model.cam_sensorsize && Model.cam_intrinsic
		&& Model.cam_sensorsize[2 * CameraIndex] > 0.0f
		&& Model.cam_sensorsize[2 * CameraIndex + 1] > 0.0f;
	if (Out.Optics.bHasIntrinsics)
	{
		const float SensorW = Model.cam_sensorsize[2 * CameraIndex];
		const float SensorH = Model.cam_sensorsize[2 * CameraIndex + 1];
		const float* Intrinsic = Model.cam_intrinsic + 4 * CameraIndex;
		Out.Optics.FocalPixelsX = Intrinsic[0] * Width / SensorW;
		Out.Optics.FocalPixelsY = Intrinsic[1] * Height / SensorH;
		Out.Optics.PrincipalPixelsX = Width * 0.5f + Intrinsic[2] * Width / SensorW;
		Out.Optics.PrincipalPixelsY = Height * 0.5f + Intrinsic[3] * Height / SensorH;
	}
	if (Model.vis.map.znear > 0.0f && Model.stat.extent > 0.0f)
		Out.Optics.NearClipCm = Model.vis.map.znear * Model.stat.extent * 100.0f;
	if (Model.vis.map.zfar > 0.0f && Model.stat.extent > 0.0f)
		Out.Optics.FarClipCm = Model.vis.map.zfar * Model.stat.extent * 100.0f;
	return Out;
}

bool AppendVisualization(const mjModel& Model, const mjData& Data,
	const FRenderVisualizationConfig& Config, FRenderFrame& Frame, FString& OutError)
{
	if (Config.Features.IsEmpty()) return true;
	mjvOption Option;
	mjv_defaultOption(&Option);
	FMemory::Memzero(Option.flags, sizeof(Option.flags));
	struct FFlag { const TCHAR* Name; int32 Flag; };
	const FFlag Flags[] = {
		{TEXT("collision"), mjVIS_CONVEXHULL}, {TEXT("com"), mjVIS_COM},
		{TEXT("inertia"), mjVIS_INERTIA},
		{TEXT("contact_points"), mjVIS_CONTACTPOINT},
		{TEXT("contact_forces"), mjVIS_CONTACTFORCE},
		{TEXT("contact_split"), mjVIS_CONTACTSPLIT},
		{TEXT("perturbation"), mjVIS_PERTFORCE},
		{TEXT("camera_markers"), mjVIS_CAMERA},
		{TEXT("light_markers"), mjVIS_LIGHT},
		{TEXT("actuators"), mjVIS_ACTUATOR},
		{TEXT("tendons"), mjVIS_TENDON},
		{TEXT("rangefinders"), mjVIS_RANGEFINDER},
		{TEXT("constraints"), mjVIS_CONSTRAINT},
		{TEXT("static_bodies"), mjVIS_STATIC},
		{TEXT("auto_connect"), mjVIS_AUTOCONNECT}
	};
	for (const FFlag& Flag : Flags)
		if (Config.IsEnabled(Flag.Name)) Option.flags[Flag.Flag] = 1;
	mjvScene Visual;
	FMemory::Memzero(Visual);
	mjv_makeScene(&Model, &Visual, 4096);
	mjv_addGeoms(&Model, const_cast<mjData*>(&Data), &Option, nullptr,
		mjCAT_ALL, &Visual);
	if (Visual.status != 0)
	{
		mjv_freeScene(&Visual);
		OutError = TEXT("MuJoCo visualization exceeds the 4096 drawable limit");
		return false;
	}
	for (int32 Index = 0; Index < Visual.ngeom; ++Index)
	{
		const mjvGeom& Geom = Visual.geoms[Index];
		// Ordinary rigid model geoms already exist in the installed neutral scene.
		const bool bSite = Geom.objtype == mjOBJ_SITE && Config.IsEnabled(TEXT("sites"));
		const bool bCollision = Geom.objtype == mjOBJ_GEOM
			&& Config.IsEnabled(TEXT("collision")) && Geom.objid >= 0
			&& Geom.objid < Model.ngeom
			&& (Model.geom_group[Geom.objid] == 3
				|| (Model.geom_contype[Geom.objid] != 0
					&& Model.geom_conaffinity[Geom.objid] != 0));
		if (Geom.category != mjCAT_DECOR && !bSite && !bCollision) continue;
		if (Frame.DebugMeshes.Num() >= 4096)
		{
			mjv_freeScene(&Visual);
			OutError = TEXT("MuJoCo visualization exceeds the debug mesh limit");
			return false;
		}
		FRenderFrame::FDebugMesh& Mesh = Frame.DebugMeshes.AddDefaulted_GetRef();
		Mesh.Name = FString::Printf(TEXT("debug_%d"), Index);
		Mesh.Color = FLinearColor(Geom.rgba[0], Geom.rgba[1],
			Geom.rgba[2], Geom.rgba[3]);
		if (bCollision) Mesh.Color = FLinearColor(1.0f, 0.0f, 1.0f, 0.35f);
		const FVector Half(FMath::Max(0.001f, Geom.size[0]) * 100.0f,
			FMath::Max(0.001f, Geom.size[1]) * 100.0f,
			FMath::Max(0.001f, Geom.size[2]) * 100.0f);
		const FVector Center = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(
			Geom.pos);
		auto World = [&Geom, &Center](const FVector& Local)
		{
			const double X = Local.X, Y = -Local.Y, Z = Local.Z;
			const double Mx = Geom.mat[0] * X + Geom.mat[1] * Y + Geom.mat[2] * Z;
			const double My = Geom.mat[3] * X + Geom.mat[4] * Y + Geom.mat[5] * Z;
			const double Mz = Geom.mat[6] * X + Geom.mat[7] * Y + Geom.mat[8] * Z;
			return Center + FVector(Mx, -My, Mz);
		};
		// A low-poly solid preserves color, pose and extent for every MuJoCo
		// decoration, including lines/arrows. The adapter owns this tessellation.
		const FVector Corners[8] = {
			{-Half.X,-Half.Y,-Half.Z}, {Half.X,-Half.Y,-Half.Z},
			{Half.X,Half.Y,-Half.Z}, {-Half.X,Half.Y,-Half.Z},
			{-Half.X,-Half.Y,Half.Z}, {Half.X,-Half.Y,Half.Z},
			{Half.X,Half.Y,Half.Z}, {-Half.X,Half.Y,Half.Z}
		};
		for (const FVector& Corner : Corners) Mesh.Positions.Add(World(Corner));
		const int32 Faces[36] = {0,2,1,0,3,2,4,5,6,4,6,7,
			0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7};
		// These faces were authored with conventional outward crosses. UE's
		// procedural-mesh front face uses the opposite cross convention.
		for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); Face += 3)
		{
			Mesh.Triangles.Add(Faces[Face]);
			Mesh.Triangles.Add(Faces[Face + 2]);
			Mesh.Triangles.Add(Faces[Face + 1]);
		}
	}
	mjv_freeScene(&Visual);
	if ((Config.IsEnabled(TEXT("wrenches"))
		|| Config.IsEnabled(TEXT("perturbation"))) && Data.xfrc_applied)
	{
		for (int32 Body = 1; Body < Model.nbody; ++Body)
		{
			const double* Wrench = Data.xfrc_applied + 6 * Body;
			const double AppliedForce[3] = {Wrench[0], Wrench[1], Wrench[2]};
			const double AppliedTorque[3] = {Wrench[3], Wrench[4], Wrench[5]};
			const FVector Force = URLabCoordinateConversion::RightHandedDirectionToUnreal(AppliedForce);
			const FVector Torque = URLabCoordinateConversion::RightHandedDirectionToUnreal(AppliedTorque);
			if (Force.SizeSquared() <= 1.e-12 && Torque.SizeSquared() <= 1.e-12) continue;
			const FVector Origin = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(
				Data.xpos + 3 * Body);
			if (Force.SizeSquared() > 1.e-12)
			{
				const FVector End = Origin + Force.GetSafeNormal()
					* FMath::Clamp(Force.Size() * 2.0, 1.0, 200.0);
				FRenderFrame::FDebugMesh& Mesh = Frame.DebugMeshes.AddDefaulted_GetRef();
				Mesh.Name = FString::Printf(TEXT("applied_force_%d"), Body);
				Mesh.Color = FLinearColor(Model.vis.rgba.force[0], Model.vis.rgba.force[1],
					Model.vis.rgba.force[2], 1.0f);
				const FVector Axis = (End - Origin).GetSafeNormal();
				const FVector Side = FVector::CrossProduct(Axis,
					FMath::Abs(Axis.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector)
					.GetSafeNormal();
				const FVector Other = FVector::CrossProduct(Axis, Side);
				Mesh.Positions = {Origin + Side, Origin - Side, Origin + Other,
					End + Side, End - Side, End + Other};
				Mesh.Triangles = {0, 3, 2, 2, 3, 5, 2, 5, 1,
					1, 5, 4, 1, 4, 0, 0, 4, 3};
			}
			if (Torque.SizeSquared() > 1.e-12)
			{
				const FVector Axis = Torque.GetSafeNormal();
				const FVector U = FVector::CrossProduct(Axis,
					FMath::Abs(Axis.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector)
					.GetSafeNormal();
				const FVector V = FVector::CrossProduct(Axis, U);
				const float Radius = FMath::Clamp(Torque.Size()
					* FMath::Max(Model.vis.map.torque, 0.1f) * 100.0, 1.0, 120.0);
				const float Width = FMath::Max(0.5f, Radius * 0.08f);
				FRenderFrame::FDebugMesh& Mesh = Frame.DebugMeshes.AddDefaulted_GetRef();
				Mesh.Name = FString::Printf(TEXT("applied_torque_%d"), Body);
				Mesh.Color = FLinearColor(Model.vis.rgba.contacttorque[0],
					Model.vis.rgba.contacttorque[1], Model.vis.rgba.contacttorque[2], 1.0f);
				for (int32 Segment = 0; Segment < 12; ++Segment)
				{
					const float Angle = 2.0f * PI * Segment / 12.0f;
					const FVector Direction = U * FMath::Cos(Angle) + V * FMath::Sin(Angle);
					Mesh.Positions.Add(Origin + Direction * (Radius - Width));
					Mesh.Positions.Add(Origin + Direction * (Radius + Width));
					const int32 A = 2 * Segment, B = 2 * ((Segment + 1) % 12);
					Mesh.Triangles.Add(A); Mesh.Triangles.Add(B); Mesh.Triangles.Add(A + 1);
					Mesh.Triangles.Add(A + 1); Mesh.Triangles.Add(B); Mesh.Triangles.Add(B + 1);
				}
			}
		}
	}
	return true;
}
}

bool FMjRenderSceneAdapter::Extract(const mjModel_* ModelPtr, const mjData_* InitialData,
	const FString& InSceneRevision,
	int32 VisibleGroupMask,
	int32 CameraMaxHeight, bool bIncludeUserCamera,
	const FMjResolvedRenderCameraOverrides* CameraOverrides,
	FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FString& OutError)
{
	OutError.Reset();
	const mjModel* Model = ModelPtr;
	if (!Model || !InitialData || InitialData->signature != Model->signature
		|| InSceneRevision.IsEmpty())
	{
		OutError = TEXT("MuJoCo scene extraction requires matching model/data and nonempty revision");
		return false;
	}
	if (Model->nbody < 0 || Model->nbody > MAX_int32 / 4 || Model->ngeom < 0
		|| Model->ngeom > MAX_int32
		|| Model->ncam < 0 || Model->ncam > MAX_int32 / 9
		|| Model->nlight < 0 || Model->nlight > MAX_int32 / 3
		|| (Model->ngeom > 0 && (!Model->geom_bodyid || !Model->geom_group
			|| !Model->geom_type || !Model->geom_size || !Model->geom_pos
			|| !Model->geom_quat || !Model->geom_dataid))
		|| (Model->ncam > 0 && (!Model->cam_bodyid || !Model->cam_mode
			|| !Model->cam_projection
			|| !Model->cam_pos || !Model->cam_quat || !Model->cam_fovy))
		|| (Model->nlight > 0 && (!Model->light_type || !Model->light_active
			|| !Model->light_castshadow || !Model->light_diffuse
			|| !Model->light_intensity || !Model->light_range
			|| !Model->light_bulbradius || !Model->light_cutoff)))
	{
		OutError = TEXT("MuJoCo rigid scene tables are incomplete or exceed renderer limits");
		return false;
	}
	FRenderSceneResources CandidateResources;
	if (!MjRenderResourceShim::Fill(CandidateResources, Model,
		InSceneRevision, OutError))
	{
		return false;
	}

	FRenderScene CandidateScene;
	CandidateScene.SceneRevision = InSceneRevision;
	CandidateScene.Bodies.SetNum(static_cast<int32>(Model->nbody));
	for (int32 BodyIndex = 0; BodyIndex < CandidateScene.Bodies.Num(); ++BodyIndex)
	{
		CandidateScene.Bodies[BodyIndex].Name = NameOf(*Model, mjOBJ_BODY, BodyIndex);
	}
	CandidateScene.Geoms.SetNum(static_cast<int32>(Model->ngeom));
	for (int32 GeomIndex = 0; GeomIndex < CandidateScene.Geoms.Num(); ++GeomIndex)
	{
		FRenderGeomDesc& Geom = CandidateScene.Geoms[GeomIndex];
		Geom.BodyIndex = Model->geom_bodyid[GeomIndex];
		Geom.Group = Model->geom_group[GeomIndex];
		Geom.Kind = RenderKind(Model->geom_type[GeomIndex]);
		Geom.SizeCm = FVector(Model->geom_size[3 * GeomIndex],
			Model->geom_size[3 * GeomIndex + 1], Model->geom_size[3 * GeomIndex + 2]) * 100.0;
		Geom.MeshResourceIndex = Model->geom_type[GeomIndex] == mjGEOM_MESH
			? Model->geom_dataid[GeomIndex]
			: (Model->geom_type[GeomIndex] == mjGEOM_HFIELD
				? Model->nmesh + Model->geom_dataid[GeomIndex] : INDEX_NONE);
		Geom.Name = NameOf(*Model, mjOBJ_GEOM, GeomIndex);
		if (Model->geom_type[GeomIndex] == mjGEOM_MESH
			&& Geom.MeshResourceIndex >= 0)
		{
			Geom.MeshName = NameOf(*Model, mjOBJ_MESH, Geom.MeshResourceIndex);
		}
		else if (Model->geom_type[GeomIndex] == mjGEOM_HFIELD
			&& Model->geom_dataid[GeomIndex] >= 0
			&& Model->geom_dataid[GeomIndex] < Model->nhfield)
		{
			Geom.MeshName = NameOf(*Model, mjOBJ_HFIELD,
				Model->geom_dataid[GeomIndex]);
		}

		double LocalPosition[3] = {Model->geom_pos[3 * GeomIndex],
			Model->geom_pos[3 * GeomIndex + 1], Model->geom_pos[3 * GeomIndex + 2]};
		double LocalQuat[4] = {Model->geom_quat[4 * GeomIndex],
			Model->geom_quat[4 * GeomIndex + 1], Model->geom_quat[4 * GeomIndex + 2],
			Model->geom_quat[4 * GeomIndex + 3]};

		if (!FinitePosition(LocalPosition) || !FiniteQuat(LocalQuat))
		{
			OutError = FString::Printf(TEXT("geom %d local pose is invalid"), GeomIndex);
			return false;
		}
		Geom.BodyLocalTransform = ConvertPose(LocalPosition, LocalQuat);
	}
	CandidateScene.Cameras.SetNum(static_cast<int32>(Model->ncam));
	TArray<FCameraMountPlan> CandidateMountPlans;
	CandidateMountPlans.SetNum(static_cast<int32>(Model->ncam));
	for (int32 CameraIndex = 0; CameraIndex < CandidateScene.Cameras.Num(); ++CameraIndex)
	{
		CandidateScene.Cameras[CameraIndex] = CameraDesc(*Model, CameraIndex, CameraMaxHeight);
		if (CameraOverrides)
			if (const FMjResolvedRenderCameraOverride* Override = CameraOverrides->Find(CameraIndex))
			{
				CandidateScene.Cameras[CameraIndex].Capture = Override->Capture;
				if (Override->CustomNearClipCm > 0.0f)
					CandidateScene.Cameras[CameraIndex].Optics.NearClipCm = Override->CustomNearClipCm;
				if (Override->CustomFarClipCm > 0.0f)
					CandidateScene.Cameras[CameraIndex].Optics.FarClipCm = Override->CustomFarClipCm;
			}
		FCameraMountPlan& Plan = CandidateMountPlans[CameraIndex];
		Plan.BodyIndex = Model->cam_bodyid[CameraIndex];
		Plan.bRequiresExplicitWorldPose = Model->cam_mode[CameraIndex] != mjCAMLIGHT_FIXED;
		const double* Position = Model->cam_pos + 3 * CameraIndex;
		const double* Quat = Model->cam_quat + 4 * CameraIndex;
		if (!FinitePosition(Position) || !FiniteQuat(Quat))
		{
			OutError = FString::Printf(TEXT("camera %d local pose is invalid"), CameraIndex);
			return false;
		}
		Plan.BodyLocalCaptureTransform = ConvertCameraPose(Position, Quat);
	}
	if (bIncludeUserCamera)
	{
		FRenderCameraDesc User;
		User.CanonicalName = TEXT("user");
		User.Optics.Resolution = FIntPoint(1280, 720);
		User.Optics.NearClipCm = CandidateScene.Cameras.Num() > 0
			? CandidateScene.Cameras[0].Optics.NearClipCm : 2.0f;
		User.Optics.FarClipCm = CandidateScene.Cameras.Num() > 0
			? CandidateScene.Cameras[0].Optics.FarClipCm : 10000.0f;
		CandidateScene.UserCamera = User;
	}
	CandidateScene.Lights.SetNum(static_cast<int32>(Model->nlight));
	for (int32 LightIndex = 0; LightIndex < CandidateScene.Lights.Num(); ++LightIndex)
	{
		if (!RenderLight(*Model, LightIndex, CandidateScene.Lights[LightIndex], OutError))
			return false;
	}
	const auto& Headlight = Model->vis.headlight;
	FRenderEnvironmentDesc& Environment = CandidateScene.Environment;
	Environment.bHeadlightEnabled = Headlight.active != 0;
	Environment.HeadlightColor = FLinearColor(Headlight.diffuse[0],
		Headlight.diffuse[1], Headlight.diffuse[2]);
	Environment.HeadlightIntensity = Environment.bHeadlightEnabled
		? 0.5f * (0.2126f * Headlight.diffuse[0]
			+ 0.7152f * Headlight.diffuse[1] + 0.0722f * Headlight.diffuse[2]) : 0.0f;
	Environment.AmbientColor = FLinearColor(Headlight.ambient[0],
		Headlight.ambient[1], Headlight.ambient[2]);
	Environment.AmbientIntensity = 0.2126f * Headlight.ambient[0]
		+ 0.7152f * Headlight.ambient[1] + 0.0722f * Headlight.ambient[2];
	for (int32 TextureId = 0; TextureId < Model->ntex; ++TextureId)
	{
		if (!Model->tex_type || Model->tex_type[TextureId] != mjTEXTURE_SKYBOX) continue;
		const FRenderTextureResource Source = CandidateResources.Textures[TextureId];
		if (Source.Width <= 0 || Source.Height != 6 * Source.Width) break;
		const int32 PixelsPerFace = Source.Width * Source.Width;
		if (PixelsPerFace <= 0 || PixelsPerFace > MAX_int32 / 4) break;
		Environment.bSkyboxEnabled = true;
		float Brightest = -1.0f, Darkest = 2.0f;
		for (int32 Face = 0; Face < 6; ++Face)
		{
			FRenderTextureResource& FaceTexture = CandidateResources.Textures.AddDefaulted_GetRef();
			FaceTexture.Width = Source.Width;
			FaceTexture.Height = Source.Width;
			FaceTexture.Rgba8.Append(Source.Rgba8.GetData() + Face * PixelsPerFace * 4,
				PixelsPerFace * 4);
			Environment.SkyboxFaceTextureIndices.Add(CandidateResources.Textures.Num() - 1);
			for (int32 Pixel = 0; Pixel < PixelsPerFace; ++Pixel)
			{
				const uint8* Rgba = FaceTexture.Rgba8.GetData() + 4 * Pixel;
				const float L = (0.2126f * Rgba[0] + 0.7152f * Rgba[1]
					+ 0.0722f * Rgba[2]) / 255.0f;
				const FLinearColor C(Rgba[0] / 255.0f, Rgba[1] / 255.0f,
					Rgba[2] / 255.0f);
				if (L > Brightest) { Brightest = L; Environment.SkyTop = C; }
				if (L < Darkest) { Darkest = L; Environment.SkyBottom = C; }
			}
		}
		break;
	}
	// MuJoCo's visualizer owns flex surface expansion, smooth corner normals,
	// front/back sheets, thickness and boundary closure. Freeze its default
	// options for this installed scene revision.
	FScopedFlexVisual InitialFlex;
	if (Model->nflex > 0 && !MakeFlexVisual(*Model, *InitialData, InitialFlex.Scene, OutError))
		return false;
	for (int32 FlexId = 0; FlexId < Model->nflex; ++FlexId)
	{
		const int32 Used = InitialFlex.Scene.flexfaceused[FlexId];
		if (Used == 0) continue;
		if (Used < 0 || Used > InitialFlex.Scene.flexfacenum[FlexId]
			|| Used > MAX_int32 / 3 || InitialFlex.Scene.flexfaceadr[FlexId] < 0)
		{
			OutError = FString::Printf(TEXT("flex %d has invalid visual face count"), FlexId);
			return false;
		}
		FRenderDeformableDesc& Desc = CandidateScene.Deformables.AddDefaulted_GetRef();
		Desc.Name = NameOf(*Model, mjOBJ_FLEX, FlexId);
		if (Desc.Name.IsEmpty()) Desc.Name = FString::Printf(TEXT("flex_%d"), FlexId);
		Desc.VertexCount = 3 * Used;
		Desc.SurfaceIndex = FlexId;
		Desc.Triangles.Reserve(3 * Used);
		for (int32 Face = 0; Face < Used; ++Face)
		{
			const int32 Base = 3 * Face;
			// UE procedural meshes use the opposite front-face convention from
			// MuJoCo. Reflecting Y already accounts for that convention change.
			Desc.Triangles.Add(Base);
			Desc.Triangles.Add(Base + 1);
			Desc.Triangles.Add(Base + 2);
		}
		if (Model->flex_texcoordadr[FlexId] >= 0)
		{
			if (!InitialFlex.Scene.flextexcoord)
			{
				OutError = FString::Printf(TEXT("flex %d has no visual texture coordinates"), FlexId);
				return false;
			}
			Desc.UVs.Reserve(Desc.VertexCount);
			const float* Coordinates = InitialFlex.Scene.flextexcoord
				+ 6 * InitialFlex.Scene.flexfaceadr[FlexId];
			for (int32 Vertex = 0; Vertex < Desc.VertexCount; ++Vertex)
			{
				const float U = Coordinates[2 * Vertex], V = Coordinates[2 * Vertex + 1];
				if (!FMath::IsFinite(U) || !FMath::IsFinite(V))
				{
					OutError = FString::Printf(TEXT("flex %d has non-finite visual UVs"), FlexId);
					return false;
				}
				Desc.UVs.Emplace(U, V);
			}
		}
	}
	for (int32 SkinId = 0; SkinId < Model->nskin; ++SkinId)
	{
		const int32 Vertices = Model->skin_vertnum[SkinId];
		const int32 Faces = Model->skin_facenum[SkinId];
		if (Vertices < 3 || Faces <= 0 || Faces > MAX_int32 / 3
			|| !Model->skin_face || Model->skin_faceadr[SkinId] < 0)
		{
			OutError = FString::Printf(TEXT("skin %d has invalid render topology"), SkinId);
			return false;
		}
		FRenderDeformableDesc& Desc = CandidateScene.Deformables.AddDefaulted_GetRef();
		Desc.Name = NameOf(*Model, mjOBJ_SKIN, SkinId);
		if (Desc.Name.IsEmpty()) Desc.Name = FString::Printf(TEXT("skin_%d"), SkinId);
		Desc.VertexCount = Vertices;
		Desc.SurfaceIndex = Model->nflex + SkinId;
		Desc.Triangles.Reserve(Faces * 3);
		for (int32 Face = 0; Face < Faces; ++Face)
		{
			const int32* I = Model->skin_face + 3 * (Model->skin_faceadr[SkinId] + Face);
			for (int32 Corner = 0; Corner < 3; ++Corner)
				if (I[Corner] < 0 || I[Corner] >= Vertices)
				{
					OutError = FString::Printf(TEXT("skin %d face index is out of bounds"), SkinId);
					return false;
				}
			// Same reflected-source / UE front-face convention as visual flexes.
			Desc.Triangles.Add(I[0]);
			Desc.Triangles.Add(I[1]);
			Desc.Triangles.Add(I[2]);
		}
		const int64 UVAddress = Model->skin_texcoordadr[SkinId];
		if (UVAddress >= 0)
		{
			if (!Model->skin_texcoord || UVAddress + Vertices > Model->nskintexvert)
			{
				OutError = FString::Printf(TEXT("skin %d has invalid texture coordinates"), SkinId);
				return false;
			}
			Desc.UVs.Reserve(Vertices);
			for (int32 Vertex = 0; Vertex < Vertices; ++Vertex)
			{
				const float* Coordinates = Model->skin_texcoord + 2 * (UVAddress + Vertex);
				if (!FMath::IsFinite(Coordinates[0]) || !FMath::IsFinite(Coordinates[1]))
				{
					OutError = FString::Printf(TEXT("skin %d has non-finite UVs"), SkinId);
					return false;
				}
				Desc.UVs.Emplace(Coordinates[0], Coordinates[1]);
			}
		}
	}

	FString SceneError;
	if (!CandidateScene.IsValid(CandidateResources, VisibleGroupMask, SceneError))
	{
		OutError = FString::Printf(TEXT("MuJoCo rigid scene is invalid: %s"), *SceneError);
		return false;
	}
	OutScene = MoveTemp(CandidateScene);
	OutResources = MoveTemp(CandidateResources);
	SceneRevision = InSceneRevision;
	NumBodies = static_cast<int32>(Model->nbody);
	CameraMountPlans = MoveTemp(CandidateMountPlans);
	Deformables = OutScene.Deformables;
	Visualization = OutScene.Visualization;
	NumLights = static_cast<int32>(Model->nlight);
	bHasUserCamera = bIncludeUserCamera;
	ModelSignature = Model->signature;
	InstalledModel = Model;
	return true;
}

bool FMjRenderSceneAdapter::FrameFromData(const mjData_* DataPtr, uint64 FrameId,
	FRenderFrame& Out, FString& OutError) const
{
	const mjModel* Model = InstalledModel;
	const mjData* Data = DataPtr;
	if (!Data || Data->signature != ModelSignature
		|| (NumBodies > 0 && (!Data->xpos || !Data->xquat))
		|| (CameraMountPlans.Num() > 0 && (!Data->cam_xpos || !Data->cam_xmat))
		|| (NumLights > 0 && (!Data->light_xpos || !Data->light_xdir)))
	{
		OutError = TEXT("mjData is null, incomplete, or belongs to a different compiled model");
		return false;
	}
	TArray<double> CameraQuaternions;
	CameraQuaternions.SetNumUninitialized(4 * CameraMountPlans.Num());
	for (int32 CameraIndex = 0; CameraIndex < CameraMountPlans.Num(); ++CameraIndex)
		mju_mat2Quat(CameraQuaternions.GetData() + 4 * CameraIndex,
			Data->cam_xmat + 9 * CameraIndex);
	FRenderFrame Candidate;
	if (!FrameFromCartesianWithCameras(MakeArrayView(Data->xpos, 3 * NumBodies),
		MakeArrayView(Data->xquat, 4 * NumBodies),
		MakeArrayView(Data->cam_xpos, 3 * CameraMountPlans.Num()), CameraQuaternions,
		MakeArrayView(Data->light_xpos, 3 * NumLights),
		MakeArrayView(Data->light_xdir, 3 * NumLights),
		FrameId, Data->time, /*bHasSourceTime=*/true, /*ClockEpoch=*/0,
		Candidate, OutError)) return false;
	if (!Model)
	{
		OutError = TEXT("render adapter has no installed MuJoCo model");
		return false;
	}
	FScopedFlexVisual DynamicFlex;
	if (Model->nflex > 0 && !MakeFlexVisual(*Model, *Data, DynamicFlex.Scene, OutError))
		return false;
	Candidate.Deformables.Reserve(Deformables.Num());
	for (const FRenderDeformableDesc& Desc : Deformables)
	{
		FRenderFrame::FDeformable& Surface = Candidate.Deformables.AddDefaulted_GetRef();
		Surface.Name = Desc.Name;
		Surface.Positions.SetNumUninitialized(Desc.VertexCount);
		if (Desc.SurfaceIndex < Model->nflex)
		{
			const int32 FlexId = Desc.SurfaceIndex;
			const int32 Used = DynamicFlex.Scene.flexfaceused[FlexId];
			if (Used < 0 || Used > DynamicFlex.Scene.flexfacenum[FlexId]
				|| Used * 3 != Desc.VertexCount)
			{
				OutError = FString::Printf(TEXT("flex %d visual topology changed; reinstall scene"), FlexId);
				return false;
			}
			const int32 First = DynamicFlex.Scene.flexfaceadr[FlexId] * 9;
			Surface.Normals.SetNumUninitialized(Desc.VertexCount);
			for (int32 Vertex = 0; Vertex < Desc.VertexCount; ++Vertex)
			{
				const float* Position = DynamicFlex.Scene.flexface + First + 3 * Vertex;
				const float* Normal = DynamicFlex.Scene.flexnormal + First + 3 * Vertex;
				const double PositionMetres[3] = {Position[0], Position[1], Position[2]};
				const double NormalDirection[3] = {Normal[0], Normal[1], Normal[2]};
				Surface.Positions[Vertex] =
					URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(PositionMetres);
				Surface.Normals[Vertex] =
					URLabCoordinateConversion::RightHandedDirectionToUnreal(NormalDirection);
			}
		}
		else
		{
			const int32 SkinId = Desc.SurfaceIndex - Model->nflex;
			const int32 VertAdr = Model->skin_vertadr[SkinId];
			TArray<double> Accum;
			Accum.SetNumZeroed(3 * Desc.VertexCount);
			TArray<double> Weights;
			Weights.SetNumZeroed(Desc.VertexCount);
			for (int32 Bone = Model->skin_boneadr[SkinId];
				Bone < Model->skin_boneadr[SkinId] + Model->skin_bonenum[SkinId]; ++Bone)
			{
				const int32 Body = Model->skin_bonebodyid[Bone];
				if (Body < 0 || Body >= NumBodies)
				{
					OutError = FString::Printf(TEXT("skin %d bone has invalid body"), SkinId);
					return false;
				}
				double InverseBind[4], RotationQuat[4], Rotation[9], Translation[3];
				const float* BindQuatSource = Model->skin_bonebindquat + 4 * Bone;
				const double BindQuat[4] = {BindQuatSource[0], BindQuatSource[1],
					BindQuatSource[2], BindQuatSource[3]};
				mju_negQuat(InverseBind, BindQuat);
				mju_mulQuat(RotationQuat, Data->xquat + 4 * Body, InverseBind);
				mju_quat2Mat(Rotation, RotationQuat);
				const float* BindPosSource = Model->skin_bonebindpos + 3 * Bone;
				const double BindPos[3] = {BindPosSource[0], BindPosSource[1],
					BindPosSource[2]};
				mju_mulMatVec3(Translation, Rotation, BindPos);
				mju_sub3(Translation, Data->xpos + 3 * Body, Translation);
				for (int32 Entry = Model->skin_bonevertadr[Bone];
					Entry < Model->skin_bonevertadr[Bone] + Model->skin_bonevertnum[Bone]; ++Entry)
				{
					const int32 Vertex = Model->skin_bonevertid[Entry];
					if (Vertex < 0 || Vertex >= Desc.VertexCount) continue;
					const double Weight = Model->skin_bonevertweight[Entry];
					double Rotated[3];
					const float* Source = Model->skin_vert + 3 * (VertAdr + Vertex);
					const double Rest[3] = {Source[0], Source[1], Source[2]};
					mju_mulMatVec3(Rotated, Rotation, Rest);
					for (int32 Axis = 0; Axis < 3; ++Axis)
						Accum[3 * Vertex + Axis] += Weight * (Rotated[Axis] + Translation[Axis]);
					Weights[Vertex] += Weight;
				}
			}
			for (int32 Vertex = 0; Vertex < Desc.VertexCount; ++Vertex)
			{
				const double Weight = Weights[Vertex];
				if (Weight > 1.e-9)
					for (int32 Axis = 0; Axis < 3; ++Axis)
						Accum[3 * Vertex + Axis] /= Weight;
			}
			const double Inflate = Model->skin_inflate ? Model->skin_inflate[SkinId] : 0.0;
			if (Inflate != 0.0)
			{
				TArray<double> Normals;
				Normals.SetNumZeroed(3 * Desc.VertexCount);
				for (int32 Face = 0; Face < Model->skin_facenum[SkinId]; ++Face)
				{
					const int32* I = Model->skin_face
						+ 3 * (Model->skin_faceadr[SkinId] + Face);
					double A[3], B[3], N[3];
					mju_sub3(A, Accum.GetData() + 3 * I[1], Accum.GetData() + 3 * I[0]);
					mju_sub3(B, Accum.GetData() + 3 * I[2], Accum.GetData() + 3 * I[0]);
					mju_cross(N, A, B);
					for (int32 Corner = 0; Corner < 3; ++Corner)
						mju_addTo3(Normals.GetData() + 3 * I[Corner], N);
				}
				for (int32 Vertex = 0; Vertex < Desc.VertexCount; ++Vertex)
				{
					double* Normal = Normals.GetData() + 3 * Vertex;
					if (mju_normalize3(Normal) > mjMINVAL)
						for (int32 Axis = 0; Axis < 3; ++Axis)
							Accum[3 * Vertex + Axis] += Inflate * Normal[Axis];
				}
			}
			for (int32 Vertex = 0; Vertex < Desc.VertexCount; ++Vertex)
			{
				Surface.Positions[Vertex] =
					URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(
						Accum.GetData() + 3 * Vertex);
			}
		}
	}
	if (!Candidate.IsValid(NumBodies, CameraMountPlans.Num(), bHasUserCamera,
		NumLights, SceneRevision, OutError)) return false;
	if (!AppendVisualization(*Model, *Data, Visualization, Candidate, OutError)) return false;
	if (!Candidate.IsValid(NumBodies, CameraMountPlans.Num(), bHasUserCamera,
		NumLights, SceneRevision, OutError)) return false;
	Out = MoveTemp(Candidate);
	return true;
}

bool FMjRenderSceneAdapter::FrameFromCartesian(TConstArrayView<double> BodyPositions,
	TConstArrayView<double> BodyQuaternions, TConstArrayView<double> LightPositions,
	TConstArrayView<double> LightDirections, uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch, FRenderFrame& Out, FString& OutError) const
{
	OutError.Reset();
	if (SceneRevision.IsEmpty() || BodyPositions.Num() != 3 * NumBodies
		|| BodyQuaternions.Num() != 4 * NumBodies
		|| LightPositions.Num() != 3 * NumLights || LightDirections.Num() != 3 * NumLights)
	{
		OutError = FString::Printf(TEXT("Cartesian frame has %d position and %d quaternion values; expected %d and %d"),
			BodyPositions.Num(), BodyQuaternions.Num(), 3 * NumBodies, 4 * NumBodies);
		return false;
	}
	FRenderFrame Candidate;
	Candidate.Identity.SceneRevision = SceneRevision;
	Candidate.Identity.FrameId = FrameId;
	Candidate.Identity.SourceTime = SourceTime;
	Candidate.Identity.bHasSourceTime = bHasSourceTime;
	Candidate.Identity.ClockEpoch = ClockEpoch;
	Candidate.BodySceneTransforms.SetNum(NumBodies);
	for (int32 BodyIndex = 0; BodyIndex < NumBodies; ++BodyIndex)
	{
		const double* Position = BodyPositions.GetData() + 3 * BodyIndex;
		const double* Quat = BodyQuaternions.GetData() + 4 * BodyIndex;
		if (!FinitePosition(Position) || !FiniteQuat(Quat))
		{
			OutError = FString::Printf(TEXT("body pose %d is non-finite or non-unit"), BodyIndex);
			return false;
		}
		Candidate.BodySceneTransforms[BodyIndex] = ConvertPose(Position, Quat);
	}
	Candidate.CameraSceneTransforms.SetNum(CameraMountPlans.Num());
	for (int32 CameraIndex = 0; CameraIndex < CameraMountPlans.Num(); ++CameraIndex)
	{
		const FCameraMountPlan& Plan = CameraMountPlans[CameraIndex];
		if (Plan.bRequiresExplicitWorldPose || !Candidate.BodySceneTransforms.IsValidIndex(Plan.BodyIndex))
		{
			OutError = FString::Printf(TEXT("camera %d requires an explicit world pose"), CameraIndex);
			return false;
		}
		Candidate.CameraSceneTransforms[CameraIndex] = Plan.BodyLocalCaptureTransform
			* Candidate.BodySceneTransforms[Plan.BodyIndex];
	}
	if (bHasUserCamera) Candidate.UserCameraSceneTransform = FTransform::Identity;
	Candidate.LightSceneTransforms.SetNum(NumLights);
	for (int32 LightIndex = 0; LightIndex < NumLights; ++LightIndex)
	{
		if (!ConvertLightPose(LightPositions.GetData() + 3 * LightIndex,
			LightDirections.GetData() + 3 * LightIndex,
			Candidate.LightSceneTransforms[LightIndex]))
		{
			OutError = FString::Printf(TEXT("light world pose %d is invalid"), LightIndex);
			return false;
		}
	}
	if (!Candidate.IsValid(NumBodies, CameraMountPlans.Num(), bHasUserCamera,
		NumLights, SceneRevision, OutError)) return false;
	Out = MoveTemp(Candidate);
	return true;
}

bool FMjRenderSceneAdapter::FrameFromCartesianWithCameras(
	TConstArrayView<double> BodyPositions, TConstArrayView<double> BodyQuaternions,
	TConstArrayView<double> CameraPositions, TConstArrayView<double> CameraQuaternions,
	TConstArrayView<double> LightPositions, TConstArrayView<double> LightDirections,
	uint64 FrameId, double SourceTime, bool bHasSourceTime, uint64 ClockEpoch,
	FRenderFrame& Out, FString& OutError) const
{
	if (CameraPositions.Num() != 3 * CameraMountPlans.Num()
		|| CameraQuaternions.Num() != 4 * CameraMountPlans.Num())
	{
		OutError = TEXT("explicit camera Cartesian arrays have the wrong size");
		return false;
	}
	FRenderFrame Candidate;
	if (!FrameFromCartesian(BodyPositions, BodyQuaternions, LightPositions, LightDirections,
		FrameId, SourceTime,
		bHasSourceTime, ClockEpoch, Candidate, OutError))
	{
		// Tracking cameras make the body-derived path reject by design; build the
		// body portion directly here, then replace every camera with its explicit pose.
		Candidate = FRenderFrame();
		Candidate.Identity = {SceneRevision, FrameId, SourceTime, ClockEpoch, bHasSourceTime};
		Candidate.BodySceneTransforms.SetNum(NumBodies);
		if (BodyPositions.Num() != 3 * NumBodies || BodyQuaternions.Num() != 4 * NumBodies)
			return false;
		for (int32 BodyIndex = 0; BodyIndex < NumBodies; ++BodyIndex)
		{
			const double* Position = BodyPositions.GetData() + 3 * BodyIndex;
			const double* Quat = BodyQuaternions.GetData() + 4 * BodyIndex;
			if (!FinitePosition(Position) || !FiniteQuat(Quat)) return false;
			Candidate.BodySceneTransforms[BodyIndex] = ConvertPose(Position, Quat);
		}
	}
	Candidate.CameraSceneTransforms.SetNum(CameraMountPlans.Num());
	for (int32 CameraIndex = 0; CameraIndex < CameraMountPlans.Num(); ++CameraIndex)
	{
		const double* Position = CameraPositions.GetData() + 3 * CameraIndex;
		const double* Quat = CameraQuaternions.GetData() + 4 * CameraIndex;
		if (!FinitePosition(Position) || !FiniteQuat(Quat))
		{
			OutError = FString::Printf(TEXT("camera world pose %d is invalid"), CameraIndex);
			return false;
		}
		Candidate.CameraSceneTransforms[CameraIndex] = ConvertCameraPose(Position, Quat);
	}
	if (bHasUserCamera && !Candidate.UserCameraSceneTransform.IsSet())
		Candidate.UserCameraSceneTransform = FTransform::Identity;
	Candidate.LightSceneTransforms.SetNum(NumLights);
	for (int32 LightIndex = 0; LightIndex < NumLights; ++LightIndex)
	{
		if (!ConvertLightPose(LightPositions.GetData() + 3 * LightIndex,
			LightDirections.GetData() + 3 * LightIndex,
			Candidate.LightSceneTransforms[LightIndex]))
		{
			OutError = FString::Printf(TEXT("light world pose %d is invalid"), LightIndex);
			return false;
		}
	}
	if (!Candidate.IsValid(NumBodies, CameraMountPlans.Num(), bHasUserCamera,
		NumLights, SceneRevision, OutError)) return false;
	Out = MoveTemp(Candidate);
	return true;
}

bool FMjRenderSceneAdapter::FrameFromSnapshot(
	const FMjSimulationCartesianSnapshot& Snapshot, uint64 ExpectedModelGeneration,
	uint64 FrameId, FRenderFrame& Out, FString& OutError) const
{
	OutError.Reset();
	if (ExpectedModelGeneration == 0
		|| Snapshot.Identity.Model.Generation != ExpectedModelGeneration)
	{
		OutError = FString::Printf(
			TEXT("Cartesian snapshot model generation %llu does not match installed generation %llu"),
			static_cast<unsigned long long>(Snapshot.Identity.Model.Generation),
			static_cast<unsigned long long>(ExpectedModelGeneration));
		return false;
	}
	if (Snapshot.Identity.ClockEpoch == 0 || Snapshot.Identity.StateRevision == 0
		|| !FMath::IsFinite(Snapshot.Identity.Time))
	{
		OutError = TEXT("Cartesian snapshot identity is incomplete or non-finite");
		return false;
	}
	return FrameFromCartesianWithCameras(Snapshot.BodyPositions, Snapshot.BodyQuaternions,
		Snapshot.CameraPositions, Snapshot.CameraQuaternions,
		Snapshot.LightPositions, Snapshot.LightDirections, FrameId,
		Snapshot.Identity.Time, /*bHasSourceTime=*/true, Snapshot.Identity.ClockEpoch,
		Out, OutError);
}

bool FMjRenderSceneAdapter::RebaseSceneRevision(const FString& ExpectedRevision,
	const FRenderScene& PatchedScene, FString& OutError)
{
	OutError.Reset();
	if (ExpectedRevision.IsEmpty() || ExpectedRevision != SceneRevision
		|| PatchedScene.SceneRevision.IsEmpty()
		|| PatchedScene.SceneRevision == SceneRevision)
	{
		OutError = TEXT("scene revision rebase requires the active revision and a distinct new revision");
		return false;
	}
	if (PatchedScene.Bodies.Num() != NumBodies
		|| PatchedScene.Cameras.Num() != CameraMountPlans.Num()
		|| PatchedScene.Lights.Num() != NumLights
		|| PatchedScene.Deformables.Num() != Deformables.Num())
	{
		OutError = TEXT("scene patch changes installed MuJoCo kinematics topology");
		return false;
	}
	for (int32 Index = 0; Index < Deformables.Num(); ++Index)
		if (PatchedScene.Deformables[Index].Name != Deformables[Index].Name
			|| PatchedScene.Deformables[Index].VertexCount != Deformables[Index].VertexCount
			|| PatchedScene.Deformables[Index].Triangles != Deformables[Index].Triangles
			|| PatchedScene.Deformables[Index].UVs != Deformables[Index].UVs)
		{
			OutError = TEXT("scene patch changes installed deformable topology");
			return false;
		}
	Visualization = PatchedScene.Visualization;
	SceneRevision = PatchedScene.SceneRevision;
	return true;
}

bool FMjRenderSceneAdapter::UserCameraTransform(const double* Position, const double* Forward,
	const double* Up, FTransform& Out, FString& OutError)
{
	if (!Position || !Forward || !Up || !FinitePosition(Position)
		|| !FinitePosition(Forward) || !FinitePosition(Up))
	{
		OutError = TEXT("user camera eye vectors are incomplete or non-finite");
		return false;
	}
	double F[3] = {Forward[0], Forward[1], Forward[2]};
	double U[3] = {Up[0], Up[1], Up[2]};
	if (mju_normalize3(F) < 1.e-9 || mju_normalize3(U) < 1.e-9)
	{
		OutError = TEXT("user camera forward or up vector is degenerate");
		return false;
	}
	double Z[3] = {-F[0], -F[1], -F[2]};
	double X[3]; mju_cross(X, U, Z);
	if (mju_normalize3(X) < 1.e-9)
	{
		OutError = TEXT("user camera forward and up vectors are parallel");
		return false;
	}
	double Y[3]; mju_cross(Y, Z, X);
	const double Mat[9] = {X[0], Y[0], Z[0], X[1], Y[1], Z[1], X[2], Y[2], Z[2]};
	double Quat[4]; mju_mat2Quat(Quat, Mat);
	Out = ConvertCameraPose(Position, Quat);
	OutError.Reset();
	return true;
}
