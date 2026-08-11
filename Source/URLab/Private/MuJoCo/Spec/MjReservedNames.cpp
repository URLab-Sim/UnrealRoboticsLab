// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjReservedNames.h"

#if URLAB_MJ_GEN

#include "MjSpecNodes.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace
{
/**
 * The `mjtObj` family an element compiles into, or `mjOBJ_UNKNOWN`.
 *
 * The name a reservation mints carries the family's own spelling, and it is
 * minted before any spec element exists to read `elemtype` off -- the names go
 * on the components, which is where the build reads them from. So the mapping
 * is stated here rather than recovered from the built spec.
 *
 * Section containers (`<asset>`, `<worldbody>`, `<actuator>`, ...) and
 * authoring-only elements (`<default>`, `<frame>`) are deliberately unknown:
 * they exist in the spec and not in the model, so naming them for binding's
 * sake would be inventing identity for something MuJoCo never gives an id to.
 */
int32 ObjTypeOfElement(psm::ElementType Type)
{
	switch (Type)
	{
		case psm::ElementType::Body:
			return mjOBJ_BODY;
		case psm::ElementType::Joint:
		case psm::ElementType::FreeJoint:
			return mjOBJ_JOINT;
		case psm::ElementType::Geom:
			return mjOBJ_GEOM;
		case psm::ElementType::Site:
			return mjOBJ_SITE;
		case psm::ElementType::Camera:
			return mjOBJ_CAMERA;
		case psm::ElementType::Light:
			return mjOBJ_LIGHT;
		case psm::ElementType::Mesh:
			return mjOBJ_MESH;
		case psm::ElementType::Skin:
			return mjOBJ_SKIN;
		case psm::ElementType::Hfield:
			return mjOBJ_HFIELD;
		case psm::ElementType::Texture:
			return mjOBJ_TEXTURE;
		case psm::ElementType::Material:
			return mjOBJ_MATERIAL;
		case psm::ElementType::Pair:
			return mjOBJ_PAIR;
		case psm::ElementType::Exclude:
			return mjOBJ_EXCLUDE;
		case psm::ElementType::Connect:
		case psm::ElementType::Weld:
		case psm::ElementType::EqualityJoint:
		case psm::ElementType::EqualityTendon:
		case psm::ElementType::EqualityFlex:
		case psm::ElementType::Flexvert:
		case psm::ElementType::Flexstrain:
			return mjOBJ_EQUALITY;
		case psm::ElementType::Spatial:
		case psm::ElementType::Fixed:
			return mjOBJ_TENDON;
		case psm::ElementType::ActuatorGeneral:
		case psm::ElementType::Motor:
		case psm::ElementType::Position:
		case psm::ElementType::Velocity:
		case psm::ElementType::IntVelocity:
		case psm::ElementType::OrientationActuator:
		case psm::ElementType::Pid:
		case psm::ElementType::Damper:
		case psm::ElementType::Cylinder:
		case psm::ElementType::Muscle:
		case psm::ElementType::Adhesion:
		case psm::ElementType::DcMotor:
		case psm::ElementType::ActuatorPlugin:
			return mjOBJ_ACTUATOR;
		case psm::ElementType::Touch:
		case psm::ElementType::Accelerometer:
		case psm::ElementType::Velocimeter:
		case psm::ElementType::Gyro:
		case psm::ElementType::Force:
		case psm::ElementType::Torque:
		case psm::ElementType::Magnetometer:
		case psm::ElementType::Camprojection:
		case psm::ElementType::Rangefinder:
		case psm::ElementType::Jointpos:
		case psm::ElementType::Jointvel:
		case psm::ElementType::Tendonpos:
		case psm::ElementType::Tendonvel:
		case psm::ElementType::Actuatorpos:
		case psm::ElementType::Actuatorvel:
		case psm::ElementType::Actuatorfrc:
		case psm::ElementType::Jointactuatorfrc:
		case psm::ElementType::Tendonactuatorfrc:
		case psm::ElementType::Ballquat:
		case psm::ElementType::Ballangvel:
		case psm::ElementType::Jointlimitpos:
		case psm::ElementType::Jointlimitvel:
		case psm::ElementType::Jointlimitfrc:
		case psm::ElementType::Tendonlimitpos:
		case psm::ElementType::Tendonlimitvel:
		case psm::ElementType::Tendonlimitfrc:
		case psm::ElementType::Framepos:
		case psm::ElementType::Framequat:
		case psm::ElementType::Framexaxis:
		case psm::ElementType::Frameyaxis:
		case psm::ElementType::Framezaxis:
		case psm::ElementType::Framelinvel:
		case psm::ElementType::Frameangvel:
		case psm::ElementType::Framelinacc:
		case psm::ElementType::Frameangacc:
		case psm::ElementType::Subtreecom:
		case psm::ElementType::Subtreelinvel:
		case psm::ElementType::Subtreeangmom:
		case psm::ElementType::Insidesite:
		case psm::ElementType::Distance:
		case psm::ElementType::Normal:
		case psm::ElementType::Fromto:
		case psm::ElementType::SensorContact:
		case psm::ElementType::EPotential:
		case psm::ElementType::EKinetic:
		case psm::ElementType::Clock:
		case psm::ElementType::Tactile:
		case psm::ElementType::SensorUser:
		case psm::ElementType::SensorPlugin:
			return mjOBJ_SENSOR;
		case psm::ElementType::Numeric:
			return mjOBJ_NUMERIC;
		case psm::ElementType::Text:
			return mjOBJ_TEXT;
		case psm::ElementType::Tuple:
			return mjOBJ_TUPLE;
		case psm::ElementType::Key:
			return mjOBJ_KEY;
		case psm::ElementType::Flex:
			return mjOBJ_FLEX;
		case psm::ElementType::PluginInstance:
			return mjOBJ_PLUGIN;
		default:
			return mjOBJ_UNKNOWN;
	}
}
} // namespace

FMjReservedNames::FMjReservedNames(const FSpecRef& Spec)
{
	// The ordinal each family is up to, counting only the reservations this walk
	// mints. `MjSpecNodesOf` hands the tree back in spec order, so the ordinal an
	// element receives is a property of the document rather than of the session
	// that read it: the same spec reserves the same names in every process.
	TMap<int32, int32> NextOrdinal;

	const FMjSpecNodes Tree = MjSpecNodesOf(Spec);
	for (UMjNodeComponent* Node : Tree.Nodes)
	{
		psm::ElementType Type{};
		if (Node == nullptr || !gen::ElementTypeOfNode(*Node, Type))
		{
			continue;
		}
		const int32 ObjType = ObjTypeOfElement(Type);
		if (ObjType == mjOBJ_UNKNOWN || Tree.Unnamable.Contains(Node))
		{
			continue;
		}
		if (Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty())
		{
			continue;
		}
		if (!MjAssetElementName(*Node).IsEmpty())
		{
			continue;
		}
		const int32 Ordinal = ++NextOrdinal.FindOrAdd(ObjType, 0);
		Node->MjName = FString::Printf(
			TEXT("%s%s:%d"), MjReservedNamePrefix, UTF8_TO_TCHAR(mju_type2Str(ObjType)), Ordinal);
		Renamed.Add(Node);
	}
}

FMjReservedNames::~FMjReservedNames()
{
	for (UMjNodeComponent* Node : Renamed)
	{
		Node->MjName.Reset();
	}
}

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
